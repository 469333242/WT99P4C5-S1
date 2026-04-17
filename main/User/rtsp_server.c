/**
 * @file rtsp_server.c
 * @brief RTSP/RTP 视频流服务器实现（H.264 编码）
 *
 * 协议栈：RTSP over TCP，RTP interleaved（RFC 2326 §10.12）
 * 视频编码：H.264 over RTP（RFC 6184）
 * 访问地址：rtsp://<设备IP>:8554/stream
 *
 * 架构：
 *   - rtsp_server_task : 接受连接 + RTSP 握手（core 1）
 *   - rtsp_send_task   : 每个客户端独立发送任务，从队列取帧发送（core 0）
 *   - rtsp_push_h264_frame : 摄像头任务调用，复制帧到队列
 *
 * H.264 RTP 封装模式：
 *   - 单 NALU 模式：NALU < MTU，直接封装
 *   - FU-A 分片模式：NALU >= MTU，分片传输
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "rtsp_server.h"

static const char *TAG = "rtsp";

/* 客户端播放状态变化回调 */
static rtsp_playing_cb_t s_playing_cb = NULL;

/* ------------------------------------------------------------------ */
/* 配置                                                                 */
/* ------------------------------------------------------------------ */
#define MAX_CLIENTS         2
#define RTP_MTU             1400        /* WiFi环境最佳MTU */
#define RTSP_BUF_SIZE       1024
#define RTSP_SERVER_TASK_STACK (12 * 1024)
#define RTSP_SEND_TASK_CORE 0
#define RTP_HEADER_LEN      12
#define RTCP_INTERLEAVE_HDR 4
#define FRAME_QUEUE_LEN     2           /* Two-frame queue absorbs short jitter without adding much latency. */
#define FRAME_POOL_SIZE     (FRAME_QUEUE_LEN + 1) /* 至少保留 1 个发送中缓冲 */
#define MAX_FRAME_SIZE      (200*1024)  /* H.264 最大帧 200KB */
#define SEND_TIMEOUT_MS     80          /* 首帧/IDR 允许更充足的发送窗口，避免黑屏 */
#define RTSP_TCP_SNDBUF     (32 * 1024) /* 保持较小缓存，但不要小到压掉关键帧 */
#define RTSP_TCP_RCVBUF     (8 * 1024)
#define KEEPALIVE_IDLE_SEC  10          /* TCP Keepalive空闲时间 */
#define KEEPALIVE_INTERVAL_SEC 5        /* TCP Keepalive探测间隔 */
#define KEEPALIVE_COUNT     3           /* TCP Keepalive探测次数 */
#define MAX_SEND_ERRORS     10          /* 最大连续发送错误次数 */
#define MAX_SEND_RETRIES    2           /* 保留少量重试，优先保证首帧出图 */
#define SEND_RETRY_DELAY_MS 2           /* 发送失败后短暂让出 CPU */
#define MAX_NALUS_PER_FRAME 16          /* 每帧最大NALU数量 */
#define RTSP_TRY_LOCK_TICKS 0           /* Keep best-effort paths non-blocking across FreeRTOS tick-rate changes. */
#define FRAME_POOL_INVALID_IDX UINT8_MAX

/* H.264 NALU 类型 */
#define H264_NALU_TYPE_MASK 0x1F
#define H264_NALU_TYPE_SPS  7
#define H264_NALU_TYPE_PPS  8
#define H264_NALU_TYPE_IDR  5
#define H264_NALU_TYPE_SLICE 1

/* FU-A 分片标志 */
#define FU_A_TYPE           28
#define FU_S_BIT            0x80        /* Start bit */
#define FU_E_BIT            0x40        /* End bit */

/* ------------------------------------------------------------------ */
/* 帧缓冲                                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t                *data;
    size_t                  len;
    esp_h264_frame_type_t   frame_type;
} frame_buf_t;

/* NALU信息（用于一次性解析，避免重复扫描）*/
typedef struct {
    size_t offset;      /* NALU在帧中的偏移 */
    size_t len;         /* NALU长度（包含header）*/
    uint8_t type;       /* NALU类型 */
} nalu_info_t;

typedef struct {
    uint8_t *data;
    size_t len;
    esp_h264_frame_type_t frame_type;
    uint32_t pts;
    uint8_t pool_idx;
    nalu_info_t nalus[MAX_NALUS_PER_FRAME];
    int nalu_count;
} frame_with_nalus_t;

/* ------------------------------------------------------------------ */
/* 客户端状态                                                           */
/* ------------------------------------------------------------------ */
typedef enum {
    CLIENT_IDLE = 0,
    CLIENT_CONNECTED,
    CLIENT_PLAYING,
} client_state_t;

typedef struct {
    int            fd;
    client_state_t state;
    uint16_t       rtp_seq;
    uint32_t       rtp_ts;
    uint32_t       ssrc;
    int            rtp_channel;
    QueueHandle_t  frame_queue;
    TaskHandle_t   send_task;
    uint8_t       *sps;              /* SPS NALU 缓存 */
    size_t         sps_len;
    uint8_t       *pps;              /* PPS NALU 缓存 */
    size_t         pps_len;
    uint32_t       frames_sent;      /* 已发送帧数统计 */
    uint32_t       frames_dropped;   /* 丢帧统计 */
    TickType_t     last_send_time;   /* 上次发送时间（用于检测阻塞）*/
    uint32_t       send_errors;      /* 连续发送错误计数 */
    uint32_t       total_bytes_sent; /* 总发送字节数 */
    TickType_t     connect_time;     /* 连接建立时间 */
    TaskHandle_t   session_task;     /* 持有该会话的服务任务 */
    bool           wait_for_idr;     /* Drop delta frames until the next IDR after loss or reconnect. */
    uint8_t       *frame_pool[FRAME_POOL_SIZE];  /* 预分配帧缓冲池 */
    bool           pool_in_use[FRAME_POOL_SIZE]; /* 缓冲占用状态 */
} rtsp_client_t;

static rtsp_client_t s_clients[MAX_CLIENTS];
static SemaphoreHandle_t s_clients_mutex;
static int s_playing_count = 0;

typedef struct {
    uint32_t frames_sent;
    uint32_t bytes_sent;
    uint32_t last_pts;
    bool     last_pts_valid;
} rtsp_tx_stats_internal_t;

static rtsp_tx_stats_internal_t s_tx_stats;
static portMUX_TYPE s_tx_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static TickType_t s_last_no_nalu_log = 0;

/* ------------------------------------------------------------------ */
/* 内部：更新播放计数并触发回调                                          */
/* ------------------------------------------------------------------ */

static void update_playing_count(int delta)
{
    int prev = s_playing_count;
    s_playing_count += delta;
    if (s_playing_count < 0) s_playing_count = 0;

    if (!s_playing_cb) return;

    if (prev == 0 && s_playing_count == 1) {
        s_playing_cb(true);
    } else if (prev == 1 && s_playing_count == 0) {
        s_playing_cb(false);
    }
}

static uint32_t get_active_client_count(void)
{
    uint32_t active = 0;

    if (!s_clients_mutex) {
        return 0;
    }

    if (xSemaphoreTake(s_clients_mutex, RTSP_TRY_LOCK_TICKS) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (s_clients[i].state == CLIENT_PLAYING) {
                active++;
            }
        }
        xSemaphoreGive(s_clients_mutex);
    }

    return active;
}

static void note_frame_sent_once(uint32_t pts, size_t len)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    if (!s_tx_stats.last_pts_valid || s_tx_stats.last_pts != pts) {
        s_tx_stats.frames_sent++;
        s_tx_stats.bytes_sent += (uint32_t)len;
        s_tx_stats.last_pts = pts;
        s_tx_stats.last_pts_valid = true;
    }
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

void rtsp_reset_tx_stats(void)
{
    portENTER_CRITICAL(&s_tx_stats_lock);
    memset(&s_tx_stats, 0, sizeof(s_tx_stats));
    portEXIT_CRITICAL(&s_tx_stats_lock);
}

void rtsp_take_tx_stats(rtsp_tx_stats_t *stats)
{
    if (!stats) {
        return;
    }

    portENTER_CRITICAL(&s_tx_stats_lock);
    stats->frames_sent = s_tx_stats.frames_sent;
    stats->bytes_sent = s_tx_stats.bytes_sent;
    memset(&s_tx_stats, 0, sizeof(s_tx_stats));
    portEXIT_CRITICAL(&s_tx_stats_lock);

    stats->active_clients = get_active_client_count();
}

static bool replace_cached_nalu_if_needed(uint8_t **dst, size_t *dst_len,
                                          const uint8_t *src, size_t src_len)
{
    if (!dst || !dst_len || !src || src_len == 0) {
        return false;
    }

    if (*dst && *dst_len == src_len && memcmp(*dst, src, src_len) == 0) {
        return true;
    }

    uint8_t *new_buf = malloc(src_len);
    if (!new_buf) {
        return false;
    }

    memcpy(new_buf, src, src_len);
    free(*dst);
    *dst = new_buf;
    *dst_len = src_len;
    return true;
}

static uint8_t reserve_frame_slot_locked(rtsp_client_t *c)
{
    for (uint8_t i = 0; i < FRAME_POOL_SIZE; i++) {
        if (!c->pool_in_use[i]) {
            c->pool_in_use[i] = true;
            return i;
        }
    }

    return FRAME_POOL_INVALID_IDX;
}

static void release_frame_slot_locked(rtsp_client_t *c, uint8_t pool_idx)
{
    if (pool_idx < FRAME_POOL_SIZE) {
        c->pool_in_use[pool_idx] = false;
    }
}

static void flush_client_queue_locked(rtsp_client_t *c)
{
    frame_with_nalus_t queued_frame;

    while (xQueueReceive(c->frame_queue, &queued_frame, 0) == pdTRUE) {
        release_frame_slot_locked(c, queued_frame.pool_idx);
    }
}

static void mark_client_wait_for_idr_locked(rtsp_client_t *c)
{
    if (!c) {
        return;
    }

    c->wait_for_idr = true;
    flush_client_queue_locked(c);
}

/* ------------------------------------------------------------------ */
/* H.264 NALU 解析                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief 一次性解析所有NALU（避免重复扫描）
 *
 * @param h264_data  H.264 数据
 * @param h264_len   数据长度
 * @param nalus      输出NALU信息数组
 * @param max_nalus  最大NALU数量
 * @return NALU数量
 */
static int parse_nalus_once(const uint8_t *h264_data, size_t h264_len,
                            nalu_info_t *nalus, int max_nalus)
{
    int count = 0;
    size_t i = 0;

    while (i + 4 <= h264_len && count < max_nalus) {
        /* 查找起始码 0x00000001 或 0x000001 */
        if (h264_data[i] == 0 && h264_data[i+1] == 0) {
            int start_code_len = 0;
            if (h264_data[i+2] == 1) {
                start_code_len = 3;
            } else if (h264_data[i+2] == 0 && h264_data[i+3] == 1) {
                start_code_len = 4;
            }

            if (start_code_len > 0) {
                size_t nalu_start = i + start_code_len;
                if (nalu_start >= h264_len) break;

                /* 查找下一个起始码 */
                size_t next = nalu_start + 1;
                bool found_next_start = false;
                while (next + 3 < h264_len) {
                    if (h264_data[next] == 0 && h264_data[next+1] == 0 &&
                        (h264_data[next+2] == 1 ||
                         (h264_data[next+2] == 0 && next+3 < h264_len && h264_data[next+3] == 1))) {
                        found_next_start = true;
                        break;
                    }
                    next++;
                }
                if (!found_next_start) {
                    next = h264_len;
                }

                nalus[count].offset = nalu_start;
                nalus[count].len = next - nalu_start;
                nalus[count].type = h264_data[nalu_start] & H264_NALU_TYPE_MASK;
                count++;

                i = next;
                continue;
            }
        }
        i++;
    }

    return count;
}

/**
 * @brief 从 H.264 流中提取 SPS 和 PPS
 *
 * @param h264_data  H.264 数据（可能包含多个 NALU）
 * @param h264_len   数据长度
 * @param sps_out    输出 SPS 指针
 * @param sps_len    输出 SPS 长度
 * @param pps_out    输出 PPS 指针
 * @param pps_len    输出 PPS 长度
 * @return 0 成功，-1 失败
 */
static int extract_sps_pps(const uint8_t *h264_data, size_t h264_len,
                           uint8_t **sps_out, size_t *sps_len,
                           uint8_t **pps_out, size_t *pps_len)
{
    nalu_info_t nalus[MAX_NALUS_PER_FRAME];
    int nalu_count = parse_nalus_once(h264_data, h264_len, nalus, MAX_NALUS_PER_FRAME);

    *sps_out = NULL;
    *pps_out = NULL;
    *sps_len = 0;
    *pps_len = 0;

    for (int i = 0; i < nalu_count; i++) {
        if (nalus[i].type == H264_NALU_TYPE_SPS) {
            *sps_out = (uint8_t *)&h264_data[nalus[i].offset];
            *sps_len = nalus[i].len;
        } else if (nalus[i].type == H264_NALU_TYPE_PPS) {
            *pps_out = (uint8_t *)&h264_data[nalus[i].offset];
            *pps_len = nalus[i].len;
        }
    }

    return (*sps_out && *pps_out) ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* RTP 发送                                                             */
/* ------------------------------------------------------------------ */

static int send_rtp_packet(int fd, int channel, const uint8_t *rtp, size_t rtp_len)
{
    /* 合并 header 和 RTP 包到一个缓冲区，减少系统调用 */
    uint8_t *send_buf = alloca(RTCP_INTERLEAVE_HDR + rtp_len);

    send_buf[0] = '$';
    send_buf[1] = (uint8_t)channel;
    send_buf[2] = (uint8_t)((rtp_len >> 8) & 0xFF);
    send_buf[3] = (uint8_t)(rtp_len & 0xFF);
    memcpy(send_buf + RTCP_INTERLEAVE_HDR, rtp, rtp_len);

    /* 循环发送，确保所有数据都发送完成 */
    size_t total = RTCP_INTERLEAVE_HDR + rtp_len;
    size_t sent_total = 0;
    int retry_count = 0;
    const int max_retries = MAX_SEND_RETRIES;

    while (sent_total < total) {
        ssize_t sent = send(fd, send_buf + sent_total, total - sent_total, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* 发送缓冲区满，等待一小段时间后重试 */
                retry_count++;
                if (retry_count >= max_retries) {
                    /* 超时，快速失败让上层丢弃此帧 */
                    return -1;
                }
                vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_DELAY_MS));
                continue;
            }
            /* 其他错误（连接断开等）*/
            return -1;
        } else if (sent == 0) {
            /* 连接已关闭 */
            return -1;
        }
        sent_total += sent;
        retry_count = 0;  /* 发送成功，重置重试计数 */
    }

    return 0;
}

/**
 * @brief 发送单个 H.264 NALU（单 NALU 模式或 FU-A 分片模式）
 *
 * @param c         客户端上下文
 * @param nalu      NALU 数据（包含 NALU header）
 * @param nalu_len  NALU 长度
 * @param marker    RTP marker bit（帧结束标志）
 * @return 0 成功，-1 失败
 */
static int rtp_send_h264_nalu(rtsp_client_t *c, const uint8_t *nalu, size_t nalu_len, int marker)
{
    uint8_t pkt[RTP_HEADER_LEN + 2 + RTP_MTU];  /* RTP header + FU header + payload */

    if (nalu_len <= RTP_MTU) {
        /* 单 NALU 模式：NALU 小于 MTU，直接封装 */
        pkt[0]  = 0x80;                             /* V=2, P=0, X=0, CC=0 */
        pkt[1]  = (uint8_t)(96 | (marker << 7));   /* PT=96 (H.264), M bit */
        pkt[2]  = (uint8_t)(c->rtp_seq >> 8);
        pkt[3]  = (uint8_t)(c->rtp_seq & 0xFF);
        pkt[4]  = (uint8_t)(c->rtp_ts >> 24);
        pkt[5]  = (uint8_t)(c->rtp_ts >> 16);
        pkt[6]  = (uint8_t)(c->rtp_ts >> 8);
        pkt[7]  = (uint8_t)(c->rtp_ts & 0xFF);
        pkt[8]  = (uint8_t)(c->ssrc >> 24);
        pkt[9]  = (uint8_t)(c->ssrc >> 16);
        pkt[10] = (uint8_t)(c->ssrc >> 8);
        pkt[11] = (uint8_t)(c->ssrc & 0xFF);
        c->rtp_seq++;

        memcpy(&pkt[RTP_HEADER_LEN], nalu, nalu_len);
        return send_rtp_packet(c->fd, c->rtp_channel, pkt, RTP_HEADER_LEN + nalu_len);

    } else {
        /* FU-A 分片模式：NALU 大于 MTU，需要分片 */
        uint8_t nalu_hdr = nalu[0];
        uint8_t nalu_type = nalu_hdr & H264_NALU_TYPE_MASK;
        uint8_t fu_indicator = (nalu_hdr & 0xE0) | FU_A_TYPE;  /* F+NRI + Type=28 */

        const uint8_t *payload = nalu + 1;  /* 跳过 NALU header */
        size_t payload_len = nalu_len - 1;
        bool first = true;

        while (payload_len > 0) {
            size_t chunk = (payload_len > RTP_MTU) ? RTP_MTU : payload_len;
            int pkt_marker = (payload_len <= RTP_MTU && marker) ? 1 : 0;

            /* RTP header */
            pkt[0]  = 0x80;
            pkt[1]  = (uint8_t)(96 | (pkt_marker << 7));
            pkt[2]  = (uint8_t)(c->rtp_seq >> 8);
            pkt[3]  = (uint8_t)(c->rtp_seq & 0xFF);
            pkt[4]  = (uint8_t)(c->rtp_ts >> 24);
            pkt[5]  = (uint8_t)(c->rtp_ts >> 16);
            pkt[6]  = (uint8_t)(c->rtp_ts >> 8);
            pkt[7]  = (uint8_t)(c->rtp_ts & 0xFF);
            pkt[8]  = (uint8_t)(c->ssrc >> 24);
            pkt[9]  = (uint8_t)(c->ssrc >> 16);
            pkt[10] = (uint8_t)(c->ssrc >> 8);
            pkt[11] = (uint8_t)(c->ssrc & 0xFF);
            c->rtp_seq++;

            /* FU indicator + FU header */
            pkt[RTP_HEADER_LEN] = fu_indicator;
            pkt[RTP_HEADER_LEN + 1] = nalu_type;
            if (first) {
                pkt[RTP_HEADER_LEN + 1] |= FU_S_BIT;  /* Start bit */
                first = false;
            }
            if (payload_len <= RTP_MTU) {
                pkt[RTP_HEADER_LEN + 1] |= FU_E_BIT;  /* End bit */
            }

            /* Payload */
            memcpy(&pkt[RTP_HEADER_LEN + 2], payload, chunk);

            if (send_rtp_packet(c->fd, c->rtp_channel, pkt, RTP_HEADER_LEN + 2 + chunk) < 0) {
                return -1;
            }

            payload += chunk;
            payload_len -= chunk;
        }
        return 0;
    }
}

/**
 * @brief 发送完整的 H.264 帧（使用预解析的NALU信息，避免重复扫描）
 *
 * @param c           客户端上下文
 * @param h264_data   H.264 数据
 * @param nalus       预解析的NALU信息
 * @param nalu_count  NALU数量
 * @return 0 成功，-1 失败
 */
static int rtp_send_h264_frame_fast(rtsp_client_t *c, const uint8_t *h264_data,
                                     const nalu_info_t *nalus, int nalu_count,
                                     uint32_t rtp_ts)
{
    c->rtp_ts = rtp_ts;

    for (int i = 0; i < nalu_count; i++) {
        int marker = (i == nalu_count - 1) ? 1 : 0;
        if (rtp_send_h264_nalu(c, &h264_data[nalus[i].offset], nalus[i].len, marker) < 0) {
            return -1;
        }
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* 每客户端发送任务                                                     */
/* ------------------------------------------------------------------ */

static void rtsp_send_task(void *arg)
{
    rtsp_client_t *c = (rtsp_client_t *)arg;
    frame_with_nalus_t frame;
    TickType_t last_stats = xTaskGetTickCount();

    while (1) {
        /* 等待入队通知，零延迟唤醒；超时 20ms 用于检测连接状态 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (xQueueReceive(c->frame_queue, &frame, 0) == pdTRUE) {
            TickType_t send_start = xTaskGetTickCount();
            int send_ret = rtp_send_h264_frame_fast(c, frame.data, frame.nalus,
                                                    frame.nalu_count, frame.pts);

            xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
            release_frame_slot_locked(c, frame.pool_idx);
            xSemaphoreGive(s_clients_mutex);

            /* 使用预解析的NALU信息快速发送 */
            if (send_ret < 0) {
                c->send_errors++;
                c->frames_dropped++;
                xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
                mark_client_wait_for_idr_locked(c);
                xSemaphoreGive(s_clients_mutex);
                ESP_LOGW(TAG, "RTP 发送失败 fd=%d (连续错误: %"PRIu32")，丢弃此帧", c->fd, c->send_errors);

                /* 只有连续错误超过阈值才退出 */
                if (c->send_errors >= MAX_SEND_ERRORS) {
                    ESP_LOGE(TAG, "RTP 连续发送失败超过 %d 次，断开连接 fd=%d", MAX_SEND_ERRORS, c->fd);
                    break;
                }

                /* 发送失败直接丢弃此帧，继续处理下一帧 */
                continue;
            }

            /* 发送成功，重置错误计数 */
            c->send_errors = 0;
            if (c->frames_sent == 0) {
                ESP_LOGI(TAG, "首帧发送成功 fd=%d | pts=%"PRIu32" | bytes=%zu | type=%d",
                         c->fd, frame.pts, frame.len, (int)frame.frame_type);
            }
            c->frames_sent++;
            c->total_bytes_sent += frame.len;
            c->last_send_time = xTaskGetTickCount();
            note_frame_sent_once(frame.pts, frame.len);

            /* 检测发送阻塞（单帧发送超过100ms表示网络拥塞）*/
            TickType_t send_duration = c->last_send_time - send_start;
            if (send_duration > pdMS_TO_TICKS(100)) {
                ESP_LOGW(TAG, "发送阻塞检测 fd=%d | 耗时: %"PRIu32" ms | 帧大小: %zu bytes",
                         c->fd, send_duration * portTICK_PERIOD_MS, frame.len);
            }

            /* 每10秒输出统计 */
            TickType_t now = xTaskGetTickCount();
            if ((now - last_stats) >= pdMS_TO_TICKS(10000)) {
                uint32_t elapsed_ms = (now - last_stats) * portTICK_PERIOD_MS;
                float actual_bitrate = (float)c->total_bytes_sent * 8.0f / elapsed_ms;  /* kbps */
                float drop_rate = (c->frames_dropped + c->frames_sent > 0) ?
                                  (float)c->frames_dropped * 100.0f / (c->frames_dropped + c->frames_sent) : 0.0f;

                //ESP_LOGI(TAG, "客户端 fd=%d 统计 | 已发送: %"PRIu32" 帧 | 丢帧: %"PRIu32" (%.1f%%) | 码率: %.0f kbps",
                //         c->fd, c->frames_sent, c->frames_dropped, drop_rate, actual_bitrate);
                c->frames_sent = 0;
                c->frames_dropped = 0;
                c->total_bytes_sent = 0;
                last_stats = now;
            }
        } else {
            /* 超时检查连接状态 */
            xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
            client_state_t st = c->state;
            xSemaphoreGive(s_clients_mutex);
            if (st != CLIENT_PLAYING) {
                break;
            }
        }
    }

    /* 通知主任务断连 */
    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    if (c->fd >= 0) {
        shutdown(c->fd, SHUT_RDWR);
    }
    c->send_task = NULL;
    TaskHandle_t session_task = c->session_task;
    xSemaphoreGive(s_clients_mutex);

    if (session_task) {
        xTaskNotifyGive(session_task);
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* rtsp_push_h264_frame                                                 */
/* ------------------------------------------------------------------ */

void rtsp_push_h264_frame(const uint8_t *h264_buf, size_t h264_len,
                          esp_h264_frame_type_t frame_type, uint32_t pts)
{
    if (!h264_buf || h264_len == 0 || h264_len > MAX_FRAME_SIZE) return;

    /* 一次性解析所有NALU（避免后续重复扫描）*/
    nalu_info_t nalus[MAX_NALUS_PER_FRAME];
    int nalu_count = parse_nalus_once(h264_buf, h264_len, nalus, MAX_NALUS_PER_FRAME);
    if (nalu_count == 0) {
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_no_nalu_log) >= pdMS_TO_TICKS(1000)) {
            uint8_t head[8] = {0};
            size_t dump_len = (h264_len < sizeof(head)) ? h264_len : sizeof(head);

            memcpy(head, h264_buf, dump_len);
            s_last_no_nalu_log = now;
            ESP_LOGW(TAG,
                     "未解析到NALU | len=%zu | type=%d | bytes=%02X %02X %02X %02X %02X %02X %02X %02X",
                     h264_len, (int)frame_type,
                     head[0], head[1], head[2], head[3],
                     head[4], head[5], head[6], head[7]);
        }
        return;
    }

    /* 如果是 IDR 帧，提取 SPS/PPS 并缓存 */
    if (frame_type == ESP_H264_FRAME_TYPE_IDR) {
        uint8_t *sps = NULL, *pps = NULL;
        size_t sps_len = 0, pps_len = 0;

        /* 从已解析的NALU中查找SPS/PPS */
        for (int i = 0; i < nalu_count; i++) {
            if (nalus[i].type == H264_NALU_TYPE_SPS) {
                sps = (uint8_t *)&h264_buf[nalus[i].offset];
                sps_len = nalus[i].len;
            } else if (nalus[i].type == H264_NALU_TYPE_PPS) {
                pps = (uint8_t *)&h264_buf[nalus[i].offset];
                pps_len = nalus[i].len;
            }
        }

        if (sps && pps) {
            if (xSemaphoreTake(s_clients_mutex, RTSP_TRY_LOCK_TICKS) == pdTRUE) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (s_clients[i].state != CLIENT_PLAYING) continue;

                    /* SPS/PPS 一般不会频繁变化，只有内容变化时才重建缓存 */
                    if (!replace_cached_nalu_if_needed(&s_clients[i].sps, &s_clients[i].sps_len,
                                                       sps, sps_len)) {
                        ESP_LOGW(TAG, "更新 SPS 缓存失败 client=%d", i);
                    }
                    if (!replace_cached_nalu_if_needed(&s_clients[i].pps, &s_clients[i].pps_len,
                                                       pps, pps_len)) {
                        ESP_LOGW(TAG, "更新 PPS 缓存失败 client=%d", i);
                    }
                }
                xSemaphoreGive(s_clients_mutex);
            }
        }
    }

    /* 为每个活跃客户端预留一块独立发送缓冲，避免发送中被下一帧覆盖 */
    frame_with_nalus_t frames[MAX_CLIENTS];
    memset(frames, 0, sizeof(frames));
    if (xSemaphoreTake(s_clients_mutex, RTSP_TRY_LOCK_TICKS) != pdTRUE) {
        return;  /* 锁被占用，直接丢弃此帧以保持低延迟 */
    }
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].state != CLIENT_PLAYING) {
            continue;
        }

        if (s_clients[i].wait_for_idr) {
            if (frame_type != ESP_H264_FRAME_TYPE_IDR) {
                continue;
            }
            flush_client_queue_locked(&s_clients[i]);
        }

        uint8_t pool_idx = reserve_frame_slot_locked(&s_clients[i]);
        if (pool_idx == FRAME_POOL_INVALID_IDX) {
            frame_with_nalus_t old_frame;

            if (xQueueReceive(s_clients[i].frame_queue, &old_frame, 0) == pdTRUE) {
                release_frame_slot_locked(&s_clients[i], old_frame.pool_idx);
                s_clients[i].frames_dropped++;
                mark_client_wait_for_idr_locked(&s_clients[i]);
                pool_idx = reserve_frame_slot_locked(&s_clients[i]);
            }
        }

        if (pool_idx == FRAME_POOL_INVALID_IDX) {
            s_clients[i].frames_dropped++;
            mark_client_wait_for_idr_locked(&s_clients[i]);
            continue;
        }

        if (s_clients[i].wait_for_idr && frame_type != ESP_H264_FRAME_TYPE_IDR) {
            release_frame_slot_locked(&s_clients[i], pool_idx);
            continue;
        }

        frames[i].pool_idx = pool_idx;
        frames[i].data = s_clients[i].frame_pool[pool_idx];
    }
    xSemaphoreGive(s_clients_mutex);

    /* 锁外准备帧数据（缓冲已预留，不会与发送线程冲突）*/
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (frames[i].data && h264_len <= MAX_FRAME_SIZE) {
            memcpy(frames[i].data, h264_buf, h264_len);
            frames[i].len = h264_len;
            frames[i].frame_type = frame_type;
            frames[i].pts = pts;
            memcpy(frames[i].nalus, nalus, sizeof(nalu_info_t) * nalu_count);
            frames[i].nalu_count = nalu_count;
        }
    }

    /* 入队阶段必须完成释放或提交，避免预留缓冲泄漏 */
    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!frames[i].data) continue;
        if (s_clients[i].state != CLIENT_PLAYING) {
            release_frame_slot_locked(&s_clients[i], frames[i].pool_idx);
            continue;
        }

        if (xQueueSend(s_clients[i].frame_queue, &frames[i], 0) != pdTRUE) {
            release_frame_slot_locked(&s_clients[i], frames[i].pool_idx);
            s_clients[i].frames_dropped++;
            mark_client_wait_for_idr_locked(&s_clients[i]);
        } else {
            if (frames[i].frame_type == ESP_H264_FRAME_TYPE_IDR) {
                s_clients[i].wait_for_idr = false;
            }
            /* 入队成功后立即通知发送任务，避免 10ms 轮询延迟 */
            if (s_clients[i].send_task) {
                xTaskNotifyGive(s_clients[i].send_task);
            }
        }
    }
    xSemaphoreGive(s_clients_mutex);
}

/* ------------------------------------------------------------------ */
/* RTSP 握手                                                            */
/* ------------------------------------------------------------------ */

static int parse_cseq(const char *req)
{
    const char *p = strstr(req, "CSeq:");
    if (!p) p = strstr(req, "CSEQ:");
    if (!p) return 1;
    return atoi(p + 5);
}

static int parse_interleaved_channel(const char *req)
{
    const char *p = strstr(req, "interleaved=");
    if (!p) return 0;
    return atoi(p + 12);
}

static bool is_tcp_interleaved_request(const char *req)
{
    return strstr(req, "RTP/AVP/TCP") != NULL && strstr(req, "interleaved=") != NULL;
}

static bool extract_request_uri(const char *req, char *dst, size_t dst_size)
{
    const char *sp1;
    const char *sp2;
    size_t len;

    if (!req || !dst || dst_size == 0) {
        return false;
    }

    sp1 = strchr(req, ' ');
    if (!sp1) {
        return false;
    }

    sp2 = strchr(sp1 + 1, ' ');
    if (!sp2 || sp2 <= sp1 + 1) {
        return false;
    }

    len = (size_t)(sp2 - (sp1 + 1));
    if (len >= dst_size) {
        len = dst_size - 1;
    }

    memcpy(dst, sp1 + 1, len);
    dst[len] = '\0';
    return true;
}

static void build_base_uri(const char *req, char *dst, size_t dst_size)
{
    if (!extract_request_uri(req, dst, dst_size)) {
        snprintf(dst, dst_size, "rtsp://0.0.0.0:%d/stream/", RTSP_PORT);
        return;
    }

    size_t len = strlen(dst);
    if (len > 0 && dst[len - 1] != '/' && len + 1 < dst_size) {
        dst[len] = '/';
        dst[len + 1] = '\0';
    }
}

static void build_track_uri(const char *req, char *dst, size_t dst_size)
{
    if (!extract_request_uri(req, dst, dst_size)) {
        snprintf(dst, dst_size, "rtsp://0.0.0.0:%d/stream/trackID=1", RTSP_PORT);
        return;
    }

    if (strstr(dst, "trackID=") || strstr(dst, "streamid=")) {
        return;
    }

    if (dst[0] != '\0' && dst[strlen(dst) - 1] != '/') {
        strncat(dst, "/", dst_size - strlen(dst) - 1);
    }
    strncat(dst, "trackID=1", dst_size - strlen(dst) - 1);
}

/**
 * @brief Base64 编码（用于 SPS/PPS）
 */
static void base64_encode(const uint8_t *src, size_t len, char *dst, size_t dst_size)
{
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t i = 0;
    size_t j = 0;

    while (i < len && j + 4 < dst_size) {
        size_t remain = len - i;
        uint32_t val = (uint32_t)src[i] << 16;
        if (remain > 1) val |= (uint32_t)src[i + 1] << 8;
        if (remain > 2) val |= (uint32_t)src[i + 2];

        dst[j++] = b64[(val >> 18) & 0x3F];
        dst[j++] = b64[(val >> 12) & 0x3F];
        dst[j++] = (remain > 1) ? b64[(val >> 6) & 0x3F] : '=';
        dst[j++] = (remain > 2) ? b64[val & 0x3F] : '=';

        i += (remain >= 3) ? 3 : remain;
    }
    dst[j] = '\0';
}

static void log_rtsp_request(rtsp_client_t *c, const char *req)
{
    const char *line_end = strstr(req, "\r\n");
    size_t line_len = line_end ? (size_t)(line_end - req) : strnlen(req, RTSP_BUF_SIZE);

    if (line_len > 120) {
        line_len = 120;
    }

    ESP_LOGI(TAG, "RTSP request fd=%d: %.*s", c->fd, (int)line_len, req);
}

static esp_err_t start_streaming(rtsp_client_t *c)
{
    bool need_start_task = false;

    xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
    if (c->state != CLIENT_PLAYING) {
        mark_client_wait_for_idr_locked(c);
        c->state = CLIENT_PLAYING;
        update_playing_count(+1);
        need_start_task = true;
    }
    xSemaphoreGive(s_clients_mutex);

    if (!need_start_task) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(rtsp_send_task, "rtsp_send",
                                             8 * 1024, c, 14, &c->send_task, RTSP_SEND_TASK_CORE);
    if (ret != pdPASS) {
        xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
        if (c->state == CLIENT_PLAYING) {
            c->state = CLIENT_CONNECTED;
            update_playing_count(-1);
        }
        c->send_task = NULL;
        xSemaphoreGive(s_clients_mutex);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTSP PLAY started fd=%d ch=%d", c->fd, c->rtp_channel);
    return ESP_OK;
}

static bool process_rtsp_request(rtsp_client_t *c, const char *req, bool *session_done)
{
    int cseq = parse_cseq(req);
    char resp[1536];

    log_rtsp_request(c, req);

    if (strncmp(req, "OPTIONS", 7) == 0) {
        snprintf(resp, sizeof(resp),
                 "RTSP/1.0 200 OK\r\n"
                 "CSeq: %d\r\n"
                 "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n",
                 cseq);
        send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
        return true;
    }

    if (strncmp(req, "DESCRIBE", 8) == 0) {
        char *sps_b64 = calloc(1, 256);
        char *pps_b64 = calloc(1, 256);
        char *sprop = calloc(1, 600);
        char *sdp = calloc(1, 768);
        char base_uri[192];
        char profile_level_id[16] = {0};

        if (!sps_b64 || !pps_b64 || !sprop || !sdp) {
            free(sps_b64);
            free(pps_b64);
            free(sprop);
            free(sdp);
            ESP_LOGE(TAG, "No memory for DESCRIBE response");
            *session_done = true;
            return false;
        }

        build_base_uri(req, base_uri, sizeof(base_uri));

        if (c->sps && c->pps) {
            base64_encode(c->sps, c->sps_len, sps_b64, 256);
            base64_encode(c->pps, c->pps_len, pps_b64, 256);
            if (c->sps_len >= 4) {
                snprintf(profile_level_id, sizeof(profile_level_id), "%02X%02X%02X",
                         c->sps[1], c->sps[2], c->sps[3]);
            }
            snprintf(sprop, 600,
                     "a=fmtp:96 packetization-mode=1;profile-level-id=%s;sprop-parameter-sets=%s,%s\r\n",
                     profile_level_id[0] ? profile_level_id : "42C01E", sps_b64, pps_b64);
        } else {
            snprintf(sprop, 600, "a=fmtp:96 packetization-mode=1;profile-level-id=42C01E\r\n");
        }

        snprintf(sdp, 768,
                 "v=0\r\n"
                 "o=- 0 0 IN IP4 0.0.0.0\r\n"
                 "s=H264 Stream\r\n"
                 "t=0 0\r\n"
                 "a=control:*\r\n"
                 "m=video 0 RTP/AVP 96\r\n"
                 "a=rtpmap:96 H264/90000\r\n"
                 "%s"
                 "a=control:trackID=1\r\n",
                 sprop);

        snprintf(resp, sizeof(resp),
                 "RTSP/1.0 200 OK\r\n"
                 "CSeq: %d\r\n"
                 "Content-Base: %s\r\n"
                 "Content-Type: application/sdp\r\n"
                 "Content-Length: %d\r\n\r\n%s",
                 cseq, base_uri, (int)strlen(sdp), sdp);
        send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

        free(sps_b64);
        free(pps_b64);
        free(sprop);
        free(sdp);
        return true;
    }

    if (strncmp(req, "SETUP", 5) == 0) {
        if (!is_tcp_interleaved_request(req)) {
            ESP_LOGW(TAG, "Unsupported SETUP transport on fd=%d, only RTP/AVP/TCP interleaved is supported", c->fd);
            snprintf(resp, sizeof(resp),
                     "RTSP/1.0 461 Unsupported Transport\r\n"
                     "CSeq: %d\r\n\r\n",
                     cseq);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
            return false;
        }

        c->rtp_channel = parse_interleaved_channel(req);
        snprintf(resp, sizeof(resp),
                 "RTSP/1.0 200 OK\r\n"
                 "CSeq: %d\r\n"
                 "Session: 1\r\n"
                 "Transport: RTP/AVP/TCP;unicast;interleaved=%d-%d;ssrc=%08"PRIx32";mode=\"PLAY\"\r\n\r\n",
                 cseq, c->rtp_channel, c->rtp_channel + 1, c->ssrc);
        send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
        return true;
    }

    if (strncmp(req, "PLAY", 4) == 0) {
        char track_uri[192];
        build_track_uri(req, track_uri, sizeof(track_uri));
        snprintf(resp, sizeof(resp),
                 "RTSP/1.0 200 OK\r\n"
                 "CSeq: %d\r\n"
                 "Session: 1\r\n"
                 "Range: npt=0.000-\r\n"
                 "RTP-Info: url=%s;seq=%u;rtptime=%"PRIu32"\r\n\r\n",
                 cseq, track_uri, (unsigned)c->rtp_seq, c->rtp_ts);
        send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

        if (start_streaming(c) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start RTSP send task for fd=%d", c->fd);
            *session_done = true;
        }
        return true;
    }

    if (strncmp(req, "TEARDOWN", 8) == 0) {
        snprintf(resp, sizeof(resp),
                 "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 1\r\n\r\n", cseq);
        send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
        *session_done = true;
        return true;
    }

    ESP_LOGW(TAG, "Unsupported RTSP request fd=%d", c->fd);
    snprintf(resp, sizeof(resp),
             "RTSP/1.0 501 Not Implemented\r\n"
             "CSeq: %d\r\n\r\n",
             cseq);
    send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
    return false;
}

static void __attribute__((unused)) handle_rtsp_session(rtsp_client_t *c)
{
    char buf[RTSP_BUF_SIZE];

    while (1) {
        int n = recv(c->fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        buf[n] = '\0';

        int cseq = parse_cseq(buf);
        char resp[1024];

        if (strncmp(buf, "OPTIONS", 7) == 0) {
            snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\n"
                "CSeq: %d\r\n"
                "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n",
                cseq);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

        } else if (strncmp(buf, "DESCRIBE", 8) == 0) {
            /* 构造 SDP（使用缓存的 SPS/PPS，如果有的话）*/
            char sps_b64[256] = {0};
            char pps_b64[256] = {0};
            char sprop[600] = {0};  /* 增大缓冲区：56 + 255 + 255 = 566 字节 */

            if (c->sps && c->pps) {
                base64_encode(c->sps, c->sps_len, sps_b64, sizeof(sps_b64));
                base64_encode(c->pps, c->pps_len, pps_b64, sizeof(pps_b64));
                snprintf(sprop, sizeof(sprop), "a=fmtp:96 packetization-mode=1;sprop-parameter-sets=%s,%s\r\n",
                         sps_b64, pps_b64);
            } else {
                snprintf(sprop, sizeof(sprop), "a=fmtp:96 packetization-mode=1\r\n");
            }

            char sdp[768];
            snprintf(sdp, sizeof(sdp),
                "v=0\r\n"
                "o=- 0 0 IN IP4 0.0.0.0\r\n"
                "s=H264 Stream\r\n"
                "t=0 0\r\n"
                "m=video 0 RTP/AVP 96\r\n"
                "a=rtpmap:96 H264/90000\r\n"
                "%s"
                "a=control:streamid=0\r\n",
                sprop);

            snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\n"
                "CSeq: %d\r\n"
                "Content-Type: application/sdp\r\n"
                "Content-Length: %d\r\n\r\n%s",
                cseq, (int)strlen(sdp), sdp);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

        } else if (strncmp(buf, "SETUP", 5) == 0) {
            c->rtp_channel = parse_interleaved_channel(buf);
            snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\n"
                "CSeq: %d\r\n"
                "Session: 1\r\n"
                "Transport: RTP/AVP/TCP;unicast;interleaved=%d-%d\r\n\r\n",
                cseq, c->rtp_channel, c->rtp_channel + 1);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

        } else if (strncmp(buf, "PLAY", 4) == 0) {
            snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\n"
                "CSeq: %d\r\n"
                "Session: 1\r\n"
                "Range: npt=0.000-\r\n\r\n",
                cseq);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);

            /* 启动发送任务 */
            xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
            c->state = CLIENT_PLAYING;
            update_playing_count(+1);
            xSemaphoreGive(s_clients_mutex);

            xTaskCreatePinnedToCore(rtsp_send_task, "rtsp_send",
                                    8 * 1024, c, 14, &c->send_task, RTSP_SEND_TASK_CORE);
            ESP_LOGI(TAG, "客户端 fd=%d 开始播放 (ch=%d)", c->fd, c->rtp_channel);

            /* 等待客户端断开或 TEARDOWN */
            while (recv(c->fd, buf, sizeof(buf), 0) > 0) {
                if (strncmp(buf, "TEARDOWN", 8) == 0) break;
            }
            break;

        } else if (strncmp(buf, "TEARDOWN", 8) == 0) {
            snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\nCSeq: %d\r\nSession: 1\r\n\r\n", cseq);
            send(c->fd, resp, strlen(resp), MSG_NOSIGNAL);
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* RTSP 服务器主任务                                                    */
/* ------------------------------------------------------------------ */

static void handle_rtsp_session_stream(rtsp_client_t *c)
{
    char buf[RTSP_BUF_SIZE];
    size_t used = 0;
    bool session_done = false;

    while (!session_done) {
        int n = recv(c->fd, buf + used, sizeof(buf) - 1 - used, 0);
        if (n <= 0) {
            break;
        }

        used += (size_t)n;
        buf[used] = '\0';

        char *cursor = buf;
        char *buffer_end = buf + used;

        while (cursor < buffer_end) {
            if (*cursor == '$') {
                if ((size_t)(buffer_end - cursor) < RTCP_INTERLEAVE_HDR) {
                    break;
                }

                size_t packet_len = RTCP_INTERLEAVE_HDR +
                                    (((uint8_t)cursor[2] << 8) | (uint8_t)cursor[3]);
                if ((size_t)(buffer_end - cursor) < packet_len) {
                    break;
                }

                cursor += packet_len;
                continue;
            }

            char *msg_end = strstr(cursor, "\r\n\r\n");
            if (!msg_end) {
                break;
            }

            char saved = *msg_end;
            *msg_end = '\0';

            process_rtsp_request(c, cursor, &session_done);
            *msg_end = saved;
            cursor = msg_end + 4;

            if (session_done) {
                return;
            }
        }

        used = (size_t)(buffer_end - cursor);
        if (used > 0) {
            memmove(buf, cursor, used);
        }

        if (used == sizeof(buf) - 1) {
            ESP_LOGW(TAG, "RTSP request too large, closing fd=%d", c->fd);
            break;
        }
    }
}

static void rtsp_server_task(void *arg)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() 失败: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(RTSP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() 失败: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_fd, MAX_CLIENTS);
    ESP_LOGI(TAG, "RTSP 服务器监听端口 %d", RTSP_PORT);

    while (1) {
        struct sockaddr_in cli_addr;
        socklen_t cli_len = sizeof(cli_addr);
        int cli_fd = accept(listen_fd, (struct sockaddr *)&cli_addr, &cli_len);
        if (cli_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* 限制 socket 侧积压，避免旧帧在内核发送队列中滞留 */
        struct timeval tv = {
            .tv_sec = SEND_TIMEOUT_MS / 1000,
            .tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000
        };
        setsockopt(cli_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        /* 缩小发送缓冲，避免旧 RTP 数据在 socket 中排队形成额外延迟 */
        int sndbuf = RTSP_TCP_SNDBUF;
        setsockopt(cli_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        /* RTSP 控制流很小，保留适度接收缓冲即可 */
        int rcvbuf = RTSP_TCP_RCVBUF;
        setsockopt(cli_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        /* 禁用 Nagle 算法（降低延迟）*/
        int nodelay = 1;
        setsockopt(cli_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        /* 启用 TCP Keepalive（检测死连接）*/
        int keepalive = 1;
        setsockopt(cli_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        int keepidle = KEEPALIVE_IDLE_SEC;
        int keepintvl = KEEPALIVE_INTERVAL_SEC;
        int keepcnt = KEEPALIVE_COUNT;
        setsockopt(cli_fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
        setsockopt(cli_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
        setsockopt(cli_fd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));

        int slot = -1;
        xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (s_clients[i].state == CLIENT_IDLE) { slot = i; break; }
        }
        if (slot >= 0) {
            s_clients[slot].fd          = cli_fd;
            s_clients[slot].state       = CLIENT_CONNECTED;
            s_clients[slot].rtp_seq     = 0;
            s_clients[slot].rtp_ts      = 0;
            s_clients[slot].ssrc        = (uint32_t)(slot + 1) * 0x12345678;
            s_clients[slot].rtp_channel = 0;
            s_clients[slot].send_task   = NULL;
            s_clients[slot].frames_sent = 0;
            s_clients[slot].frames_dropped = 0;
            s_clients[slot].last_send_time = 0;
            s_clients[slot].send_errors = 0;
            s_clients[slot].total_bytes_sent = 0;
            s_clients[slot].connect_time = xTaskGetTickCount();
            s_clients[slot].session_task = xTaskGetCurrentTaskHandle();
            s_clients[slot].wait_for_idr = true;
        }
        xSemaphoreGive(s_clients_mutex);

        if (slot < 0) {
            ESP_LOGW(TAG, "无空闲客户端槽位，拒绝连接 fd=%d", cli_fd);
            close(cli_fd);
            continue;
        }

        ESP_LOGI(TAG, "客户端已连接，slot=%d fd=%d", slot, cli_fd);
        handle_rtsp_session_stream(&s_clients[slot]);

        /* 会话结束，清理 */
        xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
        if (s_clients[slot].state == CLIENT_PLAYING) {
            update_playing_count(-1);
        }
        s_clients[slot].state = CLIENT_IDLE;
        close(s_clients[slot].fd);
        s_clients[slot].fd = -1;

        /* 释放 SPS/PPS 缓存 */
        if (s_clients[slot].sps) {
            free(s_clients[slot].sps);
            s_clients[slot].sps = NULL;
        }
        if (s_clients[slot].pps) {
            free(s_clients[slot].pps);
            s_clients[slot].pps = NULL;
        }

        TaskHandle_t send_task = s_clients[slot].send_task;
        xSemaphoreGive(s_clients_mutex);

        /* 等待发送任务退出，避免复用旧缓冲 */
        if (send_task) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        }

        xSemaphoreTake(s_clients_mutex, portMAX_DELAY);
        flush_client_queue_locked(&s_clients[slot]);
        memset(s_clients[slot].pool_in_use, 0, sizeof(s_clients[slot].pool_in_use));
        s_clients[slot].session_task = NULL;
        xSemaphoreGive(s_clients_mutex);

        ESP_LOGI(TAG, "客户端已断开，slot=%d", slot);
    }
}

/* ------------------------------------------------------------------ */
/* rtsp_set_playing_callback                                            */
/* ------------------------------------------------------------------ */

void rtsp_set_playing_callback(rtsp_playing_cb_t cb)
{
    s_playing_cb = cb;
}

/* ------------------------------------------------------------------ */
/* rtsp_server_start                                                    */
/* ------------------------------------------------------------------ */

esp_err_t rtsp_server_start(void)
{
    memset(s_clients, 0, sizeof(s_clients));
    for (int i = 0; i < MAX_CLIENTS; i++) {
        s_clients[i].fd = -1;
        s_clients[i].frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_with_nalus_t));
        if (!s_clients[i].frame_queue) return ESP_ERR_NO_MEM;

        /* 预分配帧缓冲池（避免运行时malloc）*/
        for (int j = 0; j < FRAME_POOL_SIZE; j++) {
            s_clients[i].frame_pool[j] = heap_caps_malloc(MAX_FRAME_SIZE,
                                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_clients[i].frame_pool[j]) {
                ESP_LOGE(TAG, "预分配帧缓冲池失败 client=%d buf=%d", i, j);
                return ESP_ERR_NO_MEM;
            }
        }
    }

    s_clients_mutex = xSemaphoreCreateMutex();
    if (!s_clients_mutex) return ESP_ERR_NO_MEM;

    rtsp_reset_tx_stats();

    BaseType_t ret = xTaskCreatePinnedToCore(
        rtsp_server_task, "rtsp_srv", RTSP_SERVER_TASK_STACK, NULL, 5, NULL, 1);
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
