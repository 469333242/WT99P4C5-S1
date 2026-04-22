/**
 * @file camera.c
 * @brief OV5647 MIPI-CSI 摄像头采集 + H.264 硬件编码 + RTSP 推流
 *
 * 硬件：WT99P4C5-S1 (ESP32-P4)，HBV-ZERO 摄像头（OV5647 感光芯片）
 * 接口：MIPI-CSI 2-lane，SCCB I2C（SCL=GPIO8, SDA=GPIO7）
 * 分辨率：1280x960
 * 流程：V4L2 采集（RAW8） → 硬件ISP转换（YUV420） → H.264 硬件编码 → rtsp_push_h264_frame()
 *
 * 优化说明：
 * - 使用硬件ISP将OV5647的RAW8格式转换为YUV420
 * - 使用 H.264 硬件编码器，大幅降低码率和延迟
 * - 完全依赖硬件处理，无软件格式转换
 * - 低延迟 GOP 配置，在首帧等待与编码吞吐之间折中
 */

#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_h264_enc_single_hw.h"
#include "camera.h"
#include "media_storage.h"
#include "rtsp_server.h"
#include "../../../managed_components/espressif__esp_cam_sensor/sensors/ov5647/private_include/ov5647_settings.h"

static const char *TAG = "camera";

#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS
static const esp_cam_sensor_isp_info_t s_ov5647_isp_1280x960 = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 88333333,
        .vts = 1796,
        .hts = 1093,
        .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
    }
};

static const esp_cam_sensor_format_t s_ov5647_fmt_1280x960 = {
    .name = "MIPI_2lane_24Minput_RAW10_1280x960_binning_45fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 1280,
    .height = 960,
    .regs = ov5647_mipi_2lane_24Minput_1280x960_raw10_45fps,
    .regs_size = sizeof(ov5647_mipi_2lane_24Minput_1280x960_raw10_45fps) / sizeof(ov5647_mipi_2lane_24Minput_1280x960_raw10_45fps[0]),
    .fps = 45,
    .isp_info = &s_ov5647_isp_1280x960,
    .mipi_info = {
        .mipi_clk = OV5647_MIPI_CSI_LINE_RATE_1280x960_45FPS,
        .lane_num = 2,
        .line_sync_en = CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? true : false,
    },
    .reserved = NULL,
};
#endif

#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1920X1080_30FPS
static const esp_cam_sensor_isp_info_t s_ov5647_isp_1920x1080 = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 81666700,
        .vts = 1104,
        .hts = 2416,
        .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
    }
};

static const esp_cam_sensor_format_t s_ov5647_fmt_1920x1080 = {
    .name = "MIPI_2lane_24Minput_RAW10_1920x1080_30fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 1920,
    .height = 1080,
    .regs = ov5647_mipi_2lane_24Minput_1920x1080_raw10_30fps,
    .regs_size = sizeof(ov5647_mipi_2lane_24Minput_1920x1080_raw10_30fps) / sizeof(ov5647_mipi_2lane_24Minput_1920x1080_raw10_30fps[0]),
    .fps = 30,
    .isp_info = &s_ov5647_isp_1920x1080,
    .mipi_info = {
        .mipi_clk = OV5647_MIPI_CSI_LINE_RATE_1920x1080_30FPS,
        .lane_num = 2,
        .line_sync_en = CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? true : false,
    },
    .reserved = NULL,
};
#endif

#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X800_50FPS
static const esp_cam_sensor_isp_info_t s_ov5647_isp_800x800 = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 81666700,
        .vts = 1896,
        .hts = 984,
        .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
    }
};

static const esp_cam_sensor_format_t s_ov5647_fmt_800x800 = {
    .name = "MIPI_2lane_24Minput_RAW8_800x800_50fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 800,
    .height = 800,
    .regs = ov5647_mipi_2lane_24Minput_800x800_raw8_50fps,
    .regs_size = sizeof(ov5647_mipi_2lane_24Minput_800x800_raw8_50fps) / sizeof(ov5647_mipi_2lane_24Minput_800x800_raw8_50fps[0]),
    .fps = 50,
    .isp_info = &s_ov5647_isp_800x800,
    .mipi_info = {
        .mipi_clk = OV5647_MIPI_CSI_LINE_RATE_800x800_50FPS,
        .lane_num = 2,
        .line_sync_en = CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? true : false,
    },
    .reserved = NULL,
};
#endif

#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X640_50FPS
static const esp_cam_sensor_isp_info_t s_ov5647_isp_800x640 = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 81666700,
        .vts = 1896,
        .hts = 984,
        .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
    }
};

static const esp_cam_sensor_format_t s_ov5647_fmt_800x640 = {
    .name = "MIPI_2lane_24Minput_RAW8_800x640_50fps",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 800,
    .height = 640,
    .regs = ov5647_mipi_2lane_24Minput_800x640_raw8_50fps,
    .regs_size = sizeof(ov5647_mipi_2lane_24Minput_800x640_raw8_50fps) / sizeof(ov5647_mipi_2lane_24Minput_800x640_raw8_50fps[0]),
    .fps = 50,
    .isp_info = &s_ov5647_isp_800x640,
    .mipi_info = {
        .mipi_clk = OV5647_MIPI_CSI_LINE_RATE_800x640_50FPS,
        .lane_num = 2,
        .line_sync_en = CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? true : false,
    },
    .reserved = NULL,
};
#endif

/* ------------------------------------------------------------------ */
/* 配置参数                                                             */
/* ------------------------------------------------------------------ */
#define CAM_BUF_COUNT       6           /* 低延迟与稳吞吐折中，避免采集侧缓冲过浅 */
#define CAM_SCCB_PORT       0           /* I2C 端口号 */
#define CAM_SCCB_SCL_PIN    8           /* SCCB SCL → GPIO8 */
#define CAM_SCCB_SDA_PIN    7           /* SCCB SDA → GPIO7 */
#define CAM_SCCB_FREQ       100000      /* SCCB 频率 100kHz */
#define CAM_RESET_PIN       (-1)        /* 无复位引脚 */
#define CAM_PWDN_PIN        (-1)        /* 无掉电引脚 */
#define CAM_STALE_FLUSH_MAX CAM_BUF_COUNT /* 开播时最多丢弃当前队列中的陈旧帧 */
#define CAM_STATS_REPORT_INTERVAL_US (10ULL * 1000 * 1000) /* 10秒调试统计输出周期 */

/* H.264 编码参数
 * 快速切换：只修改 H264_PROFILE
 * 1 = 1280x960
 * 2 = 1920x1080
 * 3 = 800x800
 * 4 = 800x640
 */
#define H264_PROFILE_1280X960   1
#define H264_PROFILE_1920X1080  2
#define H264_PROFILE_800X800    3
#define H264_PROFILE_800X640    4

#define H264_PROFILE            H264_PROFILE_1280X960

#if H264_PROFILE == H264_PROFILE_1280X960
#define H264_WIDTH          1280         /* 编码宽度 */
#define H264_HEIGHT         960          /* 编码高度 */
#define H264_FPS            45           /* 双客户端默认降到 30fps，降低链路堆积 */
#define H264_GOP            4            /* 约 89ms 一个 IDR，优先保证 RTSP 快速恢复 */
#define H264_BITRATE        3500000      /* 平均码率，静止时略低于，运动时略高于 */
#define H264_QP_MIN         28           /* 最小QP */
#define H264_QP_MAX         34           /* 最大QP */
#elif H264_PROFILE == H264_PROFILE_1920X1080
#define H264_WIDTH          1920         /* 编码宽度 */
#define H264_HEIGHT         1080         /* 编码高度 */
#define H264_FPS            30           /* 目标编码帧率 */
#define H264_GOP            4            /* 约 133ms 一个 IDR，兼顾低延迟与编码吞吐 */
#define H264_BITRATE        5000000      /* 码率5.0Mbps */
#define H264_QP_MIN         30           /* 最小QP */
#define H264_QP_MAX         44           /* 最大QP */
#elif H264_PROFILE == H264_PROFILE_800X800
#define H264_WIDTH          800          /* 编码宽度 */
#define H264_HEIGHT         800          /* 编码高度 */
#define H264_FPS            50           /* 目标编码帧率 */
#define H264_GOP            6            /* 约 120ms 一个 IDR，兼顾低延迟与编码吞吐 */
#define H264_BITRATE        3000000      /* 码率3.0Mbps */
#define H264_QP_MIN         28           /* 最小QP */
#define H264_QP_MAX         42           /* 最大QP */
#elif H264_PROFILE == H264_PROFILE_800X640
#define H264_WIDTH          800          /* 编码宽度 */
#define H264_HEIGHT         640          /* 编码高度 */
#define H264_FPS            50           /* 目标编码帧率 */
#define H264_GOP            6            /* 约 120ms 一个 IDR，兼顾低延迟与编码吞吐 */
#define H264_BITRATE        1800000      /* 码率1.8Mbps */
#define H264_QP_MIN         26           /* 最小QP */
#define H264_QP_MAX         40           /* 最大QP */
#else
#error "不支持的 H264_PROFILE"
#endif
#define H264_OUT_BUF_FACTOR 3            /* 输出缓冲倍数 */
#define H264_RTP_TS_STEP    (90000 / H264_FPS) /* RTP/H.264 共用 90kHz 时基 */

/* ------------------------------------------------------------------ */
/* 内部状态                                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    int                      fd;                    /* V4L2 设备文件描述符 */
    uint8_t                 *buf[CAM_BUF_COUNT];    /* MMAP 缓冲指针 */
    size_t                   buf_size;              /* 单个缓冲大小 */
    uint32_t                 width;                 /* 图像宽度 */
    uint32_t                 height;                /* 图像高度 */
    uint32_t                 pixfmt;                /* 像素格式 */
    esp_h264_enc_handle_t    h264_enc;              /* H.264 编码器句柄 */
    uint8_t                 *h264_out_buf;          /* H.264 输出缓冲 */
    size_t                   h264_out_buf_size;     /* 输出缓冲大小 */
    uint32_t                 pts;                   /* 当前时间戳 */
} cam_ctx_t;

static cam_ctx_t s_cam;

/* 用于控制采集任务启停的事件组 */
static EventGroupHandle_t s_cam_event;
#define CAM_START_BIT   BIT0    /* 置位：有客户端，开始采集 */

static void flush_stale_capture_buffers(void)
{
    uint32_t flushed = 0;

    for (uint32_t i = 0; i < CAM_STALE_FLUSH_MAX; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(s_cam.fd, VIDIOC_DQBUF, &buf) != 0) {
            if (errno != EAGAIN) {
                ESP_LOGW(TAG, "刷新陈旧采集帧失败: %d", errno);
            }
            break;
        }

        if (ioctl(s_cam.fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGW(TAG, "回收陈旧采集帧失败: %d", errno);
            break;
        }

        flushed++;
    }

    if (flushed > 0) {
        ESP_LOGI(TAG, "已丢弃 %"PRIu32" 个陈旧采集缓冲，开始实时推流", flushed);
    }
}

/**
 * @brief RTSP 播放状态回调
 *
 * 由 RTSP 服务器在客户端数量 0↔1 变化时调用。
 * 通过事件组通知 cam_task 启动或暂停采集。
 *
 * @param playing  true = 开始采集；false = 暂停采集
 */
static void on_rtsp_playing(bool playing)
{
    if (playing) {
        ESP_LOGI(TAG, "客户端已连接，启动摄像头采集");
        rtsp_reset_tx_stats();
        media_storage_start_video_record();
        xEventGroupSetBits(s_cam_event, CAM_START_BIT);
    } else {
        ESP_LOGI(TAG, "客户端已断开，暂停摄像头采集");
        rtsp_reset_tx_stats();
        media_storage_stop_video_record();
        xEventGroupClearBits(s_cam_event, CAM_START_BIT);
    }
}

static void __attribute__((unused)) log_sensor_format(const char *stage)
{
    esp_cam_sensor_format_t sensor_fmt = {0};

    if (ioctl(s_cam.fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        ESP_LOGW(TAG, "%s | 获取传感器模式失败: errno=%d", stage, errno);
        return;
    }

    ESP_LOGI(TAG,
             "%s | name=%s | %ux%u @ %u fps | sensor_fmt=%d | mipi_clk=%"PRIu32" Hz | lane=%"PRIu32" | line_sync=%s",
             stage,
             sensor_fmt.name ? sensor_fmt.name : "unknown",
             (unsigned)sensor_fmt.width,
             (unsigned)sensor_fmt.height,
             (unsigned)sensor_fmt.fps,
             (int)sensor_fmt.format,
             sensor_fmt.mipi_info.mipi_clk,
             sensor_fmt.mipi_info.lane_num,
             sensor_fmt.mipi_info.line_sync_en ? "on" : "off");
}

static const esp_cam_sensor_format_t *get_target_ov5647_sensor_format(uint32_t width, uint32_t height)
{
#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS
    if (width == 1280 && height == 960) {
        return &s_ov5647_fmt_1280x960;
    }
#endif
#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1920X1080_30FPS
    if (width == 1920 && height == 1080) {
        return &s_ov5647_fmt_1920x1080;
    }
#endif
#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X800_50FPS
    if (width == 800 && height == 800) {
        return &s_ov5647_fmt_800x800;
    }
#endif
#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X640_50FPS
    if (width == 800 && height == 640) {
        return &s_ov5647_fmt_800x640;
    }
#endif

    return NULL;
}

static esp_err_t select_sensor_mode_for_profile(void)
{
    esp_cam_sensor_format_t sensor_fmt = {0};
    const esp_cam_sensor_format_t *target_sensor_fmt = NULL;

    if (ioctl(s_cam.fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        ESP_LOGE(TAG, "读取传感器格式失败: errno=%d", errno);
        return ESP_FAIL;
    }

    if (sensor_fmt.width == H264_WIDTH && sensor_fmt.height == H264_HEIGHT) {
        return ESP_OK;
    }

    target_sensor_fmt = get_target_ov5647_sensor_format(H264_WIDTH, H264_HEIGHT);
    if (target_sensor_fmt == NULL) {
        ESP_LOGE(TAG,
                 "目标传感器格式 %ux%u 未在 sdkconfig 中启用",
                 H264_WIDTH, H264_HEIGHT);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (ioctl(s_cam.fd, VIDIOC_S_SENSOR_FMT, (void *)target_sensor_fmt) != 0) {
        ESP_LOGE(TAG, "切换传感器格式失败: errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 格式转换                                                            */
/* ------------------------------------------------------------------ */

/**
 * 注意：已移除软件RGB565转YUV420转换代码
 * 现在完全依赖硬件ISP进行RAW8到YUV420的转换
 * OV5647输出RAW8格式，通过V4L2请求YUV420时，硬件ISP自动完成转换
 */

/* ------------------------------------------------------------------ */
/* H.264 编码                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief 将原始图像帧编码为 H.264
 *
 * @param src           输入图像数据（YUV420，由硬件ISP转换）
 * @param w             图像宽度
 * @param h             图像高度
 * @param out_buf       输出缓冲指针
 * @param out_buf_size  输出缓冲大小
 * @param out_len       实际编码后的字节数
 * @param frame_type    输出帧类型（IDR/I/P）
 * @return ESP_OK 成功
 */
static esp_err_t encode_to_h264(const uint8_t *src, uint32_t w, uint32_t h,
                                 uint8_t *out_buf, size_t out_buf_size,
                                 size_t *out_len, esp_h264_frame_type_t *frame_type,
                                 uint32_t *frame_pts)
{
    uint32_t input_pts = s_cam.pts;

    /* 直接使用硬件ISP转换后的YUV420数据 */
    esp_h264_enc_in_frame_t in_frame = {
        .raw_data.buffer = (uint8_t *)src,
        .raw_data.len    = w * h * 3 / 2,
        .pts             = input_pts,
    };

    /* 准备输出帧 */
    esp_h264_enc_out_frame_t out_frame = {
        .raw_data.buffer = out_buf,
        .raw_data.len    = out_buf_size,
    };

    /* 执行编码（硬件编码器，无法进一步优化）*/
    esp_h264_err_t ret = esp_h264_enc_process(s_cam.h264_enc, &in_frame, &out_frame);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 编码失败: %d", ret);
        return ESP_FAIL;
    }

    *out_len = out_frame.length;
    *frame_type = out_frame.frame_type;
    if (frame_pts) {
        *frame_pts = input_pts;
    }
    s_cam.pts = input_pts + H264_RTP_TS_STEP;

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 采集任务                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief 摄像头帧采集任务
 *
 * 循环执行：DQBUF → H.264 编码 → QBUF → rtsp_push_h264_frame
 * 当前绑定在 core 1。
 */
static void cam_task(void *arg)
{
    struct v4l2_buffer v4l2_buf;
    uint32_t encoded_frame_cnt = 0;
    int64_t report_start_us = 0;
    uint64_t dqbuf_time_total_us = 0;
    uint64_t encode_time_total_us = 0;
    uint64_t push_time_total_us = 0;
    bool stats_window_active = false;

    while (1) {
        /* 等待 RTSP 客户端连接后再采集 */
        xEventGroupWaitBits(s_cam_event, CAM_START_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        if (!stats_window_active) {
            flush_stale_capture_buffers();
            rtsp_reset_tx_stats();
            encoded_frame_cnt = 0;
            dqbuf_time_total_us = 0;
            encode_time_total_us = 0;
            push_time_total_us = 0;
            report_start_us = esp_timer_get_time();
            stats_window_active = true;
        }

        /* 取出已填充的帧缓冲 */
        int64_t t1 = esp_timer_get_time();
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        v4l2_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(s_cam.fd, VIDIOC_DQBUF, &v4l2_buf) != 0) {
            if (errno == EAGAIN) {
                /* 缓冲区暂时无数据，短暂等待后重试 */
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            ESP_LOGE(TAG, "DQBUF 失败: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int64_t t2 = esp_timer_get_time();
        dqbuf_time_total_us += (uint64_t)(t2 - t1);

        /* 编码为 H.264 并推送给 RTSP 客户端 */
        size_t h264_len = 0;
        esp_h264_frame_type_t frame_type;
        uint32_t frame_pts = 0;
        esp_err_t ret = encode_to_h264(s_cam.buf[v4l2_buf.index],
                                        s_cam.width, s_cam.height,
                                        s_cam.h264_out_buf, s_cam.h264_out_buf_size,
                                        &h264_len, &frame_type, &frame_pts);
        int64_t t3 = esp_timer_get_time();
        encode_time_total_us += (uint64_t)(t3 - t2);
        media_storage_process_camera_frame(s_cam.buf[v4l2_buf.index], v4l2_buf.bytesused,
                                           s_cam.width, s_cam.height);

        /* 编码完成后立即归还采集缓冲，避免 CSI/ISP 因缓冲不足降帧 */
        if (ioctl(s_cam.fd, VIDIOC_QBUF, &v4l2_buf) != 0) {
            ESP_LOGW(TAG, "QBUF 失败: %d", errno);
        }

        if (ret == ESP_OK && h264_len > 0) {
            /* 推送到 RTSP 服务器 */
            int64_t t4 = esp_timer_get_time();
            rtsp_push_h264_frame(s_cam.h264_out_buf, h264_len, frame_type, frame_pts);
            int64_t t5 = esp_timer_get_time();
            push_time_total_us += (uint64_t)(t5 - t4);
            media_storage_process_h264_frame(s_cam.h264_out_buf, h264_len, frame_type, frame_pts);
            encoded_frame_cnt++;

            int64_t now_us = esp_timer_get_time();
            if ((now_us - report_start_us) >= CAM_STATS_REPORT_INTERVAL_US) {
                uint32_t elapsed_ms = (uint32_t)((now_us - report_start_us) / 1000);
                rtsp_tx_stats_t tx_stats = {0};
                float encoded_fps = 0.0f;
                float actual_fps = 0.0f;
                float actual_bitrate = 0.0f;

                rtsp_take_tx_stats(&tx_stats);
                if (elapsed_ms > 0) {
                    encoded_fps = (float)encoded_frame_cnt * 1000.0f / elapsed_ms;
                    actual_fps = (float)tx_stats.frames_sent * 1000.0f / elapsed_ms;
                    actual_bitrate = (float)tx_stats.bytes_sent * 8.0f / elapsed_ms;
                }

                ESP_LOGI(TAG,
                         "视频统计 | 当前分辨率: %"PRIu32"x%"PRIu32" | 编码帧率: %.1f fps | 实际帧率: %.1f fps | 实际码率: %.0f kbps",
                         s_cam.width, s_cam.height, encoded_fps, actual_fps, actual_bitrate);

                if (encoded_frame_cnt > 0) {
                    float avg_dqbuf_ms = (float)dqbuf_time_total_us / (1000.0f * encoded_frame_cnt);
                    float avg_encode_ms = (float)encode_time_total_us / (1000.0f * encoded_frame_cnt);
                    float avg_push_ms = (float)push_time_total_us / (1000.0f * encoded_frame_cnt);

                    ESP_LOGI(TAG,
                             "阶段耗时 | 取帧: %.2f ms | 编码: %.2f ms | 推流: %.2f ms | 总计: %.2f ms",
                             avg_dqbuf_ms, avg_encode_ms, avg_push_ms,
                             avg_dqbuf_ms + avg_encode_ms + avg_push_ms);
                }

                encoded_frame_cnt = 0;
                dqbuf_time_total_us = 0;
                encode_time_total_us = 0;
                push_time_total_us = 0;
                report_start_us = now_us;
            }
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "H.264 编码失败，跳过此帧");
        }

        if ((xEventGroupGetBits(s_cam_event) & CAM_START_BIT) == 0) {
            stats_window_active = false;
        }
    }
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* camera_init                                                          */
/* ------------------------------------------------------------------ */

esp_err_t camera_init(void)
{
    memset(&s_cam, 0, sizeof(s_cam));

    /* 创建采集控制事件组 */
    s_cam_event = xEventGroupCreate();
    if (!s_cam_event) {
        ESP_LOGE(TAG, "EventGroup 创建失败");
        return ESP_ERR_NO_MEM;
    }

    /* 注册 RTSP 播放状态回调 */
    rtsp_set_playing_callback(on_rtsp_playing);

    /* 1. 初始化 esp_video（MIPI-CSI + SCCB）*/
    esp_video_init_csi_config_t csi_cfg[] = {{
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port    = CAM_SCCB_PORT,
                .scl_pin = CAM_SCCB_SCL_PIN,
                .sda_pin = CAM_SCCB_SDA_PIN,
            },
            .freq = CAM_SCCB_FREQ,
        },
        .reset_pin = CAM_RESET_PIN,
        .pwdn_pin  = CAM_PWDN_PIN,
    }};
    esp_video_init_config_t vcfg = { .csi = csi_cfg };
    ESP_RETURN_ON_ERROR(esp_video_init(&vcfg), TAG, "esp_video_init 失败");

    /* 等待设备节点注册完成 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 2. 打开 MIPI-CSI 视频设备（使用非阻塞模式提高效率）*/
    s_cam.fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY | O_NONBLOCK);
    if (s_cam.fd < 0) {
        ESP_LOGE(TAG, "打开 %s 失败", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return ESP_FAIL;
    }

    /* 3. 枚举并选择摄像头支持的格式 */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_cam.fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "获取格式失败");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(select_sensor_mode_for_profile(), TAG, "切换传感器格式失败");

    /* 枚举所有支持的格式 */
    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_LOGI(TAG, "枚举支持的格式:");
    while (ioctl(s_cam.fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        ESP_LOGI(TAG, "  [%d] %s (0x%08x)", fmtdesc.index, fmtdesc.description, fmtdesc.pixelformat);
        fmtdesc.index++;
    }

    /* 尝试设置为 YUV420（ESP32-P4 H.264 编码器原生支持）*/
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = H264_WIDTH;
    fmt.fmt.pix.height = H264_HEIGHT;
    fmt.fmt.pix.pixelformat = 0x32315559;  /* YUV 4:2:0 */

    if (ioctl(s_cam.fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "设置 YUV420 格式失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "成功设置 YUV420 格式（硬件ISP转换）");

    /* 读取实际生效格式 */
    s_cam.width  = fmt.fmt.pix.width;
    s_cam.height = fmt.fmt.pix.height;
    s_cam.pixfmt = fmt.fmt.pix.pixelformat;

    /* 验证格式必须是 YUV420 */
    if (s_cam.pixfmt != 0x32315559) {
        ESP_LOGE(TAG, "格式错误: 期望 YUV420 (0x32315559)，实际 0x%"PRIx32, s_cam.pixfmt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "使用格式: %"PRIu32"x%"PRIu32" YUV420（硬件ISP转换）",
             s_cam.width, s_cam.height);

    /* 4. 申请 MMAP 缓冲 */
    struct v4l2_requestbuffers req = {
        .count  = CAM_BUF_COUNT,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(s_cam.fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS 失败");
        return ESP_FAIL;
    }

    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i,
        };
        ioctl(s_cam.fd, VIDIOC_QUERYBUF, &buf);
        s_cam.buf[i]  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, s_cam.fd, buf.m.offset);
        s_cam.buf_size = buf.length;
        ioctl(s_cam.fd, VIDIOC_QBUF, &buf);
    }

    /* 5. 初始化硬件 H.264 编码器（使用实际摄像头分辨率和格式）*/
    /* 使用配置的目标码率 */
    uint32_t bitrate = H264_BITRATE;

    /* ESP32-P4 H.264 编码器只支持 YUV420 格式 */
    esp_h264_raw_format_t h264_fmt = ESP_H264_RAW_FMT_O_UYY_E_VYY;

    ESP_LOGI(TAG, "H.264 编码器配置 | 分辨率: %"PRIu32"x%"PRIu32" | 帧率: %d fps | 码率: %.1f Mbps | QP: %d-%d",
             s_cam.width, s_cam.height, H264_FPS, bitrate / 1000000.0f, H264_QP_MIN, H264_QP_MAX);

    esp_h264_enc_cfg_hw_t h264_cfg = {
        .pic_type = h264_fmt,
        .gop      = H264_GOP,
        .fps      = H264_FPS,
        .res      = {
            .width  = s_cam.width,
            .height = s_cam.height,
        },
        .rc = {
            .bitrate = bitrate,
            .qp_min  = H264_QP_MIN,
            .qp_max  = H264_QP_MAX,
        },
    };

    esp_h264_err_t h264_ret = esp_h264_enc_hw_new(&h264_cfg, &s_cam.h264_enc);
    if (h264_ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 编码器创建失败: %d", h264_ret);
        return ESP_FAIL;
    }

    /* 打开编码器 */
    h264_ret = esp_h264_enc_open(s_cam.h264_enc);
    if (h264_ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 编码器打开失败: %d", h264_ret);
        esp_h264_enc_del(s_cam.h264_enc);
        return ESP_FAIL;
    }

    /* 分配编码输出缓冲（SPIRAM，64 字节对齐，增大到2倍像素数）*/
    s_cam.h264_out_buf_size = s_cam.width * s_cam.height * H264_OUT_BUF_FACTOR;
    s_cam.h264_out_buf_size = (s_cam.h264_out_buf_size + 63) & ~63;  /* 向上对齐到 64 字节 */

    s_cam.h264_out_buf = heap_caps_aligned_alloc(64, s_cam.h264_out_buf_size,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_cam.h264_out_buf) {
        ESP_LOGE(TAG, "H.264 输出缓冲分配失败");
        esp_h264_enc_close(s_cam.h264_enc);
        esp_h264_enc_del(s_cam.h264_enc);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "H.264 输出缓冲: %zu 字节 (%.1f MB)",
             s_cam.h264_out_buf_size, s_cam.h264_out_buf_size / (1024.0 * 1024.0));

    /*
     * 1280x960 下照片链路会额外占用约 6.7 MB 缓冲。
     * 启动阶段优先保证 RTSP 和录像旁路初始化成功，照片缓冲改为按需申请。
     */
    esp_err_t media_ret = media_storage_prepare_video_record(s_cam.width, s_cam.height, H264_FPS);
    if (media_ret != ESP_OK) {
        ESP_LOGW(TAG, "录像保存缓冲准备失败，RTSP 将继续运行: 0x%x", media_ret);
    }

    ESP_LOGI(TAG, "H.264 编码器初始化完成: %"PRIu32"x%"PRIu32"@%dfps, GOP=%d, 码率=%"PRIu32" bps",
             s_cam.width, s_cam.height, H264_FPS, H264_GOP, bitrate);

    /* 6. 开始视频流采集 */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_cam.fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON 失败");
        return ESP_FAIL;
    }

    /* 7. 创建采集任务。
     * 发送任务优先级更高，避免同核时编码连续跑满导致 RTP 发送滞后。 */
    xTaskCreatePinnedToCore(cam_task, "cam_task", 8 * 1024, NULL, 13, NULL, 1);

    ESP_LOGI(TAG, "摄像头初始化完成");
    return ESP_OK;
}
