/**
 * @file rtsp_server.h
 * @brief RTSP/RTP 视频流服务器头文件（H.264 编码）
 *
 * 实现基于 TCP 的 RTSP 服务器，将 H.264 帧通过 RTP 推送给客户端。
 * 访问地址：rtsp://<设备IP>:8554/stream
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_h264_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RTSP 服务监听端口 */
#define RTSP_PORT 8554

/**
 * @brief 启动 RTSP 服务器任务
 *
 * 创建 TCP 监听 socket，接受客户端连接并处理 RTSP 握手（OPTIONS/DESCRIBE/SETUP/PLAY）。
 * 握手完成后持续通过 RTP over TCP（interleaved）推送 H.264 帧。
 *
 * @return ESP_OK 成功启动任务
 */
esp_err_t rtsp_server_start(void);

/**
 * @brief 推送一帧 H.264 数据给所有已连接的 RTSP 客户端
 *
 * 由摄像头采集任务调用，将最新 H.264 帧封装为 RTP 包发送。
 *
 * @param h264_buf    H.264 数据指针（包含 NALU）
 * @param h264_len    H.264 数据长度（字节）
 * @param frame_type  帧类型（IDR/I/P）
 * @param pts         该帧的 90kHz 时间戳
 */
void rtsp_push_h264_frame(const uint8_t *h264_buf, size_t h264_len,
                          esp_h264_frame_type_t frame_type, uint32_t pts);

/**
 * @brief 客户端播放状态变化回调函数类型
 *
 * @param playing  true = 至少有一个客户端正在播放；false = 无客户端播放
 */
typedef void (*rtsp_playing_cb_t)(bool playing);

typedef struct {
    uint32_t frames_sent;     /* 去重后的已发送帧数：同一帧发给多个客户端只计一次 */
    uint32_t bytes_sent;      /* 去重后的已发送字节数 */
    uint32_t active_clients;  /* 当前处于 PLAYING 状态的客户端数量 */
} rtsp_tx_stats_t;

/**
 * @brief 注册客户端播放状态变化回调
 *
 * 当第一个客户端开始播放（PLAY）或最后一个客户端断开时触发回调，
 * 用于通知摄像头模块按需启停采集。
 * 必须在 rtsp_server_start() 之前调用。
 *
 * @param cb  回调函数指针，传 NULL 可取消注册
 */
void rtsp_set_playing_callback(rtsp_playing_cb_t cb);

/**
 * @brief 读取并清零当前 RTSP 实际发送统计
 *
 * 统计按帧时间戳去重，同一帧即使发给多个客户端，也只统计为 1 帧。
 */
void rtsp_take_tx_stats(rtsp_tx_stats_t *stats);

/**
 * @brief 清零 RTSP 发送统计窗口
 */
void rtsp_reset_tx_stats(void);

#ifdef __cplusplus
}
#endif
