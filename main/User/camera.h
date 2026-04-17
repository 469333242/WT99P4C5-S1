/**
 * @file camera.h
 * @brief OV5647 MIPI-CSI 摄像头驱动头文件（H.264 硬件编码）
 *
 * 提供摄像头初始化接口，采集到的帧通过硬件 H.264 编码器编码后，
 * 通过 rtsp_push_h264_frame() 推送给 RTSP 服务器。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 OV5647 MIPI-CSI 摄像头（H.264 编码）
 *
 * 初始化 esp_video、V4L2 采集、硬件 H.264 编码器，并启动采集任务。
 * 调用前需确保 RTSP 服务器已启动（rtsp_server_start）。
 *
 * @return ESP_OK 成功
 */
esp_err_t camera_init(void);

#ifdef __cplusplus
}
#endif
