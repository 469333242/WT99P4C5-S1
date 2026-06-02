/**
 * @file usb_thermal_camera.h
 * @brief USB 热像仪 UVC 采集头文件
 *
 * 当前模块负责 USB 热像仪热插拔监听、Y16 原始热数据持续采集、灰度帧转换，
 * 并提供独立的 RTSP H.264 推流链路。MIPI 摄像头和 USB 热像仪不同时工作。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t width;            /* 当前热像仪图像宽度 */
    uint32_t height;           /* 当前热像仪图像高度 */
    size_t y16_len;            /* Y16 原始帧长度，单位字节 */
    size_t gray8_len;          /* 8bit 灰度帧长度，单位字节 */
    size_t gray_oue_vyy_len;   /* H.264 硬件编码器灰度输入帧长度，单位字节 */
    uint32_t sequence;         /* 最新帧序号 */
    int64_t timestamp_us;      /* 最新帧时间戳，esp_timer_get_time() */
    uint16_t min_value;        /* 最新帧 Y16 最小值 */
    uint16_t max_value;        /* 最新帧 Y16 最大值 */
    uint32_t received_frames;  /* 已接收有效帧数 */
    uint32_t dropped_frames;   /* 因处理忙或缓存不足丢弃的帧数 */
    uint32_t invalid_frames;   /* 长度或格式不匹配的异常帧数 */
} usb_thermal_camera_frame_info_t;

/**
 * @brief 启动 USB 热像仪 UVC 采集任务
 *
 * 启动后会安装 USB Host 与 UVC 驱动，并在后台等待 USB UVC 设备插入。
 * 设备接入后持续采集最新一帧 Y16 原始热数据。
 * 若启动失败，仅影响 USB 热像仪功能，不影响现有摄像头和网络功能。
 *
 * @return ESP_OK 成功
 */
esp_err_t usb_thermal_camera_start(void);

/**
 * @brief 初始化 USB 热像仪 RTSP 推流链路
 *
 * 调用前需确保 RTSP 服务器已启动。该接口会创建热像仪 H.264 编码器和推流任务，
 * 但只有 RTSP 客户端播放时才会从 USB 热像仪取最新帧并编码推送。
 *
 * @return ESP_OK 成功
 */
esp_err_t usb_thermal_camera_rtsp_init(void);

/**
 * @brief 判断 USB 热像仪是否已经收到至少一帧有效 Y16 数据
 */
bool usb_thermal_camera_is_ready(void);

/**
 * @brief 获取最新热像仪帧信息
 *
 * @param out_info 输出帧信息
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 表示尚无有效帧
 */
esp_err_t usb_thermal_camera_get_frame_info(usb_thermal_camera_frame_info_t *out_info);

/**
 * @brief 拷贝最新 Y16 原始热数据
 *
 * @param out_buf      输出缓冲区
 * @param out_buf_size 输出缓冲区大小，需不小于 frame_info.y16_len
 * @param out_info     可选，输出该帧信息，可传 NULL
 * @return ESP_OK 成功
 */
esp_err_t usb_thermal_camera_get_latest_y16(uint8_t *out_buf, size_t out_buf_size,
                                            usb_thermal_camera_frame_info_t *out_info);

/**
 * @brief 将最新 Y16 原始热数据转换为 8bit 灰度图
 *
 * 灰度映射使用当前帧最小值和最大值做线性拉伸，便于第一版预览观察。
 *
 * @param out_buf      输出缓冲区
 * @param out_buf_size 输出缓冲区大小，需不小于 frame_info.gray8_len
 * @param out_info     可选，输出该帧信息，可传 NULL
 * @return ESP_OK 成功
 */
esp_err_t usb_thermal_camera_get_latest_gray8(uint8_t *out_buf, size_t out_buf_size,
                                              usb_thermal_camera_frame_info_t *out_info);

/**
 * @brief 将最新 Y16 原始热数据转换为 H.264 硬件编码器灰度输入帧
 *
 * 输出格式为 ESP_H264_RAW_FMT_O_UYY_E_VYY：
 *   - 第 0、2、4... 行按 U Y Y U Y Y... 排列
 *   - 第 1、3、5... 行按 V Y Y V Y Y... 排列
 *   - 当前灰度图 U/V 均填 128
 *
 * @param out_buf      输出缓冲区
 * @param out_buf_size 输出缓冲区大小，需不小于 frame_info.gray_oue_vyy_len
 * @param out_info     可选，输出该帧信息，可传 NULL
 * @return ESP_OK 成功
 */
esp_err_t usb_thermal_camera_get_latest_gray_oue_vyy(uint8_t *out_buf, size_t out_buf_size,
                                                     usb_thermal_camera_frame_info_t *out_info);

#ifdef __cplusplus
}
#endif
