/**
 * @file media_storage.h
 * @brief 媒体存储模块接口
 *
 * 当前版本先实现照片存储能力：
 *   - RTSP 开始推流后触发一次自动拍照
 *   - 复用摄像头 YUV420 帧，后台使用 PPA 硬件转换为 RGB565
 *   - 调用官方 JPEG 硬件编码 API 生成 JPEG
 *   - 按约定目录规则保存到 TF 卡
 *
 * 后续录像与网页访问功能可继续在该模块之上扩展。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化媒体存储模块
 *
 * 依赖条件：
 *   - NVS 已初始化
 *   - TF 卡已挂载
 *
 * @return ESP_OK 成功，其它错误码表示初始化失败
 */
esp_err_t media_storage_init(void);

/**
 * @brief 反初始化媒体存储模块
 */
void media_storage_deinit(void);

/**
 * @brief 按摄像头实际分辨率预分配照片处理缓冲
 *
 * 建议在 camera_init() 获取实际 YUV420 分辨率后调用，避免首次 RTSP 推流拍照时临时申请大块内存。
 *
 * @param width   图像宽度
 * @param height  图像高度
 * @return ESP_OK 成功，其它错误码表示失败
 */
esp_err_t media_storage_prepare_photo_buffers(uint32_t width, uint32_t height);

/**
 * @brief 请求一次自动拍照
 *
 * 该接口仅置位“待拍照”标志，真正的拍照会在摄像头任务拿到下一帧 YUV420 图像时提交给后台任务。
 */
void media_storage_request_auto_photo(void);

/**
 * @brief 用当前摄像头帧尝试提交自动拍照
 *
 * 该函数只在有待拍照请求且后台空闲时拷贝一帧图像，PPA 转换、JPEG 硬件编码和写卡均由后台任务执行。
 *
 * @param yuv420_buf  YUV420 原始图像指针
 * @param yuv420_len  图像长度，字节；为 0 时按 width * height * 3 / 2 计算
 * @param width       图像宽度
 * @param height      图像高度
 */
void media_storage_process_camera_frame(const uint8_t *yuv420_buf, size_t yuv420_len,
                                        uint32_t width, uint32_t height);

#ifdef __cplusplus
}
#endif
