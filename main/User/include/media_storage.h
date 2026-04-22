/**
 * @file media_storage.h
 * @brief 媒体存储模块接口
 *
 * 当前模块负责 TF 卡媒体保存：
 *   - 收到拍照请求后，在下一帧图像到达时保存一张照片
 *   - 复用摄像头 YUV420 帧，后台使用 PPA + JPEG 硬件编码保存照片
 *   - 复用现有 H.264 硬件编码帧，后台封装为 MP4 并按 2 分钟切段
 *   - 照片和录像共用同一个本次上电会话目录
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_h264_types.h"

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
 * @brief 使用外部传入的 Unix 毫秒时间同步系统时间
 *
 * 当前用于网页从浏览器/电脑传入实时时间。同步成功后，后续照片和录像文件名会使用真实时间。
 * 如果本次上电目录已经创建，则仅修正后续文件名时间，不重新创建新的上电目录。
 *
 * @param unix_ms  Unix 时间戳，单位毫秒，按 UTC 计时
 * @return ESP_OK 同步成功，其它错误码表示时间非法或设置失败
 */
esp_err_t media_storage_sync_time_from_unix_ms(int64_t unix_ms);

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
 * @brief 按 H.264 实际参数预分配录像旁路缓冲
 *
 * 录像只复用已经硬件编码完成的 H.264 帧，不再触发第二路视频编码。
 *
 * @param width   视频宽度
 * @param height  视频高度
 * @param fps     H.264 编码帧率
 * @return ESP_OK 成功，其它错误码表示失败
 */
esp_err_t media_storage_prepare_video_record(uint32_t width, uint32_t height, uint32_t fps);

/**
 * @brief 请求拍照一次
 *
 * 该接口仅置位“待拍照”标志，真正的拍照会在摄像头任务拿到下一帧 YUV420 图像时提交给后台任务。
 */
esp_err_t media_storage_request_photo(void);

/**
 * @brief 开始录像
 *
 * 该接口仅打开录像请求，真正的 MP4 文件会等到下一帧 IDR 且已拿到 SPS/PPS 后创建。
 */
void media_storage_start_video_record(void);

/**
 * @brief 停止录像
 *
 * 后台任务会尽快关闭当前 MP4 段并将临时文件改名为 .mp4。
 */
void media_storage_stop_video_record(void);

/**
 * @brief 用当前摄像头帧尝试处理待执行的拍照请求
 *
 * 该函数只在收到拍照请求且后台空闲时拷贝一帧图像，PPA 转换、JPEG 硬件编码和写卡均由后台任务执行。
 *
 * @param yuv420_buf  YUV420 原始图像指针
 * @param yuv420_len  图像长度，字节；为 0 时按 width * height * 3 / 2 计算
 * @param width       图像宽度
 * @param height      图像高度
 */
void media_storage_process_camera_frame(const uint8_t *yuv420_buf, size_t yuv420_len,
                                        uint32_t width, uint32_t height);

/**
 * @brief 提交一帧 H.264 压缩数据给录像后台
 *
 * 摄像头任务只做非阻塞拷贝和入队；队列满或缓冲不足时直接丢弃该帧，避免反向影响 RTSP。
 *
 * @param h264_buf    Annex-B H.264 帧数据
 * @param h264_len    数据长度
 * @param frame_type  H.264 帧类型
 * @param pts         90kHz 时基下的时间戳
 */
void media_storage_process_h264_frame(const uint8_t *h264_buf, size_t h264_len,
                                      esp_h264_frame_type_t frame_type, uint32_t pts);

#ifdef __cplusplus
}
#endif
