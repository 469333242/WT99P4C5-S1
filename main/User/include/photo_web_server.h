/**
 * @file photo_web_server.h
 * @brief SD 卡媒体网页浏览模块头文件
 *
 * 当前模块提供一个轻量网页服务，用于通过浏览器查看 TF 卡中已保存的照片和录像。
 * 已实现接口：
 *   - GET /                 : 媒体浏览网页
 *   - GET /api/photos       : 照片列表 JSON
 *   - GET /api/videos       : 视频列表 JSON
 *   - GET /api/time         : 浏览器/电脑时间同步
 *   - POST /api/capture     : 请求拍照一次
 *   - POST /api/delete      : 删除一个或多个媒体文件
 *   - GET /photo/<path>     : JPEG 原图访问
 *   - GET /video/<path>     : MP4 视频访问，支持 Range 字节请求
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 媒体网页服务监听端口 */
#define PHOTO_WEB_SERVER_PORT 80

/**
 * @brief 启动媒体网页浏览服务
 *
 * 服务启动后，可通过浏览器访问：
 *   http://<设备IP>/
 *
 * @return ESP_OK 启动成功；其它值表示启动失败
 */
esp_err_t photo_web_server_start(void);

/**
 * @brief 停止媒体网页浏览服务
 */
void photo_web_server_stop(void);

#ifdef __cplusplus
}
#endif
