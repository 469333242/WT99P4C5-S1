/**
 * @file photo_web_server.h
 * @brief SD 卡照片网页浏览模块头文件
 *
 * 当前模块提供一个轻量网页服务，用于通过浏览器查看 TF 卡中已保存的照片。
 * 已实现接口：
 *   - GET /                 : 相册网页
 *   - GET /api/photos       : 照片列表 JSON
 *   - GET /photo/<path>     : JPEG 原图访问
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 照片网页服务监听端口 */
#define PHOTO_WEB_SERVER_PORT 80

/**
 * @brief 启动照片网页浏览服务
 *
 * 服务启动后，可通过浏览器访问：
 *   http://<设备IP>/
 *
 * @return ESP_OK 启动成功；其它值表示启动失败
 */
esp_err_t photo_web_server_start(void);

/**
 * @brief 停止照片网页浏览服务
 */
void photo_web_server_stop(void);

#ifdef __cplusplus
}
#endif
