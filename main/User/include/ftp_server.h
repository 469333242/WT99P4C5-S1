/**
 * @file ftp_server.h
 * @brief TF 卡只读 FTP 文件获取服务
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FTP_SERVER_PORT 21
#define FTP_SERVER_USER "ftpuser"
#define FTP_SERVER_PASSWORD "ftpuser"

/**
 * @brief 启动 FTP 服务
 *
 * 当前服务用于从 TF 卡远程获取文件：
 *   ftp://ftpuser:ftpuser@<设备IP>
 *
 * 支持列目录和下载文件，不支持上传、删除、改名和建目录。
 */
esp_err_t ftp_server_start(void);

/**
 * @brief 停止 FTP 服务
 */
void ftp_server_stop(void);

#ifdef __cplusplus
}
#endif
