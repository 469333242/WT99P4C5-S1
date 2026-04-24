/**
 * @file device_web_config.h
 * @brief 设备网页配置模块接口
 *
 * 负责持久化保存可由网页修改的设备参数，包括：
 *   - UART0/UART1 波特率
 *   - 视频分辨率档位
 *   - Wi-Fi 静态 IP 开关与地址参数
 *
 * 这些配置默认在下次重启后生效，不影响 TF 卡中的已有媒体文件。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_WEB_CONFIG_IPV4_TEXT_LEN 16

typedef enum {
    DEVICE_WEB_CONFIG_VIDEO_PROFILE_1280X960 = 1,
    DEVICE_WEB_CONFIG_VIDEO_PROFILE_1920X1080 = 2,
    DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X800 = 3,
    DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X640 = 4,
} device_web_config_video_profile_t;

typedef struct {
    uint32_t uart0_baud_rate;
    uint32_t uart1_baud_rate;
    uint32_t video_profile;
    bool     wifi_use_static_ip;
    char     wifi_static_ip[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_gw[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_mask[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
} device_web_config_t;

/**
 * @brief 初始化设备网页配置模块
 *
 * 调用前需确保 NVS 已初始化。
 *
 * @return ESP_OK 成功；其它错误码表示读取 NVS 失败，但模块仍会保留默认配置
 */
esp_err_t device_web_config_init(void);

/**
 * @brief 读取默认配置
 */
void device_web_config_get_defaults(device_web_config_t *out_config);

/**
 * @brief 读取当前配置
 *
 * 若模块尚未初始化，则返回默认配置。
 */
void device_web_config_get(device_web_config_t *out_config);

/**
 * @brief 保存设备网页配置
 *
 * @param config 待保存配置
 * @return ESP_OK 保存成功；其它错误码表示参数非法或写入失败
 */
esp_err_t device_web_config_save(const device_web_config_t *config);

/**
 * @brief 恢复设备网页配置默认值
 *
 * 仅重置网页可配置参数，不删除 TF 卡中的媒体文件。
 */
esp_err_t device_web_config_reset_to_factory(void);

/**
 * @brief 判断视频分辨率档位是否合法且当前固件已支持
 */
bool device_web_config_is_valid_video_profile(uint32_t video_profile);

/**
 * @brief 判断波特率是否合法
 */
bool device_web_config_is_valid_baud_rate(uint32_t baud_rate);

/**
 * @brief 判断 IPv4 文本是否合法
 */
bool device_web_config_is_valid_ipv4_text(const char *text);

/**
 * @brief 获取视频分辨率档位名称
 */
const char *device_web_config_get_video_profile_name(uint32_t video_profile);

#ifdef __cplusplus
}
#endif
