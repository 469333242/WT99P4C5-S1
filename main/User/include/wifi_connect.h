/**
 * @file wifi_connect.h
 * @brief Wi-Fi AP/STA 模块头文件
 *
 * 当前设备通过网页配置选择 AP 或 STA 场景，启动后只启用其中一种模式。
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* ================================================================== */
/*  Wi-Fi 默认参数                                                       */
/* ================================================================== */

#define WIFI_AP_SSID             "WT99P4C5"
#define WIFI_AP_PASSWORD         "12345678"
#define WIFI_AP_IP               "192.168.144.2"
#define WIFI_AP_GW               "192.168.144.2"
#define WIFI_AP_MASK             "255.255.255.0"
#define WIFI_AP_MAX_CONNECTIONS  4

#define WIFI_STA_SSID            "CEEWA"
#define WIFI_STA_PASSWORD        "52285509"
#define WIFI_STA_USE_STATIC_IP   false
#define WIFI_STA_IP              "192.168.1.200"
#define WIFI_STA_GW              "192.168.1.1"
#define WIFI_STA_MASK            "255.255.255.0"

/* ================================================================== */
/*  兼容网页配置默认值的 Profile 结构体                                 */
/* ================================================================== */
typedef struct {
    const char *ssid;           // Wi-Fi 名称
    const char *password;       // Wi-Fi 密码

    bool        use_static_ip;  // 兼容旧网页配置结构，AP 模式固定为 true
    const char *ip;             // AP 固定地址
    const char *gw;             // 网关
    const char *mask;           // 子网掩码
} wifi_profile_t;

/* device_web_config 读取该默认值填充网页配置项。
 * AP 模式下 AP 网络地址默认使用 WIFI_AP_* 参数。
 */
#define WIFI_ACTIVE_PROFILE   ((wifi_profile_t){ \
    .ssid          = WIFI_AP_SSID,               \
    .password      = WIFI_AP_PASSWORD,           \
    .use_static_ip = true,                       \
    .ip            = WIFI_AP_IP,                 \
    .gw            = WIFI_AP_GW,                 \
    .mask          = WIFI_AP_MASK,               \
})

/* ================================================================== */
/*  接口声明                                                            */
/* ================================================================== */
/**
 * @brief 初始化 Wi-Fi 网络
 *
 * 根据网页保存的网络场景，只启用 AP 或 STA 其中一种模式。
 * 调用前需确保 esp_netif、事件循环以及 ESP-Hosted SDIO transport 已就绪。
 *
 * @return ESP_OK 成功
 */
esp_err_t wifi_connect_init(void);

/**
 * @brief 等待 Wi-Fi 网络就绪
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK 网络已就绪，ESP_ERR_TIMEOUT 等待超时
 */
esp_err_t wifi_connect_wait_for_ip(int timeout_ms);

/**
 * @brief 等待系统时间有效
 *
 * 当前仍由网页通过浏览器时间调用 /api/time 完成校时。
 * 若系统时间已经有效，则返回 ESP_OK。
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK 时间已有效；其它值表示当前仍需网页同步时间
 */
esp_err_t wifi_connect_wait_for_time(int timeout_ms);

/**
 * @brief 获取当前连接到 AP 的客户端数量
 */
uint32_t wifi_connect_get_ap_client_count(void);

/**
 * @brief 获取当前本次启动实际启用的 Wi-Fi 模式
 *
 * 用于区分“网页已保存的新配置”和“当前已经运行的网络模式”。
 */
uint32_t wifi_connect_get_active_wifi_mode(void);
