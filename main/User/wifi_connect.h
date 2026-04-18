/**
 * @file wifi_connect.h
 * @brief WiFi连接模块头文件
 *
 * 支持多环境 Profile：每个 Profile 独立配置 SSID/密码/IP策略。
 * 切换环境只需修改 WIFI_ACTIVE_PROFILE 一行。
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* ================================================================== */
/*  WiFi Profile 结构体                                                 */
/* ================================================================== */
typedef struct {
    const char *ssid;           // WiFi 名称
    const char *password;       // WiFi 密码

    bool        use_static_ip;  // true=静态IP  false=DHCP
    const char *ip;             // 静态IP（use_static_ip=false 时忽略）
    const char *gw;             // 网关
    const char *mask;           // 子网掩码
} wifi_profile_t;

/* ================================================================== */
/*  环境 Profile 定义                                                   */
/* ================================================================== */

/*  Profile 0 : Windows 移动热点（192.168.137.x 固定网段）
 *  使用静态IP，地址永远固定为 .200
 */
#define WIFI_PROFILE_WIN_HOTSPOT  ((wifi_profile_t){ \
    .ssid          = "WT99P4C5",          \
    .password      = "12345678",           \
    .use_static_ip = true,                 \
    .ip            = "192.168.137.200",    \
    .gw            = "192.168.137.1",      \
    .mask          = "255.255.255.0",      \
})

/*  Profile 1 : 其他热点 / 路由器（网段不确定）
 *  使用 DHCP，IP由路由器分配；连接成功后串口会打印当前IP
 */
#define WIFI_PROFILE_OTHER1        ((wifi_profile_t){ \
    .ssid          = "dyf",                \
    .password      = "dyf07100825",        \
    .use_static_ip = false,                \
    .ip            = NULL,                 \
    .gw            = NULL,                 \
    .mask          = NULL,                 \
})
/*  Profile 2 : 其他热点 / 路由器（网段不确定）
 *  使用 DHCP，IP由路由器分配；连接成功后串口会打印当前IP
 */
#define WIFI_PROFILE_OTHER2        ((wifi_profile_t){ \
    .ssid          = "CEEWA",                \
    .password      = "52285509",        \
    .use_static_ip = false,                \
    .ip            = NULL,                 \
    .gw            = NULL,                 \
    .mask          = NULL,                 \
})

/* ================================================================== */
/*    切换环境：                                       */
/*    WIFI_PROFILE_WIN_HOTSPOT  → 公司Windows 移动热点（固定 .200）         */
/*    WIFI_PROFILE_OTHER        → 其他热点（DHCP，串口日志查IP）          */
/* ================================================================== */
#define WIFI_ACTIVE_PROFILE   WIFI_PROFILE_OTHER1

/* ================================================================== */
/*  接口声明                                                            */
/* ================================================================== */
/**
 * @brief 初始化WiFi并开始连接
 *
 * 根据 WIFI_ACTIVE_PROFILE 自动决定使用静态IP还是DHCP。
 * 调用前需确保 esp_netif 和事件循环已初始化，且 SDIO transport 已就绪。
 *
 * @return ESP_OK 成功
 */
esp_err_t wifi_connect_init(void);

/**
 * @brief 等待 WiFi 拿到 IP 地址
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK 已拿到 IP，ESP_ERR_TIMEOUT 等待超时
 */
esp_err_t wifi_connect_wait_for_ip(int timeout_ms);
