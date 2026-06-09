/**
 * @file z1mini_sta_relay.h
 * @brief Z-1mini STA 入网模式辅助转发模块
 *
 * AP 模式继续使用 z1mini_bridge 的二层原始帧透传。STA 模式下 Wi-Fi STA
 * 不能等价做二层桥接，因此这里启动以太网口，并通过 Proxy ARP + IPv4
 * 帧转发让同网段主机尽量直接访问 Z-1mini。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 STA 模式下的 Z-1mini Proxy ARP 转发
 *
 * @param eth_ip    开发板连接 Z-1mini 的以太网 IP，例如 192.168.144.2
 * @param eth_gw    以太网网关地址，通常与 eth_ip 相同
 * @param eth_mask  以太网子网掩码
 * @param device_ip Z-1mini 地址，例如 192.168.144.108
 */
esp_err_t z1mini_sta_relay_start(const char *eth_ip, const char *eth_gw,
                                 const char *eth_mask, const char *device_ip);

bool z1mini_sta_relay_is_running(void);

#ifdef __cplusplus
}
#endif
