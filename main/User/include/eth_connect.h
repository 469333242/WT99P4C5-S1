/**
 * @file eth_connect.h
 * @brief 以太网连接模块接口
 */

#ifndef ETH_CONNECT_H
#define ETH_CONNECT_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化以太网连接
 *
 * @param use_static_ip 是否使用静态IP（true=静态IP，false=DHCP）
 * @param ip            静态IP地址（如 "192.168.1.100"），DHCP模式可传NULL
 * @param gw            网关地址（如 "192.168.1.1"），DHCP模式可传NULL
 * @param mask          子网掩码（如 "255.255.255.0"），DHCP模式可传NULL
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t eth_connect_init(bool use_static_ip, const char *ip,
                           const char *gw, const char *mask);

/**
 * @brief 等待以太网拿到 IP 地址
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK 已拿到 IP，ESP_ERR_TIMEOUT 等待超时
 */
esp_err_t eth_connect_wait_for_ip(int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ETH_CONNECT_H */
