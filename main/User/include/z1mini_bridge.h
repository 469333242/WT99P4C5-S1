/**
 * @file z1mini_bridge.h
 * @brief Z-1mini 吊舱网口透传模块接口
 *
 * ESP-Hosted 远端 Wi-Fi 不支持 IDF 原生二层桥，这里通过 AP 与以太网原始帧
 * 定向转发，使电脑连接 AP 后可以像网线直连一样访问固定 IP 吊舱设备。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Z-1mini 网口透传
 *
 * 调用前需确保 esp_netif、默认事件循环以及 ESP-Hosted SDIO transport 已就绪。
 * Wi-Fi AP 的 SSID/密码复用网页配置中的 AP 参数，不会写入 NVS。
 *
 * @param bridge_ip   WT99 AP 接口 IP
 * @param bridge_gw   WT99 AP 接口网关
 * @param bridge_mask WT99 AP 接口子网掩码
 * @param device_ip   吊舱固定 IP，仅用于日志提示
 * @return ESP_OK 成功
 */
esp_err_t z1mini_bridge_init(const char *bridge_ip, const char *bridge_gw,
                             const char *bridge_mask, const char *device_ip);

/**
 * @brief 等待透传 AP 启动完成
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK AP 已启动，ESP_ERR_TIMEOUT 等待超时
 */
esp_err_t z1mini_bridge_wait_for_ready(int timeout_ms);

/**
 * @brief 等待吊舱侧以太网链路连接
 *
 * 该接口只判断网线物理链路是否建立，不代表已经能访问吊舱固定 IP。
 *
 * @param timeout_ms 等待超时，单位毫秒；传负数表示一直等待
 * @return ESP_OK 以太网链路已连接，ESP_ERR_TIMEOUT 等待超时
 */
esp_err_t z1mini_bridge_wait_for_eth_link(int timeout_ms);

/**
 * @brief 判断网口透传本次启动是否已运行
 */
bool z1mini_bridge_is_running(void);

/**
 * @brief 判断吊舱侧以太网链路是否已连接
 */
bool z1mini_bridge_is_eth_link_up(void);

/**
 * @brief 获取当前连接到透传 AP 的客户端数量
 */
uint32_t z1mini_bridge_get_ap_client_count(void);

/**
 * @brief 获取透传 AP netif
 */
esp_netif_t *z1mini_bridge_get_netif(void);

#ifdef __cplusplus
}
#endif
