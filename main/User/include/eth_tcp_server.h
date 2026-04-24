/**
 * @file eth_tcp_server.h
 * @brief 开发板网口 TCP 双向通信服务接口
 *
 * 该模块用于让电脑通过网线直连 WT99P4C5-S1 后，
 * 以 TCP 方式与开发板进行双向收发。
 */

#ifndef ETH_TCP_SERVER_H
#define ETH_TCP_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ETH_TCP_SERVER_PORT 9000

typedef void (*eth_tcp_server_rx_cb_t)(const uint8_t *data, size_t len);

/**
 * @brief 启动网口 TCP 服务端
 *
 * 服务端默认监听 9000 端口，同一时刻仅保留一个客户端连接。
 * 如果未注册接收回调，收到的数据会默认原样回显给电脑。
 *
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t eth_tcp_server_start(void);

/**
 * @brief 向当前已连接的电脑客户端发送数据
 *
 * @param data 数据指针
 * @param len  数据长度
 * @return ESP_OK 发送成功，ESP_ERR_INVALID_STATE 表示当前无客户端连接
 */
esp_err_t eth_tcp_server_send(const uint8_t *data, size_t len);

/**
 * @brief 注册 TCP 接收回调
 *
 * 传入 NULL 表示恢复默认回显行为。
 *
 * @param cb 接收回调
 */
void eth_tcp_server_set_rx_callback(eth_tcp_server_rx_cb_t cb);

/**
 * @brief 查询当前是否有电脑客户端连接
 *
 * @return true 已连接
 * @return false 未连接
 */
bool eth_tcp_server_has_client(void);

#ifdef __cplusplus
}
#endif

#endif /* ETH_TCP_SERVER_H */
