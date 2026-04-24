/**
 * @file tcp_uart_server.h
 * @brief TCP-UART 双向透传模块头文件
 *
 * 每个 UART 对应一个独立的 TCP 服务器端口：
 *   - UART0 : TCP 端口 8880
 *   - UART1 : TCP 端口 8881
 *
 * 每路透传包含两个 FreeRTOS 任务：
 *   - tcp_server_task : 监听 TCP 连接，接收 TCP 数据转发至 UART
 *   - uart_rx_task    : 接收 UART 数据转发至 TCP 客户端
 *
 * 同一时刻每个端口仅支持一个 TCP 客户端连接。
 * 新客户端连接时，旧连接将被关闭。
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  硬件引脚配置（根据实际硬件修改）                                    */
/* ------------------------------------------------------------------ */

/* UART0 引脚 */
#define UART0_TX_PIN        37       /* UART0 TX GPIO */
#define UART0_RX_PIN        38       /* UART0 RX GPIO */

/* UART1 引脚 */
#define UART1_TX_PIN        4       /* UART1 TX GPIO */
#define UART1_RX_PIN        5       /* UART1 RX GPIO */

/* ------------------------------------------------------------------ */
/*  通信参数配置                                                        */
/* ------------------------------------------------------------------ */

#define UART_BAUD_RATE      115200  /* 默认波特率，可由网页配置覆盖 */
#define UART_BUF_SIZE       1024    /* UART 驱动收发缓冲区大小（字节） */
#define TCP_RX_BUF_SIZE     1024    /* TCP 接收缓冲区大小（字节） */

/* TCP 监听端口 */
#define TCP_UART0_PORT      8880    /* UART0 对应的 TCP 端口 */
#define TCP_UART1_PORT      8881    /* UART1 对应的 TCP 端口 */

/* ------------------------------------------------------------------ */
/*  公共接口                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 启动 UART0 TCP 透传服务（端口 8880）
 *
 * 初始化 UART0 驱动，创建 TCP 监听任务和 UART 接收任务。
 * 调用前需确保 WiFi 已连接并获取到 IP 地址。
 *
 * @return ESP_OK 成功；其他值表示初始化失败
 */
esp_err_t tcp_uart0_server_start(void);

/**
 * @brief 启动 UART1 TCP 透传服务（端口 8881）
 *
 * 初始化 UART1 驱动，创建 TCP 监听任务和 UART 接收任务。
 * 调用前需确保 WiFi 已连接并获取到 IP 地址。
 *
 * @return ESP_OK 成功；其他值表示初始化失败
 */
esp_err_t tcp_uart1_server_start(void);

#ifdef __cplusplus
}
#endif
