/**
 * @file tcp_uart_server.c
 * @brief TCP-UART 双向透传模块实现
 *
 * 架构说明：
 *   每路 UART 启动两个任务：
 *     1. tcp_server_task : 监听 TCP 端口，接受客户端连接；
 *                          循环接收 TCP 数据并写入 UART。
 *     2. uart_rx_task    : 循环读取 UART 数据并发送给当前 TCP 客户端。
 *
 *   两个任务通过 uart_channel_t 结构体共享客户端 socket fd，
 *   使用 mutex 保护写操作，保证线程安全。
 */

#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "tcp_uart_server.h"

/* ------------------------------------------------------------------ */
/*  内部数据结构                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 单路 UART-TCP 透传通道的运行时状态
 */
typedef struct {
    uart_port_t  uart_num;      /* UART 端口号 */
    int          tcp_port;      /* TCP 监听端口 */
    int          tx_pin;        /* UART TX GPIO */
    int          rx_pin;        /* UART RX GPIO */
    int          client_fd;     /* 当前 TCP 客户端 socket，-1 表示无连接 */
    SemaphoreHandle_t mutex;    /* 保护 client_fd 的互斥锁 */
    const char  *tag;           /* 日志标签 */
} uart_channel_t;

/* 两路通道的静态实例 */
static uart_channel_t s_ch0 = {
    .uart_num  = UART_NUM_0,
    .tcp_port  = TCP_UART0_PORT,
    .tx_pin    = UART0_TX_PIN,
    .rx_pin    = UART0_RX_PIN,
    .client_fd = -1,
    .mutex     = NULL,
    .tag       = "UART0-TCP",
};

static uart_channel_t s_ch1 = {
    .uart_num  = UART_NUM_1,
    .tcp_port  = TCP_UART1_PORT,
    .tx_pin    = UART1_TX_PIN,
    .rx_pin    = UART1_RX_PIN,
    .client_fd = -1,
    .mutex     = NULL,
    .tag       = "UART1-TCP",
};

/* ------------------------------------------------------------------ */
/*  内部辅助函数                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 初始化指定通道的 UART 驱动
 *
 * @param ch  通道指针
 * @return ESP_OK 成功
 */
static esp_err_t uart_channel_init(uart_channel_t *ch)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(ch->uart_num, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(ch->tag, "uart_param_config 失败: %d", err);
        return err;
    }

    err = uart_set_pin(ch->uart_num, ch->tx_pin, ch->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(ch->tag, "uart_set_pin 失败: %d", err);
        return err;
    }

    /* 安装驱动：TX/RX 缓冲区各 UART_BUF_SIZE，不使用事件队列 */
    err = uart_driver_install(ch->uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(ch->tag, "uart_driver_install 失败: %d", err);
        return err;
    }

    ESP_LOGI(ch->tag, "UART%d 初始化完成 (TX=%d, RX=%d, %d bps)",
             ch->uart_num, ch->tx_pin, ch->rx_pin, UART_BAUD_RATE);
    return ESP_OK;
}

/**
 * @brief 创建并绑定 TCP 监听 socket
 *
 * @param port  监听端口
 * @param tag   日志标签
 * @return 成功返回 socket fd，失败返回 -1
 */
static int create_listen_socket(int port, const char *tag)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        ESP_LOGE(tag, "socket() 失败: %d", errno);
        return -1;
    }

    /* 允许端口复用，方便重启后立即绑定 */
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(tag, "bind() 失败: %d", errno);
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        ESP_LOGE(tag, "listen() 失败: %d", errno);
        close(fd);
        return -1;
    }

    ESP_LOGI(tag, "TCP 监听已启动，端口 %d", port);
    return fd;
}

/* ------------------------------------------------------------------ */
/*  UART RX 任务：UART → TCP                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief UART 接收任务
 *
 * 持续从 UART 读取数据，若当前有 TCP 客户端则转发过去。
 * 任务参数为 uart_channel_t 指针。
 */
static void uart_rx_task(void *arg)
{
    uart_channel_t *ch = (uart_channel_t *)arg;
    uint8_t buf[UART_BUF_SIZE];

    while (1) {
        /* 阻塞读取 UART，超时 100ms */
        int len = uart_read_bytes(ch->uart_num, buf, sizeof(buf),
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        /* 获取当前客户端 fd */
        xSemaphoreTake(ch->mutex, portMAX_DELAY);
        int fd = ch->client_fd;
        xSemaphoreGive(ch->mutex);

        if (fd < 0) {
            /* 无客户端连接，丢弃数据 */
            continue;
        }

        /* 将 UART 数据发送给 TCP 客户端 */
        int sent = send(fd, buf, len, 0);
        if (sent < 0) {
            ESP_LOGW(ch->tag, "send() 失败 (fd=%d): %d，等待新连接", fd, errno);
            /* 发送失败说明连接已断开，清除 fd；tcp_server_task 会重新 accept */
            xSemaphoreTake(ch->mutex, portMAX_DELAY);
            if (ch->client_fd == fd) {
                close(ch->client_fd);
                ch->client_fd = -1;
            }
            xSemaphoreGive(ch->mutex);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  TCP Server 任务：TCP → UART                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief TCP 服务器任务
 *
 * 循环 accept 客户端连接，接收 TCP 数据后写入 UART。
 * 同一时刻仅保留一个客户端；新连接到来时关闭旧连接。
 * 任务参数为 uart_channel_t 指针。
 */
static void tcp_server_task(void *arg)
{
    uart_channel_t *ch = (uart_channel_t *)arg;
    uint8_t buf[TCP_RX_BUF_SIZE];

    /* 创建监听 socket */
    int listen_fd = create_listen_socket(ch->tcp_port, ch->tag);
    if (listen_fd < 0) {
        ESP_LOGE(ch->tag, "监听 socket 创建失败，任务退出");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        ESP_LOGI(ch->tag, "等待 TCP 客户端连接...");
        int new_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (new_fd < 0) {
            ESP_LOGE(ch->tag, "accept() 失败: %d，重试", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(ch->tag, "客户端已连接: %s:%d",
                 inet_ntoa(client_addr.sin_addr),
                 ntohs(client_addr.sin_port));

        /* 关闭旧连接，更新 client_fd */
        xSemaphoreTake(ch->mutex, portMAX_DELAY);
        if (ch->client_fd >= 0) {
            ESP_LOGW(ch->tag, "关闭旧连接 fd=%d", ch->client_fd);
            close(ch->client_fd);
        }
        ch->client_fd = new_fd;
        xSemaphoreGive(ch->mutex);

        /* 禁用 Nagle 算法，降低透传延迟 */
        int flag = 1;
        setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        /* 循环接收 TCP 数据并写入 UART */
        while (1) {
            int len = recv(new_fd, buf, sizeof(buf), 0);
            if (len <= 0) {
                /* 连接断开或出错 */
                ESP_LOGI(ch->tag, "客户端断开连接 (fd=%d, ret=%d)", new_fd, len);
                break;
            }
            /* 将 TCP 数据写入 UART */
            uart_write_bytes(ch->uart_num, (const char *)buf, len);
        }

        /* 清除 client_fd */
        xSemaphoreTake(ch->mutex, portMAX_DELAY);
        if (ch->client_fd == new_fd) {
            close(ch->client_fd);
            ch->client_fd = -1;
        }
        xSemaphoreGive(ch->mutex);
    }
}

/* ------------------------------------------------------------------ */
/*  通用启动函数                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 启动一路 UART-TCP 透传服务
 *
 * @param ch  通道指针（s_ch0 或 s_ch1）
 * @return ESP_OK 成功
 */
static esp_err_t uart_tcp_server_start(uart_channel_t *ch)
{
    /* 创建互斥锁 */
    ch->mutex = xSemaphoreCreateMutex();
    if (ch->mutex == NULL) {
        ESP_LOGE(ch->tag, "互斥锁创建失败");
        return ESP_ERR_NO_MEM;
    }

    /* 初始化 UART 驱动 */
    esp_err_t err = uart_channel_init(ch);
    if (err != ESP_OK) {
        return err;
    }

    /* 启动 UART RX 任务（UART → TCP），运行在 core 1 */
    BaseType_t ret = xTaskCreatePinnedToCore(
        uart_rx_task, ch->tag,
        4096, ch, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(ch->tag, "uart_rx_task 创建失败");
        return ESP_FAIL;
    }

    /* 启动 TCP Server 任务（TCP → UART），运行在 core 1 */
    char task_name[32];
    snprintf(task_name, sizeof(task_name), "%s_srv", ch->tag);
    ret = xTaskCreatePinnedToCore(
        tcp_server_task, task_name,
        4096, ch, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(ch->tag, "tcp_server_task 创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(ch->tag, "透传服务已启动，TCP 端口 %d ↔ UART%d",
             ch->tcp_port, ch->uart_num);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  公共接口实现                                                        */
/* ------------------------------------------------------------------ */

esp_err_t tcp_uart0_server_start(void)
{
    return uart_tcp_server_start(&s_ch0);
}

esp_err_t tcp_uart1_server_start(void)
{
    return uart_tcp_server_start(&s_ch1);
}
