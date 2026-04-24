/**
 * @file eth_tcp_server.c
 * @brief 开发板网口 TCP 双向通信服务实现
 */

#include <errno.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "eth_tcp_server.h"

static const char *TAG = "eth_tcp_srv";

#define ETH_TCP_SERVER_BACKLOG            1
#define ETH_TCP_SERVER_RX_BUF_SIZE        1024
#define ETH_TCP_SERVER_TASK_STACK_SIZE    4096
#define ETH_TCP_SERVER_TASK_PRIORITY      5
#define ETH_TCP_SERVER_TASK_CORE          1

static const char s_welcome_text[] =
    "WT99P4C5-S1 ethernet tcp ready\r\n";

typedef struct {
    int client_fd;
    SemaphoreHandle_t client_mutex;
    TaskHandle_t task_handle;
    eth_tcp_server_rx_cb_t rx_cb;
} eth_tcp_server_ctx_t;

static eth_tcp_server_ctx_t s_server = {
    .client_fd = -1,
    .client_mutex = NULL,
    .task_handle = NULL,
    .rx_cb = NULL,
};

static void eth_tcp_server_close_client_locked(void)
{
    if (s_server.client_fd >= 0) {
        close(s_server.client_fd);
        s_server.client_fd = -1;
    }
}

bool eth_tcp_server_has_client(void)
{
    bool connected;

    if (!s_server.client_mutex) {
        return false;
    }

    xSemaphoreTake(s_server.client_mutex, portMAX_DELAY);
    connected = (s_server.client_fd >= 0);
    xSemaphoreGive(s_server.client_mutex);
    return connected;
}

void eth_tcp_server_set_rx_callback(eth_tcp_server_rx_cb_t cb)
{
    s_server.rx_cb = cb;
}

esp_err_t eth_tcp_server_send(const uint8_t *data, size_t len)
{
    int fd;
    size_t sent_total = 0;

    if (!data || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_server.client_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_server.client_mutex, portMAX_DELAY);
    fd = s_server.client_fd;
    if (fd < 0) {
        xSemaphoreGive(s_server.client_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    while (sent_total < len) {
        int sent = send(fd, data + sent_total, (int)(len - sent_total), 0);
        if (sent <= 0) {
            ESP_LOGW(TAG, "send() 失败: errno=%d，关闭当前客户端", errno);
            eth_tcp_server_close_client_locked();
            xSemaphoreGive(s_server.client_mutex);
            return ESP_FAIL;
        }
        sent_total += (size_t)sent;
    }

    xSemaphoreGive(s_server.client_mutex);
    return ESP_OK;
}

static int eth_tcp_server_create_listen_socket(void)
{
    int fd;
    int opt = 1;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(ETH_TCP_SERVER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket() 失败: errno=%d", errno);
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() 失败: errno=%d", errno);
        close(fd);
        return -1;
    }

    if (listen(fd, ETH_TCP_SERVER_BACKLOG) < 0) {
        ESP_LOGE(TAG, "listen() 失败: errno=%d", errno);
        close(fd);
        return -1;
    }

    return fd;
}

static void eth_tcp_server_log_rx(const uint8_t *data, size_t len)
{
    uint8_t preview[8] = {0};
    size_t preview_len = (len < sizeof(preview)) ? len : sizeof(preview);

    memcpy(preview, data, preview_len);
    ESP_LOGI(TAG,
             "收到电脑数据 | len=%u | bytes=%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned)len,
             preview[0], preview[1], preview[2], preview[3],
             preview[4], preview[5], preview[6], preview[7]);
}

static void eth_tcp_server_task(void *arg)
{
    int listen_fd;
    uint8_t rx_buf[ETH_TCP_SERVER_RX_BUF_SIZE];

    (void)arg;

    listen_fd = eth_tcp_server_create_listen_socket();
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "创建监听 socket 失败，任务退出");
        s_server.task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP 服务已启动，监听端口 %d", ETH_TCP_SERVER_PORT);

    while (1) {
        struct sockaddr_in client_addr = {0};
        socklen_t client_len = sizeof(client_addr);
        int new_fd;
        char client_ip[16] = {0};

        ESP_LOGI(TAG, "等待电脑连接...");
        new_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (new_fd < 0) {
            ESP_LOGW(TAG, "accept() 失败: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        inet_ntoa_r(client_addr.sin_addr, client_ip, sizeof(client_ip));
        ESP_LOGI(TAG, "电脑已连接: %s:%u", client_ip, ntohs(client_addr.sin_port));

        xSemaphoreTake(s_server.client_mutex, portMAX_DELAY);
        eth_tcp_server_close_client_locked();
        s_server.client_fd = new_fd;
        xSemaphoreGive(s_server.client_mutex);

        (void)eth_tcp_server_send((const uint8_t *)s_welcome_text, strlen(s_welcome_text));

        while (1) {
            int len = recv(new_fd, rx_buf, sizeof(rx_buf), 0);
            if (len == 0) {
                ESP_LOGI(TAG, "电脑已断开连接");
                break;
            }
            if (len < 0) {
                ESP_LOGW(TAG, "recv() 失败: errno=%d", errno);
                break;
            }

            eth_tcp_server_log_rx(rx_buf, (size_t)len);
            if (s_server.rx_cb) {
                s_server.rx_cb(rx_buf, (size_t)len);
            } else {
                (void)eth_tcp_server_send(rx_buf, (size_t)len);
            }
        }

        xSemaphoreTake(s_server.client_mutex, portMAX_DELAY);
        if (s_server.client_fd == new_fd) {
            eth_tcp_server_close_client_locked();
        } else {
            close(new_fd);
        }
        xSemaphoreGive(s_server.client_mutex);
    }
}

esp_err_t eth_tcp_server_start(void)
{
    BaseType_t ret;

    if (s_server.task_handle) {
        return ESP_OK;
    }

    if (!s_server.client_mutex) {
        s_server.client_mutex = xSemaphoreCreateMutex();
        if (!s_server.client_mutex) {
            ESP_LOGE(TAG, "创建客户端互斥锁失败");
            return ESP_ERR_NO_MEM;
        }
    }

    ret = xTaskCreatePinnedToCore(eth_tcp_server_task, "eth_tcp_srv",
                                  ETH_TCP_SERVER_TASK_STACK_SIZE, NULL,
                                  ETH_TCP_SERVER_TASK_PRIORITY,
                                  &s_server.task_handle, ETH_TCP_SERVER_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 TCP 服务任务失败");
        s_server.task_handle = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}
