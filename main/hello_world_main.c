/**
 * @file hello_world_main.c
 * @brief 应用程序主入口
 *
 * 负责系统初始化与各模块启动，具体实现均在 User/ 目录下：
 *   - wifi_connect     : WiFi STA 连接（通过 ESP32-C5 SDIO 协处理器）
 *   - media_storage    : TF 卡媒体存储（当前已接入自动照片存储）
 *   - rtsp_server      : RTSP/RTP 视频流服务器（端口 8554）
 *   - camera           : OV5647 MIPI-CSI 采集 + H.264 编码 + 推流
 *   - tcp_uart_server  : TCP-UART 双向透传（端口 8880/8881）
 *
 * 访问方式：
 *   视频流  : rtsp://<设备WiFi_IP>:8554/stream
 *   ffplay -fflags nobuffer -flags low_delay -framedrop -rtsp_transport tcp "rtsp://192.168.137.200:8554/stream"
 *   串口0   : TCP <设备WiFi_IP>:8880
 *   串口1   : TCP <设备WiFi_IP>:8881
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_log.h"

/* 用户模块 */
#include "wifi_connect.h"
#include "eth_connect.h" 
#include "rtsp_server.h"
#include "camera.h"
#include "media_storage.h"
#include "photo_web_server.h"
#include "tcp_uart_server.h"
#include "tf_card.h"

/* 网络连接模式选择：0=WiFi, 1=以太网 */
#define USE_ETHERNET    0
#define HOSTED_WIFI_READY_DELAY_MS 6000
#define WIFI_IP_WAIT_SLICE_MS      15000


static const char *TAG = "main";

/* 用于等待 SDIO transport 就绪的二值信号量 */
static SemaphoreHandle_t s_hosted_up_sem;

/**
 * @brief ESP-Hosted 事件回调
 *
 * SDIO 链路建立后释放信号量，通知 app_main 继续执行 WiFi 初始化。
 */
static void hosted_event_handler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data)
{
    if (base == ESP_HOSTED_EVENT && id == ESP_HOSTED_EVENT_TRANSPORT_UP) {
        ESP_LOGI(TAG, "SDIO 传输链路已就绪");
        xSemaphoreGive(s_hosted_up_sem);
    }
}




/**
 * @brief 应用程序主入口
 *
 * 初始化顺序：
 *   1. NVS flash（WiFi 驱动依赖）
 *   2. esp_netif + 事件循环
 *   3. ESP-Hosted → 等待与 C5 建立 SDIO 链路
 *   4. WiFi 连接
 *   5. TF 卡初始化
 *   6. 媒体存储模块初始化
 *   7. RTSP 服务器启动
 *   8. UART TCP 透传服务启动
 *   9. 以太网初始化并启动 ETH TCP 透传服务
 *   10. 摄像头初始化并开始采集推流
 */
void app_main(void)
{
    /* 1. NVS 初始化 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 2. 网络接口与事件循环 */
    esp_netif_init();
    esp_event_loop_create_default();

#if USE_ETHERNET
    /* 使用以太网连接 */
    ESP_LOGI(TAG, "使用以太网连接");

    /* 以太网配置：true=静态IP, false=DHCP */
    bool use_static_ip = true;
    const char *eth_ip = "192.168.1.100";
    const char *eth_gw = "192.168.1.1";
    const char *eth_mask = "255.255.255.0";

    err = eth_connect_init(use_static_ip, eth_ip, eth_gw, eth_mask);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "以太网初始化失败: 0x%x", err);
        return;
    }

    /* 等待以太网连接 */
    vTaskDelay(pdMS_TO_TICKS(3000));

#else
    /* 使用WiFi连接 */
    ESP_LOGI(TAG, "使用WiFi连接");

    /* 3. ESP-Hosted：建立与 C5 的 SDIO 链路 */
    s_hosted_up_sem = xSemaphoreCreateBinary();
    esp_event_handler_register(ESP_HOSTED_EVENT, ESP_EVENT_ANY_ID,
                                hosted_event_handler, NULL);
    esp_hosted_init();
    esp_hosted_connect_to_slave();

    ESP_LOGI(TAG, "等待与 C5 建立 SDIO 连接...");
    xSemaphoreTake(s_hosted_up_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "SDIO 连接已建立");

    /* 4. WiFi 连接 */
    /* SDIO TRANSPORT_UP 仅代表数据链路就绪，C5 WiFi 固件与射频初始化
     * 还需要额外时间。实测过早调用会导致 reason=2/205 反复失败，
     * 等待 3s 可保证 C5 完全就绪后再发起连接。 */
    vTaskDelay(pdMS_TO_TICKS(HOSTED_WIFI_READY_DELAY_MS));
    err = wifi_connect_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 初始化失败: 0x%x", err);
        return;
    }

    ESP_LOGI(TAG, "等待 WiFi 获取 IP...");
    while ((err = wifi_connect_wait_for_ip(WIFI_IP_WAIT_SLICE_MS)) != ESP_OK) {
        ESP_LOGW(TAG, "WiFi IP 未就绪，仍在等待重连完成...");
    }
    ESP_LOGI(TAG, "WiFi IP 已就绪，开始启动各项服务");
#endif

    /* TF 卡基础驱动初始化。
     * 失败时仅记录日志，不影响现有 RTSP / WiFi / UART 功能。 */
    err = tf_card_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TF 卡初始化失败: 0x%x", err);
    }

    /* 媒体存储模块初始化。
     * 当前先启用自动照片存储能力；若 TF 卡未就绪，仅记录日志，不影响 RTSP 功能。 */
    err = media_storage_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "媒体存储模块初始化失败: 0x%x", err);
    }

    /* 启动 SD 卡照片网页浏览服务（HTTP 80 端口） */
    err = photo_web_server_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "照片网页服务启动失败: 0x%x", err);
    } else {
        ESP_LOGI(TAG, "照片网页服务已启动，访问地址: http://<IP>/");
    }

    /* 5. 启动 RTSP 服务器（端口 8554） */
    err = rtsp_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTSP 服务器启动失败: 0x%x", err);
        return;
    }
    ESP_LOGI(TAG, "RTSP 服务器已启动，访问地址: rtsp://<IP>:%d/stream", RTSP_PORT);

    /* 6. 启动 UART0 TCP 透传服务（端口 8880） */
    /*    启动 UART1 TCP 透传服务（端口 8881） */
    err = tcp_uart0_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART0 透传服务启动失败: 0x%x", err);
    }
    //err = tcp_uart1_server_start();
    //if (err != ESP_OK) {
    //    ESP_LOGE(TAG, "UART1 透传服务启动失败: 0x%x", err);
    //}

    /* 7. 初始化摄像头并开始采集推流 */
    err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败: 0x%x", err);
    }
}
