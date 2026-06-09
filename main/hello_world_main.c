/**
 * @file hello_world_main.c
 * @brief 应用程序主入口
 *
 * 负责系统初始化与各模块启动，具体实现位于 User/src 与 User/include 目录：
 *   - device_web_config: 网页设备配置读写（网络场景、波特率、分辨率）
 *   - wifi_connect     : WiFi AP/STA 网络（通过 ESP32-C5 SDIO 协处理器）
 *   - z1mini_bridge    : Z-1mini 吊舱网口透传（WiFi AP <-> 以太网）
 *   - media_storage    : TF 卡媒体存储（当前已接入自动照片存储）
 *   - rtsp_server      : RTSP/RTP 视频流服务器（端口 554，兼容 580）
 *   - camera           : OV5647 MIPI-CSI 采集 + H.264 编码 + 推流
 *   - usb_thermal_camera: USB UVC 热像仪持续采集与灰度帧转换
 *   - tcp_uart_server  : TCP-UART 双向透传（端口 8880/8881）
 *   - ftp_server       : TF 卡只读 FTP 文件获取（端口 21）
 *
 * 访问方式：
 *   视频流  : rtsp://<设备IP>
 *   吊舱流  : rtsp://192.168.144.108
 *   网页    : http://<设备IP>/
 *   FTP     : ftp://ftpuser:ftpuser@<设备IP>
 *   串口0   : TCP <设备IP>:8880
 *   串口1   : TCP <设备IP>:8881
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_log.h"

/* 用户模块 */
#include "wifi_connect.h"
#include "eth_connect.h"
#include "z1mini_bridge.h"
#include "z1mini_sta_relay.h"
#include "device_web_config.h"
#include "rtsp_server.h"
#include "camera.h"
#include "media_storage.h"
#include "photo_web_server.h"
#include "tcp_uart_server.h"
#include "tf_card.h"
#include "usb_thermal_camera.h"
#include "ftp_server.h"

/* 网络连接模式选择：开启后 AP=Z-1mini 透传 AP，STA=Z-1mini STA 代理 */
#define Z1MINI_BRIDGE_ENABLE   1
#define USE_ETHERNET           0   //切换网络连接方式：0=WiFi, 1=以太网
#define HOSTED_WIFI_READY_DELAY_MS 6000
#define WIFI_WAIT_SLICE_MS         15000
#define WIFI_TIME_WAIT_MS          8000
#define ETH_IP_WAIT_SLICE_MS       15000
#define Z1MINI_BRIDGE_WAIT_SLICE_MS 15000
#define MIPI_CAMERA_START_TASK_STACK_SIZE (12 * 1024)
#define MIPI_CAMERA_START_TASK_PRIORITY   (tskIDLE_PRIORITY + 4)

#define ETH_USE_STATIC_IP      true
#define ETH_STATIC_IP          "169.254.27.100"
#define ETH_STATIC_GW          "169.254.27.1"
#define ETH_STATIC_MASK        "255.255.255.0"

#define Z1MINI_BRIDGE_IP       "192.168.144.2"
#define Z1MINI_BRIDGE_GW       "192.168.144.2"
#define Z1MINI_BRIDGE_MASK     "255.255.255.0"
#define Z1MINI_DEVICE_IP       "192.168.144.108"
#define Z1MINI_RTSP_URL        "rtsp://192.168.144.108"

static const char *TAG = "main";

/* 用于等待 SDIO transport 就绪的二值信号量 */
static SemaphoreHandle_t s_hosted_up_sem;

static void hosted_event_handler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data);

static void wait_for_hosted_wifi_ready(void)
{
    s_hosted_up_sem = xSemaphoreCreateBinary();
    esp_event_handler_register(ESP_HOSTED_EVENT, ESP_EVENT_ANY_ID,
                               hosted_event_handler, NULL);
    esp_hosted_init();
    esp_hosted_connect_to_slave();

    ESP_LOGI(TAG, "等待 C5 建立 SDIO 连接...");
    xSemaphoreTake(s_hosted_up_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "SDIO 连接已建立");

    /*
     * TRANSPORT_UP 只代表主从数据链路就绪，远端 C5 的 Wi-Fi 固件和射频初始化
     * 还需要一点时间。保留原有启动等待，避免过早下发 Wi-Fi 配置。
     */
    vTaskDelay(pdMS_TO_TICKS(HOSTED_WIFI_READY_DELAY_MS));
}

/**
 * @brief MIPI 摄像头初始化任务
 *
 * esp_video_init 和 OV5647 初始化阶段栈占用较高，不能直接压在默认 main 任务栈上。
 * 用独立大栈任务完成初始化，初始化成功后 camera_init() 会创建实际采集任务。
 */
static void mipi_camera_start_task(void *arg)
{
    esp_err_t err;

    (void)arg;

    ESP_LOGI(TAG, "MIPI 摄像头初始化任务已启动");
    err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败: 0x%x", err);
    }

    vTaskDelete(NULL);
}

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
 *   2. 设备网页配置初始化
 *   3. esp_netif + 事件循环
 *   4. ESP-Hosted → 等待与 C5 建立 SDIO 链路
 *   5. 按编译开关选择 Z-1mini 网口透传、普通 WiFi 或普通以太网
 *   6. TF 卡初始化
 *   7. 媒体存储模块初始化
 *   8. RTSP 服务器启动
 *   9. UART TCP 透传服务启动
 *   10. 按网页配置选择 MIPI 摄像头或 USB 热像仪 RTSP 链路
 */
void app_main(void)
{
    device_web_config_t app_config = {0};

    /* 1. NVS 初始化 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    err = device_web_config_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "设备网页配置初始化失败，将继续使用默认配置: 0x%x", err);
    }
    device_web_config_get(&app_config);

    /* 2. 网络接口与事件循环 */
    esp_netif_init();
    esp_event_loop_create_default();

#if Z1MINI_BRIDGE_ENABLE
    wait_for_hosted_wifi_ready();

    if (app_config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_AP) {
        ESP_LOGI(TAG, "使用 Z-1mini 透传 AP | WT99=%s | 吊舱=%s | RTSP=%s",
                 Z1MINI_BRIDGE_IP, Z1MINI_DEVICE_IP, Z1MINI_RTSP_URL);

        err = z1mini_bridge_init(Z1MINI_BRIDGE_IP,
                                 Z1MINI_BRIDGE_GW,
                                 Z1MINI_BRIDGE_MASK,
                                 Z1MINI_DEVICE_IP);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Z-1mini 透传 AP 初始化失败: 0x%x", err);
            return;
        }

        ESP_LOGI(TAG, "等待透传 AP 就绪...");
        while ((err = z1mini_bridge_wait_for_ready(Z1MINI_BRIDGE_WAIT_SLICE_MS)) != ESP_OK) {
            ESP_LOGW(TAG, "透传 AP 尚未就绪，继续等待...");
        }

        ESP_LOGI(TAG, "透传 AP 已就绪，正在检测吊舱网口链路...");
        err = z1mini_bridge_wait_for_eth_link(Z1MINI_BRIDGE_WAIT_SLICE_MS);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "吊舱网口链路已连接 | 吊舱访问地址: %s", Z1MINI_RTSP_URL);
        } else {
            ESP_LOGW(TAG, "吊舱网口链路暂未连接，继续启动本机服务；请检查吊舱供电和网线");
        }
    } else if (app_config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) {
        ESP_LOGI(TAG, "使用 Z-1mini STA 入网模式 | STA=%s | 吊舱=%s",
                 app_config.wifi_sta_ssid, Z1MINI_DEVICE_IP);

        err = wifi_connect_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WiFi STA 初始化失败: 0x%x", err);
            return;
        }

        ESP_LOGI(TAG, "等待 WiFi STA 网络就绪...");
        while ((err = wifi_connect_wait_for_ip(WIFI_WAIT_SLICE_MS)) != ESP_OK) {
            ESP_LOGW(TAG, "WiFi STA 网络尚未就绪，继续等待...");
        }

        err = z1mini_sta_relay_start(Z1MINI_BRIDGE_IP,
                                     Z1MINI_BRIDGE_GW,
                                     Z1MINI_BRIDGE_MASK,
                                     Z1MINI_DEVICE_IP);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Z-1mini STA 代理启动失败: 0x%x", err);
            return;
        }

        ESP_LOGI(TAG, "Z-1mini STA Proxy ARP 已就绪 | QGC 可尝试 rtsp://%s",
                 Z1MINI_DEVICE_IP);
    } else {
        ESP_LOGE(TAG, "Wi-Fi 模式配置非法: %" PRIu32, app_config.wifi_mode);
        return;
    }

    err = wifi_connect_wait_for_time(WIFI_TIME_WAIT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "系统时间尚未同步，请打开媒体网页同步电脑时间");
    }

#elif USE_ETHERNET
    /* 使用以太网连接 */
    ESP_LOGI(TAG, "使用以太网连接");

    err = eth_connect_init(ETH_USE_STATIC_IP, ETH_STATIC_IP,
                           ETH_STATIC_GW, ETH_STATIC_MASK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "以太网初始化失败: 0x%x", err);
        return;
    }

    ESP_LOGI(TAG, "等待以太网获取 IP...");
    while ((err = eth_connect_wait_for_ip(ETH_IP_WAIT_SLICE_MS)) != ESP_OK) {
        ESP_LOGW(TAG, "以太网 IP 未就绪，继续等待链路建立...");
    }
    ESP_LOGI(TAG, "以太网 IP 已就绪，开始启动视频与网页服务");

#else
    ESP_LOGI(TAG, "使用 WiFi 网络模式: %s",
             device_web_config_get_wifi_mode_name(app_config.wifi_mode));

    wait_for_hosted_wifi_ready();

    err = wifi_connect_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 初始化失败: 0x%x", err);
        return;
    }

    ESP_LOGI(TAG, "等待 WiFi 网络就绪...");
    while ((err = wifi_connect_wait_for_ip(WIFI_WAIT_SLICE_MS)) != ESP_OK) {
        ESP_LOGW(TAG, "WiFi 网络尚未就绪，继续等待...");
    }
    ESP_LOGI(TAG, "WiFi 网络已就绪，开始启动各项服务");
    err = wifi_connect_wait_for_time(WIFI_TIME_WAIT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "系统时间尚未同步，请打开媒体网页同步电脑时间");
    }
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

    /* 启动 SD 卡媒体网页浏览服务（HTTP 80 端口） */
    err = photo_web_server_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "媒体网页服务启动失败: 0x%x", err);
    } else {
        ESP_LOGI(TAG, "媒体网页服务已启动，访问地址: http://<设备IP>/");
    }

    /* 启动 TF 卡只读 FTP 文件获取服务（端口 21）
     * 当前用于远程列目录和下载文件，不提供上传、删除或改名。 */
    err = ftp_server_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FTP 文件获取服务启动失败: 0x%x", err);
    } else {
        ESP_LOGI(TAG, "FTP 文件获取服务已启动，访问地址: ftp://%s:%s@<设备IP>",
                 FTP_SERVER_USER, FTP_SERVER_PASSWORD);
    }

    /* 5. 启动 RTSP 服务器（默认 554，兼容热像仪地址 580/live/6） */
    err = rtsp_server_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RTSP 服务器启动失败: 0x%x", err);
        return;
    }
    ESP_LOGI(TAG, "RTSP 服务器已启动，访问地址: rtsp://<设备IP> 和 rtsp://<设备IP>:%d/live/6",
             RTSP_THERMAL_PORT);

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

    ESP_LOGI(TAG, "当前视频源: %s",
             device_web_config_get_video_source_name(app_config.video_source));

    if (app_config.video_source == DEVICE_WEB_CONFIG_VIDEO_SOURCE_MIPI) {
        /* 7. MIPI 初始化栈占用较高，放到独立大栈任务中执行，避免 main 任务栈溢出。 */
        BaseType_t task_ret = xTaskCreatePinnedToCore(mipi_camera_start_task,
                                                      "mipi_cam_start",
                                                      MIPI_CAMERA_START_TASK_STACK_SIZE,
                                                      NULL,
                                                      MIPI_CAMERA_START_TASK_PRIORITY,
                                                      NULL,
                                                      1);
        if (task_ret != pdPASS) {
            ESP_LOGE(TAG, "创建 MIPI 摄像头初始化任务失败");
        }
    } else if (app_config.video_source == DEVICE_WEB_CONFIG_VIDEO_SOURCE_USB_THERMAL) {
        /* 7. 启动 USB 热像仪 UVC 采集。
         * 热像仪 RTSP 链路复用同一个 RTSP 服务端，和 MIPI 摄像头不同时工作。 */
        err = usb_thermal_camera_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "USB 热像仪采集启动失败: 0x%x", err);
        } else {
            err = usb_thermal_camera_rtsp_init();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "USB 热像仪 RTSP 初始化失败: 0x%x", err);
            }
        }
    } else {
        ESP_LOGW(TAG, "视频源配置非法，未启动摄像头: %" PRIu32, app_config.video_source);
    }
}
