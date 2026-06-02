/**
 * @file wifi_connect.c
 * @brief Wi-Fi SoftAP 模块实现
 *
 * 通过 ESP-Hosted 使用 ESP32-C5 协处理器创建设备热点。电脑或手机连接热点后，
 * 可直接访问网页、RTSP 和 TCP-UART 服务。
 */

#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "device_web_config.h"
#include "rtsp_server.h"
#include "tcp_uart_server.h"
#include "wifi_connect.h"

static const char *TAG = "wifi_ap";

#define WIFI_AP_STARTED_BIT BIT0
#define WIFI_VALID_UNIX_SEC 1704067200LL  /* 2024-01-01 00:00:00 UTC */

static esp_netif_t *s_ap_netif;
static EventGroupHandle_t s_wifi_event_group;
static volatile uint32_t s_ap_client_count;
static device_web_config_t s_ap_config;

static esp_err_t wifi_ap_configure_ip(esp_netif_t *netif)
{
    esp_err_t ret;
    esp_netif_ip_info_t ip_info = {0};

    ESP_RETURN_ON_FALSE(netif != NULL, ESP_ERR_INVALID_ARG, TAG, "AP netif 为空");

    ret = esp_netif_dhcps_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止 AP DHCP server 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ip_info.ip.addr = esp_ip4addr_aton(s_ap_config.wifi_static_ip);
    ip_info.gw.addr = esp_ip4addr_aton(s_ap_config.wifi_static_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(s_ap_config.wifi_static_mask);

    ret = esp_netif_set_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 AP IP 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_netif_dhcps_start(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGE(TAG, "启动 AP DHCP server 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "AP 网络已配置 | IP=%s | GW=%s | MASK=%s",
             s_ap_config.wifi_static_ip, s_ap_config.wifi_static_gw, s_ap_config.wifi_static_mask);
    return ESP_OK;
}

esp_err_t wifi_connect_wait_for_ip(int timeout_ms)
{
    EventBits_t bits;
    TickType_t ticks;

    if (!s_wifi_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_AP_STARTED_BIT,
                               pdFALSE, pdTRUE, ticks);

    return (bits & WIFI_AP_STARTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t wifi_connect_wait_for_time(int timeout_ms)
{
    struct timeval tv = {0};

    (void)timeout_ms;

    gettimeofday(&tv, NULL);
    if (tv.tv_sec >= WIFI_VALID_UNIX_SEC) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "AP 模式等待网页通过浏览器时间完成校时");
    return ESP_ERR_TIMEOUT;
}

uint32_t wifi_connect_get_ap_client_count(void)
{
    return s_ap_client_count;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;

    if (base != WIFI_EVENT) {
        return;
    }

    if (id == WIFI_EVENT_AP_START) {
        s_ap_client_count = 0;
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  Wi-Fi AP 已启动");
        ESP_LOGI(TAG, "  SSID:  %s", s_ap_config.wifi_ap_ssid);
        ESP_LOGI(TAG, "  PASS:  %s", s_ap_config.wifi_ap_password);
        ESP_LOGI(TAG, "  WEB:   http://%s/", s_ap_config.wifi_static_ip);
        ESP_LOGI(TAG, "  RTSP:  rtsp://%s:%d/stream", s_ap_config.wifi_static_ip, RTSP_PORT);
        ESP_LOGI(TAG, "  UART0: %s:%d", s_ap_config.wifi_static_ip, TCP_UART0_PORT);
        ESP_LOGI(TAG, "======================================");
        xEventGroupSetBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)data;

        if (s_ap_client_count < WIFI_AP_MAX_CONNECTIONS) {
            s_ap_client_count++;
        }
        ESP_LOGI(TAG,
                 "客户端已连接 AP | MAC=" MACSTR " | AID=%d | 当前客户端=%" PRIu32,
                 MAC2STR(event->mac), event->aid, s_ap_client_count);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)data;

        if (s_ap_client_count > 0U) {
            s_ap_client_count--;
        }
        ESP_LOGI(TAG,
                 "客户端已断开 AP | MAC=" MACSTR " | AID=%d | 当前客户端=%" PRIu32,
                 MAC2STR(event->mac), event->aid, s_ap_client_count);
    }
}

esp_err_t wifi_connect_init(void)
{
    esp_err_t ret;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {0};

    device_web_config_get(&s_ap_config);

    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group) {
            ESP_LOGE(TAG, "创建 Wi-Fi 事件组失败");
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            ESP_LOGE(TAG, "创建默认 AP netif 失败");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(wifi_ap_configure_ip(s_ap_netif), TAG, "配置 AP IP 失败");

    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 Wi-Fi 事件处理失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化 Wi-Fi 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    strncpy((char *)wifi_config.ap.ssid, s_ap_config.wifi_ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, s_ap_config.wifi_ap_password,
            sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len = strlen(s_ap_config.wifi_ap_ssid);
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 Wi-Fi AP 模式失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 AP 配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动 Wi-Fi AP 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_max_tx_power(84);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 Wi-Fi 发射功率失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Wi-Fi AP 初始化完成 | SSID=%s | 最大客户端=%d",
             s_ap_config.wifi_ap_ssid, WIFI_AP_MAX_CONNECTIONS);
    return ESP_OK;
}
