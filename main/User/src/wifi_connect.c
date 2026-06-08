/**
 * @file wifi_connect.c
 * @brief Wi-Fi AP/STA 网络模块实现
 *
 * 通过 ESP-Hosted 使用 ESP32-C5 协处理器提供 Wi-Fi 网络。网页配置中只能选择
 * AP 或 STA 其中一种模式，启动后不会同时启用 AP 与 STA。
 */

#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"

#include "device_web_config.h"
#include "rtsp_server.h"
#include "tcp_uart_server.h"
#include "wifi_connect.h"

static const char *TAG = "wifi";

#define WIFI_READY_BIT BIT0
#define WIFI_VALID_UNIX_SEC 1704067200LL  /* 2024-01-01 00:00:00 UTC */
#define WIFI_STA_CONFIRM_TIMEOUT_MS 180000

static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static EventGroupHandle_t s_wifi_event_group;
static volatile uint32_t s_ap_client_count;
static device_web_config_t s_wifi_config;
static bool s_wifi_event_registered;

static void wifi_sta_confirm_guard_task(void *arg)
{
    (void)arg;

    /*
     * STA 配置保存后设备会重启并尝试连路由器。
     * 若用户无法在网页上确认新地址可达，超时自动回退 AP，避免设备被错误 STA 配置锁死。
     */
    ESP_LOGW(TAG, "STA 模式待网页确认，%d 秒内未确认将自动回退到 AP 模式",
             WIFI_STA_CONFIRM_TIMEOUT_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(WIFI_STA_CONFIRM_TIMEOUT_MS));

    if (!device_web_config_is_sta_pending_confirm()) {
        ESP_LOGI(TAG, "STA 模式已通过网页确认，取消自动回退");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "STA 模式超时未确认，正在保存 AP 回退配置并重启");
    esp_err_t ret = device_web_config_fallback_to_ap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存 AP 回退配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void wifi_start_sta_confirm_guard_if_needed(void)
{
    if (!device_web_config_is_sta_pending_confirm()) {
        return;
    }

    /* 只有待确认的 STA 配置才启动保护任务，已确认配置正常启动不额外重启。 */
    BaseType_t task_ret = xTaskCreate(wifi_sta_confirm_guard_task,
                                      "sta_confirm_guard",
                                      4096,
                                      NULL,
                                      tskIDLE_PRIORITY + 1,
                                      NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "创建 STA 确认保护任务失败");
        esp_err_t ret = device_web_config_fallback_to_ap();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "保护任务失败后保存 AP 回退配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
}

static esp_err_t wifi_configure_ap_ip(esp_netif_t *netif)
{
    esp_err_t ret;
    esp_netif_ip_info_t ip_info = {0};

    ESP_RETURN_ON_FALSE(netif != NULL, ESP_ERR_INVALID_ARG, TAG, "AP netif 为空");

    ret = esp_netif_dhcps_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止 AP DHCP server 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ip_info.ip.addr = esp_ip4addr_aton(s_wifi_config.wifi_static_ip);
    ip_info.gw.addr = esp_ip4addr_aton(s_wifi_config.wifi_static_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(s_wifi_config.wifi_static_mask);

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
             s_wifi_config.wifi_static_ip,
             s_wifi_config.wifi_static_gw,
             s_wifi_config.wifi_static_mask);
    return ESP_OK;
}

static esp_err_t wifi_configure_sta_ip(esp_netif_t *netif)
{
    esp_err_t ret;
    esp_netif_ip_info_t ip_info = {0};

    ESP_RETURN_ON_FALSE(netif != NULL, ESP_ERR_INVALID_ARG, TAG, "STA netif 为空");

    if (!s_wifi_config.wifi_sta_use_static_ip) {
        ESP_LOGI(TAG, "STA 使用 DHCP 获取 IP");
        return ESP_OK;
    }

    ret = esp_netif_dhcpc_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止 STA DHCP client 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ip_info.ip.addr = esp_ip4addr_aton(s_wifi_config.wifi_sta_ip);
    ip_info.gw.addr = esp_ip4addr_aton(s_wifi_config.wifi_sta_gw);
    ip_info.netmask.addr = esp_ip4addr_aton(s_wifi_config.wifi_sta_mask);

    ret = esp_netif_set_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 STA 静态 IP 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "STA 静态 IP 已配置 | IP=%s | GW=%s | MASK=%s",
             s_wifi_config.wifi_sta_ip,
             s_wifi_config.wifi_sta_gw,
             s_wifi_config.wifi_sta_mask);
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
    bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_READY_BIT,
                               pdFALSE, pdTRUE, ticks);

    return (bits & WIFI_READY_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t wifi_connect_wait_for_time(int timeout_ms)
{
    struct timeval tv = {0};

    (void)timeout_ms;

    gettimeofday(&tv, NULL);
    if (tv.tv_sec >= WIFI_VALID_UNIX_SEC) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "系统时间尚未同步，请通过网页同步浏览器时间");
    return ESP_ERR_TIMEOUT;
}

uint32_t wifi_connect_get_ap_client_count(void)
{
    if (s_wifi_config.wifi_mode != DEVICE_WEB_CONFIG_WIFI_MODE_AP) {
        return 0;
    }

    return s_ap_client_count;
}

uint32_t wifi_connect_get_active_wifi_mode(void)
{
    return s_wifi_config.wifi_mode;
}

static void wifi_log_ap_ready(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Wi-Fi AP 已启动");
    ESP_LOGI(TAG, "  SSID:  %s", s_wifi_config.wifi_ap_ssid);
    ESP_LOGI(TAG, "  PASS:  %s", s_wifi_config.wifi_ap_password);
    ESP_LOGI(TAG, "  WEB:   http://%s/", s_wifi_config.wifi_static_ip);
    if (RTSP_PORT == 554) {
        ESP_LOGI(TAG, "  RTSP:  rtsp://%s", s_wifi_config.wifi_static_ip);
    } else {
        ESP_LOGI(TAG, "  RTSP:  rtsp://%s:%d/stream", s_wifi_config.wifi_static_ip, RTSP_PORT);
    }
    ESP_LOGI(TAG, "  A3热像: rtsp://%s:%d/live/6", s_wifi_config.wifi_static_ip, RTSP_THERMAL_PORT);
    ESP_LOGI(TAG, "  UART0: %s:%d", s_wifi_config.wifi_static_ip, TCP_UART0_PORT);
    ESP_LOGI(TAG, "======================================");
}

static void wifi_log_sta_ready(const esp_netif_ip_info_t *ip_info)
{
    if (!ip_info) {
        return;
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Wi-Fi STA 已连接");
    ESP_LOGI(TAG, "  SSID:  %s", s_wifi_config.wifi_sta_ssid);
    ESP_LOGI(TAG, "  IP:    " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "  GW:    " IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "  MASK:  " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "  WEB:   http://" IPSTR "/", IP2STR(&ip_info->ip));
    if (RTSP_PORT == 554) {
        ESP_LOGI(TAG, "  RTSP:  rtsp://" IPSTR, IP2STR(&ip_info->ip));
    } else {
        ESP_LOGI(TAG, "  RTSP:  rtsp://" IPSTR ":%d/stream", IP2STR(&ip_info->ip), RTSP_PORT);
    }
    ESP_LOGI(TAG, "  A3热像: rtsp://" IPSTR ":%d/live/6", IP2STR(&ip_info->ip), RTSP_THERMAL_PORT);
    ESP_LOGI(TAG, "  UART0: " IPSTR ":%d", IP2STR(&ip_info->ip), TCP_UART0_PORT);
    ESP_LOGI(TAG, "======================================");
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_START) {
            s_ap_client_count = 0;
            wifi_log_ap_ready();
            xEventGroupSetBits(s_wifi_event_group, WIFI_READY_BIT);
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
        } else if (id == WIFI_EVENT_STA_START) {
            esp_err_t ret;

            ESP_LOGI(TAG, "STA 开始连接路由器 | SSID=%s", s_wifi_config.wifi_sta_ssid);
            ret = esp_wifi_connect();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "STA 发起连接失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            }
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)data;
            esp_err_t ret;

            xEventGroupClearBits(s_wifi_event_group, WIFI_READY_BIT);
            ESP_LOGW(TAG, "STA 已断开连接 | reason=%d | 将继续重连", event ? event->reason : -1);
            ret = esp_wifi_connect();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "STA 重连失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;

        if (!event) {
            return;
        }

        wifi_log_sta_ready(&event->ip_info);
        xEventGroupSetBits(s_wifi_event_group, WIFI_READY_BIT);
    }
}

static esp_err_t wifi_register_events_once(void)
{
    esp_err_t ret;

    if (s_wifi_event_registered) {
        return ESP_OK;
    }

    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 Wi-Fi 事件处理失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                     wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 STA IP 事件处理失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    s_wifi_event_registered = true;
    return ESP_OK;
}

static esp_err_t wifi_prepare_driver(void)
{
    esp_err_t ret;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group) {
            ESP_LOGE(TAG, "创建 Wi-Fi 事件组失败");
            return ESP_ERR_NO_MEM;
        }
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_READY_BIT);

    ESP_RETURN_ON_ERROR(wifi_register_events_once(), TAG, "注册 Wi-Fi 事件失败");

    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化 Wi-Fi 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t wifi_start_ap(void)
{
    esp_err_t ret;
    wifi_config_t wifi_config = {0};

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            ESP_LOGE(TAG, "创建默认 AP netif 失败");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(wifi_configure_ap_ip(s_ap_netif), TAG, "配置 AP IP 失败");

    strncpy((char *)wifi_config.ap.ssid, s_wifi_config.wifi_ap_ssid,
            sizeof(wifi_config.ap.ssid) - 1U);
    strncpy((char *)wifi_config.ap.password, s_wifi_config.wifi_ap_password,
            sizeof(wifi_config.ap.password) - 1U);
    wifi_config.ap.ssid_len = strlen(s_wifi_config.wifi_ap_ssid);
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;

    ESP_LOGI(TAG, "准备设置 AP 配置 | SSID长度=%u | 密码长度=%u | 最大客户端=%d",
             (unsigned)strlen(s_wifi_config.wifi_ap_ssid),
             (unsigned)strlen(s_wifi_config.wifi_ap_password),
             WIFI_AP_MAX_CONNECTIONS);

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
             s_wifi_config.wifi_ap_ssid, WIFI_AP_MAX_CONNECTIONS);
    return ESP_OK;
}

static esp_err_t wifi_start_sta(void)
{
    esp_err_t ret;
    wifi_config_t wifi_config = {0};

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif) {
            ESP_LOGE(TAG, "创建默认 STA netif 失败");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(wifi_configure_sta_ip(s_sta_netif), TAG, "配置 STA IP 失败");

    strncpy((char *)wifi_config.sta.ssid, s_wifi_config.wifi_sta_ssid,
            sizeof(wifi_config.sta.ssid) - 1U);
    strncpy((char *)wifi_config.sta.password, s_wifi_config.wifi_sta_password,
            sizeof(wifi_config.sta.password) - 1U);

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 Wi-Fi STA 模式失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 STA 配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动 Wi-Fi STA 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "关闭 STA 省电模式失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    /* 视频流和网页控制更看重稳定低延迟，STA 模式关闭省电并提高发射功率。 */
    ret = esp_wifi_set_max_tx_power(84);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 Wi-Fi 发射功率失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Wi-Fi STA 初始化完成 | SSID=%s | IP模式=%s",
             s_wifi_config.wifi_sta_ssid,
             s_wifi_config.wifi_sta_use_static_ip ? "静态 IP" : "DHCP");
    wifi_start_sta_confirm_guard_if_needed();
    return ESP_OK;
}

esp_err_t wifi_connect_init(void)
{
    esp_err_t ret;

    device_web_config_get(&s_wifi_config);

    ret = wifi_prepare_driver();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "使用 Wi-Fi %s，AP 与 STA 不会同时启用",
             device_web_config_get_wifi_mode_name(s_wifi_config.wifi_mode));

    if (s_wifi_config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_AP) {
        return wifi_start_ap();
    }
    if (s_wifi_config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) {
        return wifi_start_sta();
    }

    ESP_LOGE(TAG, "Wi-Fi 模式配置非法: %" PRIu32, s_wifi_config.wifi_mode);
    return ESP_ERR_INVALID_ARG;
}
