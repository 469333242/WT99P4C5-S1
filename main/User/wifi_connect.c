/**
 * @file wifi_connect.c
 * @brief WiFi连接模块实现
 *
 * 实现WiFi STA模式的事件处理和初始化，通过ESP32-C5协处理器连接WiFi网络。
 * 支持多环境 Profile：静态IP 或 DHCP，由 wifi_connect.h 中的宏选择。
 */

#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"
#include "wifi_connect.h"

static const char *TAG = "wifi_connect";

/* 当前激活的 Profile（在 init 时从宏拷贝，避免复合字面量生命周期问题） */
static wifi_profile_t s_profile;

static TimerHandle_t  s_reconnect_timer;
static esp_netif_t   *s_sta_netif;
static EventGroupHandle_t s_wifi_event_group;
static bool s_sntp_started;

#define WIFI_GOT_IP_BIT BIT0
#define WIFI_SNTP_SERVER "ntp.aliyun.com"
#define WIFI_VALID_UNIX_SEC 1704067200LL  /* 2024-01-01 00:00:00 UTC */

/* ------------------------------------------------------------------ */
/* 静态IP辅助函数                                                        */
/* ------------------------------------------------------------------ */
/**
 * @brief 为指定 netif 配置静态 IP，并停用 DHCP 客户端
 *
 * 必须在 esp_netif 创建后、esp_wifi_start() 前调用。
 */
static esp_err_t apply_static_ip(esp_netif_t *netif, const wifi_profile_t *p)
{
    /* 先停止 DHCP 客户端，否则后续 DHCP 应答会覆盖静态配置 */
    esp_err_t ret = esp_netif_dhcpc_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止DHCP客户端失败: 0x%x", ret);
        return ret;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr      = esp_ip4addr_aton(p->ip);
    ip_info.gw.addr      = esp_ip4addr_aton(p->gw);
    ip_info.netmask.addr = esp_ip4addr_aton(p->mask);

    ret = esp_netif_set_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置静态IP失败: 0x%x", ret);
        return ret;
    }

    /* 静态 IP 不会从 DHCP 获取 DNS，这里使用网关作为 DNS，便于后续 SNTP 域名解析。 */
    esp_netif_dns_info_t dns_info = {0};
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    dns_info.ip.u_addr.ip4.addr = esp_ip4addr_aton(p->gw);
    ret = esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置静态 DNS 失败: 0x%x", ret);
    }

    ESP_LOGI(TAG, "静态IP已配置 → IP:%s  GW:%s  MASK:%s", p->ip, p->gw, p->mask);
    return ESP_OK;
}

static void wifi_sntp_sync_cb(struct timeval *tv)
{
    if (!tv || tv->tv_sec < WIFI_VALID_UNIX_SEC) {
        ESP_LOGW(TAG, "SNTP 返回时间无效");
        return;
    }

    ESP_LOGI(TAG, "SNTP 时间同步完成，unix=%" PRId64, (int64_t)tv->tv_sec);
}

static void wifi_start_sntp_once(void)
{
    esp_err_t ret;
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(WIFI_SNTP_SERVER);

    if (s_sntp_started) {
        return;
    }

    config.sync_cb = wifi_sntp_sync_cb;
    ret = esp_netif_sntp_init(&config);
    if (ret == ESP_OK) {
        s_sntp_started = true;
        ESP_LOGI(TAG, "SNTP 校时已启动，服务器: %s", WIFI_SNTP_SERVER);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        s_sntp_started = true;
        ESP_LOGW(TAG, "SNTP 已由其他模块启动");
    } else {
        ESP_LOGW(TAG, "启动 SNTP 校时失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }
}

esp_err_t wifi_connect_wait_for_ip(int timeout_ms)
{
    if (!s_wifi_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_GOT_IP_BIT,
                                           pdFALSE, pdTRUE, ticks);

    return (bits & WIFI_GOT_IP_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t wifi_connect_wait_for_time(int timeout_ms)
{
    struct timeval tv = {0};
    TickType_t ticks;
    esp_err_t ret;

    gettimeofday(&tv, NULL);
    if (tv.tv_sec >= WIFI_VALID_UNIX_SEC) {
        return ESP_OK;
    }

    if (!s_sntp_started) {
        ESP_LOGW(TAG, "SNTP 尚未启动，无法等待网络校时");
        return ESP_ERR_INVALID_STATE;
    }

    ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    ret = esp_netif_sntp_sync_wait(ticks);
    if (ret == ESP_OK) {
        gettimeofday(&tv, NULL);
        if (tv.tv_sec >= WIFI_VALID_UNIX_SEC) {
            ESP_LOGI(TAG, "系统时间已通过 SNTP 校准");
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "等待 SNTP 校时超时，请打开媒体网页用电脑时间同步");
    return ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------ */
/* 定时器回调                                                            */
/* ------------------------------------------------------------------ */
static void reconnect_timer_cb(TimerHandle_t timer)
{
    esp_wifi_connect();
}

/* ------------------------------------------------------------------ */
/* WiFi / IP 事件处理                                                   */
/* ------------------------------------------------------------------ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi 启动 → 正在连接 [%s]...", s_profile.ssid);
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        xTimerStop(s_reconnect_timer, 0);
        if (s_profile.use_static_ip) {
            ESP_LOGI(TAG, "WiFi 连接成功！固定IP: %s", s_profile.ip);
        } else {
            ESP_LOGI(TAG, "WiFi 连接成功！等待DHCP分配IP...");
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        /* 静态IP时也会触发此事件，统一在这里打印最终生效的IP */
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  IP 已生效: " IPSTR, IP2STR(&event->ip_info.ip));
        if (!s_profile.use_static_ip) {
            /* DHCP模式：明确提示用户记录这个IP */
            ESP_LOGW(TAG, "  当前为DHCP模式，IP可能每次不同");
            ESP_LOGW(TAG, "  RTSP:  rtsp://" IPSTR ":8554/stream",
                     IP2STR(&event->ip_info.ip));
            ESP_LOGW(TAG, "  UART0: " IPSTR ":8880",
                     IP2STR(&event->ip_info.ip));
        }
        ESP_LOGI(TAG, "======================================");
        wifi_start_sntp_once();
        xEventGroupSetBits(s_wifi_event_group, WIFI_GOT_IP_BIT);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconn =
            (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "WiFi 断开，reason: %d，稍后重试...", disconn->reason);
        xEventGroupClearBits(s_wifi_event_group, WIFI_GOT_IP_BIT);

        TickType_t delay;
        switch (disconn->reason) {
            case 205: /* NO_AP_FOUND：热点不可见，扫描间隔较长，多等一会 */
                delay = pdMS_TO_TICKS(6000);
                break;
            case 2:   /* AUTH_EXPIRE：C5射频未就绪导致认证超时，等久一点 */
            case 201: /* HANDSHAKE_TIMEOUT：握手超时，同上 */
                delay = pdMS_TO_TICKS(5000);
                break;
            default:
                delay = pdMS_TO_TICKS(3000);
                break;
        }
        xTimerChangePeriod(s_reconnect_timer, delay, 0);
        xTimerStart(s_reconnect_timer, 0);
    }
}

/* ------------------------------------------------------------------ */
/* 初始化入口                                                            */
/* ------------------------------------------------------------------ */
/**
 * @brief 初始化WiFi并开始连接
 *
 * 自动根据 WIFI_ACTIVE_PROFILE 决定：
 *   - use_static_ip=true  → 停DHCP，写静态IP，IP永远固定
 *   - use_static_ip=false → 保持DHCP，连接后串口打印分配到的IP
 */
esp_err_t wifi_connect_init(void)
{
    /* 将宏展开的复合字面量拷贝到静态变量，确保生命周期 */
    s_profile = WIFI_ACTIVE_PROFILE;

    ESP_LOGI(TAG, "当前网络配置: SSID=[%s]  模式=%s",
             s_profile.ssid,
             s_profile.use_static_ip ? "静态IP" : "DHCP");

    s_reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(3000),
                                     pdFALSE, NULL, reconnect_timer_cb);
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "创建 WiFi 事件组失败");
        return ESP_ERR_NO_MEM;
    }

    /* 创建默认 STA 网络接口 */
    s_sta_netif = esp_netif_create_default_wifi_sta();

    /* 静态IP：在 esp_wifi_start() 前配置，防止DHCP覆盖 */
    if (s_profile.use_static_ip) {
        ESP_ERROR_CHECK(apply_static_ip(s_sta_netif, &s_profile));
    }

    /* 注册事件处理器 */
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               wifi_event_handler, NULL);

    /* 初始化 WiFi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* 配置 STA：SSID / 密码 */
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid,
            s_profile.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password,
            s_profile.password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable    = true;
    wifi_config.sta.pmf_cfg.required   = false;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_ret != ESP_OK) {
        ESP_LOGW(TAG, "关闭 WiFi 省电模式失败: 0x%x", ps_ret);
    }

    return ESP_OK;
}
