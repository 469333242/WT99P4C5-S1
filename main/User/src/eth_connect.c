/**
 * @file eth_connect.c
 * @brief 以太网连接模块实现
 *
 * 实现以太网连接，支持静态IP或DHCP
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "eth_connect.h"

static const char *TAG = "eth_connect";

static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static EventGroupHandle_t s_eth_event_group;

#define ETH_GOT_IP_BIT BIT0

/* ------------------------------------------------------------------ */
/* 静态IP配置                                                          */
/* ------------------------------------------------------------------ */
static esp_err_t apply_static_ip(esp_netif_t *netif, const char *ip,
                                  const char *gw, const char *mask)
{
    /* 停止 DHCP 客户端 */
    esp_err_t ret = esp_netif_dhcpc_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止DHCP客户端失败: 0x%x", ret);
        return ret;
    }

    esp_netif_ip_info_t ip_info = {0};
    ip_info.ip.addr      = esp_ip4addr_aton(ip);
    ip_info.gw.addr      = esp_ip4addr_aton(gw);
    ip_info.netmask.addr = esp_ip4addr_aton(mask);

    ret = esp_netif_set_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置静态IP失败: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "静态IP已配置 → IP:%s  GW:%s  MASK:%s", ip, gw, mask);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 以太网事件处理                                                       */
/* ------------------------------------------------------------------ */
static void eth_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (base == ETH_EVENT && id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "以太网连接成功！");

    } else if (base == ETH_EVENT && id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "以太网断开连接");
        if (s_eth_event_group) {
            xEventGroupClearBits(s_eth_event_group, ETH_GOT_IP_BIT);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  以太网IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "  RTSP:  rtsp://" IPSTR ":8554/stream",
                 IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "======================================");
        if (s_eth_event_group) {
            xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
        }
    }
}

esp_err_t eth_connect_wait_for_ip(int timeout_ms)
{
    TickType_t ticks;
    EventBits_t bits;

    if (!s_eth_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    bits = xEventGroupWaitBits(s_eth_event_group, ETH_GOT_IP_BIT,
                               pdFALSE, pdTRUE, ticks);
    return (bits & ETH_GOT_IP_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------ */
/* 初始化入口                                                           */
/* ------------------------------------------------------------------ */
esp_err_t eth_connect_init(bool use_static_ip, const char *ip,
                           const char *gw, const char *mask)
{
    esp_eth_mac_t *mac;
    esp_eth_phy_t *phy;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "初始化以太网 | 模式: %s", use_static_ip ? "静态IP" : "DHCP");

    if (!s_eth_event_group) {
        s_eth_event_group = xEventGroupCreate();
        if (!s_eth_event_group) {
            ESP_LOGE(TAG, "创建 ETH 事件组失败");
            return ESP_ERR_NO_MEM;
        }
    }

    /* 创建默认以太网网络接口 */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) {
        ESP_LOGE(TAG, "创建以太网 netif 失败");
        return ESP_FAIL;
    }

    /* 如果使用静态IP，在启动前配置 */
    if (use_static_ip && ip && gw && mask) {
        esp_err_t ret = apply_static_ip(s_eth_netif, ip, gw, mask);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    /* 注册事件处理器 */
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                               eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                               eth_event_handler, NULL);

    /*
     * WT99P4C5-S1 板载 IP101GRI + RJ45。
     * 这里使用 ESP32-P4 在 IDF v5.5 的默认 RMII/SMI 引脚映射，
     * PHY 地址改为自动探测，适配单 PHY 场景。
     */
    phy_config.phy_addr = ESP_ETH_PHY_ADDR_AUTO;
    phy_config.reset_gpio_num = -1;

    /* 创建MAC实例（使用内部EMAC）*/
    mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "创建 EMAC 失败");
        return ESP_FAIL;
    }

    /* WT99P4C5-S1 板载 PHY 为 IP101GRI */
    phy = esp_eth_phy_new_ip101(&phy_config);
    if (!phy) {
        ESP_LOGE(TAG, "创建 IP101 PHY 失败");
        return ESP_FAIL;
    }

    /* 安装以太网驱动 */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config, &s_eth_handle),
                        TAG, "安装以太网驱动失败");

    /* 将以太网驱动附加到网络接口 */
    ESP_RETURN_ON_ERROR(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)),
                        TAG, "附加以太网 netif 失败");

    /* 启动以太网 */
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth_handle), TAG, "启动以太网失败");

    ESP_LOGI(TAG, "以太网初始化完成");
    return ESP_OK;
}
