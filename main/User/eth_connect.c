/**
 * @file eth_connect.c
 * @brief 以太网连接模块实现
 *
 * 实现以太网连接，支持静态IP或DHCP
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "eth_connect.h"

static const char *TAG = "eth_connect";

static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;

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

    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "  以太网IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "  RTSP:  rtsp://" IPSTR ":8554/stream",
                 IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "======================================");
    }
}

/* ------------------------------------------------------------------ */
/* 初始化入口                                                           */
/* ------------------------------------------------------------------ */
esp_err_t eth_connect_init(bool use_static_ip, const char *ip,
                           const char *gw, const char *mask)
{
    ESP_LOGI(TAG, "初始化以太网 | 模式: %s", use_static_ip ? "静态IP" : "DHCP");

    /* 创建默认以太网网络接口 */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);

    /* 如果使用静态IP，在启动前配置 */
    if (use_static_ip && ip && gw && mask) {
        ESP_ERROR_CHECK(apply_static_ip(s_eth_netif, ip, gw, mask));
    }

    /* 注册事件处理器 */
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                               eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                               eth_event_handler, NULL);

    /* 配置以太网MAC和PHY */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    /* 根据你的硬件配置PHY地址和复位引脚 */
    phy_config.phy_addr = 0;  /* 根据实际硬件修改 */
    phy_config.reset_gpio_num = -1;  /* 如果有复位引脚，修改这里 */

    /* 创建MAC实例（使用内部EMAC）*/
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    /* 创建PHY实例（根据你的PHY芯片选择，常见的有IP101、LAN8720等）*/
    /* 这里以IP101为例，如果是其他芯片需要修改 */
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    /* 安装以太网驱动 */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    /* 将以太网驱动附加到网络接口 */
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));

    /* 启动以太网 */
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    ESP_LOGI(TAG, "以太网初始化完成");
    return ESP_OK;
}
