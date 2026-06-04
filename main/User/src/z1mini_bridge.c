/**
 * @file z1mini_bridge.c
 * @brief Z-1mini 吊舱网口透传模块实现
 *
 * 拓扑：
 *   电脑 Wi-Fi 192.168.144.x <-> WT99 AP 192.168.144.1 <-> ETH <-> Z-1mini 192.168.144.108
 *
 * ESP32-P4 使用 ESP-Hosted 远端 Wi-Fi，IDF 原生 esp_netif 二层桥只支持本机 Wi-Fi，
 * 因此这里使用 Wi-Fi AP 原始帧回调与以太网原始帧回调做定向转发。
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "dhcpserver/dhcpserver.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_private/wifi.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

#include "device_web_config.h"
#include "rtsp_server.h"
#include "tcp_uart_server.h"
#include "wifi_connect.h"
#include "z1mini_bridge.h"

static const char *TAG = "z1mini_bridge";

#define Z1MINI_BRIDGE_READY_BIT       BIT0
#define Z1MINI_BRIDGE_ETH_LINK_BIT    BIT1
#define Z1MINI_BRIDGE_ETH_HEADER_LEN  14U
#define Z1MINI_BRIDGE_ETH_MIN_TX_LEN  60U
#define Z1MINI_BRIDGE_ETH_MAX_FRAME   1536U
#define Z1MINI_BRIDGE_DHCP_START_IP   "192.168.144.20"
#define Z1MINI_BRIDGE_DHCP_END_IP     "192.168.144.60"

static esp_netif_t *s_wifi_netif;
static esp_eth_handle_t s_eth_handle;
static EventGroupHandle_t s_bridge_event_group;
static esp_netif_ip_info_t s_ap_ip_info;
static volatile uint32_t s_ap_client_count;
static volatile uint32_t s_wifi_to_eth_count;
static volatile uint32_t s_eth_to_wifi_count;
static bool s_wifi_rx_registered;
static bool s_bridge_running;
static bool s_eth_link_up;

static esp_err_t z1mini_bridge_configure_dhcp_pool(esp_netif_t *netif);

static esp_err_t z1mini_bridge_fill_ip_info(esp_netif_ip_info_t *ip_info,
                                            const char *ip, const char *gw,
                                            const char *mask)
{
    ESP_RETURN_ON_FALSE(ip_info != NULL && ip != NULL && gw != NULL && mask != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "AP IP 参数为空");
    ESP_RETURN_ON_FALSE(device_web_config_is_valid_ipv4_text(ip) &&
                        device_web_config_is_valid_ipv4_text(gw) &&
                        device_web_config_is_valid_ipv4_text(mask),
                        ESP_ERR_INVALID_ARG, TAG, "AP IP 参数非法");

    memset(ip_info, 0, sizeof(*ip_info));
    ip_info->ip.addr = esp_ip4addr_aton(ip);
    ip_info->gw.addr = esp_ip4addr_aton(gw);
    ip_info->netmask.addr = esp_ip4addr_aton(mask);
    return ESP_OK;
}

static esp_err_t z1mini_bridge_configure_ap_ip(esp_netif_t *netif,
                                               esp_netif_ip_info_t *ip_info)
{
    esp_err_t ret;

    ESP_RETURN_ON_FALSE(netif != NULL && ip_info != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "AP netif 参数为空");

    ret = esp_netif_dhcps_stop(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "停止透传 AP DHCP 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(netif, ip_info),
                        TAG, "设置透传 AP IP 失败");
    ESP_RETURN_ON_ERROR(z1mini_bridge_configure_dhcp_pool(netif),
                        TAG, "设置透传 AP DHCP 地址池失败");

    ret = esp_netif_dhcps_start(netif);
    if (ret != ESP_OK && ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGE(TAG, "启动透传 AP DHCP 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "透传 AP 网络已配置 | IP=" IPSTR " | GW=" IPSTR " | MASK=" IPSTR,
             IP2STR(&ip_info->ip), IP2STR(&ip_info->gw), IP2STR(&ip_info->netmask));
    return ESP_OK;
}

static esp_err_t z1mini_bridge_configure_dhcp_pool(esp_netif_t *netif)
{
    dhcps_lease_t lease = {0};

    ESP_RETURN_ON_FALSE(netif != NULL, ESP_ERR_INVALID_ARG, TAG, "AP netif 为空");

    lease.enable = true;
    lease.start_ip.addr = esp_ip4addr_aton(Z1MINI_BRIDGE_DHCP_START_IP);
    lease.end_ip.addr = esp_ip4addr_aton(Z1MINI_BRIDGE_DHCP_END_IP);

    ESP_RETURN_ON_ERROR(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET,
                                               ESP_NETIF_REQUESTED_IP_ADDRESS,
                                               &lease, sizeof(lease)),
                        TAG, "设置透传 DHCP 地址池失败");

    ESP_LOGI(TAG, "透传 DHCP 地址池已设置 | %s - %s",
             Z1MINI_BRIDGE_DHCP_START_IP, Z1MINI_BRIDGE_DHCP_END_IP);
    return ESP_OK;
}

static void z1mini_bridge_log_ready(const device_web_config_t *config,
                                    const char *ap_ip,
                                    const char *device_ip)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Z-1mini 网口透传已启动");
    ESP_LOGI(TAG, "  热点名称:    %s", config ? config->wifi_ap_ssid : "--");
    ESP_LOGI(TAG, "  热点密码:    %s", config ? config->wifi_ap_password : "--");
    ESP_LOGI(TAG, "  WT99 网页:   http://%s/", ap_ip);
    ESP_LOGI(TAG, "  WT99 视频:   rtsp://%s:%d/stream", ap_ip, RTSP_PORT);
    ESP_LOGI(TAG, "  WT99 串口0:  %s:%d", ap_ip, TCP_UART0_PORT);
    ESP_LOGI(TAG, "  吊舱地址:    %s", device_ip);
    ESP_LOGI(TAG, "  吊舱视频:    rtsp://%s", device_ip);
    ESP_LOGI(TAG, "======================================");
}

static bool z1mini_bridge_is_valid_eth_frame(size_t len)
{
    return len >= Z1MINI_BRIDGE_ETH_HEADER_LEN &&
           len <= Z1MINI_BRIDGE_ETH_MAX_FRAME;
}

static esp_err_t z1mini_bridge_wifi_rx(void *buffer, uint16_t len, void *eb)
{
    void *eth_buffer;
    size_t eth_len;

    if (buffer != NULL && z1mini_bridge_is_valid_eth_frame(len) &&
        s_eth_handle != NULL && s_eth_link_up) {
        eth_len = (len < Z1MINI_BRIDGE_ETH_MIN_TX_LEN) ? Z1MINI_BRIDGE_ETH_MIN_TX_LEN : len;
        eth_buffer = calloc(1, eth_len);
        if (eth_buffer != NULL) {
            memcpy(eth_buffer, buffer, len);
            if (esp_eth_transmit(s_eth_handle, eth_buffer, eth_len) == ESP_OK) {
                s_wifi_to_eth_count++;
            }
            free(eth_buffer);
        }
    }

    return esp_netif_receive(s_wifi_netif, buffer, len, eb);
}

static esp_err_t z1mini_bridge_register_wifi_rx_cb(void)
{
    esp_err_t ret;

    ret = esp_wifi_internal_reg_rxcb(WIFI_IF_AP, z1mini_bridge_wifi_rx);
    if (ret != ESP_OK) {
        s_wifi_rx_registered = false;
        ESP_LOGE(TAG, "注册透传 Wi-Fi 收包回调失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    if (!s_wifi_rx_registered) {
        ESP_LOGI(TAG, "透传 Wi-Fi 收包回调已接管");
    }
    s_wifi_rx_registered = true;
    return ESP_OK;
}

static esp_err_t z1mini_bridge_eth_rx(esp_eth_handle_t eth_handle, uint8_t *buffer,
                                      uint32_t len, void *priv)
{
    void *wifi_buffer;
    esp_err_t ret = ESP_OK;

    (void)eth_handle;
    (void)priv;

    if (buffer != NULL && z1mini_bridge_is_valid_eth_frame(len)) {
        wifi_buffer = malloc(len);
        if (wifi_buffer != NULL) {
            memcpy(wifi_buffer, buffer, len);
            if (esp_wifi_internal_tx(WIFI_IF_AP, wifi_buffer, (uint16_t)len) == ESP_OK) {
                s_eth_to_wifi_count++;
            }
            free(wifi_buffer);
        } else {
            ret = ESP_ERR_NO_MEM;
        }
    }

    free(buffer);
    return ret;
}

static void z1mini_bridge_wifi_event_handler(void *arg, esp_event_base_t base,
                                             int32_t id, void *data)
{
    (void)arg;

    if (base != WIFI_EVENT) {
        return;
    }

    if (id == WIFI_EVENT_AP_START) {
        s_ap_client_count = 0;
        ESP_LOGI(TAG, "透传 Wi-Fi AP 已启动");
        if (z1mini_bridge_register_wifi_rx_cb() == ESP_OK && s_bridge_event_group) {
            xEventGroupSetBits(s_bridge_event_group, Z1MINI_BRIDGE_READY_BIT);
        }
    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)data;

        if (s_ap_client_count < WIFI_AP_MAX_CONNECTIONS) {
            s_ap_client_count++;
        }
        ESP_LOGI(TAG,
                 "客户端已连接透传 AP | MAC=" MACSTR " | AID=%d | 当前客户端=%" PRIu32,
                 MAC2STR(event->mac), event->aid, s_ap_client_count);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)data;

        if (s_ap_client_count > 0U) {
            s_ap_client_count--;
        }
        ESP_LOGI(TAG,
                 "客户端已断开透传 AP | MAC=" MACSTR " | AID=%d | 当前客户端=%" PRIu32,
                 MAC2STR(event->mac), event->aid, s_ap_client_count);
    }
}

static void z1mini_bridge_eth_event_handler(void *arg, esp_event_base_t base,
                                            int32_t id, void *data)
{
    uint8_t mac_addr[ETH_ADDR_LEN] = {0};
    esp_eth_handle_t eth_handle;

    (void)arg;

    if (base != ETH_EVENT || !data) {
        return;
    }

    eth_handle = *(esp_eth_handle_t *)data;
    if (id == ETHERNET_EVENT_CONNECTED) {
        s_eth_link_up = true;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "吊舱网口链路已连接 | MAC=%02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        if (s_bridge_event_group) {
            xEventGroupSetBits(s_bridge_event_group, Z1MINI_BRIDGE_ETH_LINK_BIT);
        }
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        s_eth_link_up = false;
        ESP_LOGW(TAG, "吊舱网口链路已断开");
        if (s_bridge_event_group) {
            xEventGroupClearBits(s_bridge_event_group, Z1MINI_BRIDGE_ETH_LINK_BIT);
        }
    } else if (id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "透传以太网已启动");
    } else if (id == ETHERNET_EVENT_STOP) {
        s_eth_link_up = false;
        ESP_LOGW(TAG, "透传以太网已停止");
    }
}

static esp_err_t z1mini_bridge_create_eth_port(const uint8_t *common_mac)
{
    esp_eth_mac_t *mac;
    esp_eth_phy_t *phy;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_config_t eth_config;

    ESP_RETURN_ON_FALSE(common_mac != NULL, ESP_ERR_INVALID_ARG, TAG, "以太网 MAC 为空");

    /*
     * WT99P4C5-S1 板载 IP101GRI + RJ45。
     * PHY 地址使用自动探测，保持与 eth_connect 模块一致。
     */
    phy_config.phy_addr = ESP_ETH_PHY_ADDR_AUTO;
    phy_config.reset_gpio_num = -1;

    mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    ESP_RETURN_ON_FALSE(mac != NULL, ESP_FAIL, TAG, "创建透传 EMAC 失败");

    phy = esp_eth_phy_new_ip101(&phy_config);
    ESP_RETURN_ON_FALSE(phy != NULL, ESP_FAIL, TAG, "创建透传 IP101 PHY 失败");

    eth_config = (esp_eth_config_t)ETH_DEFAULT_CONFIG(mac, phy);
    eth_config.stack_input = z1mini_bridge_eth_rx;

    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config, &s_eth_handle),
                        TAG, "安装透传以太网驱动失败");
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, (void *)common_mac),
                        TAG, "设置透传以太网 MAC 失败");

    return ESP_OK;
}

static esp_err_t z1mini_bridge_create_wifi_port(const device_web_config_t *config,
                                                esp_netif_ip_info_t *ip_info)
{
    esp_err_t ret;
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {0};

    ESP_RETURN_ON_FALSE(config != NULL && ip_info != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "Wi-Fi 配置为空");

    ret = esp_wifi_init(&init_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化透传 Wi-Fi 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM),
                        TAG, "设置透传 Wi-Fi 存储模式失败");

    s_wifi_netif = esp_netif_create_default_wifi_ap();
    ESP_RETURN_ON_FALSE(s_wifi_netif != NULL, ESP_ERR_NO_MEM,
                        TAG, "创建透传 Wi-Fi AP netif 失败");
    ESP_RETURN_ON_ERROR(z1mini_bridge_configure_ap_ip(s_wifi_netif, ip_info),
                        TAG, "配置透传 AP IP 失败");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   z1mini_bridge_wifi_event_handler, NULL),
                        TAG, "注册透传 Wi-Fi 事件失败");

    strncpy((char *)wifi_config.ap.ssid, config->wifi_ap_ssid,
            sizeof(wifi_config.ap.ssid) - 1U);
    strncpy((char *)wifi_config.ap.password, config->wifi_ap_password,
            sizeof(wifi_config.ap.password) - 1U);
    wifi_config.ap.ssid_len = strlen(config->wifi_ap_ssid);
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;

    ESP_LOGI(TAG, "准备设置透传 AP 配置 | SSID长度=%u | 密码长度=%u | 最大客户端=%d",
             (unsigned)strlen(config->wifi_ap_ssid),
             (unsigned)strlen(config->wifi_ap_password),
             WIFI_AP_MAX_CONNECTIONS);

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP),
                        TAG, "设置透传 Wi-Fi AP 模式失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config),
                        TAG, "设置透传 Wi-Fi AP 配置失败");

    return ESP_OK;
}

static esp_err_t z1mini_bridge_start_ports(void)
{
    bool promiscuous = true;
    esp_err_t ret;

    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                   z1mini_bridge_eth_event_handler, NULL),
                        TAG, "注册透传以太网事件失败");
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promiscuous),
                        TAG, "设置透传以太网混杂模式失败");
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth_handle), TAG, "启动透传以太网失败");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "启动透传 Wi-Fi AP 失败");

    ret = esp_wifi_set_max_tx_power(84);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置透传 Wi-Fi 发射功率失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    ret = esp_netif_set_default_netif(s_wifi_netif);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置透传 AP 为默认 netif 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "已启用 AP 与以太网原始帧转发");
    return ESP_OK;
}

esp_err_t z1mini_bridge_init(const char *bridge_ip, const char *bridge_gw,
                             const char *bridge_mask, const char *device_ip)
{
    device_web_config_t config = {0};
    uint8_t common_mac[ETH_ADDR_LEN] = {0};

    ESP_RETURN_ON_FALSE(bridge_ip != NULL && bridge_gw != NULL &&
                        bridge_mask != NULL && device_ip != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "透传启动参数为空");

    if (s_bridge_running) {
        ESP_LOGW(TAG, "Z-1mini 网口透传已启动，忽略重复初始化");
        return ESP_OK;
    }

    if (!s_bridge_event_group) {
        s_bridge_event_group = xEventGroupCreate();
        ESP_RETURN_ON_FALSE(s_bridge_event_group != NULL, ESP_ERR_NO_MEM,
                            TAG, "创建透传事件组失败");
    }
    xEventGroupClearBits(s_bridge_event_group,
                         Z1MINI_BRIDGE_READY_BIT | Z1MINI_BRIDGE_ETH_LINK_BIT);

    ESP_RETURN_ON_ERROR(z1mini_bridge_fill_ip_info(&s_ap_ip_info,
                                                   bridge_ip, bridge_gw, bridge_mask),
                        TAG, "解析 AP IP 失败");
    ESP_RETURN_ON_ERROR(esp_read_mac(common_mac, ESP_MAC_ETH),
                        TAG, "读取以太网 MAC 失败");

    device_web_config_get(&config);
    ESP_LOGI(TAG, "启动 Z-1mini 网口透传 | AP=%s | DHCP=%s-%s | 吊舱=%s",
             bridge_ip, Z1MINI_BRIDGE_DHCP_START_IP,
             Z1MINI_BRIDGE_DHCP_END_IP, device_ip);

    ESP_RETURN_ON_ERROR(z1mini_bridge_create_eth_port(common_mac),
                        TAG, "创建透传以太网端口失败");
    ESP_RETURN_ON_ERROR(z1mini_bridge_create_wifi_port(&config, &s_ap_ip_info),
                        TAG, "创建透传 Wi-Fi 端口失败");
    ESP_RETURN_ON_ERROR(z1mini_bridge_start_ports(), TAG, "启动透传端口失败");

    s_bridge_running = true;
    z1mini_bridge_log_ready(&config, bridge_ip, device_ip);
    return ESP_OK;
}

esp_err_t z1mini_bridge_wait_for_ready(int timeout_ms)
{
    EventBits_t bits;
    TickType_t ticks;

    if (!s_bridge_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    bits = xEventGroupWaitBits(s_bridge_event_group, Z1MINI_BRIDGE_READY_BIT,
                               pdFALSE, pdTRUE, ticks);
    return (bits & Z1MINI_BRIDGE_READY_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t z1mini_bridge_wait_for_eth_link(int timeout_ms)
{
    EventBits_t bits;
    TickType_t ticks;

    if (!s_bridge_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    bits = xEventGroupWaitBits(s_bridge_event_group, Z1MINI_BRIDGE_ETH_LINK_BIT,
                               pdFALSE, pdTRUE, ticks);
    return (bits & Z1MINI_BRIDGE_ETH_LINK_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool z1mini_bridge_is_running(void)
{
    return s_bridge_running;
}

bool z1mini_bridge_is_eth_link_up(void)
{
    return s_eth_link_up;
}

uint32_t z1mini_bridge_get_ap_client_count(void)
{
    return s_ap_client_count;
}

esp_netif_t *z1mini_bridge_get_netif(void)
{
    return s_wifi_netif;
}
