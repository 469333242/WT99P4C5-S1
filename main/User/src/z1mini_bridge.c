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
#include "freertos/queue.h"
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
#define Z1MINI_BRIDGE_STATS_ENABLE    0 //调试信息
#define Z1MINI_BRIDGE_STATS_INTERVAL_MS 2000U
#define Z1MINI_BRIDGE_STATS_TASK_STACK  3072U
#define Z1MINI_BRIDGE_STATS_TASK_PRIO   (tskIDLE_PRIORITY + 1)
#define Z1MINI_BRIDGE_ETH_TO_WIFI_QUEUE_LEN 16U
#define Z1MINI_BRIDGE_ETH_TO_WIFI_TASK_STACK 4096U
#define Z1MINI_BRIDGE_ETH_TO_WIFI_TASK_PRIO  (tskIDLE_PRIORITY + 10)

#define Z1MINI_BRIDGE_ETH_TYPE_IPV4 0x0800U
#define Z1MINI_BRIDGE_ETH_TYPE_VLAN 0x8100U
#define Z1MINI_BRIDGE_IP_PROTO_TCP  6U
#define Z1MINI_BRIDGE_IP_PROTO_UDP  17U
#define Z1MINI_BRIDGE_UDP_BIG_LEN   1000U
#define Z1MINI_BRIDGE_RTP_V2_MASK   0xC0U
#define Z1MINI_BRIDGE_RTP_V2_VALUE  0x80U
#define Z1MINI_BRIDGE_RTCP_TYPE_MIN 192U
#define Z1MINI_BRIDGE_RTCP_TYPE_MAX 223U

static esp_netif_t *s_wifi_netif;
static esp_eth_handle_t s_eth_handle;
static EventGroupHandle_t s_bridge_event_group;
static esp_netif_ip_info_t s_ap_ip_info;
static volatile uint32_t s_ap_client_count;
#if Z1MINI_BRIDGE_STATS_ENABLE
static volatile uint32_t s_wifi_to_eth_count;
static volatile uint32_t s_eth_to_wifi_count;
static volatile uint32_t s_wifi_to_eth_total_count;
static volatile uint32_t s_wifi_to_eth_fail_count;
static volatile uint32_t s_wifi_to_eth_no_mem_count;
static volatile uint32_t s_wifi_to_eth_invalid_count;
static volatile esp_err_t s_wifi_to_eth_last_err;
static volatile uint32_t s_eth_to_wifi_total_count;
static volatile uint32_t s_eth_to_wifi_ip_count;
static volatile uint32_t s_eth_to_wifi_tcp_count;
static volatile uint32_t s_eth_to_wifi_udp_count;
static volatile uint32_t s_eth_to_wifi_udp_big_count;
static volatile uint32_t s_eth_to_wifi_rtp_like_count;
static volatile uint32_t s_eth_to_wifi_rtp_gap_events;
static volatile uint32_t s_eth_to_wifi_rtp_gap_packets;
static volatile uint32_t s_eth_to_wifi_rtp_out_of_order_count;
static volatile uint32_t s_eth_to_wifi_rtp_seq_reset_count;
static volatile uint32_t s_eth_to_wifi_rtp_last_seq;
static volatile uint32_t s_eth_to_wifi_rtp_ssrc;
static volatile uint32_t s_eth_to_wifi_queued_count;
static volatile uint32_t s_eth_to_wifi_queue_full_count;
static volatile uint32_t s_eth_to_wifi_queue_max_depth;
static volatile uint32_t s_eth_to_wifi_fail_count;
static volatile uint32_t s_eth_to_wifi_no_mem_count;
static volatile uint32_t s_eth_to_wifi_invalid_count;
static volatile esp_err_t s_eth_to_wifi_last_err;
static volatile uint32_t s_eth_to_wifi_max_len;
#endif
static QueueHandle_t s_eth_to_wifi_queue;
static TaskHandle_t s_eth_to_wifi_task_handle;
#if Z1MINI_BRIDGE_STATS_ENABLE
static TaskHandle_t s_stats_task_handle;
static volatile bool s_eth_to_wifi_rtp_seq_valid;
#endif
static bool s_wifi_rx_registered;
static bool s_bridge_running;
static bool s_eth_link_up;

static esp_err_t z1mini_bridge_configure_dhcp_pool(esp_netif_t *netif);

#if Z1MINI_BRIDGE_STATS_ENABLE
typedef struct {
    bool ipv4;
    bool tcp;
    bool udp;
    bool udp_big;
    bool rtp_like;
    uint16_t rtp_seq;
    uint32_t rtp_ssrc;
} z1mini_bridge_frame_info_t;
#endif

typedef struct {
    uint8_t *buffer;
    uint16_t len;
} z1mini_bridge_eth_frame_t;

#if Z1MINI_BRIDGE_STATS_ENABLE
static uint16_t z1mini_bridge_read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t z1mini_bridge_read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static bool z1mini_bridge_udp_payload_is_rtp(const uint8_t *payload, uint16_t payload_len)
{
    uint8_t payload_type;

    if (!payload || payload_len < 12U) {
        return false;
    }

    if ((payload[0] & Z1MINI_BRIDGE_RTP_V2_MASK) != Z1MINI_BRIDGE_RTP_V2_VALUE) {
        return false;
    }

    payload_type = payload[1];
    if (payload_type >= Z1MINI_BRIDGE_RTCP_TYPE_MIN &&
        payload_type <= Z1MINI_BRIDGE_RTCP_TYPE_MAX) {
        return false;
    }

    return true;
}

static void z1mini_bridge_parse_frame_info(const uint8_t *frame, size_t len,
                                           z1mini_bridge_frame_info_t *info)
{
    size_t ip_offset = Z1MINI_BRIDGE_ETH_HEADER_LEN;
    uint16_t eth_type;
    uint8_t ihl;
    uint16_t total_len;
    const uint8_t *ip;

    if (!info) {
        return;
    }

    memset(info, 0, sizeof(*info));
    if (!frame || len < (Z1MINI_BRIDGE_ETH_HEADER_LEN + 20U)) {
        return;
    }

    eth_type = z1mini_bridge_read_be16(frame + 12U);
    if (eth_type == Z1MINI_BRIDGE_ETH_TYPE_VLAN) {
        if (len < (Z1MINI_BRIDGE_ETH_HEADER_LEN + 4U + 20U)) {
            return;
        }
        eth_type = z1mini_bridge_read_be16(frame + 16U);
        ip_offset += 4U;
    }

    if (eth_type != Z1MINI_BRIDGE_ETH_TYPE_IPV4) {
        return;
    }

    ip = frame + ip_offset;
    if ((ip[0] >> 4) != 4U) {
        return;
    }

    ihl = (uint8_t)((ip[0] & 0x0FU) * 4U);
    if (ihl < 20U || len < (ip_offset + ihl)) {
        return;
    }

    total_len = z1mini_bridge_read_be16(ip + 2U);
    if (total_len < ihl || len < (ip_offset + total_len)) {
        return;
    }

    info->ipv4 = true;
    if (ip[9] == Z1MINI_BRIDGE_IP_PROTO_TCP) {
        info->tcp = true;
    } else if (ip[9] == Z1MINI_BRIDGE_IP_PROTO_UDP) {
        const uint8_t *udp = ip + ihl;
        const uint8_t *udp_payload;
        uint16_t udp_len;
        uint16_t udp_payload_len;

        info->udp = true;
        if (total_len < (ihl + 8U)) {
            return;
        }

        udp_len = z1mini_bridge_read_be16(udp + 4U);
        if (udp_len < 8U || udp_len > (total_len - ihl)) {
            return;
        }

        if (udp_len >= Z1MINI_BRIDGE_UDP_BIG_LEN) {
            info->udp_big = true;
        }

        udp_payload = udp + 8U;
        udp_payload_len = (uint16_t)(udp_len - 8U);
        if (z1mini_bridge_udp_payload_is_rtp(udp_payload, udp_payload_len)) {
            info->rtp_like = true;
            info->rtp_seq = z1mini_bridge_read_be16(udp_payload + 2U);
            info->rtp_ssrc = z1mini_bridge_read_be32(udp_payload + 8U);
        }
    }
}

static void z1mini_bridge_update_rtp_sequence_stats(const z1mini_bridge_frame_info_t *info)
{
    uint16_t seq;
    uint16_t expected_seq;
    uint16_t gap;

    if (!info || !info->rtp_like) {
        return;
    }

    seq = info->rtp_seq;
    if (!s_eth_to_wifi_rtp_seq_valid ||
        s_eth_to_wifi_rtp_ssrc != info->rtp_ssrc) {
        s_eth_to_wifi_rtp_ssrc = info->rtp_ssrc;
        s_eth_to_wifi_rtp_last_seq = seq;
        s_eth_to_wifi_rtp_seq_valid = true;
        s_eth_to_wifi_rtp_seq_reset_count++;
        return;
    }

    expected_seq = (uint16_t)((uint16_t)s_eth_to_wifi_rtp_last_seq + 1U);
    if (seq == expected_seq) {
        s_eth_to_wifi_rtp_last_seq = seq;
        return;
    }

    gap = (uint16_t)(seq - expected_seq);
    if (gap < 0x8000U) {
        s_eth_to_wifi_rtp_gap_events++;
        s_eth_to_wifi_rtp_gap_packets += (uint32_t)gap;
        s_eth_to_wifi_rtp_last_seq = seq;
    } else {
        s_eth_to_wifi_rtp_out_of_order_count++;
    }
}

static void z1mini_bridge_update_eth_to_wifi_frame_stats(const uint8_t *frame, size_t len)
{
    z1mini_bridge_frame_info_t info;

    s_eth_to_wifi_total_count++;
    if (len > s_eth_to_wifi_max_len) {
        s_eth_to_wifi_max_len = (uint32_t)len;
    }

    z1mini_bridge_parse_frame_info(frame, len, &info);
    if (info.ipv4) {
        s_eth_to_wifi_ip_count++;
    }
    if (info.tcp) {
        s_eth_to_wifi_tcp_count++;
    }
    if (info.udp) {
        s_eth_to_wifi_udp_count++;
    }
    if (info.udp_big) {
        s_eth_to_wifi_udp_big_count++;
    }
    if (info.rtp_like) {
        s_eth_to_wifi_rtp_like_count++;
        z1mini_bridge_update_rtp_sequence_stats(&info);
    }
}
#endif

static void z1mini_bridge_eth_to_wifi_task(void *arg)
{
    z1mini_bridge_eth_frame_t frame;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_eth_to_wifi_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (frame.buffer && frame.len > 0U) {
            esp_err_t tx_ret = esp_wifi_internal_tx(WIFI_IF_AP, frame.buffer, frame.len);
#if Z1MINI_BRIDGE_STATS_ENABLE
            if (tx_ret == ESP_OK) {
                s_eth_to_wifi_count++;
                s_eth_to_wifi_last_err = ESP_OK;
            } else {
                s_eth_to_wifi_fail_count++;
                s_eth_to_wifi_last_err = tx_ret;
            }
#else
            (void)tx_ret;
#endif
        }

        free(frame.buffer);
    }
}

#if Z1MINI_BRIDGE_STATS_ENABLE
static void z1mini_bridge_stats_task(void *arg)
{
    uint32_t last_w2e_ok = 0;
    uint32_t last_w2e_total = 0;
    uint32_t last_w2e_fail = 0;
    uint32_t last_w2e_no_mem = 0;
    uint32_t last_w2e_invalid = 0;
    uint32_t last_e2w_ok = 0;
    uint32_t last_e2w_total = 0;
    uint32_t last_e2w_ip = 0;
    uint32_t last_e2w_tcp = 0;
    uint32_t last_e2w_udp = 0;
    uint32_t last_e2w_udp_big = 0;
    uint32_t last_e2w_rtp_like = 0;
    uint32_t last_e2w_rtp_gap_events = 0;
    uint32_t last_e2w_rtp_gap_packets = 0;
    uint32_t last_e2w_rtp_out_of_order = 0;
    uint32_t last_e2w_rtp_seq_reset = 0;
    uint32_t last_e2w_queued = 0;
    uint32_t last_e2w_queue_full = 0;
    uint32_t last_e2w_fail = 0;
    uint32_t last_e2w_no_mem = 0;
    uint32_t last_e2w_invalid = 0;

    (void)arg;

    while (1) {
        uint32_t w2e_ok;
        uint32_t w2e_total;
        uint32_t w2e_fail;
        uint32_t w2e_no_mem;
        uint32_t w2e_invalid;
        uint32_t e2w_ok;
        uint32_t e2w_total;
        uint32_t e2w_ip;
        uint32_t e2w_tcp;
        uint32_t e2w_udp;
        uint32_t e2w_udp_big;
        uint32_t e2w_rtp_like;
        uint32_t e2w_rtp_gap_events;
        uint32_t e2w_rtp_gap_packets;
        uint32_t e2w_rtp_out_of_order;
        uint32_t e2w_rtp_seq_reset;
        uint32_t e2w_queued;
        uint32_t e2w_queue_full;
        uint32_t e2w_fail;
        uint32_t e2w_no_mem;
        uint32_t e2w_invalid;
        UBaseType_t e2w_queue_depth = 0;

        vTaskDelay(pdMS_TO_TICKS(Z1MINI_BRIDGE_STATS_INTERVAL_MS));

        w2e_ok = s_wifi_to_eth_count;
        w2e_total = s_wifi_to_eth_total_count;
        w2e_fail = s_wifi_to_eth_fail_count;
        w2e_no_mem = s_wifi_to_eth_no_mem_count;
        w2e_invalid = s_wifi_to_eth_invalid_count;
        e2w_ok = s_eth_to_wifi_count;
        e2w_total = s_eth_to_wifi_total_count;
        e2w_ip = s_eth_to_wifi_ip_count;
        e2w_tcp = s_eth_to_wifi_tcp_count;
        e2w_udp = s_eth_to_wifi_udp_count;
        e2w_udp_big = s_eth_to_wifi_udp_big_count;
        e2w_rtp_like = s_eth_to_wifi_rtp_like_count;
        e2w_rtp_gap_events = s_eth_to_wifi_rtp_gap_events;
        e2w_rtp_gap_packets = s_eth_to_wifi_rtp_gap_packets;
        e2w_rtp_out_of_order = s_eth_to_wifi_rtp_out_of_order_count;
        e2w_rtp_seq_reset = s_eth_to_wifi_rtp_seq_reset_count;
        e2w_queued = s_eth_to_wifi_queued_count;
        e2w_queue_full = s_eth_to_wifi_queue_full_count;
        e2w_fail = s_eth_to_wifi_fail_count;
        e2w_no_mem = s_eth_to_wifi_no_mem_count;
        e2w_invalid = s_eth_to_wifi_invalid_count;
        if (s_eth_to_wifi_queue) {
            e2w_queue_depth = uxQueueMessagesWaiting(s_eth_to_wifi_queue);
        }

        ESP_LOGI(TAG,
                 "透传统计 | WiFi->ETH ok=%" PRIu32 "/%" PRIu32 " fail=%" PRIu32 " no_mem=%" PRIu32 " invalid=%" PRIu32 " last_err=0x%x | "
                 "ETH->WiFi ok=%" PRIu32 "/%" PRIu32 " ip=%" PRIu32 " tcp=%" PRIu32 " udp=%" PRIu32 " udp_big=%" PRIu32 " rtp_like=%" PRIu32
                 " rtp_gap=%" PRIu32 "/%" PRIu32 " rtp_ooo=%" PRIu32 " rtp_reset=%" PRIu32 " rtp_seq=%" PRIu32 " rtp_ssrc=%08" PRIx32
                 " queued=%" PRIu32 " q_full=%" PRIu32 " q_depth=%u q_max=%" PRIu32
                 " fail=%" PRIu32 " no_mem=%" PRIu32 " invalid=%" PRIu32 " last_err=0x%x max_len=%" PRIu32,
                 w2e_ok - last_w2e_ok,
                 w2e_total - last_w2e_total,
                 w2e_fail - last_w2e_fail,
                 w2e_no_mem - last_w2e_no_mem,
                 w2e_invalid - last_w2e_invalid,
                 (unsigned)s_wifi_to_eth_last_err,
                 e2w_ok - last_e2w_ok,
                 e2w_total - last_e2w_total,
                 e2w_ip - last_e2w_ip,
                 e2w_tcp - last_e2w_tcp,
                 e2w_udp - last_e2w_udp,
                 e2w_udp_big - last_e2w_udp_big,
                 e2w_rtp_like - last_e2w_rtp_like,
                 e2w_rtp_gap_events - last_e2w_rtp_gap_events,
                 e2w_rtp_gap_packets - last_e2w_rtp_gap_packets,
                 e2w_rtp_out_of_order - last_e2w_rtp_out_of_order,
                 e2w_rtp_seq_reset - last_e2w_rtp_seq_reset,
                 s_eth_to_wifi_rtp_last_seq,
                 s_eth_to_wifi_rtp_ssrc,
                 e2w_queued - last_e2w_queued,
                 e2w_queue_full - last_e2w_queue_full,
                 (unsigned)e2w_queue_depth,
                 s_eth_to_wifi_queue_max_depth,
                 e2w_fail - last_e2w_fail,
                 e2w_no_mem - last_e2w_no_mem,
                 e2w_invalid - last_e2w_invalid,
                 (unsigned)s_eth_to_wifi_last_err,
                 s_eth_to_wifi_max_len);

        last_w2e_ok = w2e_ok;
        last_w2e_total = w2e_total;
        last_w2e_fail = w2e_fail;
        last_w2e_no_mem = w2e_no_mem;
        last_w2e_invalid = w2e_invalid;
        last_e2w_ok = e2w_ok;
        last_e2w_total = e2w_total;
        last_e2w_ip = e2w_ip;
        last_e2w_tcp = e2w_tcp;
        last_e2w_udp = e2w_udp;
        last_e2w_udp_big = e2w_udp_big;
        last_e2w_rtp_like = e2w_rtp_like;
        last_e2w_rtp_gap_events = e2w_rtp_gap_events;
        last_e2w_rtp_gap_packets = e2w_rtp_gap_packets;
        last_e2w_rtp_out_of_order = e2w_rtp_out_of_order;
        last_e2w_rtp_seq_reset = e2w_rtp_seq_reset;
        last_e2w_queued = e2w_queued;
        last_e2w_queue_full = e2w_queue_full;
        last_e2w_fail = e2w_fail;
        last_e2w_no_mem = e2w_no_mem;
        last_e2w_invalid = e2w_invalid;
    }
}
#endif

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

    if (buffer == NULL || !z1mini_bridge_is_valid_eth_frame(len)) {
#if Z1MINI_BRIDGE_STATS_ENABLE
        s_wifi_to_eth_invalid_count++;
#endif
        return esp_netif_receive(s_wifi_netif, buffer, len, eb);
    }

#if Z1MINI_BRIDGE_STATS_ENABLE
    s_wifi_to_eth_total_count++;
#endif
    if (buffer != NULL && z1mini_bridge_is_valid_eth_frame(len) &&
        s_eth_handle != NULL && s_eth_link_up) {
        eth_len = (len < Z1MINI_BRIDGE_ETH_MIN_TX_LEN) ? Z1MINI_BRIDGE_ETH_MIN_TX_LEN : len;
        eth_buffer = calloc(1, eth_len);
        if (eth_buffer != NULL) {
            memcpy(eth_buffer, buffer, len);
            esp_err_t tx_ret = esp_eth_transmit(s_eth_handle, eth_buffer, eth_len);
#if Z1MINI_BRIDGE_STATS_ENABLE
            if (tx_ret == ESP_OK) {
                s_wifi_to_eth_count++;
                s_wifi_to_eth_last_err = ESP_OK;
            } else {
                s_wifi_to_eth_fail_count++;
                s_wifi_to_eth_last_err = tx_ret;
            }
#else
            (void)tx_ret;
#endif
            free(eth_buffer);
        } else {
#if Z1MINI_BRIDGE_STATS_ENABLE
            s_wifi_to_eth_no_mem_count++;
            s_wifi_to_eth_last_err = ESP_ERR_NO_MEM;
#endif
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
    esp_err_t ret = ESP_OK;
    z1mini_bridge_eth_frame_t frame = {0};

    (void)eth_handle;
    (void)priv;

    if (buffer != NULL && z1mini_bridge_is_valid_eth_frame(len)) {
#if Z1MINI_BRIDGE_STATS_ENABLE
        z1mini_bridge_update_eth_to_wifi_frame_stats((const uint8_t *)buffer, len);
#endif

        if (s_eth_to_wifi_queue) {
            frame.buffer = buffer;
            frame.len = (uint16_t)len;

            if (xQueueSend(s_eth_to_wifi_queue, &frame, 0) == pdTRUE) {
#if Z1MINI_BRIDGE_STATS_ENABLE
                UBaseType_t depth;

                s_eth_to_wifi_queued_count++;
                depth = uxQueueMessagesWaiting(s_eth_to_wifi_queue);
                if (depth > s_eth_to_wifi_queue_max_depth) {
                    s_eth_to_wifi_queue_max_depth = (uint32_t)depth;
                }
#endif
                return ESP_OK;
            }

#if Z1MINI_BRIDGE_STATS_ENABLE
            s_eth_to_wifi_queue_full_count++;
            s_eth_to_wifi_last_err = ESP_ERR_TIMEOUT;
#endif
            ret = ESP_ERR_TIMEOUT;
        } else {
#if Z1MINI_BRIDGE_STATS_ENABLE
            s_eth_to_wifi_fail_count++;
            s_eth_to_wifi_last_err = ESP_ERR_INVALID_STATE;
#endif
            ret = ESP_ERR_INVALID_STATE;
        }
    } else {
#if Z1MINI_BRIDGE_STATS_ENABLE
        s_eth_to_wifi_invalid_count++;
#endif
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
    BaseType_t task_ret;

    if (!s_eth_to_wifi_queue) {
        s_eth_to_wifi_queue = xQueueCreate(Z1MINI_BRIDGE_ETH_TO_WIFI_QUEUE_LEN,
                                           sizeof(z1mini_bridge_eth_frame_t));
        ESP_RETURN_ON_FALSE(s_eth_to_wifi_queue != NULL, ESP_ERR_NO_MEM,
                            TAG, "创建 ETH->WiFi 转发队列失败");
    }

    if (!s_eth_to_wifi_task_handle) {
        task_ret = xTaskCreate(z1mini_bridge_eth_to_wifi_task,
                               "z1_e2w_tx",
                               Z1MINI_BRIDGE_ETH_TO_WIFI_TASK_STACK,
                               NULL,
                               Z1MINI_BRIDGE_ETH_TO_WIFI_TASK_PRIO,
                               &s_eth_to_wifi_task_handle);
        ESP_RETURN_ON_FALSE(task_ret == pdPASS, ESP_ERR_NO_MEM,
                            TAG, "创建 ETH->WiFi 转发任务失败");
    }

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

#if Z1MINI_BRIDGE_STATS_ENABLE
    if (!s_stats_task_handle) {
        task_ret = xTaskCreate(z1mini_bridge_stats_task,
                               "z1_bridge_stat",
                               Z1MINI_BRIDGE_STATS_TASK_STACK,
                               NULL,
                               Z1MINI_BRIDGE_STATS_TASK_PRIO,
                               &s_stats_task_handle);
        if (task_ret != pdPASS) {
            s_stats_task_handle = NULL;
            ESP_LOGW(TAG, "创建透传统计任务失败");
        }
    }
#endif

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
