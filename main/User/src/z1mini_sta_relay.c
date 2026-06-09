/**
 * @file z1mini_sta_relay.c
 * @brief Z-1mini STA 入网模式 Proxy ARP + IP 帧转发
 *
 * 拓扑假设：
 *   PC/主路由器 192.168.144.x <-> Wi-Fi STA(WT99) <-> ETH <-> Z-1mini 192.168.144.108
 *
 * 普通 Wi-Fi STA 不能做真正二层桥接。这里使用 Proxy ARP 让两侧都把 WT99
 * 当成下一跳 MAC，再在 Wi-Fi STA 与 ETH 之间重写以太网头并转发 IPv4 帧。
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_private/wifi.h"
#include "esp_wifi.h"

#include "lwip/inet.h"

#include "z1mini_sta_relay.h"

static const char *TAG = "z1mini_sta";

#define Z1MINI_STA_RELAY_ETH_LINK_BIT        BIT0
#define Z1MINI_STA_RELAY_ETH_HEADER_LEN      14U
#define Z1MINI_STA_RELAY_ETH_MIN_TX_LEN      60U
#define Z1MINI_STA_RELAY_ETH_MAX_FRAME       1536U
#define Z1MINI_STA_RELAY_PEER_TABLE_SIZE     8U
#define Z1MINI_STA_RELAY_ARP_INTERVAL_MS     1000U
#define Z1MINI_STA_RELAY_PEER_ARP_INTERVAL_MS 2000U
#define Z1MINI_STA_RELAY_LOG_INTERVAL        200U
#define Z1MINI_STA_RELAY_STATS_ENABLE        0 // 调试信息
#define Z1MINI_STA_RELAY_STATS_INTERVAL_MS   2000U
#define Z1MINI_STA_RELAY_STATS_TASK_STACK    3072U
#define Z1MINI_STA_RELAY_STATS_TASK_PRIO     (tskIDLE_PRIORITY + 1)
#define Z1MINI_STA_RELAY_WIFI_TX_QUEUE_LEN   64U
#define Z1MINI_STA_RELAY_WIFI_TX_TASK_STACK  4096U
#define Z1MINI_STA_RELAY_WIFI_TX_TASK_PRIO   (tskIDLE_PRIORITY + 10)

#define Z1MINI_STA_RELAY_ETH_TYPE_IPV4       0x0800U
#define Z1MINI_STA_RELAY_ETH_TYPE_ARP        0x0806U
#define Z1MINI_STA_RELAY_ARP_HTYPE_ETH       0x0001U
#define Z1MINI_STA_RELAY_ARP_PTYPE_IPV4      0x0800U
#define Z1MINI_STA_RELAY_ARP_REQUEST         0x0001U
#define Z1MINI_STA_RELAY_ARP_REPLY           0x0002U
#define Z1MINI_STA_RELAY_IPV4_VERSION        4U
#if Z1MINI_STA_RELAY_STATS_ENABLE
#define Z1MINI_STA_RELAY_IP_PROTO_TCP        6U
#define Z1MINI_STA_RELAY_IP_PROTO_UDP        17U
#define Z1MINI_STA_RELAY_UDP_BIG_LEN         1000U
#define Z1MINI_STA_RELAY_RTP_V2_MASK         0xc0U
#define Z1MINI_STA_RELAY_RTP_V2_VALUE        0x80U
#define Z1MINI_STA_RELAY_RTCP_TYPE_MIN       192U
#define Z1MINI_STA_RELAY_RTCP_TYPE_MAX       223U
#endif

typedef struct __attribute__((packed)) {
    uint8_t dst[ETH_ADDR_LEN];
    uint8_t src[ETH_ADDR_LEN];
    uint16_t eth_type;
} z1mini_sta_relay_eth_hdr_t;

typedef struct __attribute__((packed)) {
    uint16_t htype;
    uint16_t ptype;
    uint8_t hlen;
    uint8_t plen;
    uint16_t oper;
    uint8_t sha[ETH_ADDR_LEN];
    uint32_t spa;
    uint8_t tha[ETH_ADDR_LEN];
    uint32_t tpa;
} z1mini_sta_relay_arp_t;

typedef struct {
    bool valid;
    uint32_t ip;
    uint8_t mac[ETH_ADDR_LEN];
    TickType_t updated_tick;
    TickType_t arp_request_tick;
} z1mini_sta_relay_peer_t;

typedef struct {
    uint8_t *buffer;
    uint16_t len;
} z1mini_sta_relay_frame_t;

#if Z1MINI_STA_RELAY_STATS_ENABLE
typedef struct {
    bool ipv4;
    bool tcp;
    bool udp;
    bool udp_big;
    bool rtp_like;
    uint16_t rtp_seq;
    uint32_t rtp_ssrc;
} z1mini_sta_relay_frame_info_t;
#endif

static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_sta_netif;
static EventGroupHandle_t s_event_group;
static QueueHandle_t s_wifi_tx_queue;
static TaskHandle_t s_wifi_tx_task;
#if Z1MINI_STA_RELAY_STATS_ENABLE
static TaskHandle_t s_stats_task;
#endif
static bool s_running;
static bool s_eth_link_up;
static bool s_wifi_rx_registered;

static uint8_t s_eth_mac[ETH_ADDR_LEN];
static uint8_t s_sta_mac[ETH_ADDR_LEN];
static uint8_t s_device_mac[ETH_ADDR_LEN];
static bool s_device_mac_valid;

static uint32_t s_bridge_ip;
static uint32_t s_bridge_mask;
static uint32_t s_device_ip;
static z1mini_sta_relay_peer_t s_peers[Z1MINI_STA_RELAY_PEER_TABLE_SIZE];
static TickType_t s_last_device_arp_tick;
static TickType_t s_last_device_announce_tick;

static uint32_t s_wifi_to_eth_count;

#if Z1MINI_STA_RELAY_STATS_ENABLE
static volatile uint32_t s_wifi_to_eth_total_count;
static volatile uint32_t s_wifi_to_eth_fail_count;
static volatile uint32_t s_wifi_to_eth_no_mem_count;
static volatile uint32_t s_wifi_to_eth_invalid_count;
static volatile esp_err_t s_wifi_to_eth_last_err;

static volatile uint32_t s_eth_to_wifi_count;
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
static volatile uint32_t s_eth_to_wifi_unicast_count;
static volatile uint32_t s_eth_to_wifi_broadcast_count;
static volatile uint32_t s_eth_to_wifi_broadcast_fallback_count;
static volatile uint32_t s_eth_to_wifi_fail_count;
static volatile uint32_t s_eth_to_wifi_no_mem_count;
static volatile uint32_t s_eth_to_wifi_invalid_count;
static volatile esp_err_t s_eth_to_wifi_last_err;
static volatile uint32_t s_eth_to_wifi_max_len;
static volatile bool s_eth_to_wifi_rtp_seq_valid;
#endif

static volatile uint32_t s_proxy_arp_wifi_count;
static volatile uint32_t s_proxy_arp_eth_count;

static esp_err_t z1mini_sta_relay_eth_rx(esp_eth_handle_t eth_handle,
                                         uint8_t *buffer,
                                         uint32_t len,
                                         void *priv);
static esp_err_t z1mini_sta_relay_wifi_rx(void *buffer, uint16_t len, void *eb);
static void z1mini_sta_relay_send_device_gratuitous_arp(void);

static uint16_t z1mini_sta_relay_read_be16(const void *ptr)
{
    const uint8_t *p = (const uint8_t *)ptr;
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

#if Z1MINI_STA_RELAY_STATS_ENABLE
static uint32_t z1mini_sta_relay_read_be32(const void *ptr)
{
    const uint8_t *p = (const uint8_t *)ptr;
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}
#endif

static void z1mini_sta_relay_write_be16(void *ptr, uint16_t value)
{
    uint8_t *p = (uint8_t *)ptr;
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static bool z1mini_sta_relay_mac_equal(const uint8_t *a, const uint8_t *b)
{
    return a != NULL && b != NULL && memcmp(a, b, ETH_ADDR_LEN) == 0;
}

static bool z1mini_sta_relay_mac_is_broadcast(const uint8_t *mac)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };

    return z1mini_sta_relay_mac_equal(mac, broadcast);
}

static bool z1mini_sta_relay_mac_is_multicast(const uint8_t *mac)
{
    return mac != NULL && (mac[0] & 0x01U) != 0U;
}

static bool z1mini_sta_relay_mac_is_usable_unicast(const uint8_t *mac)
{
    static const uint8_t zero[ETH_ADDR_LEN] = {0};

    return mac != NULL &&
           !z1mini_sta_relay_mac_equal(mac, zero) &&
           !z1mini_sta_relay_mac_is_multicast(mac);
}

static bool z1mini_sta_relay_is_valid_eth_frame(size_t len)
{
    return len >= Z1MINI_STA_RELAY_ETH_HEADER_LEN &&
           len <= Z1MINI_STA_RELAY_ETH_MAX_FRAME;
}

static bool z1mini_sta_relay_ip_in_bridge_subnet(uint32_t ip)
{
    if (ip == 0U || s_bridge_mask == 0U) {
        return false;
    }

    return (ip & s_bridge_mask) == (s_bridge_ip & s_bridge_mask);
}

static bool z1mini_sta_relay_is_ipv4_broadcast_or_multicast(uint32_t ip)
{
    uint32_t host_ip = ntohl(ip);
    uint32_t host_mask = ntohl(s_bridge_mask);
    uint32_t host_bridge_ip = ntohl(s_bridge_ip);

    if (ip == htonl(INADDR_BROADCAST)) {
        return true;
    }
    if ((host_ip & 0xf0000000UL) == 0xe0000000UL) {
        return true;
    }
    if (host_mask != 0U &&
        (host_ip & host_mask) == (host_bridge_ip & host_mask) &&
        (host_ip | host_mask) == 0xffffffffUL) {
        return true;
    }

    return false;
}

static void z1mini_sta_relay_format_ip(uint32_t ip, char *buffer, size_t buffer_size)
{
    uint8_t *p = (uint8_t *)&ip;

    if (buffer == NULL || buffer_size == 0U) {
        return;
    }

    snprintf(buffer, buffer_size, "%u.%u.%u.%u",
             (unsigned)p[0], (unsigned)p[1], (unsigned)p[2], (unsigned)p[3]);
}

static bool z1mini_sta_relay_parse_ipv4_addrs(const uint8_t *frame,
                                              size_t len,
                                              uint32_t *src_ip,
                                              uint32_t *dst_ip)
{
    const uint8_t *ip;
    uint8_t ihl;
    uint16_t total_len;

    if (frame == NULL || len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + 20U) ||
        src_ip == NULL || dst_ip == NULL) {
        return false;
    }

    ip = frame + Z1MINI_STA_RELAY_ETH_HEADER_LEN;
    if ((ip[0] >> 4) != Z1MINI_STA_RELAY_IPV4_VERSION) {
        return false;
    }

    ihl = (uint8_t)((ip[0] & 0x0fU) * 4U);
    if (ihl < 20U || len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + ihl)) {
        return false;
    }

    total_len = z1mini_sta_relay_read_be16(ip + 2U);
    if (total_len < ihl ||
        len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + total_len)) {
        return false;
    }

    memcpy(src_ip, ip + 12U, sizeof(*src_ip));
    memcpy(dst_ip, ip + 16U, sizeof(*dst_ip));
    return true;
}

#if Z1MINI_STA_RELAY_STATS_ENABLE
static bool z1mini_sta_relay_udp_payload_is_rtp(const uint8_t *payload,
                                                uint16_t payload_len)
{
    uint8_t payload_type;

    if (payload == NULL || payload_len < 12U) {
        return false;
    }
    if ((payload[0] & Z1MINI_STA_RELAY_RTP_V2_MASK) !=
        Z1MINI_STA_RELAY_RTP_V2_VALUE) {
        return false;
    }

    payload_type = payload[1];
    if (payload_type >= Z1MINI_STA_RELAY_RTCP_TYPE_MIN &&
        payload_type <= Z1MINI_STA_RELAY_RTCP_TYPE_MAX) {
        return false;
    }

    return true;
}

static void z1mini_sta_relay_parse_frame_info(const uint8_t *frame,
                                              size_t len,
                                              z1mini_sta_relay_frame_info_t *info)
{
    const uint8_t *ip;
    uint8_t ihl;
    uint16_t total_len;

    if (info == NULL) {
        return;
    }

    memset(info, 0, sizeof(*info));
    if (frame == NULL || len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + 20U)) {
        return;
    }

    ip = frame + Z1MINI_STA_RELAY_ETH_HEADER_LEN;
    if ((ip[0] >> 4) != Z1MINI_STA_RELAY_IPV4_VERSION) {
        return;
    }

    ihl = (uint8_t)((ip[0] & 0x0fU) * 4U);
    if (ihl < 20U || len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + ihl)) {
        return;
    }

    total_len = z1mini_sta_relay_read_be16(ip + 2U);
    if (total_len < ihl ||
        len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + total_len)) {
        return;
    }

    info->ipv4 = true;
    if (ip[9] == Z1MINI_STA_RELAY_IP_PROTO_TCP) {
        info->tcp = true;
    } else if (ip[9] == Z1MINI_STA_RELAY_IP_PROTO_UDP) {
        const uint8_t *udp = ip + ihl;
        const uint8_t *udp_payload;
        uint16_t udp_len;
        uint16_t udp_payload_len;

        info->udp = true;
        if (total_len < (ihl + 8U)) {
            return;
        }

        udp_len = z1mini_sta_relay_read_be16(udp + 4U);
        if (udp_len < 8U || udp_len > (total_len - ihl)) {
            return;
        }

        if (udp_len >= Z1MINI_STA_RELAY_UDP_BIG_LEN) {
            info->udp_big = true;
        }

        udp_payload = udp + 8U;
        udp_payload_len = (uint16_t)(udp_len - 8U);
        if (z1mini_sta_relay_udp_payload_is_rtp(udp_payload, udp_payload_len)) {
            info->rtp_like = true;
            info->rtp_seq = z1mini_sta_relay_read_be16(udp_payload + 2U);
            info->rtp_ssrc = z1mini_sta_relay_read_be32(udp_payload + 8U);
        }
    }
}

static void z1mini_sta_relay_update_rtp_sequence_stats(
    const z1mini_sta_relay_frame_info_t *info)
{
    uint16_t seq;
    uint16_t expected_seq;
    uint16_t gap;

    if (info == NULL || !info->rtp_like) {
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

static void z1mini_sta_relay_update_eth_to_wifi_frame_stats(const uint8_t *frame,
                                                            size_t len)
{
    z1mini_sta_relay_frame_info_t info;

    s_eth_to_wifi_total_count++;
    if (len > s_eth_to_wifi_max_len) {
        s_eth_to_wifi_max_len = (uint32_t)len;
    }

    z1mini_sta_relay_parse_frame_info(frame, len, &info);
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
        z1mini_sta_relay_update_rtp_sequence_stats(&info);
    }
}
#endif

static void z1mini_sta_relay_learn_wifi_peer(uint32_t ip, const uint8_t *mac)
{
    TickType_t now = xTaskGetTickCount();
    size_t replace_idx = 0;
    TickType_t oldest = now;

    if (ip == 0U || ip == s_device_ip || ip == s_bridge_ip ||
        !z1mini_sta_relay_ip_in_bridge_subnet(ip) ||
        !z1mini_sta_relay_mac_is_usable_unicast(mac) ||
        z1mini_sta_relay_mac_equal(mac, s_sta_mac)) {
        return;
    }

    for (size_t i = 0; i < Z1MINI_STA_RELAY_PEER_TABLE_SIZE; ++i) {
        if (s_peers[i].valid && s_peers[i].ip == ip) {
            memcpy(s_peers[i].mac, mac, ETH_ADDR_LEN);
            s_peers[i].updated_tick = now;
            return;
        }
        if (!s_peers[i].valid) {
            replace_idx = i;
            oldest = 0;
            break;
        }
        if (i == 0 || s_peers[i].updated_tick < oldest) {
            oldest = s_peers[i].updated_tick;
            replace_idx = i;
        }
    }

    (void)oldest;
    s_peers[replace_idx].valid = true;
    s_peers[replace_idx].ip = ip;
    memcpy(s_peers[replace_idx].mac, mac, ETH_ADDR_LEN);
    s_peers[replace_idx].updated_tick = now;
}

static const uint8_t *z1mini_sta_relay_find_wifi_peer(uint32_t ip)
{
    if (ip == 0U) {
        return NULL;
    }

    for (size_t i = 0; i < Z1MINI_STA_RELAY_PEER_TABLE_SIZE; ++i) {
        if (s_peers[i].valid && s_peers[i].ip == ip) {
            return s_peers[i].mac;
        }
    }

    return NULL;
}

static z1mini_sta_relay_peer_t *z1mini_sta_relay_get_or_add_wifi_peer(uint32_t ip)
{
    TickType_t now = xTaskGetTickCount();
    size_t replace_idx = 0;
    TickType_t oldest = now;

    if (ip == 0U || ip == s_device_ip || ip == s_bridge_ip ||
        !z1mini_sta_relay_ip_in_bridge_subnet(ip) ||
        z1mini_sta_relay_is_ipv4_broadcast_or_multicast(ip)) {
        return NULL;
    }

    for (size_t i = 0; i < Z1MINI_STA_RELAY_PEER_TABLE_SIZE; ++i) {
        if (s_peers[i].valid && s_peers[i].ip == ip) {
            return &s_peers[i];
        }
        if (!s_peers[i].valid) {
            replace_idx = i;
            oldest = 0;
            break;
        }
        if (i == 0 || s_peers[i].updated_tick < oldest) {
            oldest = s_peers[i].updated_tick;
            replace_idx = i;
        }
    }

    (void)oldest;
    memset(&s_peers[replace_idx], 0, sizeof(s_peers[replace_idx]));
    s_peers[replace_idx].valid = true;
    s_peers[replace_idx].ip = ip;
    s_peers[replace_idx].updated_tick = now;
    return &s_peers[replace_idx];
}

static void z1mini_sta_relay_learn_device_mac(const uint8_t *mac)
{
    if (!z1mini_sta_relay_mac_is_usable_unicast(mac) ||
        z1mini_sta_relay_mac_equal(mac, s_eth_mac)) {
        return;
    }

    if (!s_device_mac_valid ||
        !z1mini_sta_relay_mac_equal(s_device_mac, mac)) {
        memcpy(s_device_mac, mac, ETH_ADDR_LEN);
        s_device_mac_valid = true;
        ESP_LOGI(TAG, "已学习 Z-1mini MAC | %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        z1mini_sta_relay_send_device_gratuitous_arp();
    }
}

static esp_err_t z1mini_sta_relay_tx_eth(const uint8_t *frame, size_t len)
{
    uint8_t *tx_buffer;
    size_t tx_len;
    esp_err_t ret;

    if (s_eth_handle == NULL || frame == NULL ||
        !z1mini_sta_relay_is_valid_eth_frame(len)) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_wifi_to_eth_invalid_count++;
        s_wifi_to_eth_last_err = ESP_ERR_INVALID_ARG;
#endif
        return ESP_ERR_INVALID_ARG;
    }

    tx_len = (len < Z1MINI_STA_RELAY_ETH_MIN_TX_LEN) ?
             Z1MINI_STA_RELAY_ETH_MIN_TX_LEN : len;
    tx_buffer = calloc(1, tx_len);
    if (tx_buffer == NULL) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_wifi_to_eth_no_mem_count++;
        s_wifi_to_eth_last_err = ESP_ERR_NO_MEM;
#endif
        return ESP_ERR_NO_MEM;
    }

    memcpy(tx_buffer, frame, len);
    ret = esp_eth_transmit(s_eth_handle, tx_buffer, tx_len);
    free(tx_buffer);
#if Z1MINI_STA_RELAY_STATS_ENABLE
    if (ret != ESP_OK) {
        s_wifi_to_eth_fail_count++;
    }
    s_wifi_to_eth_last_err = ret;
#endif
    return ret;
}

static esp_err_t z1mini_sta_relay_tx_wifi(const uint8_t *frame, size_t len)
{
    z1mini_sta_relay_frame_t item = {0};
    uint8_t *tx_buffer;
    size_t tx_len;
#if Z1MINI_STA_RELAY_STATS_ENABLE
    UBaseType_t depth;
#endif

    if (frame == NULL || !z1mini_sta_relay_is_valid_eth_frame(len)) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_invalid_count++;
        s_eth_to_wifi_last_err = ESP_ERR_INVALID_ARG;
#endif
        return ESP_ERR_INVALID_ARG;
    }
    if (s_wifi_tx_queue == NULL) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_fail_count++;
        s_eth_to_wifi_last_err = ESP_ERR_INVALID_STATE;
#endif
        return ESP_ERR_INVALID_STATE;
    }

    tx_len = (len < Z1MINI_STA_RELAY_ETH_MIN_TX_LEN) ?
             Z1MINI_STA_RELAY_ETH_MIN_TX_LEN : len;
    tx_buffer = calloc(1, tx_len);
    if (tx_buffer == NULL) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_no_mem_count++;
        s_eth_to_wifi_last_err = ESP_ERR_NO_MEM;
#endif
        return ESP_ERR_NO_MEM;
    }

    memcpy(tx_buffer, frame, len);
    item.buffer = tx_buffer;
    item.len = (uint16_t)tx_len;
    if (xQueueSend(s_wifi_tx_queue, &item, 0) != pdTRUE) {
        free(tx_buffer);
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_queue_full_count++;
        s_eth_to_wifi_last_err = ESP_ERR_TIMEOUT;
#endif
        return ESP_ERR_TIMEOUT;
    }

#if Z1MINI_STA_RELAY_STATS_ENABLE
    s_eth_to_wifi_queued_count++;
    depth = uxQueueMessagesWaiting(s_wifi_tx_queue);
    if (depth > s_eth_to_wifi_queue_max_depth) {
        s_eth_to_wifi_queue_max_depth = (uint32_t)depth;
    }
#endif
    return ESP_OK;
}

static esp_err_t z1mini_sta_relay_queue_wifi_owned_buffer(uint8_t *buffer,
                                                          size_t len)
{
    z1mini_sta_relay_frame_t item = {0};
#if Z1MINI_STA_RELAY_STATS_ENABLE
    UBaseType_t depth;
#endif

    if (buffer == NULL || !z1mini_sta_relay_is_valid_eth_frame(len)) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_invalid_count++;
        s_eth_to_wifi_last_err = ESP_ERR_INVALID_ARG;
#endif
        return ESP_ERR_INVALID_ARG;
    }
    if (s_wifi_tx_queue == NULL) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_fail_count++;
        s_eth_to_wifi_last_err = ESP_ERR_INVALID_STATE;
#endif
        return ESP_ERR_INVALID_STATE;
    }

    item.buffer = buffer;
    item.len = (uint16_t)len;
    if (xQueueSend(s_wifi_tx_queue, &item, 0) != pdTRUE) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_queue_full_count++;
        s_eth_to_wifi_last_err = ESP_ERR_TIMEOUT;
#endif
        return ESP_ERR_TIMEOUT;
    }

#if Z1MINI_STA_RELAY_STATS_ENABLE
    s_eth_to_wifi_queued_count++;
    depth = uxQueueMessagesWaiting(s_wifi_tx_queue);
    if (depth > s_eth_to_wifi_queue_max_depth) {
        s_eth_to_wifi_queue_max_depth = (uint32_t)depth;
    }
#endif
    return ESP_OK;
}

static void z1mini_sta_relay_wifi_tx_task(void *arg)
{
    z1mini_sta_relay_frame_t frame;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_wifi_tx_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (frame.buffer != NULL && frame.len > 0U) {
            esp_err_t ret = esp_wifi_internal_tx(WIFI_IF_STA, frame.buffer, frame.len);
#if Z1MINI_STA_RELAY_STATS_ENABLE
            if (ret == ESP_OK) {
                s_eth_to_wifi_count++;
            } else {
                s_eth_to_wifi_fail_count++;
            }
            s_eth_to_wifi_last_err = ret;
#else
            (void)ret;
#endif
            free(frame.buffer);
        }
    }
}

#if Z1MINI_STA_RELAY_STATS_ENABLE
static void z1mini_sta_relay_stats_task(void *arg)
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
    uint32_t last_e2w_unicast = 0;
    uint32_t last_e2w_broadcast = 0;
    uint32_t last_e2w_fallback = 0;
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
        uint32_t e2w_unicast;
        uint32_t e2w_broadcast;
        uint32_t e2w_fallback;
        uint32_t e2w_fail;
        uint32_t e2w_no_mem;
        uint32_t e2w_invalid;
        UBaseType_t e2w_queue_depth = 0;

        vTaskDelay(pdMS_TO_TICKS(Z1MINI_STA_RELAY_STATS_INTERVAL_MS));

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
        e2w_unicast = s_eth_to_wifi_unicast_count;
        e2w_broadcast = s_eth_to_wifi_broadcast_count;
        e2w_fallback = s_eth_to_wifi_broadcast_fallback_count;
        e2w_fail = s_eth_to_wifi_fail_count;
        e2w_no_mem = s_eth_to_wifi_no_mem_count;
        e2w_invalid = s_eth_to_wifi_invalid_count;
        if (s_wifi_tx_queue != NULL) {
            e2w_queue_depth = uxQueueMessagesWaiting(s_wifi_tx_queue);
        }

        ESP_LOGI(TAG,
                 "STA stats W2E ok=%" PRIu32 "/%" PRIu32
                 " fail=%" PRIu32 " no_mem=%" PRIu32
                 " invalid=%" PRIu32 " last_err=0x%x",
                 w2e_ok - last_w2e_ok,
                 w2e_total - last_w2e_total,
                 w2e_fail - last_w2e_fail,
                 w2e_no_mem - last_w2e_no_mem,
                 w2e_invalid - last_w2e_invalid,
                 (unsigned)s_wifi_to_eth_last_err);
        ESP_LOGI(TAG,
                 "STA stats E2W ok=%" PRIu32 "/%" PRIu32
                 " ip=%" PRIu32 " tcp=%" PRIu32 " udp=%" PRIu32
                 " udp_big=%" PRIu32 " rtp=%" PRIu32
                 " rtp_gap=%" PRIu32 "/%" PRIu32
                 " rtp_ooo=%" PRIu32 " rtp_reset=%" PRIu32
                 " rtp_seq=%" PRIu32 " rtp_ssrc=%08" PRIx32
                 " queued=%" PRIu32 " q_full=%" PRIu32
                 " q_depth=%u q_max=%" PRIu32
                 " unicast=%" PRIu32 " bcast=%" PRIu32
                 " fallback=%" PRIu32 " fail=%" PRIu32
                 " no_mem=%" PRIu32 " invalid=%" PRIu32
                 " last_err=0x%x max_len=%" PRIu32,
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
                 e2w_unicast - last_e2w_unicast,
                 e2w_broadcast - last_e2w_broadcast,
                 e2w_fallback - last_e2w_fallback,
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
        last_e2w_unicast = e2w_unicast;
        last_e2w_broadcast = e2w_broadcast;
        last_e2w_fallback = e2w_fallback;
        last_e2w_fail = e2w_fail;
        last_e2w_no_mem = e2w_no_mem;
        last_e2w_invalid = e2w_invalid;
    }
}
#endif

static esp_err_t z1mini_sta_relay_start_wifi_tx_task(void)
{
    BaseType_t task_ret;

    if (s_wifi_tx_queue == NULL) {
        s_wifi_tx_queue = xQueueCreate(Z1MINI_STA_RELAY_WIFI_TX_QUEUE_LEN,
                                       sizeof(z1mini_sta_relay_frame_t));
        ESP_RETURN_ON_FALSE(s_wifi_tx_queue != NULL, ESP_ERR_NO_MEM,
                            TAG, "创建 Wi-Fi TX 队列失败");
    }

    if (s_wifi_tx_task == NULL) {
        task_ret = xTaskCreate(z1mini_sta_relay_wifi_tx_task,
                               "z1_sta_wtx",
                               Z1MINI_STA_RELAY_WIFI_TX_TASK_STACK,
                               NULL,
                               Z1MINI_STA_RELAY_WIFI_TX_TASK_PRIO,
                               &s_wifi_tx_task);
        ESP_RETURN_ON_FALSE(task_ret == pdPASS, ESP_ERR_NO_MEM,
                            TAG, "创建 Wi-Fi TX 任务失败");
    }

#if Z1MINI_STA_RELAY_STATS_ENABLE
    if (s_stats_task == NULL) {
        task_ret = xTaskCreate(z1mini_sta_relay_stats_task,
                               "z1_sta_stat",
                               Z1MINI_STA_RELAY_STATS_TASK_STACK,
                               NULL,
                               Z1MINI_STA_RELAY_STATS_TASK_PRIO,
                               &s_stats_task);
        if (task_ret != pdPASS) {
            s_stats_task = NULL;
            ESP_LOGW(TAG, "创建 STA 转发统计任务失败");
        }
    }
#endif

    return ESP_OK;
}

static void z1mini_sta_relay_build_arp(uint8_t *frame,
                                       const uint8_t *eth_dst,
                                       const uint8_t *eth_src,
                                       uint16_t oper,
                                       const uint8_t *sha,
                                       uint32_t spa,
                                       const uint8_t *tha,
                                       uint32_t tpa)
{
    z1mini_sta_relay_eth_hdr_t *eth = (z1mini_sta_relay_eth_hdr_t *)frame;
    z1mini_sta_relay_arp_t *arp =
        (z1mini_sta_relay_arp_t *)(frame + Z1MINI_STA_RELAY_ETH_HEADER_LEN);

    memcpy(eth->dst, eth_dst, ETH_ADDR_LEN);
    memcpy(eth->src, eth_src, ETH_ADDR_LEN);
    z1mini_sta_relay_write_be16(&eth->eth_type, Z1MINI_STA_RELAY_ETH_TYPE_ARP);

    z1mini_sta_relay_write_be16(&arp->htype, Z1MINI_STA_RELAY_ARP_HTYPE_ETH);
    z1mini_sta_relay_write_be16(&arp->ptype, Z1MINI_STA_RELAY_ARP_PTYPE_IPV4);
    arp->hlen = ETH_ADDR_LEN;
    arp->plen = 4U;
    z1mini_sta_relay_write_be16(&arp->oper, oper);
    memcpy(arp->sha, sha, ETH_ADDR_LEN);
    arp->spa = spa;
    memcpy(arp->tha, tha, ETH_ADDR_LEN);
    arp->tpa = tpa;
}

static void z1mini_sta_relay_send_wifi_arp_reply(const uint8_t *target_mac,
                                                 uint32_t target_ip,
                                                 uint32_t sender_ip)
{
    uint8_t frame[Z1MINI_STA_RELAY_ETH_MIN_TX_LEN] = {0};

    z1mini_sta_relay_build_arp(frame,
                               target_mac,
                               s_sta_mac,
                               Z1MINI_STA_RELAY_ARP_REPLY,
                               s_sta_mac,
                               sender_ip,
                               target_mac,
                               target_ip);
    if (z1mini_sta_relay_tx_wifi(frame, sizeof(frame)) == ESP_OK) {
        s_proxy_arp_wifi_count++;
    }
}

static void z1mini_sta_relay_send_wifi_arp_request(uint32_t target_ip)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    static const uint8_t zero[ETH_ADDR_LEN] = {0};
    uint8_t frame[Z1MINI_STA_RELAY_ETH_MIN_TX_LEN] = {0};

    z1mini_sta_relay_build_arp(frame,
                               broadcast,
                               s_sta_mac,
                               Z1MINI_STA_RELAY_ARP_REQUEST,
                               s_sta_mac,
                               s_device_ip,
                               zero,
                               target_ip);
    (void)z1mini_sta_relay_tx_wifi(frame, sizeof(frame));
}

static void z1mini_sta_relay_send_device_gratuitous_arp(void)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    TickType_t now = xTaskGetTickCount();
    uint8_t frame[Z1MINI_STA_RELAY_ETH_MIN_TX_LEN] = {0};

    if (s_last_device_announce_tick != 0U) {
        uint32_t elapsed_ms = (uint32_t)((now - s_last_device_announce_tick) *
                                         portTICK_PERIOD_MS);
        if (elapsed_ms < Z1MINI_STA_RELAY_ARP_INTERVAL_MS) {
            return;
        }
    }
    s_last_device_announce_tick = now;

    z1mini_sta_relay_build_arp(frame,
                               broadcast,
                               s_sta_mac,
                               Z1MINI_STA_RELAY_ARP_REPLY,
                               s_sta_mac,
                               s_device_ip,
                               broadcast,
                               s_device_ip);
    (void)z1mini_sta_relay_tx_wifi(frame, sizeof(frame));
    ESP_LOGI(TAG, "Proxy ARP WiFi 宣告 Z-1mini 地址");
}

static void z1mini_sta_relay_probe_wifi_peer(uint32_t ip)
{
    z1mini_sta_relay_peer_t *peer;
    TickType_t now = xTaskGetTickCount();

    peer = z1mini_sta_relay_get_or_add_wifi_peer(ip);
    if (peer == NULL) {
        return;
    }
    if (z1mini_sta_relay_mac_is_usable_unicast(peer->mac)) {
        return;
    }
    if (peer->arp_request_tick != 0U) {
        uint32_t elapsed_ms = (uint32_t)((now - peer->arp_request_tick) *
                                         portTICK_PERIOD_MS);
        if (elapsed_ms < Z1MINI_STA_RELAY_PEER_ARP_INTERVAL_MS) {
            return;
        }
    }

    peer->arp_request_tick = now;
    z1mini_sta_relay_send_wifi_arp_request(ip);
}

static void z1mini_sta_relay_send_eth_arp_reply(const uint8_t *target_mac,
                                                uint32_t target_ip,
                                                uint32_t sender_ip)
{
    uint8_t frame[Z1MINI_STA_RELAY_ETH_MIN_TX_LEN] = {0};

    z1mini_sta_relay_build_arp(frame,
                               target_mac,
                               s_eth_mac,
                               Z1MINI_STA_RELAY_ARP_REPLY,
                               s_eth_mac,
                               sender_ip,
                               target_mac,
                               target_ip);
    if (z1mini_sta_relay_tx_eth(frame, sizeof(frame)) == ESP_OK) {
        s_proxy_arp_eth_count++;
    }
}

static void z1mini_sta_relay_request_device_mac(void)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    static const uint8_t zero[ETH_ADDR_LEN] = {0};
    TickType_t now = xTaskGetTickCount();
    uint8_t frame[Z1MINI_STA_RELAY_ETH_MIN_TX_LEN] = {0};

    if (s_device_mac_valid || s_eth_handle == NULL || !s_eth_link_up) {
        return;
    }

    if (s_last_device_arp_tick != 0U) {
        uint32_t elapsed_ms = (uint32_t)((now - s_last_device_arp_tick) *
                                         portTICK_PERIOD_MS);
        if (elapsed_ms < Z1MINI_STA_RELAY_ARP_INTERVAL_MS) {
            return;
        }
    }
    s_last_device_arp_tick = now;

    z1mini_sta_relay_build_arp(frame,
                               broadcast,
                               s_eth_mac,
                               Z1MINI_STA_RELAY_ARP_REQUEST,
                               s_eth_mac,
                               s_bridge_ip,
                               zero,
                               s_device_ip);
    (void)z1mini_sta_relay_tx_eth(frame, sizeof(frame));
}

static void z1mini_sta_relay_forward_wifi_to_eth(const uint8_t *frame,
                                                 size_t len,
                                                 const uint8_t *dst_mac)
{
    uint8_t *copy;
    z1mini_sta_relay_eth_hdr_t *eth;
    esp_err_t ret;

    if (frame == NULL || dst_mac == NULL ||
        !z1mini_sta_relay_is_valid_eth_frame(len)) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_wifi_to_eth_invalid_count++;
        s_wifi_to_eth_last_err = ESP_ERR_INVALID_ARG;
#endif
        return;
    }

#if Z1MINI_STA_RELAY_STATS_ENABLE
    s_wifi_to_eth_total_count++;
#endif
    copy = malloc(len);
    if (copy == NULL) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_wifi_to_eth_no_mem_count++;
        s_wifi_to_eth_last_err = ESP_ERR_NO_MEM;
#endif
        return;
    }

    memcpy(copy, frame, len);
    eth = (z1mini_sta_relay_eth_hdr_t *)copy;
    memcpy(eth->dst, dst_mac, ETH_ADDR_LEN);
    memcpy(eth->src, s_eth_mac, ETH_ADDR_LEN);

    ret = z1mini_sta_relay_tx_eth(copy, len);
    if (ret == ESP_OK) {
        s_wifi_to_eth_count++;
        if ((s_wifi_to_eth_count % Z1MINI_STA_RELAY_LOG_INTERVAL) == 1U) {
            ESP_LOGI(TAG, "Proxy 转发 WiFi->ETH | count=%" PRIu32,
                     s_wifi_to_eth_count);
        }
    }
#if Z1MINI_STA_RELAY_STATS_ENABLE
    s_wifi_to_eth_last_err = ret;
#endif

    free(copy);
}

static bool z1mini_sta_relay_forward_eth_to_wifi_owned(uint8_t *frame,
                                                       size_t len,
                                                       const uint8_t *dst_mac,
                                                       bool fallback)
{
    z1mini_sta_relay_eth_hdr_t *eth;
    esp_err_t ret;

    if (frame == NULL || dst_mac == NULL ||
        !z1mini_sta_relay_is_valid_eth_frame(len)) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        s_eth_to_wifi_invalid_count++;
        s_eth_to_wifi_last_err = ESP_ERR_INVALID_ARG;
#endif
        return false;
    }

    eth = (z1mini_sta_relay_eth_hdr_t *)frame;
    memcpy(eth->dst, dst_mac, ETH_ADDR_LEN);
    memcpy(eth->src, s_sta_mac, ETH_ADDR_LEN);

    ret = z1mini_sta_relay_queue_wifi_owned_buffer(frame, len);
    if (ret == ESP_OK) {
#if Z1MINI_STA_RELAY_STATS_ENABLE
        if (z1mini_sta_relay_mac_is_broadcast(dst_mac) ||
            z1mini_sta_relay_mac_is_multicast(dst_mac)) {
            s_eth_to_wifi_broadcast_count++;
            if (fallback) {
                s_eth_to_wifi_broadcast_fallback_count++;
            }
        } else {
            s_eth_to_wifi_unicast_count++;
        }
#else
        (void)fallback;
#endif
        return true;
    }

    return false;
}

static void z1mini_sta_relay_handle_wifi_arp(const uint8_t *frame, size_t len)
{
    const z1mini_sta_relay_eth_hdr_t *eth;
    const z1mini_sta_relay_arp_t *arp;
    uint16_t oper;

    if (len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + sizeof(*arp))) {
        return;
    }

    eth = (const z1mini_sta_relay_eth_hdr_t *)frame;
    arp = (const z1mini_sta_relay_arp_t *)(frame + Z1MINI_STA_RELAY_ETH_HEADER_LEN);
    if (z1mini_sta_relay_read_be16(&arp->htype) != Z1MINI_STA_RELAY_ARP_HTYPE_ETH ||
        z1mini_sta_relay_read_be16(&arp->ptype) != Z1MINI_STA_RELAY_ARP_PTYPE_IPV4 ||
        arp->hlen != ETH_ADDR_LEN || arp->plen != 4U) {
        return;
    }

    z1mini_sta_relay_learn_wifi_peer(arp->spa, arp->sha);
    oper = z1mini_sta_relay_read_be16(&arp->oper);
    if (oper == Z1MINI_STA_RELAY_ARP_REQUEST && arp->tpa == s_device_ip) {
        z1mini_sta_relay_send_wifi_arp_reply(arp->sha, arp->spa, s_device_ip);
        z1mini_sta_relay_send_device_gratuitous_arp();
        z1mini_sta_relay_request_device_mac();
        ESP_LOGI(TAG,
                 "Proxy ARP WiFi 代答 Z-1mini | requester=%02x:%02x:%02x:%02x:%02x:%02x",
                 eth->src[0], eth->src[1], eth->src[2],
                 eth->src[3], eth->src[4], eth->src[5]);
    }
}

static void z1mini_sta_relay_handle_eth_arp(const uint8_t *frame, size_t len)
{
    const z1mini_sta_relay_eth_hdr_t *eth;
    const z1mini_sta_relay_arp_t *arp;
    uint16_t oper;

    if (len < (Z1MINI_STA_RELAY_ETH_HEADER_LEN + sizeof(*arp))) {
        return;
    }

    eth = (const z1mini_sta_relay_eth_hdr_t *)frame;
    arp = (const z1mini_sta_relay_arp_t *)(frame + Z1MINI_STA_RELAY_ETH_HEADER_LEN);
    if (z1mini_sta_relay_read_be16(&arp->htype) != Z1MINI_STA_RELAY_ARP_HTYPE_ETH ||
        z1mini_sta_relay_read_be16(&arp->ptype) != Z1MINI_STA_RELAY_ARP_PTYPE_IPV4 ||
        arp->hlen != ETH_ADDR_LEN || arp->plen != 4U) {
        return;
    }

    if (arp->spa == s_device_ip) {
        z1mini_sta_relay_learn_device_mac(eth->src);
    }

    oper = z1mini_sta_relay_read_be16(&arp->oper);
    if (oper == Z1MINI_STA_RELAY_ARP_REQUEST &&
        arp->tpa != s_device_ip &&
        z1mini_sta_relay_ip_in_bridge_subnet(arp->tpa)) {
        z1mini_sta_relay_send_eth_arp_reply(arp->sha, arp->spa, arp->tpa);
        ESP_LOGI(TAG, "Proxy ARP ETH 代答 WiFi 侧地址");
    }
}

static void z1mini_sta_relay_handle_wifi_ipv4(const uint8_t *frame, size_t len)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    const z1mini_sta_relay_eth_hdr_t *eth =
        (const z1mini_sta_relay_eth_hdr_t *)frame;
    uint32_t src_ip;
    uint32_t dst_ip;

    if (!z1mini_sta_relay_parse_ipv4_addrs(frame, len, &src_ip, &dst_ip)) {
        return;
    }

    z1mini_sta_relay_learn_wifi_peer(src_ip, eth->src);

    if (dst_ip == s_device_ip) {
        if (s_wifi_to_eth_count < 3U) {
            ESP_LOGI(TAG, "收到 WiFi->Z-1mini IPv4 请求");
        }
        if (!s_device_mac_valid) {
            z1mini_sta_relay_request_device_mac();
            return;
        }
        z1mini_sta_relay_forward_wifi_to_eth(frame, len, s_device_mac);
        return;
    }

    if (z1mini_sta_relay_is_ipv4_broadcast_or_multicast(dst_ip)) {
        z1mini_sta_relay_forward_wifi_to_eth(frame, len, broadcast);
    }
}

static bool z1mini_sta_relay_handle_eth_ipv4(uint8_t *frame, size_t len)
{
    static const uint8_t broadcast[ETH_ADDR_LEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    const z1mini_sta_relay_eth_hdr_t *eth =
        (const z1mini_sta_relay_eth_hdr_t *)frame;
    const uint8_t *dst_mac;
    uint32_t src_ip;
    uint32_t dst_ip;

    if (!z1mini_sta_relay_parse_ipv4_addrs(frame, len, &src_ip, &dst_ip)) {
        return false;
    }

#if Z1MINI_STA_RELAY_STATS_ENABLE
    z1mini_sta_relay_update_eth_to_wifi_frame_stats(frame, len);
#endif

    if (src_ip == s_device_ip) {
        z1mini_sta_relay_learn_device_mac(eth->src);
    }

    if (z1mini_sta_relay_is_ipv4_broadcast_or_multicast(dst_ip)) {
        return z1mini_sta_relay_forward_eth_to_wifi_owned(frame, len, broadcast, false);
    }

    dst_mac = z1mini_sta_relay_find_wifi_peer(dst_ip);
    if (dst_mac != NULL) {
        return z1mini_sta_relay_forward_eth_to_wifi_owned(frame, len, dst_mac, false);
    } else if (z1mini_sta_relay_ip_in_bridge_subnet(dst_ip)) {
        z1mini_sta_relay_probe_wifi_peer(dst_ip);
        /*
         * 还未学习到目标 MAC 时用二层广播兜底。目标主机的 IP 层仍会按
         * dst_ip 判断是否接收，比盲目开端口代理占用资源更可控。
         */
        return z1mini_sta_relay_forward_eth_to_wifi_owned(frame, len, broadcast, true);
    }

    return false;
}

static esp_err_t z1mini_sta_relay_wifi_rx(void *buffer, uint16_t len, void *eb)
{
    const z1mini_sta_relay_eth_hdr_t *eth;
    uint16_t eth_type;

    if (buffer != NULL && z1mini_sta_relay_is_valid_eth_frame(len)) {
        eth = (const z1mini_sta_relay_eth_hdr_t *)buffer;
        if (!z1mini_sta_relay_mac_equal(eth->src, s_sta_mac) &&
            (z1mini_sta_relay_mac_equal(eth->dst, s_sta_mac) ||
             z1mini_sta_relay_mac_is_multicast(eth->dst) ||
             z1mini_sta_relay_mac_is_broadcast(eth->dst))) {
            eth_type = z1mini_sta_relay_read_be16(&eth->eth_type);
            if (eth_type == Z1MINI_STA_RELAY_ETH_TYPE_ARP) {
                z1mini_sta_relay_handle_wifi_arp((const uint8_t *)buffer, len);
            } else if (eth_type == Z1MINI_STA_RELAY_ETH_TYPE_IPV4) {
                z1mini_sta_relay_handle_wifi_ipv4((const uint8_t *)buffer, len);
            }
        }
    }

    if (s_sta_netif != NULL) {
        return esp_netif_receive(s_sta_netif, buffer, len, eb);
    }

    return ESP_OK;
}

static esp_err_t z1mini_sta_relay_eth_rx(esp_eth_handle_t eth_handle,
                                         uint8_t *buffer,
                                         uint32_t len,
                                         void *priv)
{
    const z1mini_sta_relay_eth_hdr_t *eth;
    uint16_t eth_type;
    bool buffer_queued = false;

    (void)eth_handle;
    (void)priv;

    if (buffer != NULL && z1mini_sta_relay_is_valid_eth_frame(len)) {
        eth = (const z1mini_sta_relay_eth_hdr_t *)buffer;
        if (!z1mini_sta_relay_mac_equal(eth->src, s_eth_mac)) {
            eth_type = z1mini_sta_relay_read_be16(&eth->eth_type);
            if (eth_type == Z1MINI_STA_RELAY_ETH_TYPE_ARP) {
                z1mini_sta_relay_handle_eth_arp(buffer, len);
            } else if (eth_type == Z1MINI_STA_RELAY_ETH_TYPE_IPV4) {
                buffer_queued = z1mini_sta_relay_handle_eth_ipv4(buffer, len);
            }
        }
    }

    if (!buffer_queued) {
        free(buffer);
    }
    return ESP_OK;
}

static void z1mini_sta_relay_eth_event_handler(void *arg, esp_event_base_t base,
                                               int32_t id, void *data)
{
    uint8_t mac_addr[ETH_ADDR_LEN] = {0};
    esp_eth_handle_t eth_handle;

    (void)arg;

    if (base != ETH_EVENT || data == NULL) {
        return;
    }

    eth_handle = *(esp_eth_handle_t *)data;
    if (id == ETHERNET_EVENT_CONNECTED) {
        s_eth_link_up = true;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Z-1mini ETH 链路已连接 | MAC=%02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        z1mini_sta_relay_request_device_mac();
        if (s_event_group) {
            xEventGroupSetBits(s_event_group, Z1MINI_STA_RELAY_ETH_LINK_BIT);
        }
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        s_eth_link_up = false;
        s_device_mac_valid = false;
        ESP_LOGW(TAG, "Z-1mini ETH 链路已断开");
        if (s_event_group) {
            xEventGroupClearBits(s_event_group, Z1MINI_STA_RELAY_ETH_LINK_BIT);
        }
    } else if (id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "Z-1mini ETH 已启动");
    } else if (id == ETHERNET_EVENT_STOP) {
        s_eth_link_up = false;
        s_device_mac_valid = false;
        ESP_LOGW(TAG, "Z-1mini ETH 已停止");
    }
}

static esp_err_t z1mini_sta_relay_parse_ip_args(const char *eth_ip,
                                                const char *eth_mask,
                                                const char *device_ip)
{
    ESP_RETURN_ON_FALSE(eth_ip != NULL && eth_mask != NULL && device_ip != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "Proxy ARP IP 参数为空");

    s_bridge_ip = esp_ip4addr_aton(eth_ip);
    s_bridge_mask = esp_ip4addr_aton(eth_mask);
    s_device_ip = esp_ip4addr_aton(device_ip);
    ESP_RETURN_ON_FALSE(s_bridge_ip != 0U && s_bridge_mask != 0U &&
                        s_device_ip != 0U,
                        ESP_ERR_INVALID_ARG, TAG, "Proxy ARP IP 参数非法");
    ESP_RETURN_ON_FALSE(z1mini_sta_relay_ip_in_bridge_subnet(s_device_ip),
                        ESP_ERR_INVALID_ARG, TAG, "Z-1mini 不在 Proxy ARP 子网内");
    return ESP_OK;
}

static void z1mini_sta_relay_log_subnet_warning(void)
{
    esp_netif_ip_info_t sta_ip = {0};
    char sta_ip_text[16] = {0};
    char device_ip_text[16] = {0};

    if (s_sta_netif == NULL ||
        esp_netif_get_ip_info(s_sta_netif, &sta_ip) != ESP_OK ||
        sta_ip.ip.addr == 0U) {
        ESP_LOGW(TAG, "未读取到 Wi-Fi STA IP，无法确认是否与 Z-1mini 同网段");
        return;
    }

    z1mini_sta_relay_format_ip(sta_ip.ip.addr, sta_ip_text, sizeof(sta_ip_text));
    z1mini_sta_relay_format_ip(s_device_ip, device_ip_text, sizeof(device_ip_text));
    if ((sta_ip.ip.addr & s_bridge_mask) != (s_device_ip & s_bridge_mask)) {
        ESP_LOGW(TAG,
                 "Wi-Fi STA 与 Z-1mini 不在同一 Proxy ARP 子网 | STA=%s | Z-1mini=%s",
                 sta_ip_text, device_ip_text);
    } else {
        ESP_LOGI(TAG, "Wi-Fi STA 与 Z-1mini 子网一致 | STA=%s | Z-1mini=%s",
                 sta_ip_text, device_ip_text);
    }
}

static esp_err_t z1mini_sta_relay_register_wifi_rx(void)
{
    esp_err_t ret;

    if (s_wifi_rx_registered) {
        return ESP_OK;
    }

    s_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    ESP_RETURN_ON_FALSE(s_sta_netif != NULL, ESP_ERR_INVALID_STATE,
                        TAG, "未找到 Wi-Fi STA netif");

    ret = esp_wifi_get_mac(WIFI_IF_STA, s_sta_mac);
    if (ret != ESP_OK) {
        ret = esp_read_mac(s_sta_mac, ESP_MAC_WIFI_STA);
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "读取 Wi-Fi STA MAC 失败");

    ret = esp_wifi_internal_reg_rxcb(WIFI_IF_STA, z1mini_sta_relay_wifi_rx);
    ESP_RETURN_ON_ERROR(ret, TAG, "注册 Wi-Fi STA Proxy ARP RX 回调失败");

    s_wifi_rx_registered = true;
    ESP_LOGI(TAG, "Wi-Fi STA Proxy ARP RX 回调已启用 | MAC=%02x:%02x:%02x:%02x:%02x:%02x",
             s_sta_mac[0], s_sta_mac[1], s_sta_mac[2],
             s_sta_mac[3], s_sta_mac[4], s_sta_mac[5]);
    return ESP_OK;
}

static esp_err_t z1mini_sta_relay_start_eth(void)
{
    esp_eth_mac_t *mac;
    esp_eth_phy_t *phy;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_config_t eth_config;

    if (s_eth_handle != NULL) {
        return ESP_OK;
    }

    phy_config.phy_addr = ESP_ETH_PHY_ADDR_AUTO;
    phy_config.reset_gpio_num = -1;

    mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    ESP_RETURN_ON_FALSE(mac != NULL, ESP_FAIL, TAG, "创建 Z-1mini ETH MAC 失败");

    phy = esp_eth_phy_new_ip101(&phy_config);
    ESP_RETURN_ON_FALSE(phy != NULL, ESP_FAIL, TAG, "创建 Z-1mini IP101 PHY 失败");

    eth_config = (esp_eth_config_t)ETH_DEFAULT_CONFIG(mac, phy);
    eth_config.stack_input = z1mini_sta_relay_eth_rx;

    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_config, &s_eth_handle),
                        TAG, "安装 Z-1mini ETH 驱动失败");
    ESP_RETURN_ON_ERROR(esp_read_mac(s_eth_mac, ESP_MAC_ETH),
                        TAG, "读取 Z-1mini ETH MAC 失败");
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, s_eth_mac),
                        TAG, "设置 Z-1mini ETH MAC 失败");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                   z1mini_sta_relay_eth_event_handler, NULL),
                        TAG, "注册 Z-1mini ETH 事件失败");
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth_handle), TAG, "启动 Z-1mini ETH 失败");

    return ESP_OK;
}

esp_err_t z1mini_sta_relay_start(const char *eth_ip, const char *eth_gw,
                                 const char *eth_mask, const char *device_ip)
{
    esp_netif_t *sta_netif;

    (void)eth_gw;

    if (s_running) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(z1mini_sta_relay_parse_ip_args(eth_ip, eth_mask, device_ip),
                        TAG, "解析 Proxy ARP 参数失败");

    if (!s_event_group) {
        s_event_group = xEventGroupCreate();
        ESP_RETURN_ON_FALSE(s_event_group != NULL, ESP_ERR_NO_MEM,
                            TAG, "创建事件组失败");
    }
    xEventGroupClearBits(s_event_group, Z1MINI_STA_RELAY_ETH_LINK_BIT);

    ESP_RETURN_ON_ERROR(z1mini_sta_relay_start_wifi_tx_task(),
                        TAG, "启动 Wi-Fi TX 转发任务失败");
    ESP_RETURN_ON_ERROR(z1mini_sta_relay_register_wifi_rx(),
                        TAG, "启用 Wi-Fi STA Proxy ARP 失败");
    ESP_RETURN_ON_ERROR(z1mini_sta_relay_start_eth(),
                        TAG, "启动 STA 模式 Z-1mini ETH 失败");

    sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif != NULL) {
        esp_err_t ret = esp_netif_set_default_netif(sta_netif);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "恢复 Wi-Fi STA 为默认 netif 失败: 0x%x (%s)",
                     ret, esp_err_to_name(ret));
        }
    }

    s_running = true;
    z1mini_sta_relay_log_subnet_warning();
    ESP_LOGI(TAG,
             "STA 模式 Z-1mini Proxy ARP 已启动 | WT99-ETH=%s | Z-1mini=%s | 访问=rtsp://%s",
             eth_ip, device_ip, device_ip);
    ESP_LOGW(TAG,
             "该模式不是 WDS/四地址二层桥；Z-1mini 不会直接从主路由 DHCP 获取地址，仍使用固定地址并由 WT99 代答 ARP");

    if (!s_eth_link_up) {
        ESP_LOGW(TAG, "Z-1mini ETH 链路尚未连接，请确认吊舱供电和网线");
    }

    return ESP_OK;
}

bool z1mini_sta_relay_is_running(void)
{
    return s_running;
}
