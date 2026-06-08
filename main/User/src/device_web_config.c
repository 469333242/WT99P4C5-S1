/**
 * @file device_web_config.c
 * @brief 设备网页配置模块实现
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"

#include "device_web_config.h"
#include "tcp_uart_server.h"
#include "wifi_connect.h"

static const char *TAG = "device_web_cfg";

#define DEVICE_WEB_CONFIG_NVS_NAMESPACE    "device_web_cfg"
#define DEVICE_WEB_CONFIG_NVS_KEY_CONFIG   "config"
#define DEVICE_WEB_CONFIG_MAGIC            0x44574346U
#define DEVICE_WEB_CONFIG_VERSION          6U
#define DEVICE_WEB_CONFIG_VERSION_LEGACY   1U
#define DEVICE_WEB_CONFIG_VERSION_V2       2U
#define DEVICE_WEB_CONFIG_VERSION_V3       3U
#define DEVICE_WEB_CONFIG_VERSION_V4       4U
#define DEVICE_WEB_CONFIG_VERSION_V5       5U
#define DEVICE_WEB_CONFIG_MIN_BAUD_RATE    1200U
#define DEVICE_WEB_CONFIG_MAX_BAUD_RATE    2000000U
#define DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM  (1U << 0)

typedef struct {
    uint32_t uart0_baud_rate;
    uint32_t uart1_baud_rate;
    uint32_t video_profile;
    bool     wifi_use_static_ip;
    char     wifi_static_ip[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_gw[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_mask[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
} device_web_config_v1_t;

typedef struct {
    uint32_t uart0_baud_rate;
    uint32_t uart1_baud_rate;
    uint32_t video_profile;
    char     wifi_ap_ssid[DEVICE_WEB_CONFIG_WIFI_SSID_LEN];
    char     wifi_ap_password[DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN];
    bool     wifi_use_static_ip;
    char     wifi_static_ip[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_gw[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_mask[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
} device_web_config_v2_t;

typedef struct {
    uint32_t uart0_baud_rate;
    uint32_t uart1_baud_rate;
    uint32_t video_source;
    uint32_t video_profile;
    char     wifi_ap_ssid[DEVICE_WEB_CONFIG_WIFI_SSID_LEN];
    char     wifi_ap_password[DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN];
    bool     wifi_use_static_ip;
    char     wifi_static_ip[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_gw[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
    char     wifi_static_mask[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN];
} device_web_config_v3_t;

typedef struct {
    uint32_t            magic;
    uint32_t            version;
    device_web_config_t config;
    uint32_t            reserved;
} device_web_config_storage_t;

typedef struct {
    uint32_t               magic;
    uint32_t               version;
    device_web_config_v1_t config;
} device_web_config_storage_v1_t;

typedef struct {
    uint32_t               magic;
    uint32_t               version;
    device_web_config_v2_t config;
} device_web_config_storage_v2_t;

typedef struct {
    uint32_t               magic;
    uint32_t               version;
    device_web_config_v3_t config;
} device_web_config_storage_v3_t;

static SemaphoreHandle_t s_device_web_config_mutex;
static device_web_config_storage_t s_device_web_config_storage;
static bool s_device_web_config_initialized;

static bool device_web_config_validate(const device_web_config_t *config);

static esp_err_t device_web_config_copy_text(char *dst, size_t dst_size, const char *text)
{
    size_t text_len = 0;

    if (!dst || dst_size == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!text) {
        dst[0] = '\0';
        return ESP_OK;
    }

    text_len = strlen(text);
    if (text_len >= dst_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(dst, text, text_len + 1U);
    return ESP_OK;
}

static bool device_web_config_is_valid_wifi_text(const char *text, size_t min_len, size_t max_len)
{
    size_t text_len;

    if (!text) {
        return false;
    }

    text_len = strlen(text);
    if (text_len < min_len || text_len >= max_len) {
        return false;
    }

    for (const unsigned char *cursor = (const unsigned char *)text; *cursor != '\0'; cursor++) {
        if (*cursor < 0x20U || *cursor == 0x7FU ||
            *cursor == '"' || *cursor == '\\') {
            return false;
        }
    }

    return true;
}

static uint32_t device_web_config_default_video_profile(void)
{
#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS
    return DEVICE_WEB_CONFIG_VIDEO_PROFILE_1280X960;
#elif CONFIG_CAMERA_OV5647_MIPI_RAW10_1920X1080_30FPS
    return DEVICE_WEB_CONFIG_VIDEO_PROFILE_1920X1080;
#elif CONFIG_CAMERA_OV5647_MIPI_RAW8_800X800_50FPS
    return DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X800;
#elif CONFIG_CAMERA_OV5647_MIPI_RAW8_800X640_50FPS
    return DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X640;
#else
    return DEVICE_WEB_CONFIG_VIDEO_PROFILE_1280X960;
#endif
}

static void device_web_config_fill_storage(device_web_config_storage_t *storage,
                                           const device_web_config_t *config)
{
    if (!storage || !config) {
        return;
    }

    memset(storage, 0, sizeof(*storage));
    storage->magic = DEVICE_WEB_CONFIG_MAGIC;
    storage->version = DEVICE_WEB_CONFIG_VERSION;
    storage->config = *config;
}

static bool device_web_config_import_legacy(const device_web_config_storage_v1_t *legacy,
                                            device_web_config_t *out_config)
{
    if (!legacy || !out_config ||
        legacy->magic != DEVICE_WEB_CONFIG_MAGIC ||
        legacy->version != DEVICE_WEB_CONFIG_VERSION_LEGACY) {
        return false;
    }

    /*
     * 旧版 NVS blob 没有后续新增字段。
     * 迁移时先填默认配置，再只复制旧版存在且校验通过的字段，防止未初始化字节进入运行配置。
     */
    device_web_config_get_defaults(out_config);
    out_config->uart0_baud_rate = legacy->config.uart0_baud_rate;
    out_config->uart1_baud_rate = legacy->config.uart1_baud_rate;
    out_config->video_profile = legacy->config.video_profile;
    out_config->wifi_use_static_ip = true;
    if (device_web_config_is_valid_ipv4_text(legacy->config.wifi_static_ip) &&
        device_web_config_is_valid_ipv4_text(legacy->config.wifi_static_gw) &&
        device_web_config_is_valid_ipv4_text(legacy->config.wifi_static_mask)) {
        device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                    legacy->config.wifi_static_ip);
        device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                    legacy->config.wifi_static_gw);
        device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                    legacy->config.wifi_static_mask);
    }

    return device_web_config_validate(out_config);
}

static bool device_web_config_import_v2(const device_web_config_storage_v2_t *legacy,
                                        device_web_config_t *out_config)
{
    if (!legacy || !out_config ||
        legacy->magic != DEVICE_WEB_CONFIG_MAGIC ||
        legacy->version != DEVICE_WEB_CONFIG_VERSION_V2) {
        return false;
    }

    device_web_config_get_defaults(out_config);
    out_config->uart0_baud_rate = legacy->config.uart0_baud_rate;
    out_config->uart1_baud_rate = legacy->config.uart1_baud_rate;
    out_config->video_profile = legacy->config.video_profile;
    out_config->wifi_use_static_ip = legacy->config.wifi_use_static_ip;
    device_web_config_copy_text(out_config->wifi_ap_ssid, sizeof(out_config->wifi_ap_ssid),
                                legacy->config.wifi_ap_ssid);
    device_web_config_copy_text(out_config->wifi_ap_password, sizeof(out_config->wifi_ap_password),
                                legacy->config.wifi_ap_password);
    device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                legacy->config.wifi_static_ip);
    device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                legacy->config.wifi_static_gw);
    device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                legacy->config.wifi_static_mask);

    return device_web_config_validate(out_config);
}

static bool device_web_config_import_v3(const device_web_config_storage_v3_t *legacy,
                                        device_web_config_t *out_config)
{
    if (!legacy || !out_config ||
        legacy->magic != DEVICE_WEB_CONFIG_MAGIC ||
        legacy->version != DEVICE_WEB_CONFIG_VERSION_V3) {
        return false;
    }

    device_web_config_get_defaults(out_config);
    out_config->uart0_baud_rate = legacy->config.uart0_baud_rate;
    out_config->uart1_baud_rate = legacy->config.uart1_baud_rate;
    out_config->video_source = legacy->config.video_source;
    out_config->video_profile = legacy->config.video_profile;
    out_config->wifi_mode = DEVICE_WEB_CONFIG_WIFI_MODE_AP;
    out_config->wifi_use_static_ip = legacy->config.wifi_use_static_ip;
    device_web_config_copy_text(out_config->wifi_ap_ssid, sizeof(out_config->wifi_ap_ssid),
                                legacy->config.wifi_ap_ssid);
    device_web_config_copy_text(out_config->wifi_ap_password, sizeof(out_config->wifi_ap_password),
                                legacy->config.wifi_ap_password);
    device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                legacy->config.wifi_static_ip);
    device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                legacy->config.wifi_static_gw);
    device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                legacy->config.wifi_static_mask);

    return device_web_config_validate(out_config);
}

static bool device_web_config_import_v4(const device_web_config_storage_t *legacy,
                                        device_web_config_t *out_config)
{
    device_web_config_t defaults;
    bool sta_ssid_valid;

    if (!legacy || !out_config ||
        legacy->magic != DEVICE_WEB_CONFIG_MAGIC ||
        legacy->version != DEVICE_WEB_CONFIG_VERSION_V4) {
        return false;
    }

    device_web_config_get_defaults(&defaults);
    *out_config = legacy->config;

    /*
     * V4 已接近当前结构，但可能缺少新字段或保存过非法值。
     * 逐项兜底比整体丢弃配置更温和，用户已有的可用配置会被保留。
     */
    if (!device_web_config_is_valid_baud_rate(out_config->uart0_baud_rate)) {
        out_config->uart0_baud_rate = defaults.uart0_baud_rate;
    }
    if (!device_web_config_is_valid_baud_rate(out_config->uart1_baud_rate)) {
        out_config->uart1_baud_rate = defaults.uart1_baud_rate;
    }
    if (!device_web_config_is_valid_video_source(out_config->video_source)) {
        out_config->video_source = defaults.video_source;
    }
    if (!device_web_config_is_valid_video_profile(out_config->video_profile)) {
        out_config->video_profile = defaults.video_profile;
    }
    if (!device_web_config_is_valid_wifi_mode(out_config->wifi_mode)) {
        out_config->wifi_mode = defaults.wifi_mode;
    }

    if (!device_web_config_is_valid_wifi_text(out_config->wifi_ap_ssid, 1U,
                                              DEVICE_WEB_CONFIG_WIFI_SSID_LEN)) {
        device_web_config_copy_text(out_config->wifi_ap_ssid, sizeof(out_config->wifi_ap_ssid),
                                    defaults.wifi_ap_ssid);
    }
    if (!device_web_config_is_valid_wifi_text(out_config->wifi_ap_password, 8U,
                                              DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN)) {
        device_web_config_copy_text(out_config->wifi_ap_password, sizeof(out_config->wifi_ap_password),
                                    defaults.wifi_ap_password);
    }

    out_config->wifi_use_static_ip = true;
    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_static_ip)) {
        device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                    defaults.wifi_static_ip);
    }
    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_static_gw)) {
        device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                    defaults.wifi_static_gw);
    }
    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_static_mask)) {
        device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                    defaults.wifi_static_mask);
    }

    /* V4 的 STA 默认 SSID/密码为空，迁移时补齐为当前预置的入网参数。 */
    sta_ssid_valid = device_web_config_is_valid_wifi_text(out_config->wifi_sta_ssid, 1U,
                                                          DEVICE_WEB_CONFIG_WIFI_SSID_LEN);
    if (!sta_ssid_valid) {
        device_web_config_copy_text(out_config->wifi_sta_ssid, sizeof(out_config->wifi_sta_ssid),
                                    defaults.wifi_sta_ssid);
        device_web_config_copy_text(out_config->wifi_sta_password, sizeof(out_config->wifi_sta_password),
                                    defaults.wifi_sta_password);
    } else if (!device_web_config_is_valid_wifi_text(out_config->wifi_sta_password, 0U,
                                                     DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN) ||
               (strlen(out_config->wifi_sta_password) > 0U &&
                strlen(out_config->wifi_sta_password) < 8U)) {
        device_web_config_copy_text(out_config->wifi_sta_password, sizeof(out_config->wifi_sta_password),
                                    defaults.wifi_sta_password);
    }

    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_sta_ip)) {
        device_web_config_copy_text(out_config->wifi_sta_ip, sizeof(out_config->wifi_sta_ip),
                                    defaults.wifi_sta_ip);
    }
    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_sta_gw)) {
        device_web_config_copy_text(out_config->wifi_sta_gw, sizeof(out_config->wifi_sta_gw),
                                    defaults.wifi_sta_gw);
    }
    if (!device_web_config_is_valid_ipv4_text(out_config->wifi_sta_mask)) {
        device_web_config_copy_text(out_config->wifi_sta_mask, sizeof(out_config->wifi_sta_mask),
                                    defaults.wifi_sta_mask);
    }

    return device_web_config_validate(out_config);
}

static bool device_web_config_import_v5(const device_web_config_storage_t *legacy,
                                        device_web_config_t *out_config,
                                        uint32_t *out_flags)
{
    if (!legacy || !out_config || !out_flags ||
        legacy->magic != DEVICE_WEB_CONFIG_MAGIC ||
        legacy->version != DEVICE_WEB_CONFIG_VERSION_V5) {
        return false;
    }

    *out_config = legacy->config;
    /*
     * V5 还没有独立保存 pending-confirm 标志。
     * 若启动在 STA 模式，迁移后先视为待确认，避免升级后配置了错误路由器导致设备失联。
     */
    *out_flags = (legacy->config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) ?
                 DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM : 0U;

    return device_web_config_validate(out_config);
}

static bool device_web_config_validate(const device_web_config_t *config)
{
    if (!config) {
        return false;
    }

    if (!device_web_config_is_valid_baud_rate(config->uart0_baud_rate) ||
        !device_web_config_is_valid_baud_rate(config->uart1_baud_rate)) {
        return false;
    }

    if (!device_web_config_is_valid_video_source(config->video_source) ||
        !device_web_config_is_valid_video_profile(config->video_profile)) {
        return false;
    }

    if (!device_web_config_is_valid_wifi_mode(config->wifi_mode)) {
        return false;
    }

    if (!device_web_config_is_valid_wifi_text(config->wifi_ap_ssid, 1U,
                                              DEVICE_WEB_CONFIG_WIFI_SSID_LEN) ||
        !device_web_config_is_valid_wifi_text(config->wifi_ap_password, 8U,
                                              DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN)) {
        return false;
    }

    if (!config->wifi_use_static_ip ||
        !device_web_config_is_valid_ipv4_text(config->wifi_static_ip) ||
        !device_web_config_is_valid_ipv4_text(config->wifi_static_gw) ||
        !device_web_config_is_valid_ipv4_text(config->wifi_static_mask)) {
        return false;
    }

    if (config->wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) {
        if (!device_web_config_is_valid_wifi_text(config->wifi_sta_ssid, 1U,
                                                  DEVICE_WEB_CONFIG_WIFI_SSID_LEN) ||
            !device_web_config_is_valid_wifi_text(config->wifi_sta_password, 0U,
                                                  DEVICE_WEB_CONFIG_WIFI_PASSWORD_LEN)) {
            return false;
        }

        if (strlen(config->wifi_sta_password) > 0U &&
            strlen(config->wifi_sta_password) < 8U) {
            return false;
        }

        if (config->wifi_sta_use_static_ip &&
            (!device_web_config_is_valid_ipv4_text(config->wifi_sta_ip) ||
             !device_web_config_is_valid_ipv4_text(config->wifi_sta_gw) ||
             !device_web_config_is_valid_ipv4_text(config->wifi_sta_mask))) {
            return false;
        }
    }

    return true;
}

void device_web_config_get_defaults(device_web_config_t *out_config)
{
    wifi_profile_t wifi_profile;

    if (!out_config) {
        return;
    }

    memset(out_config, 0, sizeof(*out_config));
    wifi_profile = WIFI_ACTIVE_PROFILE;

    out_config->uart0_baud_rate = UART_BAUD_RATE;
    out_config->uart1_baud_rate = UART_BAUD_RATE;
    out_config->video_source = DEVICE_WEB_CONFIG_VIDEO_SOURCE_USB_THERMAL;
    out_config->video_profile = device_web_config_default_video_profile();
    out_config->wifi_mode = DEVICE_WEB_CONFIG_WIFI_MODE_AP;
    out_config->wifi_use_static_ip = wifi_profile.use_static_ip;

    device_web_config_copy_text(out_config->wifi_ap_ssid, sizeof(out_config->wifi_ap_ssid),
                                wifi_profile.ssid);
    device_web_config_copy_text(out_config->wifi_ap_password, sizeof(out_config->wifi_ap_password),
                                wifi_profile.password);
    device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                wifi_profile.ip);
    device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                wifi_profile.gw);
    device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                wifi_profile.mask);
    device_web_config_copy_text(out_config->wifi_sta_ssid, sizeof(out_config->wifi_sta_ssid),
                                WIFI_STA_SSID);
    device_web_config_copy_text(out_config->wifi_sta_password, sizeof(out_config->wifi_sta_password),
                                WIFI_STA_PASSWORD);
    out_config->wifi_sta_use_static_ip = WIFI_STA_USE_STATIC_IP;
    device_web_config_copy_text(out_config->wifi_sta_ip, sizeof(out_config->wifi_sta_ip),
                                WIFI_STA_IP);
    device_web_config_copy_text(out_config->wifi_sta_gw, sizeof(out_config->wifi_sta_gw),
                                WIFI_STA_GW);
    device_web_config_copy_text(out_config->wifi_sta_mask, sizeof(out_config->wifi_sta_mask),
                                WIFI_STA_MASK);
}

bool device_web_config_is_valid_video_profile(uint32_t video_profile)
{
    switch (video_profile) {
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_1280X960:
#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1280X960_BINNING_45FPS
            return true;
#else
            return false;
#endif
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_1920X1080:
#if CONFIG_CAMERA_OV5647_MIPI_RAW10_1920X1080_30FPS
            return true;
#else
            return false;
#endif
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X800:
#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X800_50FPS
            return true;
#else
            return false;
#endif
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X640:
#if CONFIG_CAMERA_OV5647_MIPI_RAW8_800X640_50FPS
            return true;
#else
            return false;
#endif
        default:
            return false;
    }
}

bool device_web_config_is_valid_video_source(uint32_t video_source)
{
    switch (video_source) {
        case DEVICE_WEB_CONFIG_VIDEO_SOURCE_MIPI:
        case DEVICE_WEB_CONFIG_VIDEO_SOURCE_USB_THERMAL:
            return true;
        default:
            return false;
    }
}

bool device_web_config_is_valid_wifi_mode(uint32_t wifi_mode)
{
    switch (wifi_mode) {
        case DEVICE_WEB_CONFIG_WIFI_MODE_AP:
        case DEVICE_WEB_CONFIG_WIFI_MODE_STA:
            return true;
        default:
            return false;
    }
}

bool device_web_config_is_valid_baud_rate(uint32_t baud_rate)
{
    return baud_rate >= DEVICE_WEB_CONFIG_MIN_BAUD_RATE &&
           baud_rate <= DEVICE_WEB_CONFIG_MAX_BAUD_RATE;
}

bool device_web_config_is_valid_ipv4_text(const char *text)
{
    const char *cursor = text;

    if (!text || text[0] == '\0') {
        return false;
    }

    for (int part = 0; part < 4; part++) {
        uint32_t value = 0;
        uint32_t digits = 0;

        if (*cursor < '0' || *cursor > '9') {
            return false;
        }

        while (*cursor >= '0' && *cursor <= '9') {
            value = value * 10U + (uint32_t)(*cursor - '0');
            if (value > 255U) {
                return false;
            }
            digits++;
            cursor++;
        }

        if (digits == 0U) {
            return false;
        }

        if (part == 3) {
            break;
        }

        if (*cursor != '.') {
            return false;
        }
        cursor++;
    }

    return *cursor == '\0';
}

const char *device_web_config_get_video_profile_name(uint32_t video_profile)
{
    switch (video_profile) {
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_1280X960:
            return "1280x960";
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_1920X1080:
            return "1920x1080";
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X800:
            return "800x800";
        case DEVICE_WEB_CONFIG_VIDEO_PROFILE_800X640:
            return "800x640";
        default:
            return "unknown";
    }
}

const char *device_web_config_get_video_source_name(uint32_t video_source)
{
    switch (video_source) {
        case DEVICE_WEB_CONFIG_VIDEO_SOURCE_MIPI:
            return "MIPI 摄像头";
        case DEVICE_WEB_CONFIG_VIDEO_SOURCE_USB_THERMAL:
            return "USB 热像仪";
        default:
            return "unknown";
    }
}

const char *device_web_config_get_wifi_mode_name(uint32_t wifi_mode)
{
    switch (wifi_mode) {
        case DEVICE_WEB_CONFIG_WIFI_MODE_AP:
            return "AP 热点模式";
        case DEVICE_WEB_CONFIG_WIFI_MODE_STA:
            return "STA 入网模式";
        default:
            return "unknown";
    }
}

esp_err_t device_web_config_init(void)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs = 0;
    size_t blob_size = 0;
    device_web_config_storage_t loaded_storage;
    device_web_config_storage_v1_t loaded_storage_v1;
    device_web_config_storage_v2_t loaded_storage_v2;
    device_web_config_storage_v3_t loaded_storage_v3;
    device_web_config_t default_config;
    device_web_config_t imported_config;
    uint32_t imported_flags = 0U;

    if (!s_device_web_config_mutex) {
        s_device_web_config_mutex = xSemaphoreCreateMutex();
        if (!s_device_web_config_mutex) {
            ESP_LOGE(TAG, "创建设备网页配置互斥锁失败");
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(s_device_web_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (s_device_web_config_initialized) {
        xSemaphoreGive(s_device_web_config_mutex);
        return ESP_OK;
    }

    device_web_config_get_defaults(&default_config);
    device_web_config_fill_storage(&s_device_web_config_storage, &default_config);

    /*
     * NVS 中只保存一个 blob，但历史版本结构体大小不同。
     * 先按 blob 大小判断候选版本，再走对应迁移函数，保证老固件升级后仍能读取配置。
     */
    ret = nvs_open(DEVICE_WEB_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret == ESP_OK) {
        memset(&loaded_storage, 0, sizeof(loaded_storage));
        ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG, NULL, &blob_size);
        if (ret == ESP_OK) {
            if (blob_size == sizeof(loaded_storage)) {
                ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                                   &loaded_storage, &blob_size);
            } else if (blob_size == sizeof(loaded_storage_v3)) {
                memset(&loaded_storage_v3, 0, sizeof(loaded_storage_v3));
                ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                                   &loaded_storage_v3, &blob_size);
            } else if (blob_size == sizeof(loaded_storage_v2)) {
                memset(&loaded_storage_v2, 0, sizeof(loaded_storage_v2));
                ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                                   &loaded_storage_v2, &blob_size);
            } else if (blob_size == sizeof(loaded_storage_v1)) {
                memset(&loaded_storage_v1, 0, sizeof(loaded_storage_v1));
                ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                                   &loaded_storage_v1, &blob_size);
            }
        }
        if (ret == ESP_OK) {
            if (blob_size == sizeof(loaded_storage) &&
                loaded_storage.magic == DEVICE_WEB_CONFIG_MAGIC &&
                loaded_storage.version == DEVICE_WEB_CONFIG_VERSION &&
                device_web_config_validate(&loaded_storage.config)) {
                s_device_web_config_storage = loaded_storage;
                ESP_LOGI(TAG, "已加载设备网页配置 | 网络=%s | AP=%s | STA=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                         device_web_config_get_wifi_mode_name(loaded_storage.config.wifi_mode),
                         loaded_storage.config.wifi_ap_ssid,
                         loaded_storage.config.wifi_sta_ssid,
                         device_web_config_get_video_source_name(loaded_storage.config.video_source),
                         device_web_config_get_video_profile_name(loaded_storage.config.video_profile),
                         loaded_storage.config.uart0_baud_rate,
                         loaded_storage.config.uart1_baud_rate);
                ret = ESP_OK;
            } else {
                memset(&imported_config, 0, sizeof(imported_config));
                imported_flags = 0U;
                if (blob_size == sizeof(loaded_storage) &&
                    device_web_config_import_v5(&loaded_storage, &imported_config, &imported_flags)) {
                    device_web_config_fill_storage(&s_device_web_config_storage, &imported_config);
                    s_device_web_config_storage.reserved = imported_flags;
                    ESP_LOGI(TAG, "已迁移 V5 设备网页配置 | 网络=%s | AP=%s | STA=%s | STA待确认=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                             device_web_config_get_wifi_mode_name(imported_config.wifi_mode),
                             imported_config.wifi_ap_ssid,
                             imported_config.wifi_sta_ssid,
                             (imported_flags & DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM) ? "是" : "否",
                             device_web_config_get_video_source_name(imported_config.video_source),
                             device_web_config_get_video_profile_name(imported_config.video_profile),
                             imported_config.uart0_baud_rate,
                             imported_config.uart1_baud_rate);
                    ret = ESP_OK;
                } else if (blob_size == sizeof(loaded_storage) &&
                    device_web_config_import_v4(&loaded_storage, &imported_config)) {
                    device_web_config_fill_storage(&s_device_web_config_storage, &imported_config);
                    if (imported_config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) {
                        s_device_web_config_storage.reserved = DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM;
                    }
                    ESP_LOGI(TAG, "已迁移 V4 设备网页配置 | 网络=%s | AP=%s | STA=%s | STA待确认=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                             device_web_config_get_wifi_mode_name(imported_config.wifi_mode),
                             imported_config.wifi_ap_ssid,
                             imported_config.wifi_sta_ssid,
                             (s_device_web_config_storage.reserved & DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM) ? "是" : "否",
                             device_web_config_get_video_source_name(imported_config.video_source),
                             device_web_config_get_video_profile_name(imported_config.video_profile),
                             imported_config.uart0_baud_rate,
                             imported_config.uart1_baud_rate);
                    ret = ESP_OK;
                } else if (blob_size == sizeof(loaded_storage_v3) &&
                    device_web_config_import_v3(&loaded_storage_v3, &imported_config)) {
                    device_web_config_fill_storage(&s_device_web_config_storage, &imported_config);
                    ESP_LOGI(TAG, "已迁移 V3 设备网页配置 | 网络=%s | AP=%s | STA=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                             device_web_config_get_wifi_mode_name(imported_config.wifi_mode),
                             imported_config.wifi_ap_ssid,
                             imported_config.wifi_sta_ssid,
                             device_web_config_get_video_source_name(imported_config.video_source),
                             device_web_config_get_video_profile_name(imported_config.video_profile),
                             imported_config.uart0_baud_rate,
                             imported_config.uart1_baud_rate);
                    ret = ESP_OK;
                } else if (blob_size == sizeof(loaded_storage_v2) &&
                    device_web_config_import_v2(&loaded_storage_v2, &imported_config)) {
                    device_web_config_fill_storage(&s_device_web_config_storage, &imported_config);
                    ESP_LOGI(TAG, "已迁移 V2 设备网页配置 | 网络=%s | AP=%s | STA=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                             device_web_config_get_wifi_mode_name(imported_config.wifi_mode),
                             imported_config.wifi_ap_ssid,
                             imported_config.wifi_sta_ssid,
                             device_web_config_get_video_source_name(imported_config.video_source),
                             device_web_config_get_video_profile_name(imported_config.video_profile),
                             imported_config.uart0_baud_rate,
                             imported_config.uart1_baud_rate);
                    ret = ESP_OK;
                } else if (blob_size == sizeof(loaded_storage_v1) &&
                    device_web_config_import_legacy(&loaded_storage_v1, &imported_config)) {
                    device_web_config_fill_storage(&s_device_web_config_storage, &imported_config);
                    ESP_LOGI(TAG, "已迁移旧版设备网页配置 | 网络=%s | AP=%s | STA=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                             device_web_config_get_wifi_mode_name(imported_config.wifi_mode),
                             imported_config.wifi_ap_ssid,
                             imported_config.wifi_sta_ssid,
                             device_web_config_get_video_source_name(imported_config.video_source),
                             device_web_config_get_video_profile_name(imported_config.video_profile),
                             imported_config.uart0_baud_rate,
                             imported_config.uart1_baud_rate);
                    ret = ESP_OK;
                } else {
                    ESP_LOGW(TAG, "检测到设备网页配置无效，已回退为默认配置");
                    ret = ESP_OK;
                }
            }
        } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "未发现已保存的设备网页配置，使用默认配置");
            ret = ESP_OK;
        } else {
            ESP_LOGW(TAG, "读取设备网页配置失败: 0x%x (%s)，继续使用默认配置",
                     ret, esp_err_to_name(ret));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "打开设备网页配置命名空间失败: 0x%x (%s)，继续使用默认配置",
                 ret, esp_err_to_name(ret));
    }

    s_device_web_config_initialized = true;
    xSemaphoreGive(s_device_web_config_mutex);
    return ret;
}

void device_web_config_get(device_web_config_t *out_config)
{
    if (!out_config) {
        return;
    }

    if (!s_device_web_config_initialized || !s_device_web_config_mutex) {
        device_web_config_get_defaults(out_config);
        return;
    }

    if (xSemaphoreTake(s_device_web_config_mutex, portMAX_DELAY) != pdTRUE) {
        device_web_config_get_defaults(out_config);
        return;
    }

    *out_config = s_device_web_config_storage.config;
    xSemaphoreGive(s_device_web_config_mutex);
}

static esp_err_t device_web_config_save_with_flags(const device_web_config_t *config,
                                                   uint32_t flags)
{
    esp_err_t ret;
    nvs_handle_t nvs = 0;
    device_web_config_storage_t new_storage;

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_device_web_config_initialized) {
        ret = device_web_config_init();
        if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "初始化设备网页配置模块失败: 0x%x (%s)",
                     ret, esp_err_to_name(ret));
        }
    }

    if (!device_web_config_validate(config)) {
        ESP_LOGW(TAG, "拒绝保存非法设备网页配置");
        return ESP_ERR_INVALID_ARG;
    }

    device_web_config_fill_storage(&new_storage, config);
    new_storage.reserved = flags;

    /* reserved 当前用作运行保护标志位，不放进公开配置结构，避免网页配置和保护状态互相覆盖。 */
    ret = nvs_open(DEVICE_WEB_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开设备网页配置命名空间失败: 0x%x (%s)",
                 ret, esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                       &new_storage, sizeof(new_storage));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存设备网页配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    if (s_device_web_config_mutex &&
        xSemaphoreTake(s_device_web_config_mutex, portMAX_DELAY) == pdTRUE) {
        s_device_web_config_storage = new_storage;
        s_device_web_config_initialized = true;
        xSemaphoreGive(s_device_web_config_mutex);
    }

    ESP_LOGI(TAG,
             "设备网页配置已保存 | 网络=%s | AP=%s | STA=%s | STA待确认=%s | 视频源=%s | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
             device_web_config_get_wifi_mode_name(config->wifi_mode),
             config->wifi_ap_ssid,
             config->wifi_sta_ssid,
             (flags & DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM) ? "是" : "否",
             device_web_config_get_video_source_name(config->video_source),
             device_web_config_get_video_profile_name(config->video_profile),
             config->uart0_baud_rate,
             config->uart1_baud_rate);
    return ESP_OK;
}

esp_err_t device_web_config_save(const device_web_config_t *config)
{
    return device_web_config_save_with_flags(config, 0U);
}

esp_err_t device_web_config_save_sta_pending_confirm(const device_web_config_t *config)
{
    uint32_t flags = 0U;

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) {
        flags |= DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM;
    }

    return device_web_config_save_with_flags(config, flags);
}

bool device_web_config_is_sta_pending_confirm(void)
{
    bool pending = false;

    if (!s_device_web_config_initialized) {
        esp_err_t ret = device_web_config_init();
        if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "读取 STA 待确认状态前初始化配置失败: 0x%x (%s)",
                     ret, esp_err_to_name(ret));
        }
    }

    if (s_device_web_config_mutex &&
        xSemaphoreTake(s_device_web_config_mutex, portMAX_DELAY) == pdTRUE) {
        pending = (s_device_web_config_storage.config.wifi_mode == DEVICE_WEB_CONFIG_WIFI_MODE_STA) &&
                  ((s_device_web_config_storage.reserved & DEVICE_WEB_CONFIG_FLAG_STA_PENDING_CONFIRM) != 0U);
        xSemaphoreGive(s_device_web_config_mutex);
    }

    return pending;
}

esp_err_t device_web_config_clear_sta_pending_confirm(void)
{
    device_web_config_t config = {0};

    device_web_config_get(&config);
    return device_web_config_save_with_flags(&config, 0U);
}

esp_err_t device_web_config_fallback_to_ap(void)
{
    device_web_config_t config = {0};

    device_web_config_get(&config);
    config.wifi_mode = DEVICE_WEB_CONFIG_WIFI_MODE_AP;
    return device_web_config_save_with_flags(&config, 0U);
}

esp_err_t device_web_config_reset_to_factory(void)
{
    esp_err_t ret;
    nvs_handle_t nvs = 0;
    device_web_config_t default_config;

    if (!s_device_web_config_initialized) {
        ret = device_web_config_init();
        if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "初始化设备网页配置模块失败: 0x%x (%s)",
                     ret, esp_err_to_name(ret));
        }
    }

    device_web_config_get_defaults(&default_config);

    ret = nvs_open(DEVICE_WEB_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret == ESP_OK) {
        ret = nvs_erase_key(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ret = ESP_OK;
        }
        if (ret == ESP_OK) {
            ret = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "恢复设备网页默认配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    if (s_device_web_config_mutex &&
        xSemaphoreTake(s_device_web_config_mutex, portMAX_DELAY) == pdTRUE) {
        device_web_config_fill_storage(&s_device_web_config_storage, &default_config);
        s_device_web_config_initialized = true;
        xSemaphoreGive(s_device_web_config_mutex);
    }

    ESP_LOGI(TAG, "设备网页配置已恢复默认值");
    return ESP_OK;
}
