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
#define DEVICE_WEB_CONFIG_VERSION          1U
#define DEVICE_WEB_CONFIG_MIN_BAUD_RATE    1200U
#define DEVICE_WEB_CONFIG_MAX_BAUD_RATE    2000000U

typedef struct {
    uint32_t            magic;
    uint32_t            version;
    device_web_config_t config;
} device_web_config_storage_t;

static SemaphoreHandle_t s_device_web_config_mutex;
static device_web_config_storage_t s_device_web_config_storage;
static bool s_device_web_config_initialized;

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

static bool device_web_config_validate(const device_web_config_t *config)
{
    if (!config) {
        return false;
    }

    if (!device_web_config_is_valid_baud_rate(config->uart0_baud_rate) ||
        !device_web_config_is_valid_baud_rate(config->uart1_baud_rate)) {
        return false;
    }

    if (!device_web_config_is_valid_video_profile(config->video_profile)) {
        return false;
    }

    if (config->wifi_use_static_ip) {
        if (!device_web_config_is_valid_ipv4_text(config->wifi_static_ip) ||
            !device_web_config_is_valid_ipv4_text(config->wifi_static_gw) ||
            !device_web_config_is_valid_ipv4_text(config->wifi_static_mask)) {
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
    out_config->video_profile = device_web_config_default_video_profile();
    out_config->wifi_use_static_ip = wifi_profile.use_static_ip;

    device_web_config_copy_text(out_config->wifi_static_ip, sizeof(out_config->wifi_static_ip),
                                wifi_profile.ip);
    device_web_config_copy_text(out_config->wifi_static_gw, sizeof(out_config->wifi_static_gw),
                                wifi_profile.gw);
    device_web_config_copy_text(out_config->wifi_static_mask, sizeof(out_config->wifi_static_mask),
                                wifi_profile.mask);
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

esp_err_t device_web_config_init(void)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs = 0;
    size_t blob_size = sizeof(device_web_config_storage_t);
    device_web_config_storage_t loaded_storage;
    device_web_config_t default_config;

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

    ret = nvs_open(DEVICE_WEB_CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret == ESP_OK) {
        memset(&loaded_storage, 0, sizeof(loaded_storage));
        ret = nvs_get_blob(nvs, DEVICE_WEB_CONFIG_NVS_KEY_CONFIG,
                           &loaded_storage, &blob_size);
        if (ret == ESP_OK) {
            if (blob_size == sizeof(loaded_storage) &&
                loaded_storage.magic == DEVICE_WEB_CONFIG_MAGIC &&
                loaded_storage.version == DEVICE_WEB_CONFIG_VERSION &&
                device_web_config_validate(&loaded_storage.config)) {
                s_device_web_config_storage = loaded_storage;
                ESP_LOGI(TAG, "已加载设备网页配置 | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32,
                         device_web_config_get_video_profile_name(loaded_storage.config.video_profile),
                         loaded_storage.config.uart0_baud_rate,
                         loaded_storage.config.uart1_baud_rate);
                ret = ESP_OK;
            } else {
                ESP_LOGW(TAG, "检测到设备网页配置无效，已回退为默认配置");
                ret = ESP_OK;
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

esp_err_t device_web_config_save(const device_web_config_t *config)
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
             "设备网页配置已保存 | 分辨率=%s | UART0=%" PRIu32 " | UART1=%" PRIu32 " | 静态IP=%s",
             device_web_config_get_video_profile_name(config->video_profile),
             config->uart0_baud_rate,
             config->uart1_baud_rate,
             config->wifi_use_static_ip ? "开启" : "关闭");
    return ESP_OK;
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
