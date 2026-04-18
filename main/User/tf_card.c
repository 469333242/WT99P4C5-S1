/**
 * @file tf_card.c
 * @brief TF 卡基础驱动实现
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "soc/soc_caps.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl.h"
#include "sd_pwr_ctrl_interface.h"
#endif
#include "tf_card.h"

static const char *TAG = "tf_card";

#define TF_CARD_SPEED_TEST_FILE_NAME  TF_CARD_MOUNT_POINT "/TFTEST.BIN"

#if SOC_SDMMC_IO_POWER_EXTERNAL
typedef struct {
    esp_ldo_channel_handle_t ldo_chan;
    int voltage_mv;
} tf_card_power_ctrl_ctx_t;
#endif

typedef struct {
    bool                        mounted;
    sdmmc_card_t               *card;
    SemaphoreHandle_t           lock;
    tf_card_speed_test_result_t last_speed_test;
#if SOC_SDMMC_IO_POWER_EXTERNAL
    sd_pwr_ctrl_handle_t        pwr_ctrl_handle;
#endif
} tf_card_ctx_t;

static tf_card_ctx_t s_tf;

static esp_err_t tf_card_ensure_lock(void)
{
    if (s_tf.lock) {
        return ESP_OK;
    }

    s_tf.lock = xSemaphoreCreateMutex();
    if (!s_tf.lock) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void tf_card_fill_slot_config(sdmmc_slot_config_t *slot_cfg)
{
    *slot_cfg = (sdmmc_slot_config_t) {
        .clk = TF_CARD_CLK_PIN,
        .cmd = TF_CARD_CMD_PIN,
        .d0 = TF_CARD_D0_PIN,
        .d1 = TF_CARD_D1_PIN,
        .d2 = TF_CARD_D2_PIN,
        .d3 = TF_CARD_D3_PIN,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = TF_CARD_CD_PIN,
        .wp = TF_CARD_WP_PIN,
        .width = TF_CARD_BUS_WIDTH,
        .flags = TF_CARD_ENABLE_INTERNAL_PULLUP ? SDMMC_SLOT_FLAG_INTERNAL_PULLUP : 0,
    };
}

static int tf_card_get_io_voltage_mv(const sdmmc_host_t *host)
{
    int voltage_mv = (int)(host->io_voltage * 1000.0f);

    if (voltage_mv <= 0) {
        voltage_mv = TF_CARD_IO_VOLTAGE_MV;
    }

    return voltage_mv;
}

#if SOC_SDMMC_IO_POWER_EXTERNAL
static esp_err_t tf_card_power_ctrl_set_voltage(void *ctx, int voltage_mv)
{
    tf_card_power_ctrl_ctx_t *pwr_ctx = (tf_card_power_ctrl_ctx_t *)ctx;

    if (!pwr_ctx || !pwr_ctx->ldo_chan) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(esp_ldo_channel_adjust_voltage(pwr_ctx->ldo_chan, voltage_mv),
                        TAG, "调整 TF LDO 电压失败");
    pwr_ctx->voltage_mv = voltage_mv;
    return ESP_OK;
}
#endif

static void tf_card_release_power_ctrl_locked(void)
{
#if SOC_SDMMC_IO_POWER_EXTERNAL
    tf_card_power_ctrl_ctx_t *pwr_ctx = NULL;
    esp_err_t ret = ESP_OK;

    if (!s_tf.pwr_ctrl_handle) {
        return;
    }

    pwr_ctx = (tf_card_power_ctrl_ctx_t *)s_tf.pwr_ctrl_handle->ctx;
    if (pwr_ctx && pwr_ctx->ldo_chan) {
        ret = esp_ldo_release_channel(pwr_ctx->ldo_chan);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "释放 TF 供电控制失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return;
    }

    free(pwr_ctx);
    free(s_tf.pwr_ctrl_handle);
    s_tf.pwr_ctrl_handle = NULL;
#endif
}

static esp_err_t tf_card_prepare_power_ctrl_locked(sdmmc_host_t *host)
{
#if SOC_SDMMC_IO_POWER_EXTERNAL
    esp_err_t ret;
    const int voltage_mv = tf_card_get_io_voltage_mv(host);
    tf_card_power_ctrl_ctx_t *pwr_ctx = NULL;

    if (!s_tf.pwr_ctrl_handle) {
        esp_ldo_channel_config_t ldo_cfg = {
            .chan_id = TF_CARD_LDO_CHANNEL_ID,
            .voltage_mv = voltage_mv,
            .flags.adjustable = true,
        };

        /* P4 板卡在 TF 协商前需要先打开 TF IO 电源。 */
        s_tf.pwr_ctrl_handle = calloc(1, sizeof(sd_pwr_ctrl_drv_t));
        if (!s_tf.pwr_ctrl_handle) {
            ESP_LOGE(TAG, "创建 TF 供电控制句柄失败");
            return ESP_ERR_NO_MEM;
        }

        pwr_ctx = calloc(1, sizeof(tf_card_power_ctrl_ctx_t));
        if (!pwr_ctx) {
            free(s_tf.pwr_ctrl_handle);
            s_tf.pwr_ctrl_handle = NULL;
            ESP_LOGE(TAG, "创建 TF 供电控制上下文失败");
            return ESP_ERR_NO_MEM;
        }

        ret = esp_ldo_acquire_channel(&ldo_cfg, &pwr_ctx->ldo_chan);
        if (ret != ESP_OK) {
            free(pwr_ctx);
            free(s_tf.pwr_ctrl_handle);
            s_tf.pwr_ctrl_handle = NULL;
            ESP_LOGE(TAG, "申请 TF LDO 通道失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            return ret;
        }

        pwr_ctx->voltage_mv = voltage_mv;
        s_tf.pwr_ctrl_handle->set_io_voltage = tf_card_power_ctrl_set_voltage;
        s_tf.pwr_ctrl_handle->ctx = pwr_ctx;
    } else {
        pwr_ctx = (tf_card_power_ctrl_ctx_t *)s_tf.pwr_ctrl_handle->ctx;
    }

    if (!pwr_ctx) {
        tf_card_release_power_ctrl_locked();
        ESP_LOGE(TAG, "TF 供电控制上下文无效");
        return ESP_ERR_INVALID_STATE;
    }

    if (pwr_ctx->voltage_mv != voltage_mv) {
        ret = tf_card_power_ctrl_set_voltage(pwr_ctx, voltage_mv);
        if (ret != ESP_OK) {
            tf_card_release_power_ctrl_locked();
            ESP_LOGE(TAG, "设置 TF IO 电压失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            return ret;
        }
    }

    host->io_voltage = (float)voltage_mv / 1000.0f;
    host->pwr_ctrl_handle = s_tf.pwr_ctrl_handle;
    vTaskDelay(pdMS_TO_TICKS(TF_CARD_POWER_UP_DELAY_MS));
#else
    (void)host;
#endif

    return ESP_OK;
}

static esp_err_t tf_card_get_info_locked(tf_card_info_t *out_info)
{
    esp_err_t ret;
    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;

    if (!out_info) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_tf.mounted || !s_tf.card) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(out_info, 0, sizeof(*out_info));

    ret = esp_vfs_fat_info(TF_CARD_MOUNT_POINT, &total_bytes, &free_bytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取 FATFS 信息失败: 0x%x", ret);
        return ret;
    }

    out_info->mounted = true;
    out_info->card_ok = (sdmmc_get_status(s_tf.card) == ESP_OK);
    out_info->total_bytes = total_bytes;
    out_info->free_bytes = free_bytes;
    out_info->sector_size = (uint32_t)s_tf.card->csd.sector_size;
    out_info->sector_count = (uint64_t)s_tf.card->csd.capacity;
    out_info->max_freq_khz = s_tf.card->max_freq_khz;
    out_info->real_freq_khz = s_tf.card->real_freq_khz;

    memcpy(out_info->card_name, s_tf.card->cid.name, sizeof(out_info->card_name) - 1);
    out_info->card_name[sizeof(out_info->card_name) - 1] = '\0';

    return ESP_OK;
}

static uint32_t bytes_per_sec_to_kbps(uint64_t bytes, int64_t elapsed_us)
{
    if (elapsed_us <= 0) {
        return 0;
    }

    return (uint32_t)((bytes * 1000000ULL) / ((uint64_t)elapsed_us * 1024ULL));
}

esp_err_t tf_card_init(void)
{
    esp_err_t ret;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_cfg;
    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = TF_CARD_MAX_OPEN_FILES,
        .allocation_unit_size = TF_CARD_ALLOC_UNIT_SIZE,
        .disk_status_check_enable = true,
        .use_one_fat = false,
    };

    ESP_RETURN_ON_ERROR(tf_card_ensure_lock(), TAG, "初始化互斥锁失败");

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    if (s_tf.mounted) {
        xSemaphoreGive(s_tf.lock);
        return ESP_OK;
    }

    memset(&s_tf.last_speed_test, 0, sizeof(s_tf.last_speed_test));

    /* Slot1 已给 ESP-Hosted 使用，TF 卡固定走 Slot0。 */
    host.slot = TF_CARD_SDMMC_SLOT;
    host.max_freq_khz = TF_CARD_MAX_FREQ_KHZ;

    tf_card_fill_slot_config(&slot_cfg);
    ret = tf_card_prepare_power_ctrl_locked(&host);
    if (ret != ESP_OK) {
        xSemaphoreGive(s_tf.lock);
        return ret;
    }

    ret = esp_vfs_fat_sdmmc_mount(TF_CARD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_tf.card);
    if (ret != ESP_OK) {
        s_tf.card = NULL;
        tf_card_release_power_ctrl_locked();
        xSemaphoreGive(s_tf.lock);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "TF 卡挂载失败: 0x%x (%s)，请优先检查 TF 供电和 SDMMC 连线",
                     ret, esp_err_to_name(ret));
        } else {
            ESP_LOGE(TAG, "TF 卡挂载失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        }
        return ret;
    }

    s_tf.mounted = true;
    xSemaphoreGive(s_tf.lock);

    ESP_LOGI(TAG, "TF 卡已挂载到 %s", TF_CARD_MOUNT_POINT);
    tf_card_log_status();
    return ESP_OK;
}

esp_err_t tf_card_deinit(void)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(tf_card_ensure_lock(), TAG, "初始化互斥锁失败");

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    if (!s_tf.mounted || !s_tf.card) {
        tf_card_release_power_ctrl_locked();
        xSemaphoreGive(s_tf.lock);
        return ESP_OK;
    }

    ret = esp_vfs_fat_sdcard_unmount(TF_CARD_MOUNT_POINT, s_tf.card);
    if (ret == ESP_OK) {
        s_tf.mounted = false;
        s_tf.card = NULL;
        memset(&s_tf.last_speed_test, 0, sizeof(s_tf.last_speed_test));
        tf_card_release_power_ctrl_locked();
        ESP_LOGI(TAG, "TF 卡已卸载");
    } else {
        ESP_LOGE(TAG, "TF 卡卸载失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }
    xSemaphoreGive(s_tf.lock);

    return ret;
}

bool tf_card_is_mounted(void)
{
    bool mounted = false;

    if (tf_card_ensure_lock() != ESP_OK) {
        return false;
    }

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    mounted = s_tf.mounted;
    xSemaphoreGive(s_tf.lock);

    return mounted;
}

const char *tf_card_get_mount_point(void)
{
    return TF_CARD_MOUNT_POINT;
}

esp_err_t tf_card_get_info(tf_card_info_t *out_info)
{
    esp_err_t ret;

    ESP_RETURN_ON_ERROR(tf_card_ensure_lock(), TAG, "初始化互斥锁失败");

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    ret = tf_card_get_info_locked(out_info);
    xSemaphoreGive(s_tf.lock);

    return ret;
}

esp_err_t tf_card_run_speed_test(tf_card_speed_test_result_t *out_result)
{
    esp_err_t ret;
    tf_card_speed_test_result_t result = {0};
    tf_card_info_t info = {0};
    uint8_t *buf = NULL;
    FILE *fp = NULL;
    uint32_t test_size = TF_CARD_SPEED_TEST_FILE_SIZE;
    uint32_t chunk_size = TF_CARD_SPEED_TEST_BUFFER_SIZE;
    uint32_t remaining;
    size_t io_len;
    int64_t t_start;
    int64_t t_end;

    ESP_RETURN_ON_ERROR(tf_card_ensure_lock(), TAG, "初始化互斥锁失败");

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    ret = tf_card_get_info_locked(&info);
    xSemaphoreGive(s_tf.lock);
    ESP_RETURN_ON_ERROR(ret, TAG, "TF 卡尚未就绪");

    if (info.free_bytes <= (2 * TF_CARD_SPEED_TEST_MIN_FILE_SIZE)) {
        ESP_LOGW(TAG, "TF 卡剩余空间不足，无法执行测速");
        return ESP_ERR_NO_MEM;
    }

    if (info.free_bytes < (uint64_t)(TF_CARD_SPEED_TEST_FILE_SIZE + TF_CARD_SPEED_TEST_MIN_FILE_SIZE)) {
        test_size = (uint32_t)(info.free_bytes / 2);
    }

    test_size = (test_size / chunk_size) * chunk_size;
    if (test_size < TF_CARD_SPEED_TEST_MIN_FILE_SIZE) {
        ESP_LOGW(TAG, "TF 卡可用于测速的空间过小");
        return ESP_ERR_NO_MEM;
    }

    buf = heap_caps_malloc(chunk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(chunk_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "申请测速缓冲区失败");

    for (uint32_t i = 0; i < chunk_size; i++) {
        buf[i] = (uint8_t)(i & 0xFF);
    }

    remove(TF_CARD_SPEED_TEST_FILE_NAME);

    fp = fopen(TF_CARD_SPEED_TEST_FILE_NAME, "wb");
    if (!fp) {
        free(buf);
        ESP_LOGE(TAG, "创建 TF 测速文件失败");
        return ESP_FAIL;
    }

    remaining = test_size;
    t_start = esp_timer_get_time();
    while (remaining > 0) {
        io_len = (remaining > chunk_size) ? chunk_size : remaining;
        if (fwrite(buf, 1, io_len, fp) != io_len) {
            fclose(fp);
            remove(TF_CARD_SPEED_TEST_FILE_NAME);
            free(buf);
            ESP_LOGE(TAG, "写入 TF 测速文件失败");
            return ESP_FAIL;
        }
        remaining -= (uint32_t)io_len;
    }

    fflush(fp);
    fsync(fileno(fp));
    t_end = esp_timer_get_time();
    result.write_speed_kbps = bytes_per_sec_to_kbps(test_size, t_end - t_start);

    fclose(fp);
    fp = fopen(TF_CARD_SPEED_TEST_FILE_NAME, "rb");
    if (!fp) {
        remove(TF_CARD_SPEED_TEST_FILE_NAME);
        free(buf);
        ESP_LOGE(TAG, "打开 TF 测速文件失败");
        return ESP_FAIL;
    }

    remaining = test_size;
    t_start = esp_timer_get_time();
    while (remaining > 0) {
        io_len = (remaining > chunk_size) ? chunk_size : remaining;
        if (fread(buf, 1, io_len, fp) != io_len) {
            fclose(fp);
            remove(TF_CARD_SPEED_TEST_FILE_NAME);
            free(buf);
            ESP_LOGE(TAG, "读取 TF 测速文件失败");
            return ESP_FAIL;
        }
        remaining -= (uint32_t)io_len;
    }
    t_end = esp_timer_get_time();
    result.read_speed_kbps = bytes_per_sec_to_kbps(test_size, t_end - t_start);

    fclose(fp);
    remove(TF_CARD_SPEED_TEST_FILE_NAME);
    free(buf);

    result.valid = true;
    result.file_size_bytes = test_size;
    result.write_speed_too_low = (result.write_speed_kbps < TF_CARD_MIN_WRITE_SPEED_KBPS);
    result.read_speed_too_low = (result.read_speed_kbps < TF_CARD_MIN_READ_SPEED_KBPS);

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    s_tf.last_speed_test = result;
    xSemaphoreGive(s_tf.lock);

    if (out_result) {
        *out_result = result;
    }

    ESP_LOGI(TAG, "TF 卡测速完成 | 文件: %u 字节 | 写入: %u KB/s | 读取: %u KB/s",
             result.file_size_bytes, result.write_speed_kbps, result.read_speed_kbps);
    if (result.write_speed_too_low || result.read_speed_too_low) {
        ESP_LOGW(TAG, "TF 卡速度低于阈值 | 写入阈值: %u KB/s | 读取阈值: %u KB/s",
                 TF_CARD_MIN_WRITE_SPEED_KBPS, TF_CARD_MIN_READ_SPEED_KBPS);
    }

    return ESP_OK;
}

void tf_card_log_status(void)
{
    tf_card_info_t info = {0};
    tf_card_speed_test_result_t speed = {0};
    esp_err_t ret;

    ret = tf_card_get_info(&info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TF 卡未挂载，跳过状态打印");
        return;
    }

    xSemaphoreTake(s_tf.lock, portMAX_DELAY);
    speed = s_tf.last_speed_test;
    xSemaphoreGive(s_tf.lock);

    ESP_LOGI(TAG, "TF 状态 | 名称: %s | 总容量: %" PRIu64 " MB | 剩余: %" PRIu64 " MB",
             info.card_name[0] ? info.card_name : "未知",
             info.total_bytes / (1024ULL * 1024ULL),
             info.free_bytes / (1024ULL * 1024ULL));
    ESP_LOGI(TAG, "TF 参数 | 扇区数: %" PRIu64 " | 扇区大小: %" PRIu32 " | 实际频率: %" PRIi32 " kHz",
             info.sector_count, info.sector_size, info.real_freq_khz);

    if (speed.valid) {
        ESP_LOGI(TAG, "TF 最近测速 | 写入: %u KB/s | 读取: %u KB/s | 低于阈值(写/读)=%d/%d",
                 speed.write_speed_kbps, speed.read_speed_kbps,
                 speed.write_speed_too_low, speed.read_speed_too_low);
    }
}
