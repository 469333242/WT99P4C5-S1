/**
 * @file tf_card.h
 * @brief TF 卡基础驱动接口
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/sdmmc_default_configs.h"
#include "driver/sdmmc_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* WT99P4C5-S1 的 TF 卡硬件连接配置。
 * Slot1 已被 ESP-Hosted 占用，因此 TF 卡默认使用 Slot0。
 */
#define TF_CARD_SDMMC_SLOT              SDMMC_HOST_SLOT_0
#define TF_CARD_CLK_PIN                 GPIO_NUM_43
#define TF_CARD_CMD_PIN                 GPIO_NUM_44
#define TF_CARD_D0_PIN                  GPIO_NUM_39
#define TF_CARD_D1_PIN                  GPIO_NUM_40
#define TF_CARD_D2_PIN                  GPIO_NUM_41
#define TF_CARD_D3_PIN                  GPIO_NUM_42
#define TF_CARD_CD_PIN                  GPIO_NUM_NC
#define TF_CARD_WP_PIN                  GPIO_NUM_NC
#define TF_CARD_BUS_WIDTH               4
#define TF_CARD_MAX_FREQ_KHZ            SDMMC_FREQ_DEFAULT
#define TF_CARD_IO_VOLTAGE_MV           3300
#define TF_CARD_ENABLE_INTERNAL_PULLUP  1
#define TF_CARD_LDO_CHANNEL_ID          4
#define TF_CARD_POWER_UP_DELAY_MS       20

#define TF_CARD_MOUNT_POINT             "/sdcard"

/* 驱动参数 */
#define TF_CARD_MAX_OPEN_FILES              8
#define TF_CARD_ALLOC_UNIT_SIZE             (16 * 1024)
#define TF_CARD_SPEED_TEST_BUFFER_SIZE      (64 * 1024)
#define TF_CARD_SPEED_TEST_FILE_SIZE        (4 * 1024 * 1024)
#define TF_CARD_SPEED_TEST_MIN_FILE_SIZE    (1 * 1024 * 1024)
#define TF_CARD_MIN_WRITE_SPEED_KBPS        1024
#define TF_CARD_MIN_READ_SPEED_KBPS         1024

typedef struct {
    bool     mounted;
    bool     card_ok;
    uint64_t total_bytes;
    uint64_t free_bytes;
    uint32_t sector_size;
    uint64_t sector_count;
    uint32_t max_freq_khz;
    int32_t  real_freq_khz;
    char     card_name[8];
} tf_card_info_t;

typedef struct {
    bool     valid;
    uint32_t file_size_bytes;
    uint32_t write_speed_kbps;
    uint32_t read_speed_kbps;
    bool     write_speed_too_low;
    bool     read_speed_too_low;
} tf_card_speed_test_result_t;

esp_err_t tf_card_init(void);
esp_err_t tf_card_deinit(void);
bool tf_card_is_mounted(void);
const char *tf_card_get_mount_point(void);
esp_err_t tf_card_get_info(tf_card_info_t *out_info);
esp_err_t tf_card_run_speed_test(tf_card_speed_test_result_t *out_result);
void tf_card_log_status(void);

#ifdef __cplusplus
}
#endif
