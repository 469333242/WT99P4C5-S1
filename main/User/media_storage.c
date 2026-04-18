/**
 * @file media_storage.c
 * @brief 媒体存储模块实现
 *
 * 当前实现照片保存功能：
 *   - RTSP 开始推流后，请求一次自动拍照
 *   - 复用 RTSP 当前 YUV420 帧，先拷贝到独立缓冲，避免后台处理阻塞采集缓冲归还
 *   - 后台任务使用 PPA 硬件将 YUV420 转为 RGB565
 *   - 调用官方 JPEG 硬件编码 API 将 RGB565 压缩为 JPEG
 *   - 按约定目录规则写入 TF 卡
 *
 * 目录规则：
 *   /sdcard/001_19800106/photo/1980-01-06T00-00-01-234.jpeg
 *
 * 其中：
 *   - 001 为历史“上电且实际触发拍照保存”的次序号
 *   - 同一次上电首次拍照确定根目录日期，后续继续复用同一目录
 *   - 若系统时间尚未同步，目录日期固定回退为 19800106
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/jpeg_encode.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "media_storage.h"
#include "tf_card.h"

static const char *TAG = "media_storage";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define MEDIA_STORAGE_NVS_NAMESPACE      "media_storage"
#define MEDIA_STORAGE_NVS_KEY_BOOT_SEQ   "boot_seq"
#define MEDIA_STORAGE_TIMEZONE           "CST-8"
#define MEDIA_STORAGE_VALID_UNIX_SEC     1704067200LL  /* 2024-01-01 00:00:00 UTC */
#define MEDIA_STORAGE_DEFAULT_DATE_TAG   "19800106"
#define MEDIA_STORAGE_DEFAULT_DATE_TEXT  "1980-01-06"
#define MEDIA_STORAGE_PHOTO_SUBDIR       "photo"
#define MEDIA_STORAGE_JPEG_QUALITY       80
#define MEDIA_STORAGE_JPEG_TIMEOUT_MS    5000
#define MEDIA_STORAGE_TASK_STACK_SIZE    (10 * 1024)
#define MEDIA_STORAGE_TASK_PRIORITY      5
#define MEDIA_STORAGE_TASK_CORE          0
#define MEDIA_STORAGE_DMA_ALIGN          64
#define MEDIA_STORAGE_MAX_PATH_LEN       192
#define MEDIA_STORAGE_DATE_TAG_LEN       9
#define MEDIA_STORAGE_TIMESTAMP_LEN      32

/* ------------------------------------------------------------------ */
/* 内部状态                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    bool                  initialized;
    bool                  session_ready;
    bool                  auto_photo_pending;
    bool                  photo_job_busy;

    TaskHandle_t          task_handle;
    ppa_client_handle_t   ppa_client;
    jpeg_encoder_handle_t jpeg_handle;

    uint32_t              session_seq;
    uint32_t              photo_count;
    char                  session_date_tag[MEDIA_STORAGE_DATE_TAG_LEN];
    char                  session_root_dir[MEDIA_STORAGE_MAX_PATH_LEN];
    char                  photo_dir[MEDIA_STORAGE_MAX_PATH_LEN];

    uint8_t              *yuv420_buf;
    size_t                yuv420_buf_size;
    uint8_t              *rgb565_buf;
    size_t                rgb565_buf_size;
    uint8_t              *jpeg_out_buf;
    size_t                jpeg_out_buf_size;

    uint32_t              frame_width;
    uint32_t              frame_height;
    size_t                frame_len;
} media_storage_ctx_t;

static media_storage_ctx_t s_media;
static portMUX_TYPE s_media_lock = portMUX_INITIALIZER_UNLOCKED;
static void media_storage_photo_task(void *arg);

/* ------------------------------------------------------------------ */
/* 内部工具函数                                                        */
/* ------------------------------------------------------------------ */
static size_t media_storage_align_size(size_t size)
{
    return (size + MEDIA_STORAGE_DMA_ALIGN - 1U) & ~(MEDIA_STORAGE_DMA_ALIGN - 1U);
}

static size_t media_storage_yuv420_size(uint32_t width, uint32_t height)
{
    return (size_t)width * (size_t)height * 3U / 2U;
}

static size_t media_storage_rgb565_size(uint32_t width, uint32_t height)
{
    return (size_t)width * (size_t)height * 2U;
}

static esp_err_t media_storage_append_text(char *dst, size_t dst_size,
                                           size_t *offset, const char *text)
{
    size_t text_len;

    if (!dst || dst_size == 0 || !offset || !text) {
        return ESP_ERR_INVALID_ARG;
    }

    text_len = strlen(text);
    if (*offset + text_len >= dst_size) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(dst + *offset, text, text_len);
    *offset += text_len;
    dst[*offset] = '\0';
    return ESP_OK;
}

static esp_err_t media_storage_copy_text(char *dst, size_t dst_size, const char *text)
{
    size_t offset = 0;

    if (!dst || dst_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    return media_storage_append_text(dst, dst_size, &offset, text);
}

static esp_err_t media_storage_join_path(char *dst, size_t dst_size,
                                         const char *dir, const char *name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !dir || !name) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = media_storage_append_text(dst, dst_size, &offset, dir);
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, name);
    }

    return ret;
}

static esp_err_t media_storage_build_session_root(char *dst, size_t dst_size,
                                                  const char *mount_point,
                                                  uint32_t seq, const char *date_tag)
{
    int seq_len;
    char seq_text[16] = {0};
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !mount_point || !date_tag) {
        return ESP_ERR_INVALID_ARG;
    }

    seq_len = snprintf(seq_text, sizeof(seq_text), "%03" PRIu32, seq);
    if (seq_len < 0 || seq_len >= (int)sizeof(seq_text)) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = media_storage_append_text(dst, dst_size, &offset, mount_point);
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, seq_text);
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "_");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, date_tag);
    }

    return ret;
}

static esp_err_t media_storage_build_photo_file_path(char *dst, size_t dst_size,
                                                     const char *photo_dir,
                                                     const char *timestamp)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !photo_dir || !timestamp) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = media_storage_append_text(dst, dst_size, &offset, photo_dir);
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, timestamp);
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, ".jpeg");
    }

    return ret;
}

static bool media_storage_try_take_photo_request(void)
{
    bool accepted = false;

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.initialized && s_media.auto_photo_pending && !s_media.photo_job_busy) {
        s_media.auto_photo_pending = false;
        s_media.photo_job_busy = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&s_media_lock);
    return accepted;
}

static void media_storage_finish_photo_job(void)
{
    portENTER_CRITICAL(&s_media_lock);
    s_media.photo_job_busy = false;
    portEXIT_CRITICAL(&s_media_lock);
}

static void media_storage_cancel_photo_job(bool retry)
{
    portENTER_CRITICAL(&s_media_lock);
    s_media.photo_job_busy = false;
    if (retry) {
        s_media.auto_photo_pending = true;
    }
    portEXIT_CRITICAL(&s_media_lock);
}

static void media_storage_set_timezone_once(void)
{
    static bool s_tz_ready = false;

    if (s_tz_ready) {
        return;
    }

    setenv("TZ", MEDIA_STORAGE_TIMEZONE, 1);
    tzset();
    s_tz_ready = true;
}

static bool media_storage_is_time_valid(time_t unix_sec)
{
    return unix_sec >= MEDIA_STORAGE_VALID_UNIX_SEC;
}

static void media_storage_build_timestamp(char *date_tag, size_t date_tag_size,
                                          char *timestamp, size_t timestamp_size)
{
    struct timeval tv = {0};

    gettimeofday(&tv, NULL);

    if (media_storage_is_time_valid((time_t)tv.tv_sec)) {
        struct tm tm_info = {0};
        uint32_t msec = (uint32_t)(tv.tv_usec / 1000U);
        time_t sec = (time_t)tv.tv_sec;

        localtime_r(&sec, &tm_info);

        snprintf(date_tag, date_tag_size, "%04d%02d%02d",
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday);
        snprintf(timestamp, timestamp_size, "%04d-%02d-%02dT%02d-%02d-%02d-%03" PRIu32,
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, msec);
    } else {
        uint64_t boot_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
        uint32_t msec = (uint32_t)(boot_ms % 1000ULL);
        uint32_t total_sec = (uint32_t)((boot_ms / 1000ULL) % 86400ULL);
        uint32_t hour = total_sec / 3600U;
        uint32_t minute = (total_sec % 3600U) / 60U;
        uint32_t second = total_sec % 60U;

        snprintf(date_tag, date_tag_size, "%s", MEDIA_STORAGE_DEFAULT_DATE_TAG);
        snprintf(timestamp, timestamp_size, "%sT%02" PRIu32 "-%02" PRIu32 "-%02" PRIu32 "-%03" PRIu32,
                 MEDIA_STORAGE_DEFAULT_DATE_TEXT, hour, minute, second, msec);
    }
}

static esp_err_t media_storage_ensure_dir(const char *path)
{
    if (!path || path[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (mkdir(path, 0775) == 0 || errno == EEXIST) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "创建目录失败: %s, errno=%d", path, errno);
    return ESP_FAIL;
}

static esp_err_t media_storage_alloc_session_seq(uint32_t *out_seq)
{
    esp_err_t ret;
    nvs_handle_t nvs = 0;
    uint32_t seq = 0;

    if (!out_seq) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = nvs_open(MEDIA_STORAGE_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开 NVS 命名空间失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_get_u32(nvs, MEDIA_STORAGE_NVS_KEY_BOOT_SEQ, &seq);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "读取历史拍照次序失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        nvs_close(nvs);
        return ret;
    }

    seq += 1;

    ret = nvs_set_u32(nvs, MEDIA_STORAGE_NVS_KEY_BOOT_SEQ, seq);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写入历史拍照次序失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    *out_seq = seq;
    return ESP_OK;
}

static esp_err_t media_storage_prepare_session(const char *date_tag)
{
    esp_err_t ret;
    uint32_t seq = 0;

    if (s_media.session_ready) {
        return ESP_OK;
    }

    if (!date_tag || date_tag[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    ret = media_storage_alloc_session_seq(&seq);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = media_storage_build_session_root(s_media.session_root_dir,
                                           sizeof(s_media.session_root_dir),
                                           tf_card_get_mount_point(), seq, date_tag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "照片根目录路径过长");
        return ret;
    }

    ret = media_storage_join_path(s_media.photo_dir, sizeof(s_media.photo_dir),
                                  s_media.session_root_dir, MEDIA_STORAGE_PHOTO_SUBDIR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "照片目录路径过长");
        return ret;
    }

    ret = media_storage_ensure_dir(s_media.session_root_dir);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = media_storage_ensure_dir(s_media.photo_dir);
    if (ret != ESP_OK) {
        return ret;
    }

    s_media.session_seq = seq;
    ret = media_storage_copy_text(s_media.session_date_tag,
                                  sizeof(s_media.session_date_tag), date_tag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "照片目录日期标签过长");
        return ret;
    }
    s_media.session_ready = true;

    ESP_LOGI(TAG, "本次上电照片目录已确定: %s", s_media.session_root_dir);
    return ESP_OK;
}

static void media_storage_free_photo_buffers(void)
{
    if (s_media.yuv420_buf) {
        free(s_media.yuv420_buf);
        s_media.yuv420_buf = NULL;
    }
    if (s_media.rgb565_buf) {
        free(s_media.rgb565_buf);
        s_media.rgb565_buf = NULL;
    }
    if (s_media.jpeg_out_buf) {
        free(s_media.jpeg_out_buf);
        s_media.jpeg_out_buf = NULL;
    }

    s_media.yuv420_buf_size = 0;
    s_media.rgb565_buf_size = 0;
    s_media.jpeg_out_buf_size = 0;
    s_media.frame_width = 0;
    s_media.frame_height = 0;
    s_media.frame_len = 0;
}

static esp_err_t media_storage_prepare_ppa_client(void)
{
    ppa_client_config_t ppa_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
        /* 抓拍属于低频后台任务，限制 PPA burst，尽量减少与 RTSP/H.264 的总线竞争。 */
        .data_burst_length = PPA_DATA_BURST_LENGTH_64,
    };

    if (s_media.ppa_client) {
        return ESP_OK;
    }

    esp_err_t ret = ppa_register_client(&ppa_config, &s_media.ppa_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 PPA 照片转换客户端失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t media_storage_prepare_jpeg_encoder(void)
{
    jpeg_encode_engine_cfg_t eng_cfg = {
        .intr_priority = 0,
        .timeout_ms = MEDIA_STORAGE_JPEG_TIMEOUT_MS,
    };

    if (s_media.jpeg_handle) {
        return ESP_OK;
    }

    esp_err_t ret = jpeg_new_encoder_engine(&eng_cfg, &s_media.jpeg_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建 JPEG 硬件编码器失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t media_storage_ensure_photo_task(void)
{
    TaskHandle_t task_handle = NULL;

    if (s_media.task_handle) {
        return ESP_OK;
    }

    if (xTaskCreatePinnedToCore(media_storage_photo_task, "media_photo",
                                MEDIA_STORAGE_TASK_STACK_SIZE, NULL,
                                MEDIA_STORAGE_TASK_PRIORITY,
                                &task_handle,
                                MEDIA_STORAGE_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "创建照片保存后台任务失败");
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.task_handle == NULL) {
        s_media.task_handle = task_handle;
        task_handle = NULL;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (task_handle) {
        vTaskDelete(task_handle);
    }

    return ESP_OK;
}

static esp_err_t media_storage_convert_yuv420_to_rgb565(uint32_t width, uint32_t height)
{
    esp_err_t ret = media_storage_prepare_ppa_client();
    if (ret != ESP_OK) {
        return ret;
    }

    ppa_srm_oper_config_t oper_config = {
        .in = {
            .buffer = s_media.yuv420_buf,
            .pic_w = width,
            .pic_h = height,
            .block_w = width,
            .block_h = height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
            .yuv_range = PPA_COLOR_RANGE_FULL,
            .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
        },
        .out = {
            .buffer = s_media.rgb565_buf,
            .buffer_size = s_media.rgb565_buf_size,
            .pic_w = width,
            .pic_h = height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1.0f,
        .scale_y = 1.0f,
        .mirror_x = false,
        .mirror_y = false,
        .alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    ret = ppa_do_scale_rotate_mirror(s_media.ppa_client, &oper_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PPA 硬件转换 YUV420->RGB565 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t media_storage_encode_rgb565_to_jpeg(uint32_t width, uint32_t height,
                                                     uint32_t *out_jpeg_len)
{
    jpeg_encode_cfg_t jpeg_cfg = {
        .height = height,
        .width = width,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = MEDIA_STORAGE_JPEG_QUALITY,
    };
    uint32_t jpeg_len = 0;
    size_t rgb565_len = media_storage_rgb565_size(width, height);

    if (!out_jpeg_len) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = media_storage_prepare_jpeg_encoder();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = jpeg_encoder_process(s_media.jpeg_handle, &jpeg_cfg,
                               s_media.rgb565_buf, (uint32_t)rgb565_len,
                               s_media.jpeg_out_buf, (uint32_t)s_media.jpeg_out_buf_size,
                               &jpeg_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG 硬件编码失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    if (jpeg_len == 0 || jpeg_len > s_media.jpeg_out_buf_size) {
        ESP_LOGE(TAG, "JPEG 输出长度异常: %" PRIu32, jpeg_len);
        return ESP_FAIL;
    }

    *out_jpeg_len = jpeg_len;
    return ESP_OK;
}

static esp_err_t media_storage_write_file(const char *path, const uint8_t *data, size_t len)
{
    FILE *fp = NULL;

    if (!path || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    fp = fopen(path, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "打开照片文件失败: %s, errno=%d", path, errno);
        return ESP_FAIL;
    }

    if (fwrite(data, 1, len, fp) != len) {
        fclose(fp);
        unlink(path);
        ESP_LOGE(TAG, "写入照片文件失败: %s, errno=%d", path, errno);
        return ESP_FAIL;
    }

    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    return ESP_OK;
}

static void media_storage_photo_task(void *arg)
{
    (void)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t width = s_media.frame_width;
        uint32_t height = s_media.frame_height;
        uint32_t jpeg_len = 0;
        char date_tag[MEDIA_STORAGE_DATE_TAG_LEN] = {0};
        char timestamp[MEDIA_STORAGE_TIMESTAMP_LEN] = {0};
        char file_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};

        if (!tf_card_is_mounted()) {
            ESP_LOGW(TAG, "TF 卡未挂载，跳过本次自动拍照");
            media_storage_finish_photo_job();
            continue;
        }

        media_storage_build_timestamp(date_tag, sizeof(date_tag), timestamp, sizeof(timestamp));

        esp_err_t ret = media_storage_convert_yuv420_to_rgb565(width, height);
        if (ret == ESP_OK) {
            ret = media_storage_encode_rgb565_to_jpeg(width, height, &jpeg_len);
        }
        if (ret == ESP_OK) {
            ret = media_storage_prepare_session(date_tag);
        }
        if (ret == ESP_OK) {
            ret = media_storage_build_photo_file_path(file_path, sizeof(file_path),
                                                      s_media.photo_dir, timestamp);
        }
        if (ret == ESP_OK) {
            ret = media_storage_write_file(file_path, s_media.jpeg_out_buf, jpeg_len);
        }

        if (ret == ESP_OK) {
            s_media.photo_count++;
            ESP_LOGI(TAG, "照片保存成功 | 次序=%03" PRIu32 " | 本次上电第 %" PRIu32 " 张 | 路径=%s",
                     s_media.session_seq, s_media.photo_count, file_path);
        } else {
            ESP_LOGE(TAG, "照片保存失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        }

        media_storage_finish_photo_job();
    }
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
esp_err_t media_storage_init(void)
{
    if (s_media.initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(tf_card_is_mounted(), ESP_ERR_INVALID_STATE, TAG,
                        "TF 卡未挂载，无法初始化媒体存储模块");

    memset(&s_media, 0, sizeof(s_media));
    media_storage_set_timezone_once();
    s_media.initialized = true;
    ESP_LOGI(TAG, "media storage init ok, photo pipeline lazy create");
    return ESP_OK;
}

void media_storage_deinit(void)
{
    if (s_media.task_handle) {
        vTaskDelete(s_media.task_handle);
        s_media.task_handle = NULL;
    }

    if (s_media.jpeg_handle) {
        jpeg_del_encoder_engine(s_media.jpeg_handle);
        s_media.jpeg_handle = NULL;
    }

    if (s_media.ppa_client) {
        ppa_unregister_client(s_media.ppa_client);
        s_media.ppa_client = NULL;
    }

    media_storage_free_photo_buffers();
    memset(&s_media, 0, sizeof(s_media));
}

esp_err_t media_storage_prepare_photo_buffers(uint32_t width, uint32_t height)
{
    size_t yuv420_size;
    size_t rgb565_size;
    size_t jpeg_out_size;
    jpeg_encode_memory_alloc_cfg_t out_mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };

    if (!s_media.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (width == 0 || height == 0 || (width % 2U) != 0 || (height % 2U) != 0) {
        ESP_LOGE(TAG, "照片缓冲分辨率非法: %" PRIu32 "x%" PRIu32, width, height);
        return ESP_ERR_INVALID_ARG;
    }

    yuv420_size = media_storage_yuv420_size(width, height);
    rgb565_size = media_storage_rgb565_size(width, height);
    jpeg_out_size = rgb565_size;

    if (s_media.yuv420_buf && s_media.rgb565_buf && s_media.jpeg_out_buf &&
        s_media.frame_width == width && s_media.frame_height == height &&
        s_media.yuv420_buf_size >= yuv420_size &&
        s_media.rgb565_buf_size >= rgb565_size &&
        s_media.jpeg_out_buf_size >= jpeg_out_size) {
        return ESP_OK;
    }

    media_storage_free_photo_buffers();

    s_media.yuv420_buf_size = media_storage_align_size(yuv420_size);
    s_media.rgb565_buf_size = media_storage_align_size(rgb565_size);

    s_media.yuv420_buf = (uint8_t *)heap_caps_aligned_calloc(MEDIA_STORAGE_DMA_ALIGN, 1,
                                                             s_media.yuv420_buf_size,
                                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    s_media.rgb565_buf = (uint8_t *)heap_caps_aligned_calloc(MEDIA_STORAGE_DMA_ALIGN, 1,
                                                             s_media.rgb565_buf_size,
                                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    s_media.jpeg_out_buf = (uint8_t *)jpeg_alloc_encoder_mem(jpeg_out_size, &out_mem_cfg,
                                                             &s_media.jpeg_out_buf_size);

    if (!s_media.yuv420_buf || !s_media.rgb565_buf || !s_media.jpeg_out_buf) {
        ESP_LOGE(TAG, "申请照片缓冲失败 | YUV=%zu | RGB565=%zu | JPEG=%zu",
                 s_media.yuv420_buf_size, s_media.rgb565_buf_size, jpeg_out_size);
        media_storage_free_photo_buffers();
        return ESP_ERR_NO_MEM;
    }

    s_media.frame_width = width;
    s_media.frame_height = height;

    ESP_LOGI(TAG, "照片缓冲已准备 | %" PRIu32 "x%" PRIu32 " | YUV=%zu | RGB565=%zu | JPEG=%zu",
             width, height, s_media.yuv420_buf_size, s_media.rgb565_buf_size,
             s_media.jpeg_out_buf_size);
    return ESP_OK;
}

void media_storage_request_auto_photo(void)
{
    esp_err_t ret;

    if (!s_media.initialized) {
        return;
    }

    portENTER_CRITICAL(&s_media_lock);
    s_media.auto_photo_pending = true;
    portEXIT_CRITICAL(&s_media_lock);
    ret = media_storage_ensure_photo_task();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "照片后台任务暂未就绪，收到视频帧后会继续重试: 0x%x", ret);
    }

    ESP_LOGI(TAG, "已收到自动拍照请求，等待下一帧图像");
}

void media_storage_process_camera_frame(const uint8_t *yuv420_buf, size_t yuv420_len,
                                        uint32_t width, uint32_t height)
{
    size_t expected_len;

    if (!s_media.initialized || !yuv420_buf ||
        width == 0 || height == 0) {
        return;
    }

    if (!media_storage_try_take_photo_request()) {
        return;
    }

    if (media_storage_ensure_photo_task() != ESP_OK) {
        media_storage_cancel_photo_job(true);
        return;
    }

    expected_len = media_storage_yuv420_size(width, height);
    if (yuv420_len != 0 && yuv420_len < expected_len) {
        ESP_LOGE(TAG, "YUV420 帧长度不足: %zu < %zu", yuv420_len, expected_len);
        media_storage_cancel_photo_job(false);
        return;
    }

    if (media_storage_prepare_photo_buffers(width, height) != ESP_OK) {
        media_storage_cancel_photo_job(false);
        return;
    }

    /* 摄像头采集缓冲会很快归还给 V4L2，这里只做一次帧拷贝，后续硬件转换/编码/写卡全部后台执行。 */
    memcpy(s_media.yuv420_buf, yuv420_buf, expected_len);
    s_media.frame_width = width;
    s_media.frame_height = height;
    s_media.frame_len = expected_len;

    xTaskNotifyGive(s_media.task_handle);
}
