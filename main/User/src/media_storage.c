/**
 * @file media_storage.c
 * @brief 媒体存储模块实现
 *
 * 当前实现包含两条后台存储链路：
 *   - 照片链路：YUV420 -> PPA RGB565 -> JPEG 硬件编码 -> TF 卡
 *   - 录像链路：复用现有 H.264 硬编码帧 -> MP4 封装 -> TF 卡
 *
 * 设计原则：
 *   - 摄像头线程不做文件写入，不做 MP4 封装，不等待 SD 卡
 *   - 录像旁路仅做非阻塞 memcpy + 入队，队列满时直接丢帧
 *   - MP4 目标每段 2 分钟，但只在下一帧 IDR 处切段
 *   - 照片与录像共用同一次上电会话目录
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/jpeg_encode.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_h264_types.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "media_mp4_writer.h"
#include "media_storage.h"
#include "tf_card.h"

static const char *TAG = "media_storage";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define MEDIA_STORAGE_NVS_NAMESPACE           "media_storage"
#define MEDIA_STORAGE_NVS_KEY_BOOT_SEQ        "boot_seq"
#define MEDIA_STORAGE_TIMEZONE                "CST-8"
#define MEDIA_STORAGE_VALID_UNIX_SEC          1704067200LL  /* 2024-01-01 00:00:00 UTC */
#define MEDIA_STORAGE_DEFAULT_DATE_TAG        "19800106"
#define MEDIA_STORAGE_DEFAULT_DATE_TEXT       "1980-01-06"
#define MEDIA_STORAGE_UNIX_MSEC_PER_SEC       1000LL
#define MEDIA_STORAGE_PHOTO_SUBDIR            "photo"
#define MEDIA_STORAGE_VIDEO_SUBDIR            "video"
#define MEDIA_STORAGE_JPEG_QUALITY            80
#define MEDIA_STORAGE_JPEG_TIMEOUT_MS         5000
#define MEDIA_STORAGE_PHOTO_TASK_STACK_SIZE   (10 * 1024)
#define MEDIA_STORAGE_PHOTO_TASK_PRIORITY     5
#define MEDIA_STORAGE_PHOTO_TASK_CORE         0
#define MEDIA_STORAGE_VIDEO_TASK_STACK_SIZE   (12 * 1024)
#define MEDIA_STORAGE_VIDEO_TASK_PRIORITY     8
#define MEDIA_STORAGE_VIDEO_TASK_CORE         0
#define MEDIA_STORAGE_DMA_ALIGN               64
#define MEDIA_STORAGE_MAX_PATH_LEN            192
#define MEDIA_STORAGE_DATE_TAG_LEN            9
#define MEDIA_STORAGE_TIMESTAMP_LEN           32
#define MEDIA_STORAGE_VIDEO_QUEUE_LEN         60
#define MEDIA_STORAGE_VIDEO_QUEUE_LEN_1080P   12
#define MEDIA_STORAGE_VIDEO_QUEUE_LEN_SXGA    30
#define MEDIA_STORAGE_VIDEO_WAIT_MS           200
#define MEDIA_STORAGE_VIDEO_SEGMENT_SEC       120U
#define MEDIA_STORAGE_VIDEO_SEGMENT_US        ((int64_t)MEDIA_STORAGE_VIDEO_SEGMENT_SEC * 1000000LL)
#define MEDIA_STORAGE_VIDEO_TIMESCALE         90000U
#define MEDIA_STORAGE_VIDEO_FRAME_BUF_SIZE    (128U * 1024U)
#define MEDIA_STORAGE_VIDEO_DROP_LOG_MS       1000
#define MEDIA_STORAGE_VIDEO_SAVE_GOP_INTERVAL 1U
#define MEDIA_STORAGE_VIDEO_ADAPTIVE_INTERVAL_MAX 1U
#define MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS   0U
#define MEDIA_STORAGE_VIDEO_RECOVERY_GOPS        2U
#define MEDIA_STORAGE_VIDEO_TASK_YIELD_FRAMES 120U
#define MEDIA_STORAGE_VIDEO_NON_IDR_SKIP_MOD_HIGH 3U
#define MEDIA_STORAGE_VIDEO_NON_IDR_SKIP_MOD_CRITICAL 2U
#define MEDIA_STORAGE_VIDEO_NALU_TYPE_MASK    0x1F
#define MEDIA_STORAGE_VIDEO_NALU_TYPE_SPS     7
#define MEDIA_STORAGE_VIDEO_NALU_TYPE_PPS     8
#define MEDIA_STORAGE_TF_MIN_PHOTO_FREE_BYTES      (2ULL * 1024ULL * 1024ULL)
#define MEDIA_STORAGE_TF_MIN_VIDEO_START_FREE_BYTES (12ULL * 1024ULL * 1024ULL)
#define MEDIA_STORAGE_TF_MIN_VIDEO_RESUME_FREE_BYTES (16ULL * 1024ULL * 1024ULL)
#define MEDIA_STORAGE_TF_VIDEO_START_GUARD_SEC     15ULL
#define MEDIA_STORAGE_TF_VIDEO_RESUME_GUARD_SEC    20ULL

/* ------------------------------------------------------------------ */
/* 内部类型                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    size_t offset;
    size_t len;
    uint8_t type;
} media_storage_nalu_info_t;

typedef struct {
    uint8_t *buf;
    size_t capacity;
    size_t len;
    esp_h264_frame_type_t frame_type;
    uint32_t pts;
} media_storage_video_frame_t;

typedef struct {
    bool                  initialized;
    bool                  session_ready;
    bool                  photo_pending;
    bool                  photo_job_busy;
    uint32_t              photo_skip_frames;

    TaskHandle_t          photo_task_handle;
    ppa_client_handle_t   ppa_client;
    jpeg_encoder_handle_t jpeg_handle;
    SemaphoreHandle_t     session_mutex;

    uint32_t              session_seq;
    uint32_t              photo_count;
    uint64_t              photo_avg_size_bytes;
    char                  session_date_tag[MEDIA_STORAGE_DATE_TAG_LEN];
    char                  session_root_dir[MEDIA_STORAGE_MAX_PATH_LEN];
    char                  photo_dir[MEDIA_STORAGE_MAX_PATH_LEN];
    char                  video_dir[MEDIA_STORAGE_MAX_PATH_LEN];

    uint8_t              *yuv420_buf;
    size_t                yuv420_buf_size;
    uint8_t              *rgb565_buf;
    size_t                rgb565_buf_size;
    uint8_t              *jpeg_out_buf;
    size_t                jpeg_out_buf_size;

    uint32_t              frame_width;
    uint32_t              frame_height;
    size_t                frame_len;

    bool                  video_prepared;
    bool                  video_record_requested;
    bool                  video_overwrite_allowed;
    bool                  tf_overwriting_old_video;
    bool                  video_gap_pending;
    bool                  video_save_current_gop;
    TaskHandle_t          video_task_handle;
    QueueHandle_t         video_queue;
    media_storage_video_frame_t video_frames[MEDIA_STORAGE_VIDEO_QUEUE_LEN];
    bool                  video_slot_in_use[MEDIA_STORAGE_VIDEO_QUEUE_LEN];
    uint32_t              video_slot_count;
    size_t                video_frame_buf_size;
    uint32_t              video_width;
    uint32_t              video_height;
    uint32_t              video_fps;
    uint32_t              video_sample_duration;
    uint64_t              video_avg_bytes_per_sec;
    media_mp4_writer_t    video_writer;
    uint8_t              *video_sps;
    size_t                video_sps_len;
    uint8_t              *video_pps;
    size_t                video_pps_len;
    uint32_t              video_segment_index;
    uint32_t              video_segment_frame_count;
    int64_t               video_segment_start_us;
    bool                  video_switch_pending;
    char                  video_tmp_path[MEDIA_STORAGE_MAX_PATH_LEN];
    char                  video_final_path[MEDIA_STORAGE_MAX_PATH_LEN];
    uint32_t              video_drop_count;
    uint32_t              video_save_gop_count;
    uint32_t              video_adaptive_save_interval;
    uint32_t              video_pressure_skip_gops;
    uint32_t              video_recovery_gop_count;
    uint32_t              video_last_idr_used_slots;
    uint32_t              video_non_idr_frame_count;
    TickType_t            video_last_drop_log_tick;
} media_storage_ctx_t;

static media_storage_ctx_t s_media;
static portMUX_TYPE s_media_lock = portMUX_INITIALIZER_UNLOCKED;

static void media_storage_photo_task(void *arg);
static void media_storage_video_task(void *arg);
static esp_err_t media_storage_join_path(char *dst, size_t dst_size,
                                         const char *dir, const char *name);

/* ------------------------------------------------------------------ */
/* 通用工具                                                            */
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

static size_t media_storage_calc_video_frame_buf_size(uint32_t width, uint32_t height)
{
    (void)width;
    (void)height;

    /*
     * Store compressed H.264 frames only. 128 KB keeps the 45-slot pool
     * below the previous 24 x 256 KB footprint while still leaving IDR margin.
     */
    return media_storage_align_size(MEDIA_STORAGE_VIDEO_FRAME_BUF_SIZE);
}

static uint32_t media_storage_calc_video_slot_count(uint32_t width, uint32_t height)
{
    const uint32_t pixels = width * height;

    if (pixels >= (1920U * 1080U)) {
        return MEDIA_STORAGE_VIDEO_QUEUE_LEN_1080P;
    }
    if (pixels >= (1280U * 960U)) {
        return MEDIA_STORAGE_VIDEO_QUEUE_LEN_SXGA;
    }

    return MEDIA_STORAGE_VIDEO_QUEUE_LEN;
}

static uint32_t media_storage_video_queue_high_watermark(void)
{
    return (s_media.video_slot_count * 2U) / 3U;
}

static uint32_t media_storage_video_queue_low_watermark(void)
{
    return (s_media.video_slot_count + 1U) / 2U;
}

static uint32_t media_storage_video_queue_critical_watermark(void)
{
    return (s_media.video_slot_count * 5U) / 6U;
}

static esp_err_t media_storage_append_text(char *dst, size_t dst_size,
                                           size_t *offset, const char *text)
{
    size_t text_len;

    if (!dst || dst_size == 0U || !offset || !text) {
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

    if (!dst || dst_size == 0U || !text) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    return media_storage_append_text(dst, dst_size, &offset, text);
}

static uint64_t media_storage_max_u64(uint64_t left, uint64_t right)
{
    return (left > right) ? left : right;
}

static bool media_storage_is_errno_no_space(int err_no)
{
    if (err_no == ENOSPC) {
        return true;
    }
#ifdef EDQUOT
    if (err_no == EDQUOT) {
        return true;
    }
#endif
    return false;
}

static uint64_t media_storage_default_photo_size_bytes(void)
{
    uint32_t width = s_media.frame_width;
    uint32_t height = s_media.frame_height;
    uint64_t pixels;

    if (width == 0U || height == 0U) {
        width = s_media.video_width;
        height = s_media.video_height;
    }
    if (width == 0U || height == 0U) {
        return 512ULL * 1024ULL;
    }

    pixels = (uint64_t)width * (uint64_t)height;
    return media_storage_max_u64(256ULL * 1024ULL, pixels / 4ULL);
}

static uint64_t media_storage_default_video_bytes_per_sec(void)
{
    uint64_t pixels = (uint64_t)s_media.video_width * (uint64_t)s_media.video_height;

    if (pixels >= (uint64_t)1920U * 1080U) {
        return 450ULL * 1024ULL;
    }
    if (pixels >= (uint64_t)1280U * 960U) {
        return 320ULL * 1024ULL;
    }
    if (pixels >= (uint64_t)800U * 800U) {
        return 220ULL * 1024ULL;
    }
    if (pixels >= (uint64_t)800U * 640U) {
        return 180ULL * 1024ULL;
    }
    return 256ULL * 1024ULL;
}

static uint64_t media_storage_get_photo_estimate_bytes(void)
{
    if (s_media.photo_avg_size_bytes > 0ULL) {
        return s_media.photo_avg_size_bytes;
    }
    return media_storage_default_photo_size_bytes();
}

static uint64_t media_storage_get_video_estimate_bytes_per_sec(void)
{
    if (s_media.video_avg_bytes_per_sec > 0ULL) {
        return s_media.video_avg_bytes_per_sec;
    }
    return media_storage_default_video_bytes_per_sec();
}

static uint64_t media_storage_get_photo_min_free_bytes(void)
{
    return media_storage_max_u64(MEDIA_STORAGE_TF_MIN_PHOTO_FREE_BYTES,
                                 media_storage_get_photo_estimate_bytes() * 2ULL);
}

static uint64_t media_storage_get_video_start_min_free_bytes(void)
{
    return media_storage_max_u64(MEDIA_STORAGE_TF_MIN_VIDEO_START_FREE_BYTES,
                                 media_storage_get_video_estimate_bytes_per_sec() *
                                 MEDIA_STORAGE_TF_VIDEO_START_GUARD_SEC);
}

static uint64_t media_storage_get_video_resume_min_free_bytes(void)
{
    return media_storage_max_u64(MEDIA_STORAGE_TF_MIN_VIDEO_RESUME_FREE_BYTES,
                                 media_storage_get_video_estimate_bytes_per_sec() *
                                 MEDIA_STORAGE_TF_VIDEO_RESUME_GUARD_SEC);
}

static bool media_storage_can_capture_with_info(const tf_card_info_t *info)
{
    return info && info->mounted && info->card_ok &&
           info->free_bytes >= media_storage_get_photo_min_free_bytes();
}

static bool media_storage_can_start_record_with_info(const tf_card_info_t *info)
{
    return info && info->mounted && info->card_ok &&
           info->free_bytes >= media_storage_get_video_start_min_free_bytes();
}

static bool media_storage_can_resume_record_with_info(const tf_card_info_t *info)
{
    return info && info->mounted && info->card_ok &&
           info->free_bytes >= media_storage_get_video_resume_min_free_bytes();
}

static void media_storage_set_video_overwrite_allowed(bool enabled)
{
    portENTER_CRITICAL(&s_media_lock);
    s_media.video_overwrite_allowed = enabled;
    portEXIT_CRITICAL(&s_media_lock);
}

static bool media_storage_is_video_overwrite_allowed(void)
{
    bool enabled;

    portENTER_CRITICAL(&s_media_lock);
    enabled = s_media.video_overwrite_allowed;
    portEXIT_CRITICAL(&s_media_lock);
    return enabled;
}

static void media_storage_set_tf_overwriting_old_video(bool enabled)
{
    portENTER_CRITICAL(&s_media_lock);
    s_media.tf_overwriting_old_video = enabled;
    portEXIT_CRITICAL(&s_media_lock);
}

static bool media_storage_is_tf_overwriting_old_video(void)
{
    bool enabled;

    portENTER_CRITICAL(&s_media_lock);
    enabled = s_media.tf_overwriting_old_video;
    portEXIT_CRITICAL(&s_media_lock);
    return enabled;
}

static void media_storage_update_photo_average(size_t file_size)
{
    uint64_t size_bytes = (uint64_t)file_size;

    if (size_bytes == 0ULL) {
        return;
    }
    if (s_media.photo_avg_size_bytes == 0ULL) {
        s_media.photo_avg_size_bytes = size_bytes;
    } else {
        s_media.photo_avg_size_bytes =
            (s_media.photo_avg_size_bytes * 3ULL + size_bytes) / 4ULL;
    }
}

static void media_storage_update_video_average(uint64_t file_size, uint32_t duration_ms)
{
    uint64_t bytes_per_sec;

    if (file_size == 0ULL || duration_ms == 0U) {
        return;
    }

    bytes_per_sec = (file_size * 1000ULL) / (uint64_t)duration_ms;
    if (bytes_per_sec == 0ULL) {
        return;
    }

    if (s_media.video_avg_bytes_per_sec == 0ULL) {
        s_media.video_avg_bytes_per_sec = bytes_per_sec;
    } else {
        s_media.video_avg_bytes_per_sec =
            (s_media.video_avg_bytes_per_sec * 3ULL + bytes_per_sec) / 4ULL;
    }
}

static bool media_storage_parse_session_seq(const char *name, uint32_t *out_seq)
{
    const char *cursor = name;
    uint32_t value = 0;

    if (!name || !out_seq) {
        return false;
    }

    while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10U + (uint32_t)(*cursor - '0');
        cursor++;
    }

    if (cursor == name || *cursor != '_') {
        return false;
    }

    *out_seq = value;
    return true;
}

static bool media_storage_has_mp4_suffix(const char *name)
{
    size_t len;

    if (!name) {
        return false;
    }

    len = strlen(name);
    return len >= 4U && strcasecmp(name + len - 4U, ".mp4") == 0;
}

static bool media_storage_is_dir(const char *path)
{
    struct stat st = {0};

    return path && stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

static bool media_storage_is_regular_file(const char *path, struct stat *out_st)
{
    struct stat st = {0};

    if (!path || stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        return false;
    }

    if (out_st) {
        *out_st = st;
    }
    return true;
}

static bool media_storage_dir_is_empty(const char *path)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;

    if (!media_storage_is_dir(path)) {
        return false;
    }

    dir = opendir(path);
    if (!dir) {
        return false;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
            closedir(dir);
            return false;
        }
    }

    closedir(dir);
    return true;
}

static void media_storage_try_remove_empty_dir(const char *path)
{
    if (!path || !media_storage_dir_is_empty(path)) {
        return;
    }

    if (remove(path) != 0 && errno != ENOENT) {
        ESP_LOGW(TAG, "删除空目录失败: %s, errno=%d", path, errno);
    }
}

static esp_err_t media_storage_delete_oldest_video_file(char *deleted_path, size_t deleted_path_size)
{
    DIR *root_dir = NULL;
    struct dirent *entry = NULL;
    uint32_t selected_seq = UINT32_MAX;
    char selected_session_dir[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    char selected_video_dir[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    char selected_file_name[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    char selected_file_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    bool found = false;

    root_dir = opendir(tf_card_get_mount_point());
    if (!root_dir) {
        ESP_LOGW(TAG, "打开 TF 根目录失败，无法清理旧视频: errno=%d", errno);
        return ESP_FAIL;
    }

    while ((entry = readdir(root_dir)) != NULL) {
        char session_dir[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
        char video_dir[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
        DIR *video_dir_handle = NULL;
        struct dirent *video_entry = NULL;
        uint32_t session_seq = 0;
        bool has_video = false;
        char first_file_name[MEDIA_STORAGE_MAX_PATH_LEN] = {0};

        if (entry->d_name[0] == '.' ||
            !media_storage_parse_session_seq(entry->d_name, &session_seq)) {
            continue;
        }

        if (media_storage_join_path(session_dir, sizeof(session_dir),
                                    tf_card_get_mount_point(), entry->d_name) != ESP_OK ||
            media_storage_join_path(video_dir, sizeof(video_dir),
                                    session_dir, MEDIA_STORAGE_VIDEO_SUBDIR) != ESP_OK ||
            !media_storage_is_dir(video_dir)) {
            continue;
        }

        video_dir_handle = opendir(video_dir);
        if (!video_dir_handle) {
            continue;
        }

        while ((video_entry = readdir(video_dir_handle)) != NULL) {
            if (!media_storage_has_mp4_suffix(video_entry->d_name)) {
                continue;
            }
            if (!has_video || strcmp(video_entry->d_name, first_file_name) < 0) {
                if (media_storage_copy_text(first_file_name, sizeof(first_file_name),
                                            video_entry->d_name) != ESP_OK) {
                    ESP_LOGW(TAG, "视频文件名过长，已跳过: %s", video_entry->d_name);
                    continue;
                }
                has_video = true;
            }
        }
        closedir(video_dir_handle);

        if (!has_video) {
            media_storage_try_remove_empty_dir(video_dir);
            media_storage_try_remove_empty_dir(session_dir);
            continue;
        }

        if (!found || session_seq < selected_seq ||
            (session_seq == selected_seq && strcmp(first_file_name, selected_file_name) < 0)) {
            selected_seq = session_seq;
            if (media_storage_copy_text(selected_session_dir, sizeof(selected_session_dir),
                                        session_dir) != ESP_OK ||
                media_storage_copy_text(selected_video_dir, sizeof(selected_video_dir),
                                        video_dir) != ESP_OK ||
                media_storage_copy_text(selected_file_name, sizeof(selected_file_name),
                                        first_file_name) != ESP_OK) {
                ESP_LOGW(TAG, "旧视频路径过长，已跳过当前目录");
                continue;
            }
            found = true;
        }
    }
    closedir(root_dir);

    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(media_storage_join_path(selected_file_path, sizeof(selected_file_path),
                                                selected_video_dir, selected_file_name),
                        TAG, "构建旧视频删除路径失败");

    if (!media_storage_is_regular_file(selected_file_path, NULL)) {
        return ESP_ERR_NOT_FOUND;
    }

    if (remove(selected_file_path) != 0) {
        ESP_LOGW(TAG, "删除旧视频失败: %s, errno=%d", selected_file_path, errno);
        return ESP_FAIL;
    }

    media_storage_try_remove_empty_dir(selected_video_dir);
    media_storage_try_remove_empty_dir(selected_session_dir);

    if (deleted_path && deleted_path_size > 0U) {
        if (media_storage_copy_text(deleted_path, deleted_path_size,
                                    selected_file_path) != ESP_OK) {
            deleted_path[0] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t media_storage_join_path(char *dst, size_t dst_size,
                                         const char *dir, const char *name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0U || !dir || !name) {
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

    if (!dst || dst_size == 0U || !mount_point || !date_tag) {
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

    if (!dst || dst_size == 0U || !photo_dir || !timestamp) {
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

static esp_err_t media_storage_build_video_file_path(char *dst, size_t dst_size,
                                                     const char *video_dir,
                                                     const char *timestamp,
                                                     uint32_t segment_index,
                                                     const char *suffix)
{
    esp_err_t ret;
    size_t offset = 0;
    int index_len;
    char index_text[16] = {0};

    if (!dst || dst_size == 0U || !video_dir || !timestamp || !suffix) {
        return ESP_ERR_INVALID_ARG;
    }

    index_len = snprintf(index_text, sizeof(index_text), "%04" PRIu32, segment_index);
    if (index_len < 0 || index_len >= (int)sizeof(index_text)) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = media_storage_append_text(dst, dst_size, &offset, video_dir);
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, timestamp);
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, "_");
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, index_text);
    }
    if (ret == ESP_OK) {
        ret = media_storage_append_text(dst, dst_size, &offset, suffix);
    }

    return ret;
}

static bool media_storage_try_take_photo_request(void)
{
    bool accepted = false;

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.initialized && s_media.photo_pending && !s_media.photo_job_busy) {
        if (s_media.photo_skip_frames > 0U) {
            s_media.photo_skip_frames--;
        } else {
            s_media.photo_pending = false;
            s_media.photo_job_busy = true;
            accepted = true;
        }
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
        s_media.photo_pending = true;
    }
    portEXIT_CRITICAL(&s_media_lock);
}

static bool media_storage_is_video_record_requested(void)
{
    bool requested = false;

    portENTER_CRITICAL(&s_media_lock);
    requested = s_media.video_record_requested;
    portEXIT_CRITICAL(&s_media_lock);
    return requested;
}

static uint32_t media_storage_base_save_interval(void)
{
    return (MEDIA_STORAGE_VIDEO_SAVE_GOP_INTERVAL == 0U) ? 1U
                                                         : MEDIA_STORAGE_VIDEO_SAVE_GOP_INTERVAL;
}

static uint32_t media_storage_count_video_slots_in_use_locked(void)
{
    uint32_t used_slots = 0;

    for (uint32_t i = 0; i < s_media.video_slot_count; i++) {
        if (s_media.video_slot_in_use[i]) {
            used_slots++;
        }
    }

    return used_slots;
}

static void media_storage_log_video_pressure(uint32_t used_slots,
                                             uint32_t save_interval,
                                             uint32_t skip_gops,
                                             uint32_t non_idr_skip_mod)
{
    static TickType_t s_last_pressure_log_tick = 0;
    TickType_t now = xTaskGetTickCount();
    uint32_t slot_count = s_media.video_slot_count;

    if (slot_count == 0U) {
        slot_count = MEDIA_STORAGE_VIDEO_QUEUE_LEN;
    }

    if (s_last_pressure_log_tick != 0 &&
        (now - s_last_pressure_log_tick) < pdMS_TO_TICKS(MEDIA_STORAGE_VIDEO_DROP_LOG_MS)) {
        return;
    }

    s_last_pressure_log_tick = now;
    if (non_idr_skip_mod > 1U) {
        ESP_LOGW(TAG,
                 "录像旁路积压偏高 | 占用=%" PRIu32 "/%u | 当前保存间隔=%" PRIu32 " | 临时跳过=%" PRIu32 " 个 GOP | 非IDR抽帧=1/%" PRIu32,
                 used_slots, (unsigned)slot_count, save_interval, skip_gops,
                 non_idr_skip_mod);
    } else {
        ESP_LOGW(TAG,
                 "录像旁路积压偏高 | 占用=%" PRIu32 "/%u | 当前保存间隔=%" PRIu32 " | 临时跳过=%" PRIu32 " 个 GOP",
                 used_slots, (unsigned)slot_count, save_interval, skip_gops);
    }
}

static bool media_storage_should_skip_video_input(esp_h264_frame_type_t frame_type)
{
    bool skip;
    uint32_t used_slots = 0;
    uint32_t save_interval = 0;
    uint32_t skip_gops = 0;
    uint32_t non_idr_skip_mod = 0;
    bool log_pressure = false;
    bool pressure_worsened = false;
    const uint32_t base_interval = media_storage_base_save_interval();

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.video_adaptive_save_interval < base_interval) {
        s_media.video_adaptive_save_interval = base_interval;
    }
    used_slots = media_storage_count_video_slots_in_use_locked();
    if (used_slots >= media_storage_video_queue_critical_watermark()) {
        non_idr_skip_mod = MEDIA_STORAGE_VIDEO_NON_IDR_SKIP_MOD_CRITICAL;
    } else if (used_slots >= media_storage_video_queue_high_watermark()) {
        non_idr_skip_mod = MEDIA_STORAGE_VIDEO_NON_IDR_SKIP_MOD_HIGH;
    }

    if (frame_type == ESP_H264_FRAME_TYPE_IDR) {
        pressure_worsened = (used_slots > s_media.video_last_idr_used_slots);
        if (used_slots >= media_storage_video_queue_high_watermark()) {
            if (pressure_worsened &&
                s_media.video_adaptive_save_interval < MEDIA_STORAGE_VIDEO_ADAPTIVE_INTERVAL_MAX) {
                s_media.video_adaptive_save_interval++;
            }
            if (MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS > 0U &&
                pressure_worsened &&
                s_media.video_pressure_skip_gops < MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS) {
                s_media.video_pressure_skip_gops = MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS;
            }
            s_media.video_recovery_gop_count = 0;
            log_pressure = pressure_worsened ||
                           (s_media.video_last_idr_used_slots < media_storage_video_queue_high_watermark());
        } else if (used_slots <= media_storage_video_queue_low_watermark()) {
            if (s_media.video_adaptive_save_interval > base_interval) {
                s_media.video_recovery_gop_count++;
                if (s_media.video_recovery_gop_count >= MEDIA_STORAGE_VIDEO_RECOVERY_GOPS) {
                    s_media.video_adaptive_save_interval--;
                    s_media.video_recovery_gop_count = 0;
                }
            } else {
                s_media.video_recovery_gop_count = 0;
            }
        } else {
            s_media.video_recovery_gop_count = 0;
        }

        s_media.video_save_gop_count++;
        if (s_media.video_pressure_skip_gops > 0U) {
            s_media.video_pressure_skip_gops--;
            s_media.video_save_current_gop = false;
        } else {
            s_media.video_save_current_gop =
                (((s_media.video_save_gop_count - 1U) % s_media.video_adaptive_save_interval) == 0U);
        }
        save_interval = s_media.video_adaptive_save_interval;
        skip_gops = s_media.video_pressure_skip_gops;
        s_media.video_last_idr_used_slots = used_slots;
    } else if (used_slots >= media_storage_video_queue_critical_watermark() &&
               used_slots > s_media.video_last_idr_used_slots &&
               s_media.video_save_current_gop) {
        if (s_media.video_adaptive_save_interval < MEDIA_STORAGE_VIDEO_ADAPTIVE_INTERVAL_MAX) {
            s_media.video_adaptive_save_interval++;
        }
        if (MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS > 0U &&
            s_media.video_pressure_skip_gops < MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS) {
            s_media.video_pressure_skip_gops = MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS;
        }
        s_media.video_recovery_gop_count = 0;
        s_media.video_gap_pending = true;
        s_media.video_save_current_gop = false;
        save_interval = s_media.video_adaptive_save_interval;
        skip_gops = s_media.video_pressure_skip_gops;
        log_pressure = true;
    }
    skip = !s_media.video_save_current_gop;
    if (!skip && frame_type != ESP_H264_FRAME_TYPE_IDR) {
        if (non_idr_skip_mod > 1U) {
            s_media.video_non_idr_frame_count++;
            if ((s_media.video_non_idr_frame_count % non_idr_skip_mod) == 0U) {
                skip = true;
            }
        }
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (log_pressure) {
        media_storage_log_video_pressure(used_slots, save_interval, skip_gops, non_idr_skip_mod);
    }

    return skip;
}

static int media_storage_acquire_video_slot(void)
{
    int slot = -1;

    portENTER_CRITICAL(&s_media_lock);
    for (uint32_t i = 0; i < s_media.video_slot_count; i++) {
        if (!s_media.video_slot_in_use[i]) {
            s_media.video_slot_in_use[i] = true;
            slot = (int)i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_media_lock);
    return slot;
}

static void media_storage_release_video_slot(int slot)
{
    if (slot < 0 || (uint32_t)slot >= s_media.video_slot_count) {
        return;
    }

    portENTER_CRITICAL(&s_media_lock);
    s_media.video_slot_in_use[slot] = false;
    portEXIT_CRITICAL(&s_media_lock);
}

static void media_storage_log_video_drop(const char *reason, size_t len)
{
    TickType_t now;
    uint32_t drop_count;
    uint32_t used_slots;
    uint32_t save_interval;
    uint32_t skip_gops;
    bool need_log = false;
    const uint32_t base_interval = media_storage_base_save_interval();

    if (!reason) {
        reason = "unknown";
    }

    now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_media_lock);
    s_media.video_gap_pending = true;
    s_media.video_save_current_gop = false;
    s_media.video_drop_count++;
    if (s_media.video_adaptive_save_interval < base_interval) {
        s_media.video_adaptive_save_interval = base_interval;
    }
    if (s_media.video_adaptive_save_interval < MEDIA_STORAGE_VIDEO_ADAPTIVE_INTERVAL_MAX) {
        s_media.video_adaptive_save_interval++;
    }
    if (s_media.video_pressure_skip_gops < MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS) {
        s_media.video_pressure_skip_gops = MEDIA_STORAGE_VIDEO_PRESSURE_SKIP_GOPS;
    }
    s_media.video_recovery_gop_count = 0;
    drop_count = s_media.video_drop_count;
    used_slots = media_storage_count_video_slots_in_use_locked();
    s_media.video_last_idr_used_slots = used_slots;
    save_interval = s_media.video_adaptive_save_interval;
    skip_gops = s_media.video_pressure_skip_gops;
    if (s_media.video_last_drop_log_tick == 0 ||
        (now - s_media.video_last_drop_log_tick) >= pdMS_TO_TICKS(MEDIA_STORAGE_VIDEO_DROP_LOG_MS)) {
        s_media.video_last_drop_log_tick = now;
        need_log = true;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (need_log) {
        media_storage_log_video_pressure(used_slots, save_interval, skip_gops, 0U);
        ESP_LOGW(TAG, "录像旁路丢帧 | 原因=%s | 长度=%zu | 累计=%" PRIu32,
                 reason, len, drop_count);
    }
}

static void media_storage_log_video_gap_continue(void)
{
    static TickType_t s_last_gap_continue_log_tick = 0;
    TickType_t now = xTaskGetTickCount();

    if (s_last_gap_continue_log_tick == 0 ||
        (now - s_last_gap_continue_log_tick) >= pdMS_TO_TICKS(MEDIA_STORAGE_VIDEO_DROP_LOG_MS)) {
        s_last_gap_continue_log_tick = now;
        ESP_LOGW(TAG, "录像旁路发生丢帧，已从下一帧 IDR 继续写入当前分段");
    }
}

static bool media_storage_take_video_gap(void)
{
    bool gap_pending;

    portENTER_CRITICAL(&s_media_lock);
    gap_pending = s_media.video_gap_pending;
    s_media.video_gap_pending = false;
    portEXIT_CRITICAL(&s_media_lock);
    return gap_pending;
}

static bool media_storage_should_skip_video_gap_frame(bool is_idr)
{
    bool gap_pending;

    portENTER_CRITICAL(&s_media_lock);
    gap_pending = s_media.video_gap_pending;
    if (gap_pending && is_idr) {
        s_media.video_gap_pending = false;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (!gap_pending) {
        return false;
    }

    if (!is_idr) {
        return true;
    }

    media_storage_log_video_gap_continue();
    return false;
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
        time_t sec = (time_t)tv.tv_sec;
        uint32_t msec = (uint32_t)(tv.tv_usec / 1000U);

        localtime_r(&sec, &tm_info);

        snprintf(date_tag, date_tag_size, "%04d%02d%02d",
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday);
        snprintf(timestamp, timestamp_size, "%04d-%02d-%02dT%02d-%02d-%02d-%03" PRIu32,
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, msec);
    } else {
        static bool s_default_time_warned = false;
        uint64_t boot_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
        uint32_t msec = (uint32_t)(boot_ms % 1000ULL);
        uint32_t total_sec = (uint32_t)((boot_ms / 1000ULL) % 86400ULL);
        uint32_t hour = total_sec / 3600U;
        uint32_t minute = (total_sec % 3600U) / 60U;
        uint32_t second = total_sec % 60U;

        snprintf(date_tag, date_tag_size, "%s", MEDIA_STORAGE_DEFAULT_DATE_TAG);
        snprintf(timestamp, timestamp_size, "%sT%02" PRIu32 "-%02" PRIu32 "-%02" PRIu32 "-%03" PRIu32,
                 MEDIA_STORAGE_DEFAULT_DATE_TEXT, hour, minute, second, msec);
        if (!s_default_time_warned) {
            s_default_time_warned = true;
            ESP_LOGW(TAG, "系统时间尚未同步，本次媒体命名暂用默认日期 %s", MEDIA_STORAGE_DEFAULT_DATE_TEXT);
        }
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
        ESP_LOGE(TAG, "读取历史保存次序失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        nvs_close(nvs);
        return ret;
    }

    seq += 1U;

    ret = nvs_set_u32(nvs, MEDIA_STORAGE_NVS_KEY_BOOT_SEQ, seq);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写入历史保存次序失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    *out_seq = seq;
    return ESP_OK;
}

static esp_err_t media_storage_prepare_session(const char *date_tag)
{
    esp_err_t ret;
    uint32_t seq = 0;

    if (!date_tag || date_tag[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_media.session_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_media.session_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (s_media.session_ready) {
        xSemaphoreGive(s_media.session_mutex);
        return ESP_OK;
    }

    ret = media_storage_alloc_session_seq(&seq);
    if (ret == ESP_OK) {
        ret = media_storage_build_session_root(s_media.session_root_dir,
                                               sizeof(s_media.session_root_dir),
                                               tf_card_get_mount_point(), seq, date_tag);
    }
    if (ret == ESP_OK) {
        ret = media_storage_join_path(s_media.photo_dir, sizeof(s_media.photo_dir),
                                      s_media.session_root_dir, MEDIA_STORAGE_PHOTO_SUBDIR);
    }
    if (ret == ESP_OK) {
        ret = media_storage_join_path(s_media.video_dir, sizeof(s_media.video_dir),
                                      s_media.session_root_dir, MEDIA_STORAGE_VIDEO_SUBDIR);
    }
    if (ret == ESP_OK) {
        ret = media_storage_ensure_dir(s_media.session_root_dir);
    }
    if (ret == ESP_OK) {
        ret = media_storage_ensure_dir(s_media.photo_dir);
    }
    if (ret == ESP_OK) {
        ret = media_storage_ensure_dir(s_media.video_dir);
    }
    if (ret == ESP_OK) {
        ret = media_storage_copy_text(s_media.session_date_tag,
                                      sizeof(s_media.session_date_tag), date_tag);
    }

    if (ret == ESP_OK) {
        s_media.session_seq = seq;
        s_media.session_ready = true;
        ESP_LOGI(TAG, "本次上电媒体目录已确定: %s", s_media.session_root_dir);
    } else {
        s_media.session_root_dir[0] = '\0';
        s_media.photo_dir[0] = '\0';
        s_media.video_dir[0] = '\0';
        s_media.session_date_tag[0] = '\0';
    }

    xSemaphoreGive(s_media.session_mutex);
    return ret;
}

static esp_err_t media_storage_reclaim_space_for_record(tf_card_info_t *out_info)
{
    tf_card_info_t info = {0};
    esp_err_t ret;
    bool deleted_any = false;

    ret = tf_card_get_info(&info);
    if (ret != ESP_OK) {
        return ret;
    }

    while (!media_storage_can_resume_record_with_info(&info)) {
        char deleted_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};

        ret = media_storage_delete_oldest_video_file(deleted_path, sizeof(deleted_path));
        if (ret != ESP_OK) {
            if (deleted_any) {
                ESP_LOGW(TAG, "TF 卡已满，但没有更多旧视频可删除，无法继续录像");
            }
            if (out_info) {
                *out_info = info;
            }
            return ret;
        }

        deleted_any = true;
        media_storage_set_tf_overwriting_old_video(true);
        ESP_LOGW(TAG, "TF 卡已满，已删除旧视频继续录像: %s", deleted_path);

        ret = tf_card_get_info(&info);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    if (out_info) {
        *out_info = info;
    }
    return ESP_OK;
}

esp_err_t media_storage_get_tf_status(bool run_speed_test,
                                      media_storage_tf_status_t *out_status)
{
    media_storage_tf_status_t status = {0};
    tf_card_info_t info = {0};
    tf_card_speed_test_result_t speed = {0};
    esp_err_t ret;

    if (!out_status) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = tf_card_get_info(&info);
    if (ret == ESP_OK) {
        status.tf_mounted = info.mounted;
        status.tf_card_ok = info.card_ok;
        status.tf_total_bytes = info.total_bytes;
        status.tf_free_bytes = info.free_bytes;
        status.tf_used_bytes = (info.total_bytes > info.free_bytes) ?
                               (info.total_bytes - info.free_bytes) : 0ULL;
        status.tf_can_capture = media_storage_can_capture_with_info(&info);
        status.tf_can_start_record = media_storage_can_start_record_with_info(&info);
        status.tf_est_photo_count = (uint32_t)(info.free_bytes /
                                  media_storage_max_u64(1ULL, media_storage_get_photo_estimate_bytes()));
        status.tf_est_record_seconds = (uint32_t)(info.free_bytes /
                                     media_storage_max_u64(1ULL, media_storage_get_video_estimate_bytes_per_sec()));
    } else {
        status.tf_mounted = false;
        status.tf_card_ok = false;
        status.tf_can_capture = false;
        status.tf_can_start_record = false;
    }

    if (run_speed_test) {
        if (media_storage_is_video_record_requested()) {
            status.tf_speed_test_skipped = true;
            snprintf(status.tf_speed_text, sizeof(status.tf_speed_text), "%s",
                     "录像中，已跳过测速");
            tf_card_get_last_speed_test(&speed);
        } else {
            ret = tf_card_run_speed_test(&speed);
            if (ret != ESP_OK) {
                tf_card_get_last_speed_test(&speed);
                if (ret == ESP_ERR_NO_MEM) {
                    snprintf(status.tf_speed_text, sizeof(status.tf_speed_text), "%s",
                             "剩余空间不足，无法测速");
                } else {
                    snprintf(status.tf_speed_text, sizeof(status.tf_speed_text), "%s",
                             "TF 卡测速失败");
                }
            }
        }
    } else {
        tf_card_get_last_speed_test(&speed);
    }

    if (speed.valid) {
        status.tf_speed_test_valid = true;
        status.tf_write_speed_kbps = speed.write_speed_kbps;
        status.tf_read_speed_kbps = speed.read_speed_kbps;
        status.tf_speed_too_low = speed.write_speed_too_low || speed.read_speed_too_low;
        if (status.tf_speed_text[0] == '\0') {
            snprintf(status.tf_speed_text, sizeof(status.tf_speed_text),
                     "写入 %" PRIu32 " KB/s，读取 %" PRIu32 " KB/s%s",
                     speed.write_speed_kbps, speed.read_speed_kbps,
                     status.tf_speed_too_low ? "，速度过慢" : "");
        }
    } else if (status.tf_speed_text[0] == '\0') {
        snprintf(status.tf_speed_text, sizeof(status.tf_speed_text), "%s", "未执行测速");
    }

    status.tf_overwriting_old_video = media_storage_is_tf_overwriting_old_video();
    status.tf_full = status.tf_mounted &&
                     ((!status.tf_can_capture) ||
                      (!status.tf_can_start_record) ||
                      status.tf_overwriting_old_video);

    if (!status.tf_mounted) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s", "TF 卡未挂载");
    } else if (!status.tf_card_ok) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s", "TF 卡异常");
    } else if (status.tf_overwriting_old_video) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s",
                 "TF 卡已满，正在覆盖旧视频");
    } else if (!status.tf_can_start_record && !status.tf_can_capture) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s",
                 "TF 卡已满，无法拍照和启动新录像");
    } else if (!status.tf_can_start_record) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s",
                 "TF 卡剩余空间不足，无法启动新录像");
    } else if (!status.tf_can_capture) {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s",
                 "TF 卡剩余空间不足，无法拍照");
    } else {
        snprintf(status.tf_status_text, sizeof(status.tf_status_text), "%s", "TF 卡正常");
    }

    *out_status = status;
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 照片链路                                                            */
/* ------------------------------------------------------------------ */
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
        .data_burst_length = PPA_DATA_BURST_LENGTH_64,
    };

    if (s_media.ppa_client) {
        return ESP_OK;
    }

    esp_err_t ret = ppa_register_client(&ppa_config, &s_media.ppa_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 PPA 客户端失败: 0x%x (%s)", ret, esp_err_to_name(ret));
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

    if (s_media.photo_task_handle) {
        return ESP_OK;
    }

    if (xTaskCreatePinnedToCore(media_storage_photo_task, "media_photo",
                                MEDIA_STORAGE_PHOTO_TASK_STACK_SIZE, NULL,
                                MEDIA_STORAGE_PHOTO_TASK_PRIORITY,
                                &task_handle,
                                MEDIA_STORAGE_PHOTO_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "创建照片保存后台任务失败");
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.photo_task_handle == NULL) {
        s_media.photo_task_handle = task_handle;
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
        ESP_LOGE(TAG, "PPA 硬件转换 YUV420->RGB565 失败: 0x%x (%s)",
                 ret, esp_err_to_name(ret));
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

    if (jpeg_len == 0U || jpeg_len > s_media.jpeg_out_buf_size) {
        ESP_LOGE(TAG, "JPEG 输出长度异常: %" PRIu32, jpeg_len);
        return ESP_FAIL;
    }

    *out_jpeg_len = jpeg_len;
    return ESP_OK;
}

static esp_err_t media_storage_write_file(const char *path, const uint8_t *data, size_t len)
{
    FILE *fp = NULL;
    int saved_errno = 0;

    if (!path || !data || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    fp = fopen(path, "wb");
    if (!fp) {
        saved_errno = errno;
        ESP_LOGE(TAG, "打开文件失败: %s, errno=%d", path, saved_errno);
        if (media_storage_is_errno_no_space(saved_errno)) {
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }
        return ESP_FAIL;
    }

    if (fwrite(data, 1, len, fp) != len) {
        saved_errno = ferror(fp) ? errno : 0;
        fclose(fp);
        remove(path);
        ESP_LOGE(TAG, "写入文件失败: %s, errno=%d", path, saved_errno);
        if (media_storage_is_errno_no_space(saved_errno)) {
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }
        return ESP_FAIL;
    }

    if (fflush(fp) != 0) {
        saved_errno = errno;
        fclose(fp);
        remove(path);
        ESP_LOGE(TAG, "刷新文件失败: %s, errno=%d", path, saved_errno);
        if (media_storage_is_errno_no_space(saved_errno)) {
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }
        return ESP_FAIL;
    }

    if (fsync(fileno(fp)) != 0) {
        saved_errno = errno;
        fclose(fp);
        remove(path);
        ESP_LOGE(TAG, "同步文件失败: %s, errno=%d", path, saved_errno);
        if (media_storage_is_errno_no_space(saved_errno)) {
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }
        return ESP_FAIL;
    }

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
        tf_card_info_t tf_info = {0};

        if (!tf_card_is_mounted()) {
            ESP_LOGW(TAG, "TF 卡未挂载，跳过本次拍照任务");
            media_storage_finish_photo_job();
            continue;
        }

        media_storage_build_timestamp(date_tag, sizeof(date_tag), timestamp, sizeof(timestamp));

        esp_err_t ret = media_storage_convert_yuv420_to_rgb565(width, height);
        if (ret == ESP_OK) {
            ret = media_storage_encode_rgb565_to_jpeg(width, height, &jpeg_len);
        }
        if (ret == ESP_OK) {
            ret = tf_card_get_info(&tf_info);
        }
        if (ret == ESP_OK && !media_storage_can_capture_with_info(&tf_info)) {
            ret = ESP_ERR_MEDIA_STORAGE_TF_FULL;
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
            media_storage_update_photo_average(jpeg_len);
            ESP_LOGI(TAG, "照片保存成功 | 次序=%03" PRIu32 " | 本次上电第 %" PRIu32 " 张 | 路径=%s",
                     s_media.session_seq, s_media.photo_count, file_path);
        } else if (ret == ESP_ERR_MEDIA_STORAGE_TF_FULL) {
            ESP_LOGW(TAG, "TF 卡剩余空间不足，本次拍照未保存");
        } else {
            ESP_LOGE(TAG, "照片保存失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        }

        media_storage_finish_photo_job();
    }
}

/* ------------------------------------------------------------------ */
/* 录像链路                                                            */
/* ------------------------------------------------------------------ */
static void media_storage_clear_video_segment_paths(void)
{
    s_media.video_tmp_path[0] = '\0';
    s_media.video_final_path[0] = '\0';
}

static void media_storage_reset_video_segment_state(void)
{
    s_media.video_segment_frame_count = 0;
    s_media.video_segment_start_us = 0;
    s_media.video_switch_pending = false;
    media_storage_clear_video_segment_paths();
}

static void media_storage_free_video_parameter_sets(void)
{
    if (s_media.video_sps) {
        free(s_media.video_sps);
        s_media.video_sps = NULL;
    }
    if (s_media.video_pps) {
        free(s_media.video_pps);
        s_media.video_pps = NULL;
    }

    s_media.video_sps_len = 0;
    s_media.video_pps_len = 0;
}

static void media_storage_free_video_buffers(void)
{
    if (media_mp4_writer_is_open(&s_media.video_writer)) {
        media_mp4_writer_deinit(&s_media.video_writer);
    }

    if (s_media.video_queue) {
        xQueueReset(s_media.video_queue);
    }

    for (int i = 0; i < MEDIA_STORAGE_VIDEO_QUEUE_LEN; i++) {
        if (s_media.video_frames[i].buf) {
            free(s_media.video_frames[i].buf);
            s_media.video_frames[i].buf = NULL;
        }
        s_media.video_frames[i].capacity = 0;
        s_media.video_frames[i].len = 0;
        s_media.video_frames[i].pts = 0;
        s_media.video_frames[i].frame_type = 0;
        s_media.video_slot_in_use[i] = false;
    }

    s_media.video_frame_buf_size = 0;
    s_media.video_slot_count = 0;
    s_media.video_width = 0;
    s_media.video_height = 0;
    s_media.video_fps = 0;
    s_media.video_sample_duration = 0;
    s_media.video_prepared = false;
    s_media.video_record_requested = false;
    s_media.video_overwrite_allowed = false;
    s_media.tf_overwriting_old_video = false;
    s_media.video_gap_pending = false;
    s_media.video_save_current_gop = false;
    s_media.video_drop_count = 0;
    s_media.video_save_gop_count = 0;
    s_media.video_adaptive_save_interval = 0;
    s_media.video_pressure_skip_gops = 0;
    s_media.video_recovery_gop_count = 0;
    s_media.video_last_idr_used_slots = 0;
    s_media.video_non_idr_frame_count = 0;
    s_media.video_last_drop_log_tick = 0;
    s_media.video_avg_bytes_per_sec = 0ULL;
    media_storage_free_video_parameter_sets();
    media_storage_reset_video_segment_state();
}

static esp_err_t media_storage_ensure_video_task(void)
{
    TaskHandle_t task_handle = NULL;

    if (s_media.video_task_handle) {
        return ESP_OK;
    }

    if (xTaskCreatePinnedToCore(media_storage_video_task, "media_video",
                                MEDIA_STORAGE_VIDEO_TASK_STACK_SIZE, NULL,
                                MEDIA_STORAGE_VIDEO_TASK_PRIORITY,
                                &task_handle,
                                MEDIA_STORAGE_VIDEO_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "创建录像保存后台任务失败");
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.video_task_handle == NULL) {
        s_media.video_task_handle = task_handle;
        task_handle = NULL;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (task_handle) {
        vTaskDelete(task_handle);
    }

    return ESP_OK;
}

static int media_storage_parse_h264_nalus(const uint8_t *h264_data, size_t h264_len,
                                          media_storage_nalu_info_t *nalus,
                                          int max_nalus)
{
    int count = 0;
    size_t i = 0;

    if (!h264_data || h264_len == 0U || !nalus || max_nalus <= 0) {
        return 0;
    }

    while (i + 4U <= h264_len && count < max_nalus) {
        if (h264_data[i] == 0U && h264_data[i + 1U] == 0U) {
            int start_code_len = 0;

            if (h264_data[i + 2U] == 1U) {
                start_code_len = 3;
            } else if ((i + 3U < h264_len) &&
                       h264_data[i + 2U] == 0U &&
                       h264_data[i + 3U] == 1U) {
                start_code_len = 4;
            }

            if (start_code_len > 0) {
                size_t nalu_start = i + (size_t)start_code_len;
                size_t next = nalu_start + 1U;
                bool found_next = false;

                if (nalu_start >= h264_len) {
                    break;
                }

                while (next + 3U < h264_len) {
                    if (h264_data[next] == 0U &&
                        h264_data[next + 1U] == 0U &&
                        (h264_data[next + 2U] == 1U ||
                         (h264_data[next + 2U] == 0U &&
                          h264_data[next + 3U] == 1U))) {
                        found_next = true;
                        break;
                    }
                    next++;
                }

                if (!found_next) {
                    next = h264_len;
                }

                nalus[count].offset = nalu_start;
                nalus[count].len = next - nalu_start;
                nalus[count].type = h264_data[nalu_start] & MEDIA_STORAGE_VIDEO_NALU_TYPE_MASK;
                count++;

                i = next;
                continue;
            }
        }

        i++;
    }

    return count;
}

static esp_err_t media_storage_replace_nalu(uint8_t **dst, size_t *dst_len,
                                            const uint8_t *src, size_t src_len)
{
    uint8_t *buf = NULL;

    if (!dst || !dst_len || !src || src_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (*dst && *dst_len == src_len && memcmp(*dst, src, src_len) == 0) {
        return ESP_OK;
    }

    buf = (uint8_t *)malloc(src_len);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(buf, src, src_len);
    free(*dst);
    *dst = buf;
    *dst_len = src_len;
    return ESP_OK;
}

static void media_storage_update_video_parameter_sets(const uint8_t *h264_buf, size_t h264_len)
{
    media_storage_nalu_info_t nalus[16];
    int nalu_count;

    nalu_count = media_storage_parse_h264_nalus(h264_buf, h264_len,
                                                nalus, (int)(sizeof(nalus) / sizeof(nalus[0])));
    if (nalu_count <= 0) {
        return;
    }

    for (int i = 0; i < nalu_count; i++) {
        const uint8_t *nalu = &h264_buf[nalus[i].offset];

        if (nalus[i].type == MEDIA_STORAGE_VIDEO_NALU_TYPE_SPS) {
            esp_err_t ret = media_storage_replace_nalu(&s_media.video_sps, &s_media.video_sps_len,
                                                       nalu, nalus[i].len);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "更新视频 SPS 失败: 0x%x", ret);
            }
        } else if (nalus[i].type == MEDIA_STORAGE_VIDEO_NALU_TYPE_PPS) {
            esp_err_t ret = media_storage_replace_nalu(&s_media.video_pps, &s_media.video_pps_len,
                                                       nalu, nalus[i].len);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "更新视频 PPS 失败: 0x%x", ret);
            }
        }
    }
}

static esp_err_t media_storage_video_open_segment(void)
{
    esp_err_t ret;
    media_mp4_writer_config_t writer_cfg;
    char date_tag[MEDIA_STORAGE_DATE_TAG_LEN] = {0};
    char timestamp[MEDIA_STORAGE_TIMESTAMP_LEN] = {0};
    char tmp_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    char final_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    uint32_t segment_index = s_media.video_segment_index + 1U;
    tf_card_info_t tf_info = {0};

    if (media_mp4_writer_is_open(&s_media.video_writer)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!tf_card_is_mounted()) {
        ESP_LOGW(TAG, "TF 卡未挂载，无法创建录像分段");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_media.video_sps || !s_media.video_pps ||
        s_media.video_sps_len == 0U || s_media.video_pps_len == 0U) {
        static TickType_t s_last_param_log_tick = 0;
        TickType_t now = xTaskGetTickCount();

        if (s_last_param_log_tick == 0 ||
            now - s_last_param_log_tick >= pdMS_TO_TICKS(1000)) {
            s_last_param_log_tick = now;
            ESP_LOGW(TAG, "录像等待 H.264 SPS/PPS 参数，暂不创建 MP4 分段");
        }
        return ESP_ERR_INVALID_STATE;
    }

    ret = tf_card_get_info(&tf_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取 TF 卡状态失败，无法创建录像分段: 0x%x (%s)",
                 ret, esp_err_to_name(ret));
        return ret;
    }

    if (!media_storage_can_resume_record_with_info(&tf_info)) {
        if (!media_storage_is_video_overwrite_allowed()) {
            ESP_LOGW(TAG, "TF 卡剩余空间不足，禁止启动新录像");
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }

        ret = media_storage_reclaim_space_for_record(&tf_info);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "TF 卡已满，删除旧视频后仍无法继续录像: 0x%x (%s)",
                     ret, esp_err_to_name(ret));
            return ESP_ERR_MEDIA_STORAGE_TF_FULL;
        }
    }

    media_storage_build_timestamp(date_tag, sizeof(date_tag), timestamp, sizeof(timestamp));

    ret = media_storage_prepare_session(date_tag);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = media_storage_build_video_file_path(tmp_path, sizeof(tmp_path),
                                              s_media.video_dir, timestamp,
                                              segment_index, ".tmp");
    if (ret != ESP_OK) {
        return ret;
    }

    ret = media_storage_build_video_file_path(final_path, sizeof(final_path),
                                              s_media.video_dir, timestamp,
                                              segment_index, ".mp4");
    if (ret != ESP_OK) {
        return ret;
    }

    memset(&writer_cfg, 0, sizeof(writer_cfg));
    writer_cfg.width = s_media.video_width;
    writer_cfg.height = s_media.video_height;
    writer_cfg.timescale = MEDIA_STORAGE_VIDEO_TIMESCALE;
    writer_cfg.default_sample_duration = s_media.video_sample_duration;
    writer_cfg.sps = s_media.video_sps;
    writer_cfg.sps_len = s_media.video_sps_len;
    writer_cfg.pps = s_media.video_pps;
    writer_cfg.pps_len = s_media.video_pps_len;

    ret = media_mp4_writer_open(&s_media.video_writer, tmp_path, &writer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开 MP4 分段失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        remove(tmp_path);
        return ret;
    }

    ret = media_storage_copy_text(s_media.video_tmp_path, sizeof(s_media.video_tmp_path), tmp_path);
    if (ret == ESP_OK) {
        ret = media_storage_copy_text(s_media.video_final_path, sizeof(s_media.video_final_path), final_path);
    }
    if (ret != ESP_OK) {
        media_mp4_writer_deinit(&s_media.video_writer);
        remove(tmp_path);
        media_storage_reset_video_segment_state();
        return ret;
    }

    s_media.video_segment_index = segment_index;
    s_media.video_segment_start_us = esp_timer_get_time();
    s_media.video_segment_frame_count = 0;
    s_media.video_switch_pending = false;
    media_storage_set_video_overwrite_allowed(true);

    ESP_LOGI(TAG, "录像分段已创建 | 序号=%04" PRIu32 " | 临时文件=%s",
             s_media.video_segment_index, s_media.video_tmp_path);
    return ESP_OK;
}

static void media_storage_video_abort_segment(void)
{
    if (media_mp4_writer_is_open(&s_media.video_writer)) {
        media_mp4_writer_deinit(&s_media.video_writer);
    }

    if (s_media.video_tmp_path[0] != '\0') {
        remove(s_media.video_tmp_path);
    }

    ESP_LOGW(TAG, "录像分段已丢弃 | 序号=%04" PRIu32, s_media.video_segment_index);
    media_storage_reset_video_segment_state();
}

static esp_err_t media_storage_video_close_segment(void)
{
    esp_err_t ret = ESP_OK;
    uint32_t segment_index = s_media.video_segment_index;
    uint32_t frame_count = s_media.video_segment_frame_count;
    char tmp_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    char final_path[MEDIA_STORAGE_MAX_PATH_LEN] = {0};
    uint32_t duration_ms = 0;

    if (!media_mp4_writer_is_open(&s_media.video_writer)) {
        media_storage_reset_video_segment_state();
        return ESP_OK;
    }

    media_storage_copy_text(tmp_path, sizeof(tmp_path), s_media.video_tmp_path);
    media_storage_copy_text(final_path, sizeof(final_path), s_media.video_final_path);

    ret = media_mp4_writer_close(&s_media.video_writer);
    if (ret != ESP_OK || frame_count == 0U) {
        remove(tmp_path);
        media_storage_reset_video_segment_state();

        if (ret == ESP_OK && frame_count == 0U) {
            return ESP_FAIL;
        }

        ESP_LOGE(TAG, "关闭 MP4 分段失败 | 序号=%04" PRIu32 " | ret=0x%x (%s)",
                 segment_index, ret, esp_err_to_name(ret));
        return ret;
    }

    if (rename(tmp_path, final_path) != 0) {
        remove(tmp_path);
        media_storage_reset_video_segment_state();
        ESP_LOGE(TAG, "MP4 临时文件改名失败 | %s -> %s | errno=%d",
                 tmp_path, final_path, errno);
        return ESP_FAIL;
    }

    if (s_media.video_sample_duration > 0U) {
        uint64_t duration_ms_u64 =
            ((uint64_t)frame_count * (uint64_t)s_media.video_sample_duration * 1000ULL) /
            (uint64_t)MEDIA_STORAGE_VIDEO_TIMESCALE;
        if (duration_ms_u64 > UINT32_MAX) {
            duration_ms = UINT32_MAX;
        } else {
            duration_ms = (uint32_t)duration_ms_u64;
        }
    }

    if (duration_ms > 0U) {
        struct stat st = {0};

        if (stat(final_path, &st) == 0 && st.st_size > 0) {
            media_storage_update_video_average((uint64_t)st.st_size, duration_ms);
        }
    }

    ESP_LOGI(TAG, "录像保存成功 | 序号=%04" PRIu32 " | 帧数=%" PRIu32 " | 路径=%s",
             segment_index, frame_count, final_path);
    media_storage_reset_video_segment_state();
    return ESP_OK;
}

static void media_storage_handle_video_frame(const media_storage_video_frame_t *frame)
{
    bool is_idr;
    esp_err_t ret;

    if (!frame || !frame->buf || frame->len == 0U) {
        return;
    }

    is_idr = (frame->frame_type == ESP_H264_FRAME_TYPE_IDR);
    if (is_idr ||
        !s_media.video_sps || s_media.video_sps_len == 0U ||
        !s_media.video_pps || s_media.video_pps_len == 0U) {
        media_storage_update_video_parameter_sets(frame->buf, frame->len);
    }

    if (!media_storage_is_video_record_requested()) {
        media_storage_take_video_gap();
        if (media_mp4_writer_is_open(&s_media.video_writer)) {
            media_storage_video_close_segment();
        }
        return;
    }

    /* 丢帧后不关闭分段，只等待下一帧 IDR，避免持续生成 1 帧小文件。 */
    if (media_storage_should_skip_video_gap_frame(is_idr)) {
        return;
    }

    if (media_mp4_writer_is_open(&s_media.video_writer)) {
        int64_t elapsed_us = 0;

        if (s_media.video_segment_start_us <= 0) {
            s_media.video_segment_start_us = esp_timer_get_time();
        }

        elapsed_us = esp_timer_get_time() - s_media.video_segment_start_us;

        if (!s_media.video_switch_pending && elapsed_us >= MEDIA_STORAGE_VIDEO_SEGMENT_US) {
            s_media.video_switch_pending = true;
            ESP_LOGI(TAG, "录像分段时长已到，等待下一帧 IDR 切段");
        }

        if (s_media.video_switch_pending && is_idr) {
            ret = media_storage_video_close_segment();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "关闭旧 MP4 分段返回错误: 0x%x", ret);
            }
        }
    }

    if (!media_mp4_writer_is_open(&s_media.video_writer)) {
        if (!is_idr) {
            return;
        }

        ret = media_storage_video_open_segment();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "创建新 MP4 分段失败: 0x%x (%s)", ret, esp_err_to_name(ret));
            return;
        }
    }

    /*
     * Use a compact recording timeline. If the recorder has to skip frames
     * under SD pressure, playback jumps over the missing frames instead of
     * freezing on the previous frame for the elapsed wall-clock gap.
     */
    ret = media_mp4_writer_write_frame(&s_media.video_writer,
                                       frame->buf, frame->len,
                                       s_media.video_segment_frame_count * s_media.video_sample_duration,
                                       is_idr);
    if (ret != ESP_OK) {
        tf_card_info_t tf_info = {0};
        bool tf_space_low = false;

        if (tf_card_get_info(&tf_info) == ESP_OK) {
            tf_space_low = !media_storage_can_resume_record_with_info(&tf_info);
        }

        if (tf_space_low) {
            ESP_LOGW(TAG, "TF 卡空间不足，当前录像分段写入失败，等待下一帧 IDR 后尝试删除旧视频继续录像");
        } else {
            ESP_LOGE(TAG, "写入 MP4 帧失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        }
        media_storage_video_abort_segment();
        return;
    }

    s_media.video_segment_frame_count++;
}

static void media_storage_video_task(void *arg)
{
    uint32_t slot_index = 0;
    uint32_t handled_since_yield = 0;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_media.video_queue, &slot_index,
                          pdMS_TO_TICKS(MEDIA_STORAGE_VIDEO_WAIT_MS)) == pdTRUE) {
            if (slot_index < s_media.video_slot_count) {
                media_storage_handle_video_frame(&s_media.video_frames[slot_index]);
                s_media.video_frames[slot_index].len = 0;
                media_storage_release_video_slot((int)slot_index);
                handled_since_yield++;
                if (handled_since_yield >= MEDIA_STORAGE_VIDEO_TASK_YIELD_FRAMES) {
                    UBaseType_t pending_items = uxQueueMessagesWaiting(s_media.video_queue);

                    handled_since_yield = 0;
                    if (pending_items == 0U) {
                        vTaskDelay(1);
                    } else {
                        taskYIELD();
                    }
                }
            }
        } else if (!media_storage_is_video_record_requested() &&
                   media_mp4_writer_is_open(&s_media.video_writer)) {
            media_storage_video_close_segment();
            handled_since_yield = 0;
        } else {
            handled_since_yield = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
esp_err_t media_storage_sync_time_from_unix_ms(int64_t unix_ms)
{
    struct timeval tv = {0};
    struct tm tm_info = {0};
    time_t sec;
    int64_t valid_unix_ms = MEDIA_STORAGE_VALID_UNIX_SEC * MEDIA_STORAGE_UNIX_MSEC_PER_SEC;
    int64_t msec;

    if (unix_ms < valid_unix_ms) {
        ESP_LOGW(TAG, "忽略无效时间同步请求: unix_ms=%" PRId64, unix_ms);
        return ESP_ERR_INVALID_ARG;
    }

    media_storage_set_timezone_once();

    sec = (time_t)(unix_ms / MEDIA_STORAGE_UNIX_MSEC_PER_SEC);
    msec = unix_ms % MEDIA_STORAGE_UNIX_MSEC_PER_SEC;
    tv.tv_sec = sec;
    tv.tv_usec = (suseconds_t)(msec * 1000LL);

    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "设置系统时间失败: errno=%d", errno);
        return ESP_FAIL;
    }

    localtime_r(&sec, &tm_info);
    ESP_LOGI(TAG, "系统时间已同步: %04d-%02d-%02d %02d:%02d:%02d.%03" PRId64,
             tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, msec);
    return ESP_OK;
}

esp_err_t media_storage_init(void)
{
    if (s_media.initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(tf_card_is_mounted(), ESP_ERR_INVALID_STATE, TAG,
                        "TF 卡未挂载，无法初始化媒体存储模块");

    memset(&s_media, 0, sizeof(s_media));
    media_storage_set_timezone_once();

    s_media.session_mutex = xSemaphoreCreateMutex();
    if (!s_media.session_mutex) {
        return ESP_ERR_NO_MEM;
    }

    media_mp4_writer_init(&s_media.video_writer);
    s_media.initialized = true;
    ESP_LOGI(TAG, "媒体存储模块已初始化，照片与录像链路按需创建");
    return ESP_OK;
}

void media_storage_deinit(void)
{
    if (s_media.photo_task_handle) {
        vTaskDelete(s_media.photo_task_handle);
        s_media.photo_task_handle = NULL;
    }

    if (s_media.video_task_handle) {
        vTaskDelete(s_media.video_task_handle);
        s_media.video_task_handle = NULL;
    }

    if (s_media.video_queue) {
        vQueueDelete(s_media.video_queue);
        s_media.video_queue = NULL;
    }

    if (s_media.jpeg_handle) {
        jpeg_del_encoder_engine(s_media.jpeg_handle);
        s_media.jpeg_handle = NULL;
    }

    if (s_media.ppa_client) {
        ppa_unregister_client(s_media.ppa_client);
        s_media.ppa_client = NULL;
    }

    if (s_media.session_mutex) {
        vSemaphoreDelete(s_media.session_mutex);
        s_media.session_mutex = NULL;
    }

    media_mp4_writer_deinit(&s_media.video_writer);
    media_storage_free_photo_buffers();
    media_storage_free_video_buffers();
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

    if (width == 0U || height == 0U || (width % 2U) != 0U || (height % 2U) != 0U) {
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

esp_err_t media_storage_prepare_video_record(uint32_t width, uint32_t height, uint32_t fps)
{
    size_t frame_buf_size;
    uint32_t slot_count;
    esp_err_t ret;
    bool record_requested_before;

    if (!s_media.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (width == 0U || height == 0U || fps == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    frame_buf_size = media_storage_calc_video_frame_buf_size(width, height);
    slot_count = media_storage_calc_video_slot_count(width, height);

    if (s_media.video_prepared &&
        s_media.video_width == width &&
        s_media.video_height == height &&
        s_media.video_fps == fps &&
        s_media.video_frame_buf_size == frame_buf_size &&
        s_media.video_slot_count == slot_count) {
        return media_storage_ensure_video_task();
    }

    record_requested_before = media_storage_is_video_record_requested();
    media_storage_free_video_buffers();

    if (!s_media.video_queue) {
        s_media.video_queue = xQueueCreate(MEDIA_STORAGE_VIDEO_QUEUE_LEN, sizeof(uint32_t));
        if (!s_media.video_queue) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        xQueueReset(s_media.video_queue);
    }

    s_media.video_slot_count = slot_count;

    for (uint32_t i = 0; i < s_media.video_slot_count; i++) {
        s_media.video_frames[i].buf = (uint8_t *)heap_caps_calloc(1, frame_buf_size,
                                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_media.video_frames[i].buf) {
            ESP_LOGE(TAG, "申请录像帧缓冲失败 | index=%d | size=%zu", i, frame_buf_size);
            media_storage_free_video_buffers();
            return ESP_ERR_NO_MEM;
        }

        s_media.video_frames[i].capacity = frame_buf_size;
    }

    s_media.video_width = width;
    s_media.video_height = height;
    s_media.video_fps = fps;
    s_media.video_frame_buf_size = frame_buf_size;
    s_media.video_sample_duration = MEDIA_STORAGE_VIDEO_TIMESCALE / fps;
    if (s_media.video_sample_duration == 0U) {
        s_media.video_sample_duration = 1U;
    }
    s_media.video_prepared = true;
    s_media.video_adaptive_save_interval = media_storage_base_save_interval();
    s_media.video_pressure_skip_gops = 0;
    s_media.video_recovery_gop_count = 0;
    s_media.video_last_idr_used_slots = 0;
    s_media.video_non_idr_frame_count = 0;

    if (record_requested_before) {
        portENTER_CRITICAL(&s_media_lock);
        s_media.video_record_requested = true;
        portEXIT_CRITICAL(&s_media_lock);
    }

    ret = media_storage_ensure_video_task();
    if (ret != ESP_OK) {
        media_storage_free_video_buffers();
        return ret;
    }

    ESP_LOGI(TAG, "录像缓冲已准备 | %ux%u@%u | 队列=%u/%u | 帧缓冲=%zu",
             (unsigned)width, (unsigned)height, (unsigned)fps,
             (unsigned)s_media.video_slot_count,
             (unsigned)MEDIA_STORAGE_VIDEO_QUEUE_LEN, frame_buf_size);
    ESP_LOGI(TAG, "录像保存策略 | 每 %" PRIu32 " 个 GOP 保存 1 个，优先保证 RTSP 实时性",
             (uint32_t)MEDIA_STORAGE_VIDEO_SAVE_GOP_INTERVAL);
    if (record_requested_before) {
        ESP_LOGI(TAG, "录像缓冲准备完成，恢复此前保留的录像请求");
    }
    return ESP_OK;
}

esp_err_t media_storage_request_photo(void)
{
    esp_err_t ret;
    bool accepted = false;
    tf_card_info_t tf_info = {0};

    if (!s_media.initialized) {
        ESP_LOGW(TAG, "媒体存储模块未初始化，拒绝拍照请求");
        return ESP_ERR_INVALID_STATE;
    }

    if (!tf_card_is_mounted()) {
        ESP_LOGW(TAG, "TF 卡未挂载，拒绝拍照请求");
        return ESP_ERR_INVALID_STATE;
    }

    ret = tf_card_get_info(&tf_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取 TF 卡状态失败，拒绝拍照请求: 0x%x (%s)",
                 ret, esp_err_to_name(ret));
        return ret;
    }
    if (!media_storage_can_capture_with_info(&tf_info)) {
        ESP_LOGW(TAG, "TF 卡剩余空间不足，拒绝拍照请求");
        return ESP_ERR_MEDIA_STORAGE_TF_FULL;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (!s_media.photo_pending && !s_media.photo_job_busy) {
        s_media.photo_pending = true;
        s_media.photo_skip_frames = 0;
        accepted = true;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (!accepted) {
        ESP_LOGW(TAG, "已有拍照任务正在处理中，忽略新的拍照请求");
        return ESP_ERR_INVALID_STATE;
    }

    ret = media_storage_ensure_photo_task();
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&s_media_lock);
        s_media.photo_pending = false;
        s_media.photo_skip_frames = 0;
        portEXIT_CRITICAL(&s_media_lock);
        ESP_LOGW(TAG, "拍照后台任务未就绪: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "已收到拍照请求，等待下一帧图像");
    return ESP_OK;
}

void media_storage_start_video_record(void)
{
    esp_err_t ret;
    bool need_log = false;
    bool prepared = false;
    tf_card_info_t tf_info = {0};

    if (!s_media.initialized) {
        return;
    }

    if (!tf_card_is_mounted()) {
        ESP_LOGW(TAG, "TF 卡未挂载，忽略录像请求");
        return;
    }

    ret = tf_card_get_info(&tf_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读取 TF 卡状态失败，忽略录像请求: 0x%x (%s)",
                 ret, esp_err_to_name(ret));
        return;
    }
    if (!media_storage_can_start_record_with_info(&tf_info)) {
        ESP_LOGW(TAG, "TF 卡剩余空间不足，拒绝启动新录像");
        return;
    }

    prepared = s_media.video_prepared;
    if (!prepared) {
        portENTER_CRITICAL(&s_media_lock);
        if (!s_media.video_record_requested) {
            s_media.video_record_requested = true;
            s_media.video_overwrite_allowed = false;
            s_media.tf_overwriting_old_video = false;
            s_media.video_gap_pending = false;
            s_media.video_save_current_gop = false;
            s_media.video_save_gop_count = 0;
            s_media.video_adaptive_save_interval = media_storage_base_save_interval();
            s_media.video_pressure_skip_gops = 0;
            s_media.video_recovery_gop_count = 0;
            s_media.video_last_idr_used_slots = 0;
            s_media.video_non_idr_frame_count = 0;
            need_log = true;
        }
        portEXIT_CRITICAL(&s_media_lock);

        if (need_log) {
            ESP_LOGW(TAG, "录像缓冲尚未准备完成，已保留录像请求");
        }
        return;
    }

    ret = media_storage_ensure_video_task();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "录像后台任务尚未就绪: 0x%x", ret);
        return;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (!s_media.video_record_requested) {
        s_media.video_record_requested = true;
        s_media.video_overwrite_allowed = false;
        s_media.tf_overwriting_old_video = false;
        s_media.video_gap_pending = false;
        s_media.video_save_current_gop = false;
        s_media.video_save_gop_count = 0;
        s_media.video_adaptive_save_interval = media_storage_base_save_interval();
        s_media.video_pressure_skip_gops = 0;
        s_media.video_recovery_gop_count = 0;
        s_media.video_last_idr_used_slots = 0;
        s_media.video_non_idr_frame_count = 0;
        need_log = true;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (need_log) {
        ESP_LOGI(TAG, "录像请求已生效，等待下一帧 IDR 创建 MP4 分段");
    }
}

void media_storage_stop_video_record(void)
{
    bool need_log = false;

    if (!s_media.initialized) {
        return;
    }

    portENTER_CRITICAL(&s_media_lock);
    if (s_media.video_record_requested) {
        s_media.video_record_requested = false;
        s_media.video_overwrite_allowed = false;
        s_media.tf_overwriting_old_video = false;
        s_media.video_save_current_gop = false;
        need_log = true;
    }
    portEXIT_CRITICAL(&s_media_lock);

    if (need_log) {
        ESP_LOGI(TAG, "录像请求已停止，后台将关闭当前 MP4 分段");
    }
}

void media_storage_process_camera_frame(const uint8_t *yuv420_buf, size_t yuv420_len,
                                        uint32_t width, uint32_t height)
{
    size_t expected_len;

    if (!s_media.initialized || !yuv420_buf || width == 0U || height == 0U) {
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
    if (yuv420_len != 0U && yuv420_len < expected_len) {
        ESP_LOGE(TAG, "YUV420 帧长度不足: %zu < %zu", yuv420_len, expected_len);
        media_storage_cancel_photo_job(false);
        return;
    }

    if (media_storage_prepare_photo_buffers(width, height) != ESP_OK) {
        media_storage_cancel_photo_job(false);
        return;
    }

    memcpy(s_media.yuv420_buf, yuv420_buf, expected_len);
    s_media.frame_width = width;
    s_media.frame_height = height;
    s_media.frame_len = expected_len;

    xTaskNotifyGive(s_media.photo_task_handle);
}

void media_storage_process_h264_frame(const uint8_t *h264_buf, size_t h264_len,
                                      esp_h264_frame_type_t frame_type, uint32_t pts)
{
    int slot_index;
    uint32_t queue_value;
    media_storage_video_frame_t *frame;

    if (!s_media.initialized || !s_media.video_prepared || !h264_buf || h264_len == 0U) {
        return;
    }

    if (!media_storage_is_video_record_requested()) {
        return;
    }

    if (!s_media.video_queue) {
        return;
    }

    if (media_storage_should_skip_video_input(frame_type)) {
        return;
    }

    if (h264_len > s_media.video_frame_buf_size) {
        media_storage_log_video_drop("frame_too_large", h264_len);
        return;
    }

    slot_index = media_storage_acquire_video_slot();
    if (slot_index < 0) {
        media_storage_log_video_drop("no_free_slot", h264_len);
        return;
    }

    frame = &s_media.video_frames[slot_index];
    memcpy(frame->buf, h264_buf, h264_len);
    frame->len = h264_len;
    frame->frame_type = frame_type;
    frame->pts = pts;

    queue_value = (uint32_t)slot_index;
    if (xQueueSend(s_media.video_queue, &queue_value, 0) != pdTRUE) {
        frame->len = 0;
        media_storage_release_video_slot(slot_index);
        media_storage_log_video_drop("queue_full", h264_len);
    }
}
