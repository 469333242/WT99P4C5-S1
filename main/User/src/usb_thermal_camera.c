/**
 * @file usb_thermal_camera.c
 * @brief USB 热像仪 UVC 持续采集与灰度帧转换
 *
 * 当前阶段目标：
 *   1. 监听 USB UVC 热像仪热插拔事件
 *   2. 设备接入后持续采集最新一帧 Y16 原始热数据
 *   3. 提供最新 Y16、8bit 灰度图和 H.264 编码器灰度输入帧读取接口
 *
 * 本模块提供独立的热像仪 RTSP 推流链路；通过主入口的视频源参数保证
 * USB 热像仪和 OV5647 MIPI 摄像头不同时工作。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "esp_h264_enc_single_hw.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "esp_private/uvc_stream.h"

#include "rtsp_server.h"
#include "usb_thermal_camera.h"

static const char *TAG = "usb_thermal";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define USB_THERMAL_CAMERA_ENABLE              1       /* USB 热像仪总开关：0=关闭，1=开启 */
#define USB_THERMAL_CAMERA_PRINT_DESC_ENABLE   0       /* 完整 USB 描述符打印开关：调试 VID/PID 时再开启 */
#define USB_THERMAL_CAMERA_ALLOW_UNKNOWN_FMT   1       /* 允许尝试 UVC 未识别格式，常见于 Y16/GRAY16 热像仪 */
#define USB_THERMAL_CAMERA_PERIPHERAL_MAP      0x00    /* USB 外设选择：0=使用 IDF 默认，0x1/0x2 可用于切换控制器 */
#define USB_THERMAL_CAMERA_READY_DELAY_MS      500     /* USB Host/UVC 安装完成后打开根端口前的稳定等待时间 */
#define USB_THERMAL_CAMERA_DEVICE_QUEUE_LEN    4       /* UVC 接入事件队列长度 */
#define USB_THERMAL_CAMERA_USB_TASK_STACK      4096    /* USB Host 事件任务栈 */
#define USB_THERMAL_CAMERA_USB_TASK_PRIORITY   15      /* USB Host 事件任务优先级 */
#define USB_THERMAL_CAMERA_UVC_TASK_STACK      4096    /* UVC 驱动任务栈 */
#define USB_THERMAL_CAMERA_UVC_TASK_PRIORITY   16      /* UVC 驱动任务优先级 */
#define USB_THERMAL_CAMERA_CAPTURE_TASK_STACK  4096    /* 热像仪采集任务栈 */
#define USB_THERMAL_CAMERA_CAPTURE_TASK_PRIORITY 12    /* 热像仪采集任务优先级 */
#define USB_THERMAL_CAMERA_OPEN_TIMEOUT_MS     5000    /* 打开 UVC 流超时时间 */
#define USB_THERMAL_CAMERA_CLOSE_RETRY_COUNT   5       /* 关闭 UVC 流重试次数 */
#define USB_THERMAL_CAMERA_CLOSE_RETRY_MS      200     /* 关闭 UVC 流失败后的重试间隔 */
#define USB_THERMAL_CAMERA_PAUSE_DRAIN_MS      250     /* 停止抓帧后等待 BULK 传输回调收尾 */
#define USB_THERMAL_CAMERA_FRAME_BUF_COUNT     2       /* UVC 帧缓冲数量，降低常驻内存占用 */
#define USB_THERMAL_CAMERA_URB_COUNT           4       /* USB 传输 URB 数量 */
#define USB_THERMAL_CAMERA_URB_SIZE            (10 * 1024)
#define USB_THERMAL_CAMERA_DEFAULT_FPS         50.0f   /* 描述符未给出帧率时的请求帧率 */
#define USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL 2U      /* 当前热像仪一像素 16bit */
#define USB_THERMAL_CAMERA_FRAME_WAIT_MS       50      /* 读取最新帧时等待互斥锁的最长时间 */
#define USB_THERMAL_CAMERA_STATS_INTERVAL_US   (5ULL * 1000ULL * 1000ULL)
#define USB_THERMAL_CAMERA_CONVERT_LOG_US      (5ULL * 1000ULL * 1000ULL)
#define USB_THERMAL_CAMERA_NEUTRAL_CHROMA      128     /* 灰度视频帧的中性 U/V 值 */
#define USB_THERMAL_CAMERA_RTSP_TASK_STACK     (8 * 1024)
#define USB_THERMAL_CAMERA_RTSP_TASK_PRIORITY  13
#define USB_THERMAL_CAMERA_RTSP_TASK_CORE      1
#define USB_THERMAL_CAMERA_RTSP_FPS            25U     /* 热像仪 RTSP 输出帧率，低于 USB 采集帧率以减轻编码压力 */
#define USB_THERMAL_CAMERA_RTSP_GOP            5U
#define USB_THERMAL_CAMERA_RTSP_BITRATE        1200000U
#define USB_THERMAL_CAMERA_RTSP_QP_MIN         28U
#define USB_THERMAL_CAMERA_RTSP_QP_MAX         42U
#define USB_THERMAL_CAMERA_H264_OUT_BUF_SIZE   (256U * 1024U) /* 热像仪灰度 H.264 单帧输出缓冲，避免挤占 SDIO RX 内存 */
#define USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS  20U
#define USB_THERMAL_CAMERA_RTSP_STATS_US       (10ULL * 1000ULL * 1000ULL)
#define USB_THERMAL_CAMERA_RTSP_WIDTH          512U    /* 当前热像仪实测宽度 */
#define USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT     390U    /* 当前热像仪实测高度 */
#define USB_THERMAL_CAMERA_RTSP_ENC_HEIGHT     400U    /* H.264 按 16 像素宏块对齐，底部补黑边 */
#define USB_THERMAL_CAMERA_RTSP_IDLE_RELEASE_MS 1000U  /* 客户端断开后释放编码资源，给 SDIO/WiFi 留内存 */

#if USB_THERMAL_CAMERA_ENABLE

typedef struct {
    uint8_t dev_addr;          /* USB 设备地址 */
    uint8_t stream_index;      /* UVC 视频流索引 */
    size_t  frame_info_num;    /* 当前流支持的格式数量 */
} usb_thermal_camera_event_t;

typedef struct {
    SemaphoreHandle_t frame_mutex;
    SemaphoreHandle_t convert_mutex;
    uvc_host_stream_hdl_t stream_hdl;
    uint8_t *latest_y16;
    size_t latest_y16_size;
    uint8_t *convert_y16;
    size_t convert_y16_size;
    uint32_t width;
    uint32_t height;
    size_t y16_len;
    uint32_t sequence;
    int64_t timestamp_us;
    uint16_t min_value;
    uint16_t max_value;
    uint32_t received_frames;
    uint32_t dropped_frames;
    uint32_t invalid_frames;
    uint32_t stats_frames;
    int64_t stats_start_us;
    int64_t last_gray8_log_us;
    int64_t last_video_log_us;
    bool ready;
    bool connected;
    bool first_frame_logged;
    bool stream_closed;
} usb_thermal_camera_capture_t;

typedef struct {
    EventGroupHandle_t event;
    TaskHandle_t task_handle;
    esp_h264_enc_handle_t h264_enc;
    uint8_t *raw_buf;
    size_t raw_buf_size;
    uint8_t *h264_out_buf;
    size_t h264_out_buf_size;
    uint32_t width;
    uint32_t height;
    uint32_t rtp_ts_step;
    uint32_t pts;
    bool initialized;
    bool encoder_ready;
} usb_thermal_camera_rtsp_t;

static QueueHandle_t s_device_queue;
static TaskHandle_t s_usb_task_handle;
static TaskHandle_t s_capture_task_handle;
static bool s_started;
static usb_thermal_camera_capture_t s_capture;
static usb_thermal_camera_rtsp_t s_rtsp;

#define USB_THERMAL_RTSP_START_BIT BIT0

static const char *s_uvc_format_name[] = {
    "DEFAULT",
    "MJPEG",
    "YUY2",
    "H264",
    "H265",
};

static void usb_thermal_camera_release_capture_resources(void)
{
    free(s_capture.latest_y16);
    free(s_capture.convert_y16);

    if (s_capture.frame_mutex) {
        vSemaphoreDelete(s_capture.frame_mutex);
    }
    if (s_capture.convert_mutex) {
        vSemaphoreDelete(s_capture.convert_mutex);
    }

    memset(&s_capture, 0, sizeof(s_capture));
}

static float uvc_interval_to_fps(uint32_t interval)
{
    if (interval == 0U) {
        return 0.0f;
    }

    return 10000000.0f / (float)interval;
}

static const char *uvc_format_to_string(enum uvc_host_stream_format format)
{
    if ((int)format < 0 ||
        (size_t)format >= (sizeof(s_uvc_format_name) / sizeof(s_uvc_format_name[0]))) {
        return "UNKNOWN";
    }

    return s_uvc_format_name[format];
}

static bool uvc_format_is_unknown(enum uvc_host_stream_format format)
{
    return (int)format < 0 ||
           (size_t)format >= (sizeof(s_uvc_format_name) / sizeof(s_uvc_format_name[0]));
}

static bool uvc_format_can_capture(enum uvc_host_stream_format format)
{
#if USB_THERMAL_CAMERA_ALLOW_UNKNOWN_FMT
    /* 当前热像仪上报为 UNKNOWN，实测帧长匹配 16bit 原始热数据。
     * 不把 MJPEG/YUY2/H264 当作 Y16 处理，避免误解码。 */
    return uvc_format_is_unknown(format);
#else
    return false;
#endif
}

static uint32_t uvc_format_priority(enum uvc_host_stream_format format)
{
    switch (format) {
        default:
            return uvc_format_is_unknown(format) ? 0 : 100;
    }
}

static bool uvc_is_better_capture_format(const uvc_host_frame_info_t *candidate,
                                         const uvc_host_frame_info_t *current)
{
    uint32_t candidate_pixels = candidate->h_res * candidate->v_res;
    uint32_t current_pixels = current->h_res * current->v_res;
    uint32_t candidate_priority = uvc_format_priority(candidate->format);
    uint32_t current_priority = uvc_format_priority(current->format);

    if (candidate_pixels != current_pixels) {
        return candidate_pixels < current_pixels;
    }

    return candidate_priority < current_priority;
}

static float uvc_select_request_fps(const uvc_host_frame_info_t *info)
{
    float fps;

    if (!info) {
        return USB_THERMAL_CAMERA_DEFAULT_FPS;
    }

    fps = uvc_interval_to_fps(info->default_interval);
    if (fps <= 0.0f) {
        fps = USB_THERMAL_CAMERA_DEFAULT_FPS;
    }

    return fps;
}

static void uvc_log_frame_intervals(const uvc_host_frame_info_t *info)
{
    if (info->interval_type == 0U) {
        ESP_LOGI(TAG,
                 "      帧率范围: %.1f-%.1f fps | 步进 %.1f fps",
                 uvc_interval_to_fps(info->interval_max),
                 uvc_interval_to_fps(info->interval_min),
                 uvc_interval_to_fps(info->interval_step));
        return;
    }

    for (uint8_t i = 0; i < info->interval_type && i < CONFIG_UVC_INTERVAL_ARRAY_SIZE; i++) {
        ESP_LOGI(TAG, "      可选帧率[%u]: %.1f fps",
                 i, uvc_interval_to_fps(info->interval[i]));
    }
}

static void uvc_log_frame_info_list(const uvc_host_frame_info_t *frame_list,
                                    size_t frame_count)
{
    ESP_LOGI(TAG, "USB UVC 视频格式列表:");

    for (size_t i = 0; i < frame_count; i++) {
        const uvc_host_frame_info_t *info = &frame_list[i];

        ESP_LOGI(TAG,
                 "  [%u] %s | %ux%u | 默认 %.1f fps",
                 (unsigned)i,
                 uvc_format_to_string(info->format),
                 info->h_res,
                 info->v_res,
                 uvc_interval_to_fps(info->default_interval));
        uvc_log_frame_intervals(info);
    }
}

static const uvc_host_frame_info_t *uvc_select_capture_format(const uvc_host_frame_info_t *frame_list,
                                                              size_t frame_count)
{
    const uvc_host_frame_info_t *selected = NULL;

    for (size_t i = 0; i < frame_count; i++) {
        const uvc_host_frame_info_t *info = &frame_list[i];

        if (!uvc_format_can_capture(info->format)) {
            continue;
        }

        if (selected == NULL || uvc_is_better_capture_format(info, selected)) {
            selected = info;
        }
    }

    return selected;
}

static uint16_t usb_thermal_read_y16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint8_t usb_thermal_y16_to_gray(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    uint32_t range;

    if (max_value <= min_value) {
        return 0;
    }

    range = (uint32_t)max_value - (uint32_t)min_value;
    return (uint8_t)(((uint32_t)value - (uint32_t)min_value) * 255U / range);
}

static void usb_thermal_scan_y16_range(const uint8_t *data, uint32_t pixels,
                                       uint16_t *out_min, uint16_t *out_max)
{
    uint16_t min_value = UINT16_MAX;
    uint16_t max_value = 0;

    if (!data || pixels == 0U) {
        *out_min = 0;
        *out_max = 0;
        return;
    }

    for (uint32_t i = 0; i < pixels; i++) {
        uint16_t value = usb_thermal_read_y16_le(data + (size_t)i * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);

        if (value < min_value) {
            min_value = value;
        }
        if (value > max_value) {
            max_value = value;
        }
    }

    *out_min = min_value;
    *out_max = max_value;
}

static void usb_thermal_fill_frame_info_locked(usb_thermal_camera_frame_info_t *out_info)
{
    if (!out_info) {
        return;
    }

    out_info->width = s_capture.width;
    out_info->height = s_capture.height;
    out_info->y16_len = s_capture.y16_len;
    out_info->gray8_len = (size_t)s_capture.width * (size_t)s_capture.height;
    out_info->gray_oue_vyy_len = (size_t)s_capture.width * (size_t)s_capture.height * 3U / 2U;
    out_info->sequence = s_capture.sequence;
    out_info->timestamp_us = s_capture.timestamp_us;
    out_info->min_value = s_capture.min_value;
    out_info->max_value = s_capture.max_value;
    out_info->received_frames = s_capture.received_frames;
    out_info->dropped_frames = s_capture.dropped_frames;
    out_info->invalid_frames = s_capture.invalid_frames;
}

static esp_err_t usb_thermal_prepare_latest_buffer(uint32_t width, uint32_t height)
{
    size_t y16_len = (size_t)width * (size_t)height * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL;
    uint8_t *new_latest_buf = NULL;
    uint8_t *new_convert_buf = NULL;
    bool need_latest_buf;
    bool need_convert_buf;

    if (width == 0U || height == 0U || y16_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    need_latest_buf = (!s_capture.latest_y16 || s_capture.latest_y16_size < y16_len);
    need_convert_buf = (!s_capture.convert_y16 || s_capture.convert_y16_size < y16_len);

    if (need_latest_buf) {
#if CONFIG_SPIRAM
        new_latest_buf = heap_caps_malloc(y16_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
        new_latest_buf = heap_caps_malloc(y16_len, MALLOC_CAP_8BIT);
#endif
        if (!new_latest_buf) {
            ESP_LOGE(TAG, "USB 热像仪最新帧缓存分配失败: %zu 字节", y16_len);
            return ESP_ERR_NO_MEM;
        }
    }

    if (need_convert_buf) {
#if CONFIG_SPIRAM
        new_convert_buf = heap_caps_malloc(y16_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
        new_convert_buf = heap_caps_malloc(y16_len, MALLOC_CAP_8BIT);
#endif
        if (!new_convert_buf) {
            free(new_latest_buf);
            ESP_LOGE(TAG, "USB 热像仪转换帧缓存分配失败: %zu 字节", y16_len);
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_capture.convert_mutex &&
        xSemaphoreTake(s_capture.convert_mutex, portMAX_DELAY) != pdTRUE) {
        free(new_latest_buf);
        free(new_convert_buf);
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, portMAX_DELAY) == pdTRUE) {
        if (new_latest_buf) {
            free(s_capture.latest_y16);
            s_capture.latest_y16 = new_latest_buf;
            s_capture.latest_y16_size = y16_len;
        }
        if (new_convert_buf) {
            free(s_capture.convert_y16);
            s_capture.convert_y16 = new_convert_buf;
            s_capture.convert_y16_size = y16_len;
        }
        s_capture.width = width;
        s_capture.height = height;
        s_capture.y16_len = y16_len;
        s_capture.ready = false;
        s_capture.first_frame_logged = false;
        s_capture.stream_closed = false;
        xSemaphoreGive(s_capture.frame_mutex);
    } else {
        if (s_capture.convert_mutex) {
            xSemaphoreGive(s_capture.convert_mutex);
        }
        free(new_latest_buf);
        free(new_convert_buf);
        return ESP_FAIL;
    }

    if (s_capture.convert_mutex) {
        xSemaphoreGive(s_capture.convert_mutex);
    }

    ESP_LOGI(TAG,
             "USB 热像仪 Y16 帧缓存已准备: %"PRIu32"x%"PRIu32" | 最新帧=%zu 字节 | 转换帧=%zu 字节",
             width, height, s_capture.latest_y16_size, s_capture.convert_y16_size);
    return ESP_OK;
}

static bool usb_thermal_frame_matches_y16(const uvc_host_frame_t *frame)
{
    size_t expected_len;

    if (!frame || !frame->data || frame->vs_format.h_res == 0U || frame->vs_format.v_res == 0U) {
        return false;
    }

    expected_len = (size_t)frame->vs_format.h_res *
                   (size_t)frame->vs_format.v_res *
                   USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL;

    return frame->data_len == expected_len;
}

static void usb_thermal_log_capture_stats_locked(int64_t now_us)
{
    int64_t elapsed_us;
    float fps;

    if (s_capture.stats_start_us == 0) {
        s_capture.stats_start_us = now_us;
        s_capture.stats_frames = 0;
        return;
    }

    elapsed_us = now_us - s_capture.stats_start_us;
    if (elapsed_us < (int64_t)USB_THERMAL_CAMERA_STATS_INTERVAL_US) {
        return;
    }

    fps = (float)s_capture.stats_frames * 1000000.0f / (float)elapsed_us;
    ESP_LOGI(TAG,
             "USB 热像仪采集统计 | %"PRIu32"x%"PRIu32" | %.1f fps | Y16范围=%u-%u | 收到=%"PRIu32" | 丢弃=%"PRIu32" | 异常=%"PRIu32,
             s_capture.width,
             s_capture.height,
             fps,
             s_capture.min_value,
             s_capture.max_value,
             s_capture.received_frames,
             s_capture.dropped_frames,
             s_capture.invalid_frames);

    s_capture.stats_start_us = now_us;
    s_capture.stats_frames = 0;
}

static bool uvc_frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    usb_thermal_camera_capture_t *capture = (usb_thermal_camera_capture_t *)user_ctx;
    uvc_host_stream_hdl_t stream_hdl = NULL;
    bool frame_returned = false;

    if (!frame || !capture) {
        return true;
    }

    stream_hdl = capture->stream_hdl;

    if (!usb_thermal_frame_matches_y16(frame)) {
        if (xSemaphoreTake(capture->frame_mutex, 0) == pdTRUE) {
            capture->invalid_frames++;
            xSemaphoreGive(capture->frame_mutex);
        }
        goto return_frame;
    }

    if (xSemaphoreTake(capture->frame_mutex, 0) == pdTRUE) {
        size_t y16_len = frame->data_len;
        uint16_t min_value = 0;
        uint16_t max_value = 0;
        int64_t now_us = esp_timer_get_time();

        if (capture->latest_y16 && capture->latest_y16_size >= y16_len) {
            usb_thermal_scan_y16_range(frame->data,
                                       frame->vs_format.h_res * frame->vs_format.v_res,
                                       &min_value, &max_value);
            memcpy(capture->latest_y16, frame->data, y16_len);

            capture->width = frame->vs_format.h_res;
            capture->height = frame->vs_format.v_res;
            capture->y16_len = y16_len;
            capture->sequence++;
            capture->timestamp_us = now_us;
            capture->min_value = min_value;
            capture->max_value = max_value;
            capture->received_frames++;
            capture->stats_frames++;
            capture->ready = true;
            capture->connected = true;

            if (!capture->first_frame_logged) {
                capture->first_frame_logged = true;
                ESP_LOGI(TAG,
                         "USB 热像仪收到首帧 | %ux%u | %s | %u 字节 | Y16范围=%u-%u",
                         frame->vs_format.h_res,
                         frame->vs_format.v_res,
                         uvc_format_to_string(frame->vs_format.format),
                         (unsigned)frame->data_len,
                         min_value,
                         max_value);

                if (frame->data_len >= 8U) {
                    ESP_LOGI(TAG,
                             "USB 热像仪首帧前 8 字节: %02x %02x %02x %02x %02x %02x %02x %02x",
                             frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                             frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
                }
            }

            usb_thermal_log_capture_stats_locked(now_us);
        } else {
            capture->dropped_frames++;
        }

        xSemaphoreGive(capture->frame_mutex);
    } else {
        /* 回调中不等待互斥锁，避免阻塞 UVC BULK 传输。 */
        if (xSemaphoreTake(capture->frame_mutex, 0) == pdTRUE) {
            capture->dropped_frames++;
            xSemaphoreGive(capture->frame_mutex);
        }
    }

return_frame:
    if (stream_hdl) {
        esp_err_t ret = uvc_host_frame_return(stream_hdl, (uvc_host_frame_t *)frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "USB 热像仪帧缓冲归还失败: 0x%x", ret);
        } else {
            frame_returned = true;
        }
    }

    return !frame_returned;
}

static void uvc_stream_event_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    usb_thermal_camera_capture_t *capture = (usb_thermal_camera_capture_t *)user_ctx;

    if (!event || !capture) {
        return;
    }

    switch (event->type) {
        case UVC_HOST_TRANSFER_ERROR:
            ESP_LOGE(TAG, "USB 热像仪传输错误: 0x%x", event->transfer_error.error);
            break;
        case UVC_HOST_DEVICE_DISCONNECTED:
            ESP_LOGW(TAG, "USB 热像仪已断开");
            if (xSemaphoreTake(capture->frame_mutex, portMAX_DELAY) == pdTRUE) {
                capture->ready = false;
                capture->connected = false;
                capture->stream_closed = false;
                xSemaphoreGive(capture->frame_mutex);
            }

            if (uvc_host_stream_close(event->device_disconnected.stream_hdl) != ESP_OK) {
                ESP_LOGW(TAG, "USB 热像仪断开后关闭视频流失败");
            }
            if (xSemaphoreTake(capture->frame_mutex, portMAX_DELAY) == pdTRUE) {
                if (capture->stream_hdl == event->device_disconnected.stream_hdl) {
                    capture->stream_hdl = NULL;
                }
                capture->stream_closed = true;
                xSemaphoreGive(capture->frame_mutex);
            }
            break;
        case UVC_HOST_FRAME_BUFFER_OVERFLOW:
            ESP_LOGW(TAG, "USB 热像仪帧缓冲溢出，可能需要增大 frame_size 或降低分辨率");
            break;
        case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
            ESP_LOGW(TAG, "USB 热像仪帧缓冲不足，可能是处理过慢或缓冲数量过少");
            break;
#ifdef UVC_HOST_SUSPEND_RESUME_API_SUPPORTED
        case UVC_HOST_DEVICE_SUSPENDED:
            ESP_LOGW(TAG, "USB 热像仪已挂起");
            break;
        case UVC_HOST_DEVICE_RESUMED:
            ESP_LOGI(TAG, "USB 热像仪已恢复");
            break;
#endif
        default:
            ESP_LOGW(TAG, "USB 热像仪未知事件: %d", event->type);
            break;
    }
}

static esp_err_t usb_thermal_camera_close_stream(uvc_host_stream_hdl_t stream_hdl)
{
    esp_err_t err = ESP_OK;

    if (!stream_hdl) {
        return ESP_OK;
    }

    err = uvc_host_stream_pause(stream_hdl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "暂停 USB 热像仪视频流失败: 0x%x", err);
    }
    vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_PAUSE_DRAIN_MS));

    for (uint32_t retry = 0; retry < USB_THERMAL_CAMERA_CLOSE_RETRY_COUNT; retry++) {
        err = uvc_host_stream_close(stream_hdl);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        ESP_LOGW(TAG,
                 "关闭 USB 热像仪视频流失败: 0x%x，等待后重试[%u/%u]",
                 err,
                 (unsigned)(retry + 1U),
                 (unsigned)USB_THERMAL_CAMERA_CLOSE_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_CLOSE_RETRY_MS));
    }

    return err;
}

static void usb_thermal_camera_capture_stream(const usb_thermal_camera_event_t *device_event)
{
    uvc_host_frame_info_t *frame_list = NULL;
    const uvc_host_frame_info_t *selected = NULL;
    size_t frame_list_size = device_event->frame_info_num;
    size_t expected_frame_len;
    esp_err_t err;

    if (frame_list_size == 0U) {
        ESP_LOGW(TAG, "USB UVC 设备未上报可用视频格式");
        return;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, portMAX_DELAY) == pdTRUE) {
        bool active = (s_capture.stream_hdl != NULL);
        xSemaphoreGive(s_capture.frame_mutex);

        if (active) {
            ESP_LOGW(TAG, "USB 热像仪视频流已在运行，忽略新的 UVC 接入事件");
            return;
        }
    }

    frame_list = calloc(frame_list_size, sizeof(uvc_host_frame_info_t));
    if (!frame_list) {
        ESP_LOGE(TAG, "USB UVC 格式列表内存分配失败");
        return;
    }

    err = uvc_host_get_frame_list(device_event->dev_addr,
                                  device_event->stream_index,
                                  (uvc_host_frame_info_t (*)[])frame_list,
                                  &frame_list_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取 USB UVC 格式列表失败: 0x%x", err);
        free(frame_list);
        return;
    }

    uvc_log_frame_info_list(frame_list, frame_list_size);
    selected = uvc_select_capture_format(frame_list, frame_list_size);
    if (!selected) {
        ESP_LOGW(TAG,
                 "未找到当前 UVC 驱动可直接抓帧的格式；若热像仪输出私有热数据，需要后续按厂商协议处理");
        free(frame_list);
        return;
    }

    expected_frame_len = (size_t)selected->h_res *
                         (size_t)selected->v_res *
                         USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL;

    if (uvc_format_is_unknown(selected->format)) {
        ESP_LOGW(TAG,
                 "USB 热像仪格式未被驱动识别，将按 Y16 原始热数据持续采集 | %ux%u | 期望帧长=%zu",
                 selected->h_res,
                 selected->v_res,
                 expected_frame_len);
    } else {
        ESP_LOGI(TAG,
                 "USB 热像仪采集格式选择: %s | %ux%u | 请求 %.1f fps | 期望帧长=%zu",
                 uvc_format_to_string(selected->format),
                 selected->h_res,
                 selected->v_res,
                 uvc_select_request_fps(selected),
                 expected_frame_len);
    }

    err = usb_thermal_prepare_latest_buffer(selected->h_res, selected->v_res);
    if (err != ESP_OK) {
        free(frame_list);
        return;
    }

    uvc_host_stream_config_t stream_config = {
        .event_cb = uvc_stream_event_callback,
        .frame_cb = uvc_frame_callback,
        .user_ctx = &s_capture,
        .usb = {
            .dev_addr = device_event->dev_addr,
            .vid = UVC_HOST_ANY_VID,
            .pid = UVC_HOST_ANY_PID,
            .uvc_stream_index = device_event->stream_index,
        },
        .vs_format = {
            .h_res = selected->h_res,
            .v_res = selected->v_res,
            .fps = uvc_select_request_fps(selected),
            .format = selected->format,
        },
        .advanced = {
            .number_of_frame_buffers = USB_THERMAL_CAMERA_FRAME_BUF_COUNT,
            .frame_size = expected_frame_len,
#if CONFIG_SPIRAM
            .frame_heap_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
#else
            .frame_heap_caps = MALLOC_CAP_8BIT,
#endif
            .number_of_urbs = USB_THERMAL_CAMERA_URB_COUNT,
            .urb_size = USB_THERMAL_CAMERA_URB_SIZE,
        },
    };

    uvc_host_stream_hdl_t stream_hdl = NULL;
    err = uvc_host_stream_open(&stream_config,
                               pdMS_TO_TICKS(USB_THERMAL_CAMERA_OPEN_TIMEOUT_MS),
                               &stream_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "打开 USB 热像仪视频流失败: 0x%x", err);
        free(frame_list);
        return;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, portMAX_DELAY) == pdTRUE) {
        s_capture.stream_hdl = stream_hdl;
        s_capture.connected = true;
        s_capture.ready = false;
        s_capture.received_frames = 0;
        s_capture.dropped_frames = 0;
        s_capture.invalid_frames = 0;
        s_capture.stats_frames = 0;
        s_capture.stats_start_us = 0;
        s_capture.first_frame_logged = false;
        s_capture.stream_closed = false;
        xSemaphoreGive(s_capture.frame_mutex);
    }

    uvc_host_stream_format_t active_format = {0};
    if (uvc_host_stream_format_get(stream_hdl, &active_format) == ESP_OK) {
        ESP_LOGI(TAG,
                 "USB 热像仪协商格式: %s | %ux%u | %.1f fps",
                 uvc_format_to_string(active_format.format),
                 active_format.h_res,
                 active_format.v_res,
                 active_format.fps);
    }

#if USB_THERMAL_CAMERA_PRINT_DESC_ENABLE
    /* 调试设备枚举信息时可开启，日志较长，默认关闭避免影响正常观察。 */
    uvc_host_desc_print(stream_hdl);
#endif

    err = uvc_host_stream_start(stream_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 USB 热像仪视频流失败: 0x%x", err);
        usb_thermal_camera_close_stream(stream_hdl);
        if (xSemaphoreTake(s_capture.frame_mutex, portMAX_DELAY) == pdTRUE) {
            s_capture.stream_hdl = NULL;
            s_capture.connected = false;
            s_capture.ready = false;
            xSemaphoreGive(s_capture.frame_mutex);
        }
        free(frame_list);
        return;
    }

    ESP_LOGI(TAG, "USB 热像仪持续采集已启动，等待最新 Y16 帧");
    free(frame_list);
}

static void usb_thermal_camera_capture_task(void *arg)
{
    usb_thermal_camera_event_t device_event;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_device_queue, &device_event, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG,
                     "开始打开 USB UVC 视频流 | addr=%u | stream=%u | 格式数=%u",
                     device_event.dev_addr,
                     device_event.stream_index,
                     (unsigned)device_event.frame_info_num);
            usb_thermal_camera_capture_stream(&device_event);
        }
    }
}

static void usb_thermal_camera_driver_event_callback(const uvc_host_driver_event_data_t *event,
                                                     void *user_ctx)
{
    (void)user_ctx;

    if (!event || !s_device_queue) {
        return;
    }

    switch (event->type) {
        case UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED: {
            usb_thermal_camera_event_t device_event = {
                .dev_addr = event->device_connected.dev_addr,
                .stream_index = event->device_connected.uvc_stream_index,
                .frame_info_num = event->device_connected.frame_info_num,
            };

            ESP_LOGI(TAG,
                     "检测到 USB UVC 视频接口 | addr=%u | stream=%u | 格式数=%u",
                     device_event.dev_addr,
                     device_event.stream_index,
                     (unsigned)device_event.frame_info_num);

            if (xQueueSend(s_device_queue, &device_event, 0) != pdPASS) {
                ESP_LOGW(TAG, "USB UVC 事件队列已满，忽略本次设备事件");
            }
            break;
        }
        default:
            break;
    }
}

static void usb_host_lib_task(void *arg)
{
    (void)arg;

    while (1) {
        uint32_t event_flags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "USB Host 事件处理失败: 0x%x", err);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0) {
            usb_host_device_free_all();
        }

        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) != 0) {
            ESP_LOGI(TAG, "USB 设备资源已释放，继续等待热插拔");
        }
    }
}

#endif /* USB_THERMAL_CAMERA_ENABLE */

#if USB_THERMAL_CAMERA_ENABLE

static void usb_thermal_rtsp_on_playing(bool playing)
{
    if (!s_rtsp.event) {
        return;
    }

    if (playing) {
        rtsp_reset_tx_stats();
        xEventGroupSetBits(s_rtsp.event, USB_THERMAL_RTSP_START_BIT);
    } else {
        rtsp_reset_tx_stats();
        xEventGroupClearBits(s_rtsp.event, USB_THERMAL_RTSP_START_BIT);
    }
}

static void usb_thermal_rtsp_release_encoder(void)
{
    if (s_rtsp.h264_enc) {
        esp_h264_enc_close(s_rtsp.h264_enc);
        esp_h264_enc_del(s_rtsp.h264_enc);
        s_rtsp.h264_enc = NULL;
    }

    free(s_rtsp.h264_out_buf);
    free(s_rtsp.raw_buf);
    s_rtsp.h264_out_buf = NULL;
    s_rtsp.raw_buf = NULL;
    s_rtsp.h264_out_buf_size = 0;
    s_rtsp.raw_buf_size = 0;
    s_rtsp.pts = 0;
    s_rtsp.encoder_ready = false;
}

static esp_err_t usb_thermal_rtsp_create_encoder(void)
{
    if (s_rtsp.encoder_ready) {
        return ESP_OK;
    }

    /* H.264 编码器和输出缓冲较占内存。
     * 延迟到 RTSP PLAY 后再创建，避免 AP 客户端接入/DHCP 阶段挤占 ESP-Hosted SDIO RX 缓冲。 */
    s_rtsp.raw_buf_size = (size_t)s_rtsp.width * (size_t)s_rtsp.height * 3U / 2U;
    s_rtsp.h264_out_buf_size = USB_THERMAL_CAMERA_H264_OUT_BUF_SIZE;
    s_rtsp.h264_out_buf_size = (s_rtsp.h264_out_buf_size + 63U) & ~63U;

#if CONFIG_SPIRAM
    s_rtsp.raw_buf = heap_caps_aligned_alloc(64, s_rtsp.raw_buf_size,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    s_rtsp.raw_buf = heap_caps_aligned_alloc(64, s_rtsp.raw_buf_size,
                                             MALLOC_CAP_8BIT);
#endif
    if (!s_rtsp.raw_buf) {
        ESP_LOGE(TAG, "USB 热像仪 RTSP 原始帧缓冲分配失败: %zu 字节", s_rtsp.raw_buf_size);
        usb_thermal_rtsp_release_encoder();
        return ESP_ERR_NO_MEM;
    }

#if CONFIG_SPIRAM
    s_rtsp.h264_out_buf = heap_caps_aligned_alloc(64, s_rtsp.h264_out_buf_size,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    s_rtsp.h264_out_buf = heap_caps_aligned_alloc(64, s_rtsp.h264_out_buf_size,
                                                  MALLOC_CAP_8BIT);
#endif
    if (!s_rtsp.h264_out_buf) {
        ESP_LOGE(TAG, "USB 热像仪 RTSP H.264 输出缓冲分配失败: %zu 字节", s_rtsp.h264_out_buf_size);
        usb_thermal_rtsp_release_encoder();
        return ESP_ERR_NO_MEM;
    }

    esp_h264_enc_cfg_hw_t h264_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = USB_THERMAL_CAMERA_RTSP_GOP,
        .fps = USB_THERMAL_CAMERA_RTSP_FPS,
        .res = {
            .width = s_rtsp.width,
            .height = s_rtsp.height,
        },
        .rc = {
            .bitrate = USB_THERMAL_CAMERA_RTSP_BITRATE,
            .qp_min = USB_THERMAL_CAMERA_RTSP_QP_MIN,
            .qp_max = USB_THERMAL_CAMERA_RTSP_QP_MAX,
        },
    };

    esp_h264_err_t h264_ret = esp_h264_enc_hw_new(&h264_cfg, &s_rtsp.h264_enc);
    if (h264_ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "USB 热像仪 H.264 编码器创建失败: %d", h264_ret);
        usb_thermal_rtsp_release_encoder();
        return ESP_FAIL;
    }

    h264_ret = esp_h264_enc_open(s_rtsp.h264_enc);
    if (h264_ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "USB 热像仪 H.264 编码器打开失败: %d", h264_ret);
        usb_thermal_rtsp_release_encoder();
        return ESP_FAIL;
    }

    s_rtsp.encoder_ready = true;
    ESP_LOGI(TAG,
             "USB 热像仪 RTSP 编码资源已创建 | 编码 %"PRIu32"x%"PRIu32"@%"PRIu32"fps | raw=%zu | h264=%zu",
             s_rtsp.width,
             s_rtsp.height,
             (uint32_t)USB_THERMAL_CAMERA_RTSP_FPS,
             s_rtsp.raw_buf_size,
             s_rtsp.h264_out_buf_size);

    return ESP_OK;
}

static esp_err_t usb_thermal_encode_to_h264(const uint8_t *src,
                                            uint8_t *out_buf,
                                            size_t out_buf_size,
                                            size_t *out_len,
                                            esp_h264_frame_type_t *frame_type,
                                            uint32_t *frame_pts)
{
    uint32_t input_pts = s_rtsp.pts;

    esp_h264_enc_in_frame_t in_frame = {
        .raw_data.buffer = (uint8_t *)src,
        .raw_data.len = s_rtsp.width * s_rtsp.height * 3U / 2U,
        .pts = input_pts,
    };
    esp_h264_enc_out_frame_t out_frame = {
        .raw_data.buffer = out_buf,
        .raw_data.len = out_buf_size,
    };

    esp_h264_err_t ret = esp_h264_enc_process(s_rtsp.h264_enc, &in_frame, &out_frame);
    if (ret != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "USB 热像仪 H.264 编码失败: %d", ret);
        return ESP_FAIL;
    }

    *out_len = out_frame.length;
    *frame_type = out_frame.frame_type;
    if (frame_pts) {
        *frame_pts = input_pts;
    }
    s_rtsp.pts = input_pts + s_rtsp.rtp_ts_step;

    return ESP_OK;
}

static void usb_thermal_pad_oue_vyy_frame(uint8_t *frame_buf,
                                          uint32_t width,
                                          uint32_t src_height,
                                          uint32_t enc_height)
{
    size_t row_stride;

    if (!frame_buf || width == 0U || src_height >= enc_height) {
        return;
    }

    row_stride = (size_t)width * 3U / 2U;
    for (uint32_t y = src_height; y < enc_height; y++) {
        uint8_t *dst_row = frame_buf + (size_t)y * row_stride;

        for (uint32_t x = 0; x < width; x += 2U) {
            size_t dst_offset = (size_t)(x / 2U) * 3U;

            dst_row[dst_offset] = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
            dst_row[dst_offset + 1U] = 0;
            dst_row[dst_offset + 2U] = 0;
        }
    }
}

static void usb_thermal_rtsp_task(void *arg)
{
    uint32_t encoded_frame_cnt = 0;
    uint64_t convert_time_total_us = 0;
    uint64_t encode_time_total_us = 0;
    uint64_t push_time_total_us = 0;
    int64_t report_start_us = 0;
    TickType_t frame_delay_ticks = pdMS_TO_TICKS(1000U / USB_THERMAL_CAMERA_RTSP_FPS);

    (void)arg;

    if (frame_delay_ticks == 0) {
        frame_delay_ticks = 1;
    }

    while (1) {
        xEventGroupWaitBits(s_rtsp.event, USB_THERMAL_RTSP_START_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        if (!usb_thermal_camera_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS));
            continue;
        }

        if (!s_rtsp.encoder_ready) {
            esp_err_t enc_ret = usb_thermal_rtsp_create_encoder();
            if (enc_ret != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS));
                continue;
            }
        }

        if (report_start_us == 0) {
            report_start_us = esp_timer_get_time();
            encoded_frame_cnt = 0;
            convert_time_total_us = 0;
            encode_time_total_us = 0;
            push_time_total_us = 0;
        }

        usb_thermal_camera_frame_info_t frame_info = {0};
        int64_t t1 = esp_timer_get_time();
        esp_err_t ret = usb_thermal_camera_get_latest_gray_oue_vyy(s_rtsp.raw_buf,
                                                                   s_rtsp.raw_buf_size,
                                                                   &frame_info);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS));
            continue;
        }
        if (frame_info.width != USB_THERMAL_CAMERA_RTSP_WIDTH ||
            frame_info.height != USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT) {
            ESP_LOGW(TAG,
                     "USB 热像仪分辨率变化，暂不推流 | 当前=%"PRIu32"x%"PRIu32" | 期望=%"PRIu32"x%"PRIu32,
                     frame_info.width,
                     frame_info.height,
                     (uint32_t)USB_THERMAL_CAMERA_RTSP_WIDTH,
                     (uint32_t)USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT);
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS));
            continue;
        }
        usb_thermal_pad_oue_vyy_frame(s_rtsp.raw_buf,
                                      s_rtsp.width,
                                      USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT,
                                      s_rtsp.height);
        int64_t t2 = esp_timer_get_time();
        convert_time_total_us += (uint64_t)(t2 - t1);

        size_t h264_len = 0;
        esp_h264_frame_type_t frame_type = ESP_H264_FRAME_TYPE_INVALID;
        uint32_t frame_pts = 0;
        ret = usb_thermal_encode_to_h264(s_rtsp.raw_buf,
                                         s_rtsp.h264_out_buf,
                                         s_rtsp.h264_out_buf_size,
                                         &h264_len,
                                         &frame_type,
                                         &frame_pts);
        int64_t t3 = esp_timer_get_time();
        encode_time_total_us += (uint64_t)(t3 - t2);

        if (ret == ESP_OK && h264_len > 0U) {
            rtsp_push_h264_frame(s_rtsp.h264_out_buf, h264_len, frame_type, frame_pts);
            int64_t t4 = esp_timer_get_time();

            push_time_total_us += (uint64_t)(t4 - t3);
            encoded_frame_cnt++;

            if ((t4 - report_start_us) >= (int64_t)USB_THERMAL_CAMERA_RTSP_STATS_US) {
                uint32_t elapsed_ms = (uint32_t)((t4 - report_start_us) / 1000);
                rtsp_tx_stats_t tx_stats = {0};
                float encoded_fps = 0.0f;
                float actual_fps = 0.0f;
                float actual_bitrate = 0.0f;

                rtsp_take_tx_stats(&tx_stats);
                if (elapsed_ms > 0U) {
                    encoded_fps = (float)encoded_frame_cnt * 1000.0f / (float)elapsed_ms;
                    actual_fps = (float)tx_stats.frames_sent * 1000.0f / (float)elapsed_ms;
                    actual_bitrate = (float)tx_stats.bytes_sent * 8.0f / (float)elapsed_ms;
                }

                ESP_LOGI(TAG,
                         "USB 热像仪 RTSP 统计 | %"PRIu32"x%"PRIu32" | 编码 %.1f fps | 实际 %.1f fps | 码率 %.0f kbps | Y16范围=%u-%u",
                         s_rtsp.width,
                         s_rtsp.height,
                         encoded_fps,
                         actual_fps,
                         actual_bitrate,
                         frame_info.min_value,
                         frame_info.max_value);

                if (encoded_frame_cnt > 0U) {
                    ESP_LOGI(TAG,
                             "USB 热像仪 RTSP 耗时 | 转换 %.2f ms | 编码 %.2f ms | 推流 %.2f ms",
                             (double)convert_time_total_us / (1000.0 * encoded_frame_cnt),
                             (double)encode_time_total_us / (1000.0 * encoded_frame_cnt),
                             (double)push_time_total_us / (1000.0 * encoded_frame_cnt));
                }

                report_start_us = t4;
                encoded_frame_cnt = 0;
                convert_time_total_us = 0;
                encode_time_total_us = 0;
                push_time_total_us = 0;
            }
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "USB 热像仪 H.264 编码失败，跳过当前帧");
        }

        if ((xEventGroupGetBits(s_rtsp.event) & USB_THERMAL_RTSP_START_BIT) == 0) {
            report_start_us = 0;
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_IDLE_RELEASE_MS));
            if ((xEventGroupGetBits(s_rtsp.event) & USB_THERMAL_RTSP_START_BIT) == 0) {
                ESP_LOGI(TAG, "USB 热像仪 RTSP 无客户端，释放编码资源");
                usb_thermal_rtsp_release_encoder();
            }
        }

        vTaskDelay(frame_delay_ticks);
    }
}

#endif /* USB_THERMAL_CAMERA_ENABLE */

esp_err_t usb_thermal_camera_start(void)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    ESP_LOGI(TAG, "USB 热像仪采集未启用");
    return ESP_OK;
#else
    if (s_started) {
        return ESP_OK;
    }

    memset(&s_capture, 0, sizeof(s_capture));
    s_capture.frame_mutex = xSemaphoreCreateMutex();
    if (!s_capture.frame_mutex) {
        ESP_LOGE(TAG, "USB 热像仪帧互斥锁创建失败");
        return ESP_ERR_NO_MEM;
    }
    s_capture.convert_mutex = xSemaphoreCreateMutex();
    if (!s_capture.convert_mutex) {
        ESP_LOGE(TAG, "USB 热像仪转换互斥锁创建失败");
        usb_thermal_camera_release_capture_resources();
        return ESP_ERR_NO_MEM;
    }

    s_device_queue = xQueueCreate(USB_THERMAL_CAMERA_DEVICE_QUEUE_LEN,
                                  sizeof(usb_thermal_camera_event_t));
    if (!s_device_queue) {
        ESP_LOGE(TAG, "USB 热像仪事件队列创建失败");
        usb_thermal_camera_release_capture_resources();
        return ESP_ERR_NO_MEM;
    }

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = true,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .peripheral_map = USB_THERMAL_CAMERA_PERIPHERAL_MAP,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB Host 安装失败，无法采集 USB 热像仪: 0x%x", err);
        vQueueDelete(s_device_queue);
        s_device_queue = NULL;
        usb_thermal_camera_release_capture_resources();
        return err;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(usb_host_lib_task,
                                                 "usb_host_lib",
                                                 USB_THERMAL_CAMERA_USB_TASK_STACK,
                                                 NULL,
                                                 USB_THERMAL_CAMERA_USB_TASK_PRIORITY,
                                                 &s_usb_task_handle,
                                                 tskNO_AFFINITY);
    if (task_ok != pdTRUE) {
        ESP_LOGE(TAG, "USB Host 事件任务创建失败");
        usb_host_uninstall();
        vQueueDelete(s_device_queue);
        s_device_queue = NULL;
        usb_thermal_camera_release_capture_resources();
        return ESP_ERR_NO_MEM;
    }

    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = USB_THERMAL_CAMERA_UVC_TASK_STACK,
        .driver_task_priority = USB_THERMAL_CAMERA_UVC_TASK_PRIORITY,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = usb_thermal_camera_driver_event_callback,
    };

    err = uvc_host_install(&uvc_driver_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UVC 驱动安装失败: 0x%x", err);
        vTaskDelete(s_usb_task_handle);
        s_usb_task_handle = NULL;
        usb_host_uninstall();
        vQueueDelete(s_device_queue);
        s_device_queue = NULL;
        usb_thermal_camera_release_capture_resources();
        return err;
    }

    task_ok = xTaskCreatePinnedToCore(usb_thermal_camera_capture_task,
                                      "usb_thermal_cap",
                                      USB_THERMAL_CAMERA_CAPTURE_TASK_STACK,
                                      NULL,
                                      USB_THERMAL_CAMERA_CAPTURE_TASK_PRIORITY,
                                      &s_capture_task_handle,
                                      tskNO_AFFINITY);
    if (task_ok != pdTRUE) {
        ESP_LOGE(TAG, "USB 热像仪采集任务创建失败");
        uvc_host_uninstall();
        vTaskDelete(s_usb_task_handle);
        s_usb_task_handle = NULL;
        usb_host_uninstall();
        vQueueDelete(s_device_queue);
        s_device_queue = NULL;
        usb_thermal_camera_release_capture_resources();
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_READY_DELAY_MS));
    err = usb_host_lib_set_root_port_power(true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB Host 根端口上电失败，无法采集 USB 热像仪: 0x%x", err);
        s_started = false;
        uvc_host_uninstall();
        vTaskDelete(s_capture_task_handle);
        s_capture_task_handle = NULL;
        vTaskDelete(s_usb_task_handle);
        s_usb_task_handle = NULL;
        usb_host_uninstall();
        vQueueDelete(s_device_queue);
        s_device_queue = NULL;
        usb_thermal_camera_release_capture_resources();
        return err;
    }

    ESP_LOGI(TAG,
             "USB 热像仪 UVC 采集已启动，根端口已上电 | peripheral_map=0x%x | 请插入标准 UVC 热像仪",
             USB_THERMAL_CAMERA_PERIPHERAL_MAP);
    return ESP_OK;
#endif
}

esp_err_t usb_thermal_camera_rtsp_init(void)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    ESP_LOGI(TAG, "USB 热像仪 RTSP 未启用");
    return ESP_OK;
#else
    if (s_rtsp.initialized) {
        return ESP_OK;
    }

    memset(&s_rtsp, 0, sizeof(s_rtsp));
    s_rtsp.width = USB_THERMAL_CAMERA_RTSP_WIDTH;
    s_rtsp.height = USB_THERMAL_CAMERA_RTSP_ENC_HEIGHT;
    s_rtsp.rtp_ts_step = 90000U / USB_THERMAL_CAMERA_RTSP_FPS;
    if (s_rtsp.rtp_ts_step == 0U) {
        s_rtsp.rtp_ts_step = 1U;
    }

    s_rtsp.event = xEventGroupCreate();
    if (!s_rtsp.event) {
        ESP_LOGE(TAG, "USB 热像仪 RTSP 事件组创建失败");
        return ESP_ERR_NO_MEM;
    }

    rtsp_set_playing_callback(usb_thermal_rtsp_on_playing);

    BaseType_t task_ok = xTaskCreatePinnedToCore(usb_thermal_rtsp_task,
                                                 "usb_thermal_rtsp",
                                                 USB_THERMAL_CAMERA_RTSP_TASK_STACK,
                                                 NULL,
                                                 USB_THERMAL_CAMERA_RTSP_TASK_PRIORITY,
                                                 &s_rtsp.task_handle,
                                                 USB_THERMAL_CAMERA_RTSP_TASK_CORE);
    if (task_ok != pdTRUE) {
        ESP_LOGE(TAG, "USB 热像仪 RTSP 任务创建失败");
        usb_thermal_rtsp_release_encoder();
        vEventGroupDelete(s_rtsp.event);
        memset(&s_rtsp, 0, sizeof(s_rtsp));
        return ESP_ERR_NO_MEM;
    }

    s_rtsp.initialized = true;
    ESP_LOGI(TAG,
             "USB 热像仪 RTSP 链路已初始化 | 采集 %"PRIu32"x%"PRIu32" -> 编码 %"PRIu32"x%"PRIu32"@%"PRIu32"fps | 客户端播放时创建编码资源",
             (uint32_t)USB_THERMAL_CAMERA_RTSP_WIDTH,
             (uint32_t)USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT,
             s_rtsp.width,
             s_rtsp.height,
             (uint32_t)USB_THERMAL_CAMERA_RTSP_FPS);

    return ESP_OK;
#endif
}

bool usb_thermal_camera_is_ready(void)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return false;
#else
    bool ready = false;

    if (!s_capture.frame_mutex) {
        return false;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) == pdTRUE) {
        ready = s_capture.ready;
        xSemaphoreGive(s_capture.frame_mutex);
    }

    return ready;
#endif
}

esp_err_t usb_thermal_camera_get_frame_info(usb_thermal_camera_frame_info_t *out_info)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return ESP_ERR_INVALID_STATE;
#else
    esp_err_t ret = ESP_OK;

    if (!out_info) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_capture.ready) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        usb_thermal_fill_frame_info_locked(out_info);
    }

    xSemaphoreGive(s_capture.frame_mutex);
    return ret;
#endif
}

esp_err_t usb_thermal_camera_get_latest_y16(uint8_t *out_buf, size_t out_buf_size,
                                            usb_thermal_camera_frame_info_t *out_info)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return ESP_ERR_INVALID_STATE;
#else
    esp_err_t ret = ESP_OK;

    if (!out_buf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_capture.ready || !s_capture.latest_y16) {
        ret = ESP_ERR_INVALID_STATE;
    } else if (out_buf_size < s_capture.y16_len) {
        ret = ESP_ERR_INVALID_SIZE;
    } else {
        memcpy(out_buf, s_capture.latest_y16, s_capture.y16_len);
        usb_thermal_fill_frame_info_locked(out_info);
    }

    xSemaphoreGive(s_capture.frame_mutex);
    return ret;
#endif
}

esp_err_t usb_thermal_camera_get_latest_gray8(uint8_t *out_buf, size_t out_buf_size,
                                              usb_thermal_camera_frame_info_t *out_info)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return ESP_ERR_INVALID_STATE;
#else
    esp_err_t ret = ESP_OK;
    size_t pixels;
    uint8_t *local_y16 = NULL;
    size_t local_y16_size = 0;
    uint16_t min_value = 0;
    uint16_t max_value = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    int64_t start_us = 0;
    int64_t end_us = 0;
    usb_thermal_camera_frame_info_t frame_info = {0};

    if (!out_buf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex || !s_capture.convert_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_capture.convert_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        xSemaphoreGive(s_capture.convert_mutex);
        return ESP_ERR_TIMEOUT;
    }

    pixels = (size_t)s_capture.width * (size_t)s_capture.height;
    if (!s_capture.ready || !s_capture.latest_y16) {
        ret = ESP_ERR_INVALID_STATE;
    } else if (out_buf_size < pixels) {
        ret = ESP_ERR_INVALID_SIZE;
    } else if (!s_capture.convert_y16 || s_capture.convert_y16_size < s_capture.y16_len) {
        ret = ESP_ERR_NO_MEM;
    } else {
        memcpy(s_capture.convert_y16, s_capture.latest_y16, s_capture.y16_len);
        local_y16 = s_capture.convert_y16;
        local_y16_size = s_capture.y16_len;
        min_value = s_capture.min_value;
        max_value = s_capture.max_value;
        width = s_capture.width;
        height = s_capture.height;
        usb_thermal_fill_frame_info_locked(&frame_info);
    }

    xSemaphoreGive(s_capture.frame_mutex);

    if (ret == ESP_OK) {
        pixels = (size_t)width * (size_t)height;
        if (local_y16_size < pixels * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL) {
            ret = ESP_ERR_INVALID_SIZE;
        } else {
            start_us = esp_timer_get_time();
            for (size_t i = 0; i < pixels; i++) {
                uint16_t value = usb_thermal_read_y16_le(local_y16 + i * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                out_buf[i] = usb_thermal_y16_to_gray(value, min_value, max_value);
            }
            end_us = esp_timer_get_time();

            if (out_info) {
                *out_info = frame_info;
            }

            if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) == pdTRUE) {
                if ((end_us - s_capture.last_gray8_log_us) >= (int64_t)USB_THERMAL_CAMERA_CONVERT_LOG_US) {
                    s_capture.last_gray8_log_us = end_us;
                    ESP_LOGI(TAG,
                             "USB 热像仪灰度转换完成 | gray8 | %"PRIu32"x%"PRIu32" | 耗时 %.2f ms | Y16范围=%u-%u",
                             width,
                             height,
                             (double)(end_us - start_us) / 1000.0,
                             min_value,
                             max_value);
                }
                xSemaphoreGive(s_capture.frame_mutex);
            }
        }
    }

    xSemaphoreGive(s_capture.convert_mutex);
    return ret;
#endif
}

esp_err_t usb_thermal_camera_get_latest_gray_oue_vyy(uint8_t *out_buf, size_t out_buf_size,
                                                     usb_thermal_camera_frame_info_t *out_info)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return ESP_ERR_INVALID_STATE;
#else
    esp_err_t ret = ESP_OK;
    size_t expected_len;
    size_t row_stride;
    uint8_t *local_y16 = NULL;
    size_t local_y16_size = 0;
    uint16_t min_value = 0;
    uint16_t max_value = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    int64_t start_us = 0;
    int64_t end_us = 0;
    usb_thermal_camera_frame_info_t frame_info = {0};

    if (!out_buf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex || !s_capture.convert_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_capture.convert_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        xSemaphoreGive(s_capture.convert_mutex);
        return ESP_ERR_TIMEOUT;
    }

    expected_len = (size_t)s_capture.width * (size_t)s_capture.height * 3U / 2U;
    if (!s_capture.ready || !s_capture.latest_y16) {
        ret = ESP_ERR_INVALID_STATE;
    } else if ((s_capture.width & 1U) != 0U || (s_capture.height & 1U) != 0U) {
        ret = ESP_ERR_INVALID_SIZE;
    } else if (out_buf_size < expected_len) {
        ret = ESP_ERR_INVALID_SIZE;
    } else if (!s_capture.convert_y16 || s_capture.convert_y16_size < s_capture.y16_len) {
        ret = ESP_ERR_NO_MEM;
    } else {
        memcpy(s_capture.convert_y16, s_capture.latest_y16, s_capture.y16_len);
        local_y16 = s_capture.convert_y16;
        local_y16_size = s_capture.y16_len;
        min_value = s_capture.min_value;
        max_value = s_capture.max_value;
        width = s_capture.width;
        height = s_capture.height;
        usb_thermal_fill_frame_info_locked(&frame_info);
    }

    xSemaphoreGive(s_capture.frame_mutex);

    if (ret == ESP_OK) {
        expected_len = (size_t)width * (size_t)height * 3U / 2U;
        row_stride = (size_t)width * 3U / 2U;
        if (local_y16_size < (size_t)width * (size_t)height * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL) {
            ret = ESP_ERR_INVALID_SIZE;
        } else {
            start_us = esp_timer_get_time();
            for (uint32_t y = 0; y < height; y++) {
                const uint8_t *src_row = local_y16 +
                                         (size_t)y * (size_t)width * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL;
                uint8_t *dst_row = out_buf + (size_t)y * row_stride;

                for (uint32_t x = 0; x < width; x += 2U) {
                    uint16_t value0 = usb_thermal_read_y16_le(src_row + (size_t)x * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                    uint16_t value1 = usb_thermal_read_y16_le(src_row + ((size_t)x + 1U) * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                    size_t dst_offset = (size_t)(x / 2U) * 3U;

                    dst_row[dst_offset] = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
                    dst_row[dst_offset + 1U] = usb_thermal_y16_to_gray(value0, min_value, max_value);
                    dst_row[dst_offset + 2U] = usb_thermal_y16_to_gray(value1, min_value, max_value);
                }
            }
            end_us = esp_timer_get_time();

            if (out_info) {
                *out_info = frame_info;
            }

            if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) == pdTRUE) {
                if ((end_us - s_capture.last_video_log_us) >= (int64_t)USB_THERMAL_CAMERA_CONVERT_LOG_US) {
                    s_capture.last_video_log_us = end_us;
                    ESP_LOGI(TAG,
                             "USB 热像仪灰度转换完成 | O_UYY_E_VYY | %"PRIu32"x%"PRIu32" | 耗时 %.2f ms | 输出=%zu 字节",
                             width,
                             height,
                             (double)(end_us - start_us) / 1000.0,
                             expected_len);
                }
                xSemaphoreGive(s_capture.frame_mutex);
            }
        }
    }

    xSemaphoreGive(s_capture.convert_mutex);
    return ret;
#endif
}
