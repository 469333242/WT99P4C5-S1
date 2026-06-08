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
#include "media_storage.h"
#include "usb_thermal_camera.h"

static const char *TAG = "usb_thermal";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define USB_THERMAL_CAMERA_ENABLE              1       /* USB 热像仪总开关：0=关闭，1=开启 */
#define USB_THERMAL_CAMERA_PRINT_DESC_ENABLE   0       /* 完整 USB 描述符打印开关：调试 VID/PID 时再开启 */
#define USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE  0       /* 调试日志开关：开启后打印格式列表、周期统计和转换耗时 */
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
#define USB_THERMAL_CAMERA_NEUTRAL_CHROMA      128     /* 默认中性 U/V 值 */
#define USB_THERMAL_CAMERA_RTSP_TASK_STACK     (8 * 1024)
#define USB_THERMAL_CAMERA_RTSP_TASK_PRIORITY  13
#define USB_THERMAL_CAMERA_RTSP_TASK_CORE      1
#define USB_THERMAL_CAMERA_RTSP_FPS            25U     /* 热像仪 RTSP 输出帧率，低于 USB 采集帧率以减轻编码压力 */
#define USB_THERMAL_CAMERA_RTSP_GOP            5U
#define USB_THERMAL_CAMERA_RTSP_BITRATE        1200000U
#define USB_THERMAL_CAMERA_RTSP_QP_MIN         28U
#define USB_THERMAL_CAMERA_RTSP_QP_MAX         42U
#define USB_THERMAL_CAMERA_H264_OUT_BUF_SIZE   (256U * 1024U) /* 热像仪 H.264 单帧输出缓冲，避免挤占 SDIO RX 内存 */
#define USB_THERMAL_CAMERA_RTSP_READY_WAIT_MS  20U
#define USB_THERMAL_CAMERA_RTSP_STATS_US       (10ULL * 1000ULL * 1000ULL)
#define USB_THERMAL_CAMERA_RTSP_WIDTH          512U    /* 当前热像仪实测宽度 */
#define USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT     390U    /* 当前热像仪实测高度 */
#define USB_THERMAL_CAMERA_RTSP_ENC_HEIGHT     400U    /* H.264 按 16 像素宏块对齐，底部补黑边 */
#define USB_THERMAL_CAMERA_RTSP_IDLE_RELEASE_MS 1000U  /* 客户端断开后释放编码资源，给 SDIO/WiFi 留内存 */
#define USB_THERMAL_CAMERA_CTRL_EVENT_WAIT_MS  100U    /* 等待 USB 控制传输事件的轮询间隔 */
#define USB_THERMAL_CAMERA_CTRL_CLIENT_EVENTS  4       /* 热像仪控制命令客户端事件缓存数量 */
#define USB_THERMAL_CAMERA_CTRL_BREQUEST       0x01U   /* 厂商私有控制请求，若实测不生效优先调整这里 */
#define USB_THERMAL_CAMERA_CTRL_WVALUE         0x0000U
#define USB_THERMAL_CAMERA_CTRL_WINDEX         0x0000U
#define USB_THERMAL_CAMERA_CTRL_BMREQUEST_TYPE (USB_BM_REQUEST_TYPE_DIR_OUT | \
                                                USB_BM_REQUEST_TYPE_TYPE_VENDOR | \
                                                USB_BM_REQUEST_TYPE_RECIP_DEVICE)
#define USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE 0  /* 厂商私有成像命令开关：当前 Y16 推流使用本地伪彩，避免网页请求阻塞 */

const char *usb_thermal_camera_get_effect_name(usb_thermal_camera_effect_t effect)
{
    switch (effect) {
        case USB_THERMAL_CAMERA_EFFECT_WHITE_HOT:
            return "白热";
        case USB_THERMAL_CAMERA_EFFECT_BLACK_HOT:
            return "黑热";
        case USB_THERMAL_CAMERA_EFFECT_IRON_RED:
            return "铁红";
        default:
            return "未知";
    }
}

#if USB_THERMAL_CAMERA_ENABLE

typedef struct {
    uint8_t dev_addr;          /* USB 设备地址 */
    uint8_t stream_index;      /* UVC 视频流索引 */
    size_t  frame_info_num;    /* 当前流支持的格式数量 */
} usb_thermal_camera_event_t;

typedef struct {
    SemaphoreHandle_t frame_mutex;   /* 保护 latest_y16 和帧元信息 */
    SemaphoreHandle_t convert_mutex; /* 串行化灰度/伪彩转换，复用 convert_y16 临时缓冲 */
    uvc_host_stream_hdl_t stream_hdl;
    uint8_t dev_addr;
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
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    int64_t last_gray8_log_us;
    int64_t last_video_log_us;
#endif
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

#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
typedef struct {
    SemaphoreHandle_t done;
    usb_transfer_status_t status;
    int actual_num_bytes;
} usb_thermal_camera_control_ctx_t;
#endif

static QueueHandle_t s_device_queue;
static TaskHandle_t s_usb_task_handle;
static TaskHandle_t s_capture_task_handle;
#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
static SemaphoreHandle_t s_control_mutex;
#endif
static bool s_started;
static bool s_usb_thermal_external_active_requested;
static usb_thermal_camera_effect_t s_current_effect = USB_THERMAL_CAMERA_EFFECT_WHITE_HOT;
static usb_thermal_camera_capture_t s_capture;
static usb_thermal_camera_rtsp_t s_rtsp;
static uint8_t s_iron_red_y[256];
static uint8_t s_iron_red_u[256];
static uint8_t s_iron_red_v[256];
static bool s_iron_red_palette_ready;

#define USB_THERMAL_RTSP_START_BIT     BIT0
#define USB_THERMAL_EXTERNAL_START_BIT BIT1
#define USB_THERMAL_ACTIVE_BITS        (USB_THERMAL_RTSP_START_BIT | USB_THERMAL_EXTERNAL_START_BIT)

static const char *s_uvc_format_name[] = {
    "DEFAULT",
    "MJPEG",
    "YUY2",
    "H264",
    "H265",
};

#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
static const uint8_t s_usb_thermal_effect_white_hot_cmd[] = {
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x6e, 0x00, 0x00, 0x10, 0x00, 0x02, 0xbc, 0x9a,
    0x00, 0x00, 0x00, 0x00,
};

static const uint8_t s_usb_thermal_effect_black_hot_cmd[] = {
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x6e, 0x00, 0x00, 0x10, 0x00, 0x02, 0xbc, 0x9a,
    0x00, 0x01, 0x10, 0x21,
};

static const uint8_t s_usb_thermal_effect_iron_red_cmd[] = {
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x42, 0x48, 0x48, 0x55, 0x01, 0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x6e, 0x00, 0x00, 0x10, 0x00, 0x02, 0xbc, 0x9a,
    0x00, 0x06, 0x60, 0xc6,
};
#endif

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
#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
    if (s_control_mutex) {
        vSemaphoreDelete(s_control_mutex);
        s_control_mutex = NULL;
    }
#endif

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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
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
#endif

static void uvc_log_frame_info_list(const uvc_host_frame_info_t *frame_list,
                                    size_t frame_count)
{
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
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
#else
    (void)frame_list;
    (void)frame_count;
#endif
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

static bool usb_thermal_camera_effect_is_valid(usb_thermal_camera_effect_t effect)
{
    switch (effect) {
        case USB_THERMAL_CAMERA_EFFECT_WHITE_HOT:
        case USB_THERMAL_CAMERA_EFFECT_BLACK_HOT:
        case USB_THERMAL_CAMERA_EFFECT_IRON_RED:
            return true;
        default:
            return false;
    }
}

#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
static const uint8_t *usb_thermal_camera_get_effect_payload(usb_thermal_camera_effect_t effect,
                                                            size_t *payload_len)
{
    if (!payload_len) {
        return NULL;
    }

    switch (effect) {
        case USB_THERMAL_CAMERA_EFFECT_WHITE_HOT:
            *payload_len = sizeof(s_usb_thermal_effect_white_hot_cmd);
            return s_usb_thermal_effect_white_hot_cmd;
        case USB_THERMAL_CAMERA_EFFECT_BLACK_HOT:
            *payload_len = sizeof(s_usb_thermal_effect_black_hot_cmd);
            return s_usb_thermal_effect_black_hot_cmd;
        case USB_THERMAL_CAMERA_EFFECT_IRON_RED:
            *payload_len = sizeof(s_usb_thermal_effect_iron_red_cmd);
            return s_usb_thermal_effect_iron_red_cmd;
        default:
            *payload_len = 0;
            return NULL;
    }
}

static esp_err_t usb_thermal_camera_get_connected_dev_addr(uint8_t *dev_addr)
{
    esp_err_t ret = ESP_OK;

    if (!dev_addr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_capture.frame_mutex,
                       pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_capture.connected || s_capture.dev_addr == 0U) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        *dev_addr = s_capture.dev_addr;
    }

    xSemaphoreGive(s_capture.frame_mutex);
    return ret;
}

static void usb_thermal_camera_control_client_event_cb(const usb_host_client_event_msg_t *event_msg,
                                                       void *arg)
{
    (void)arg;

    if (!event_msg) {
        return;
    }

    if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGW(TAG, "USB 热像仪控制通道检测到设备断开");
    }
}

static void usb_thermal_camera_control_transfer_cb(usb_transfer_t *transfer)
{
    usb_thermal_camera_control_ctx_t *ctx;

    if (!transfer || !transfer->context) {
        return;
    }

    ctx = (usb_thermal_camera_control_ctx_t *)transfer->context;
    ctx->status = transfer->status;
    ctx->actual_num_bytes = transfer->actual_num_bytes;

    if (ctx->done) {
        xSemaphoreGive(ctx->done);
    }
}

static esp_err_t usb_thermal_camera_wait_control_done(usb_host_client_handle_t client_hdl,
                                                      usb_thermal_camera_control_ctx_t *ctx)
{
    esp_err_t ret;
    bool error_logged = false;

    if (!client_hdl || !ctx || !ctx->done) {
        return ESP_ERR_INVALID_ARG;
    }

    while (xSemaphoreTake(ctx->done, 0) != pdTRUE) {
        ret = usb_host_client_handle_events(client_hdl,
                                            pdMS_TO_TICKS(USB_THERMAL_CAMERA_CTRL_EVENT_WAIT_MS));
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            if (!error_logged) {
                ESP_LOGW(TAG, "等待 USB 热像仪控制传输事件异常: 0x%x (%s)",
                         ret, esp_err_to_name(ret));
                error_logged = true;
            }
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_CTRL_EVENT_WAIT_MS));
        }
    }

    if (ctx->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "USB 热像仪控制传输未完成 | status=%d | actual=%d",
                 ctx->status, ctx->actual_num_bytes);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t usb_thermal_camera_send_control_payload(const uint8_t *payload,
                                                         size_t payload_len)
{
    uint8_t dev_addr = 0;
    usb_host_client_handle_t client_hdl = NULL;
    usb_device_handle_t dev_hdl = NULL;
    usb_transfer_t *transfer = NULL;
    usb_thermal_camera_control_ctx_t ctrl_ctx = {0};
    esp_err_t ret;

    if (!payload || payload_len == 0U || payload_len > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_started || !s_control_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = usb_thermal_camera_get_connected_dev_addr(&dev_addr);
    if (ret != ESP_OK) {
        return ret;
    }

    if (xSemaphoreTake(s_control_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = USB_THERMAL_CAMERA_CTRL_CLIENT_EVENTS,
        .async = {
            .client_event_callback = usb_thermal_camera_control_client_event_cb,
            .callback_arg = NULL,
        },
    };

    ret = usb_host_client_register(&client_config, &client_hdl);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "USB 热像仪控制客户端注册失败: 0x%x", ret);
        goto exit;
    }

    ret = usb_host_device_open(client_hdl, dev_addr, &dev_hdl);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "打开 USB 热像仪控制设备失败: addr=%u | ret=0x%x", dev_addr, ret);
        goto exit;
    }

    ctrl_ctx.done = xSemaphoreCreateBinary();
    if (!ctrl_ctx.done) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }

    ret = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + payload_len, 0, &transfer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "USB 热像仪控制传输缓存分配失败: 0x%x", ret);
        goto exit;
    }

    usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
    setup->bmRequestType = USB_THERMAL_CAMERA_CTRL_BMREQUEST_TYPE;
    setup->bRequest = USB_THERMAL_CAMERA_CTRL_BREQUEST;
    setup->wValue = USB_THERMAL_CAMERA_CTRL_WVALUE;
    setup->wIndex = USB_THERMAL_CAMERA_CTRL_WINDEX;
    setup->wLength = (uint16_t)payload_len;
    memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), payload, payload_len);

    transfer->num_bytes = (int)(sizeof(usb_setup_packet_t) + payload_len);
    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = 0;
    transfer->callback = usb_thermal_camera_control_transfer_cb;
    transfer->context = &ctrl_ctx;

    ret = usb_host_transfer_submit_control(client_hdl, transfer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "提交 USB 热像仪控制命令失败: 0x%x", ret);
        goto exit;
    }

    ret = usb_thermal_camera_wait_control_done(client_hdl, &ctrl_ctx);

exit:
    if (transfer) {
        usb_host_transfer_free(transfer);
    }
    if (ctrl_ctx.done) {
        vSemaphoreDelete(ctrl_ctx.done);
    }
    if (dev_hdl && client_hdl) {
        esp_err_t close_ret = usb_host_device_close(client_hdl, dev_hdl);
        if (close_ret != ESP_OK) {
            ESP_LOGW(TAG, "关闭 USB 热像仪控制设备失败: 0x%x", close_ret);
        }
    }
    if (client_hdl) {
        esp_err_t dereg_ret = usb_host_client_deregister(client_hdl);
        if (dereg_ret != ESP_OK) {
            ESP_LOGW(TAG, "注销 USB 热像仪控制客户端失败: 0x%x", dereg_ret);
        }
    }

    xSemaphoreGive(s_control_mutex);
    return ret;
}
#endif

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

static uint8_t usb_thermal_clamp_u8_i32(int32_t value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }

    return (uint8_t)value;
}

static void usb_thermal_rgb_to_yuv(uint8_t r, uint8_t g, uint8_t b,
                                   uint8_t *out_y, uint8_t *out_u, uint8_t *out_v)
{
    int32_t y;
    int32_t u;
    int32_t v;

    if (!out_y || !out_u || !out_v) {
        return;
    }

    /* 使用 BT.601 全范围近似公式，匹配当前 PPA/JPEG 链路的 YUV 处理方式。 */
    y = ((77 * (int32_t)r) + (150 * (int32_t)g) + (29 * (int32_t)b)) / 256;
    u = 128 + (((-43 * (int32_t)r) - (85 * (int32_t)g) + (128 * (int32_t)b)) / 256);
    v = 128 + (((128 * (int32_t)r) - (107 * (int32_t)g) - (21 * (int32_t)b)) / 256);

    *out_y = usb_thermal_clamp_u8_i32(y);
    *out_u = usb_thermal_clamp_u8_i32(u);
    *out_v = usb_thermal_clamp_u8_i32(v);
}

static void usb_thermal_iron_red_rgb_from_gray(uint8_t gray,
                                               uint8_t *out_r,
                                               uint8_t *out_g,
                                               uint8_t *out_b)
{
    uint32_t t;

    if (!out_r || !out_g || !out_b) {
        return;
    }

    /* 铁红伪彩：低温黑红，中温红橙，高温黄白。 */
    if (gray < 64U) {
        t = gray;
        *out_r = (uint8_t)(t * 128U / 63U);
        *out_g = 0;
        *out_b = 0;
    } else if (gray < 128U) {
        t = (uint32_t)gray - 64U;
        *out_r = (uint8_t)(128U + (t * 127U / 63U));
        *out_g = (uint8_t)(t * 64U / 63U);
        *out_b = 0;
    } else if (gray < 192U) {
        t = (uint32_t)gray - 128U;
        *out_r = 255;
        *out_g = (uint8_t)(64U + (t * 191U / 63U));
        *out_b = 0;
    } else {
        t = (uint32_t)gray - 192U;
        *out_r = 255;
        *out_g = 255;
        *out_b = (uint8_t)(t * 255U / 63U);
    }
}

static void usb_thermal_prepare_iron_red_palette(void)
{
    if (s_iron_red_palette_ready) {
        return;
    }

    for (uint32_t i = 0; i < 256U; i++) {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;

        usb_thermal_iron_red_rgb_from_gray((uint8_t)i, &r, &g, &b);
        usb_thermal_rgb_to_yuv(r, g, b,
                               &s_iron_red_y[i],
                               &s_iron_red_u[i],
                               &s_iron_red_v[i]);
    }

    s_iron_red_palette_ready = true;
}

static void usb_thermal_effect_yuv_from_gray(usb_thermal_camera_effect_t effect,
                                             uint8_t gray,
                                             uint8_t *out_y,
                                             uint8_t *out_u,
                                             uint8_t *out_v)
{
    if (!out_y || !out_u || !out_v) {
        return;
    }

    switch (effect) {
        case USB_THERMAL_CAMERA_EFFECT_BLACK_HOT:
            *out_y = (uint8_t)(255U - (uint32_t)gray);
            *out_u = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
            *out_v = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
            break;
        case USB_THERMAL_CAMERA_EFFECT_IRON_RED:
            usb_thermal_prepare_iron_red_palette();
            *out_y = s_iron_red_y[gray];
            *out_u = s_iron_red_u[gray];
            *out_v = s_iron_red_v[gray];
            break;
        case USB_THERMAL_CAMERA_EFFECT_WHITE_HOT:
        default:
            *out_y = gray;
            *out_u = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
            *out_v = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
            break;
    }
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

    /*
     * 转换函数会先拿 convert_mutex 再拿 frame_mutex。
     * 这里重建缓冲也保持同样锁顺序，避免热插拔/分辨率变化时形成死锁。
     */
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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    ESP_LOGI(TAG,
             "USB 热像仪 Y16 帧缓存已准备: %"PRIu32"x%"PRIu32" | 最新帧=%zu 字节 | 转换帧=%zu 字节",
             width, height, s_capture.latest_y16_size, s_capture.convert_y16_size);
#endif
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
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
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
#else
    (void)now_us;
#endif
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

    /*
     * UVC 回调运行在 USB 传输路径上，不能等待应用层转换完成。
     * 拿不到锁就丢当前帧，优先保证 USB BULK 传输持续回收缓冲。
     */
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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
                if (frame->data_len >= 8U) {
                    ESP_LOGI(TAG,
                             "USB 热像仪首帧前 8 字节: %02x %02x %02x %02x %02x %02x %02x %02x",
                             frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                             frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
                }
#endif
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
                capture->dev_addr = 0;
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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    if (uvc_format_is_unknown(selected->format)) {
        ESP_LOGI(TAG,
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
#endif

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
        s_capture.dev_addr = device_event->dev_addr;
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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    uvc_host_stream_format_t active_format = {0};
    if (uvc_host_stream_format_get(stream_hdl, &active_format) == ESP_OK) {
        ESP_LOGI(TAG,
                 "USB 热像仪协商格式: %s | %ux%u | %.1f fps",
                 uvc_format_to_string(active_format.format),
                 active_format.h_res,
                 active_format.v_res,
                 active_format.fps);
    }
#endif

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
            s_capture.dev_addr = 0;
            s_capture.connected = false;
            s_capture.ready = false;
            xSemaphoreGive(s_capture.frame_mutex);
        }
        free(frame_list);
        return;
    }

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    ESP_LOGI(TAG, "USB 热像仪持续采集已启动，等待最新 Y16 帧");
#endif
    free(frame_list);
}

static void usb_thermal_camera_capture_task(void *arg)
{
    usb_thermal_camera_event_t device_event;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_device_queue, &device_event, portMAX_DELAY) == pdPASS) {
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            ESP_LOGI(TAG,
                     "开始打开 USB UVC 视频流 | addr=%u | stream=%u | 格式数=%u",
                     device_event.dev_addr,
                     device_event.stream_index,
                     (unsigned)device_event.frame_info_num);
#endif
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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            ESP_LOGI(TAG,
                     "检测到 USB UVC 视频接口 | addr=%u | stream=%u | 格式数=%u",
                     device_event.dev_addr,
                     device_event.stream_index,
                     (unsigned)device_event.frame_info_num);
#endif

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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) != 0) {
            ESP_LOGI(TAG, "USB 设备资源已释放，继续等待热插拔");
        }
#endif
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

    /* 录像复用当前 H.264 帧，不额外创建第二路编码器。 */
    esp_err_t media_ret = media_storage_prepare_video_record(s_rtsp.width,
                                                             s_rtsp.height,
                                                             USB_THERMAL_CAMERA_RTSP_FPS);
    if (media_ret != ESP_OK) {
        ESP_LOGW(TAG, "USB 热像仪录像缓冲准备失败，RTSP 将继续运行: 0x%x", media_ret);
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
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    uint32_t encoded_frame_cnt = 0;
    uint64_t convert_time_total_us = 0;
    uint64_t encode_time_total_us = 0;
    uint64_t push_time_total_us = 0;
    int64_t report_start_us = 0;
#endif
    TickType_t frame_delay_ticks = pdMS_TO_TICKS(1000U / USB_THERMAL_CAMERA_RTSP_FPS);

    (void)arg;

    if (frame_delay_ticks == 0) {
        frame_delay_ticks = 1;
    }

    while (1) {
        xEventGroupWaitBits(s_rtsp.event, USB_THERMAL_ACTIVE_BITS,
                            pdFALSE, pdFALSE, portMAX_DELAY);

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

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
        if (report_start_us == 0) {
            report_start_us = esp_timer_get_time();
            encoded_frame_cnt = 0;
            convert_time_total_us = 0;
            encode_time_total_us = 0;
            push_time_total_us = 0;
        }
#endif

        usb_thermal_camera_frame_info_t frame_info = {0};
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
        int64_t t1 = esp_timer_get_time();
#endif
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
        /* 拍照只保存热像仪实际 390 行图像，底部补黑边仅用于 H.264 宏块对齐。 */
        media_storage_process_camera_frame(s_rtsp.raw_buf,
                                           (size_t)s_rtsp.width *
                                           (size_t)USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT * 3U / 2U,
                                           s_rtsp.width,
                                           USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT);
        usb_thermal_pad_oue_vyy_frame(s_rtsp.raw_buf,
                                      s_rtsp.width,
                                      USB_THERMAL_CAMERA_RTSP_SRC_HEIGHT,
                                      s_rtsp.height);
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
        int64_t t2 = esp_timer_get_time();
        convert_time_total_us += (uint64_t)(t2 - t1);
#endif

        size_t h264_len = 0;
        esp_h264_frame_type_t frame_type = ESP_H264_FRAME_TYPE_INVALID;
        uint32_t frame_pts = 0;
        ret = usb_thermal_encode_to_h264(s_rtsp.raw_buf,
                                         s_rtsp.h264_out_buf,
                                         s_rtsp.h264_out_buf_size,
                                         &h264_len,
                                         &frame_type,
                                         &frame_pts);
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
        int64_t t_encode_done = esp_timer_get_time();
        encode_time_total_us += (uint64_t)(t_encode_done - t2);
#endif

        if (ret == ESP_OK && h264_len > 0U) {
            rtsp_push_h264_frame(s_rtsp.h264_out_buf, h264_len, frame_type, frame_pts);
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            int64_t t_push_done = esp_timer_get_time();

            push_time_total_us += (uint64_t)(t_push_done - t_encode_done);
            encoded_frame_cnt++;
#endif

            media_storage_process_h264_frame(s_rtsp.h264_out_buf, h264_len, frame_type, frame_pts);

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            if ((t_push_done - report_start_us) >= (int64_t)USB_THERMAL_CAMERA_RTSP_STATS_US) {
                uint32_t elapsed_ms = (uint32_t)((t_push_done - report_start_us) / 1000);
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

                report_start_us = t_push_done;
                encoded_frame_cnt = 0;
                convert_time_total_us = 0;
                encode_time_total_us = 0;
                push_time_total_us = 0;
            }
#endif
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "USB 热像仪 H.264 编码失败，跳过当前帧");
        }

        if ((xEventGroupGetBits(s_rtsp.event) & USB_THERMAL_ACTIVE_BITS) == 0) {
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            report_start_us = 0;
#endif
            vTaskDelay(pdMS_TO_TICKS(USB_THERMAL_CAMERA_RTSP_IDLE_RELEASE_MS));
            if ((xEventGroupGetBits(s_rtsp.event) & USB_THERMAL_ACTIVE_BITS) == 0) {
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
#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
    s_control_mutex = xSemaphoreCreateMutex();
    if (!s_control_mutex) {
        ESP_LOGE(TAG, "USB 热像仪控制互斥锁创建失败");
        usb_thermal_camera_release_capture_resources();
        return ESP_ERR_NO_MEM;
    }
#endif

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
    if (s_usb_thermal_external_active_requested) {
        xEventGroupSetBits(s_rtsp.event, USB_THERMAL_EXTERNAL_START_BIT);
    }

    rtsp_set_playing_callback(usb_thermal_rtsp_on_playing);
    if (rtsp_get_active_client_count() > 0U) {
        xEventGroupSetBits(s_rtsp.event, USB_THERMAL_RTSP_START_BIT);
    }

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

void usb_thermal_camera_set_external_active(bool active)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    (void)active;
#else
    s_usb_thermal_external_active_requested = active;

    if (!s_rtsp.event) {
        return;
    }

    if (active) {
        xEventGroupSetBits(s_rtsp.event, USB_THERMAL_EXTERNAL_START_BIT);
    } else {
        xEventGroupClearBits(s_rtsp.event, USB_THERMAL_EXTERNAL_START_BIT);
    }
#endif
}

usb_thermal_camera_effect_t usb_thermal_camera_get_effect(void)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    return USB_THERMAL_CAMERA_EFFECT_WHITE_HOT;
#else
    return s_current_effect;
#endif
}

esp_err_t usb_thermal_camera_set_effect(usb_thermal_camera_effect_t effect)
{
#if USB_THERMAL_CAMERA_ENABLE == 0
    ESP_LOGI(TAG, "USB 热像仪采集未启用，忽略成像效果设置");
    return ESP_ERR_INVALID_STATE;
#else
#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE
    const uint8_t *payload;
    size_t payload_len = 0;
    esp_err_t ret;
#endif

    if (!usb_thermal_camera_effect_is_valid(effect)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 当前热像仪 UVC 输出为 Y16 原始热数据，RTSP 和录像颜色由本地 Y16->YUV 转换决定。
     * 先更新本地效果，避免厂商私有 USB 控制命令阻塞网页请求。
     */
    s_current_effect = effect;

#if USB_THERMAL_CAMERA_VENDOR_EFFECT_CMD_ENABLE == 0
    ESP_LOGI(TAG, "USB 热像仪本地成像效果已设置为%s",
             usb_thermal_camera_get_effect_name(effect));
    return ESP_OK;
#else
    payload = usb_thermal_camera_get_effect_payload(effect, &payload_len);
    if (!payload || payload_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = usb_thermal_camera_send_control_payload(payload, payload_len);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "USB 热像仪厂商成像命令已下发，当前本地效果为%s",
                 usb_thermal_camera_get_effect_name(effect));
        return ESP_OK;
    }

    if (usb_thermal_camera_is_ready()) {
        ESP_LOGW(TAG,
                 "USB 热像仪厂商成像命令失败，已切换本地 RTSP 成像效果 | 效果=%s | ret=0x%x (%s)",
                 usb_thermal_camera_get_effect_name(effect), ret, esp_err_to_name(ret));
        return ESP_OK;
    }

    ESP_LOGW(TAG, "USB 热像仪成像效果设置失败 | 效果=%s | ret=0x%x (%s)",
             usb_thermal_camera_get_effect_name(effect), ret, esp_err_to_name(ret));
    return ret;
#endif
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
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    int64_t start_us = 0;
    int64_t end_us = 0;
#endif
    usb_thermal_camera_frame_info_t frame_info = {0};

    if (!out_buf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex || !s_capture.convert_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 转换耗时明显长于复制最新帧。
     * 先把 latest_y16 拷到 convert_y16，再释放 frame_mutex，让采集回调尽快继续写入新帧。
     */
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
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            start_us = esp_timer_get_time();
#endif
            for (size_t i = 0; i < pixels; i++) {
                uint16_t value = usb_thermal_read_y16_le(local_y16 + i * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                out_buf[i] = usb_thermal_y16_to_gray(value, min_value, max_value);
            }
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            end_us = esp_timer_get_time();
#endif

            if (out_info) {
                *out_info = frame_info;
            }

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
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
#endif
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
    usb_thermal_camera_effect_t effect = USB_THERMAL_CAMERA_EFFECT_WHITE_HOT;
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
    int64_t start_us = 0;
    int64_t end_us = 0;
#endif
    usb_thermal_camera_frame_info_t frame_info = {0};

    if (!out_buf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_capture.frame_mutex || !s_capture.convert_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 输出给 H.264 编码器的是 O_UYY_E_VYY：两像素共用色度，偶数行放 U、奇数行放 V。
     * Y16 到伪彩的转换在本地完成，不依赖厂商私有成像命令。
     */
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
        effect = s_current_effect;
        usb_thermal_fill_frame_info_locked(&frame_info);
    }

    xSemaphoreGive(s_capture.frame_mutex);

    if (ret == ESP_OK) {
        expected_len = (size_t)width * (size_t)height * 3U / 2U;
        row_stride = (size_t)width * 3U / 2U;
        if (local_y16_size < (size_t)width * (size_t)height * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL) {
            ret = ESP_ERR_INVALID_SIZE;
        } else {
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            start_us = esp_timer_get_time();
#endif
            for (uint32_t y = 0; y < height; y++) {
                const uint8_t *src_row = local_y16 +
                                         (size_t)y * (size_t)width * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL;
                uint8_t *dst_row = out_buf + (size_t)y * row_stride;

                for (uint32_t x = 0; x < width; x += 2U) {
                    uint16_t value0 = usb_thermal_read_y16_le(src_row + (size_t)x * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                    uint16_t value1 = usb_thermal_read_y16_le(src_row + ((size_t)x + 1U) * USB_THERMAL_CAMERA_Y16_BYTES_PER_PIXEL);
                    uint8_t gray0 = usb_thermal_y16_to_gray(value0, min_value, max_value);
                    uint8_t gray1 = usb_thermal_y16_to_gray(value1, min_value, max_value);
                    uint8_t y0 = 0;
                    uint8_t u0 = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
                    uint8_t v0 = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
                    uint8_t y1 = 0;
                    uint8_t u1 = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
                    uint8_t v1 = USB_THERMAL_CAMERA_NEUTRAL_CHROMA;
                    size_t dst_offset = (size_t)(x / 2U) * 3U;

                    usb_thermal_effect_yuv_from_gray(effect, gray0, &y0, &u0, &v0);
                    usb_thermal_effect_yuv_from_gray(effect, gray1, &y1, &u1, &v1);

                    /* O_UYY_E_VYY 每两像素共用一个色度值，偶数行写 U，奇数行写 V。 */
                    dst_row[dst_offset] = (uint8_t)(((uint16_t)((y & 1U) == 0U ? u0 : v0) +
                                                     (uint16_t)((y & 1U) == 0U ? u1 : v1)) / 2U);
                    dst_row[dst_offset + 1U] = y0;
                    dst_row[dst_offset + 2U] = y1;
                }
            }
#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            end_us = esp_timer_get_time();
#endif

            if (out_info) {
                *out_info = frame_info;
            }

#if USB_THERMAL_CAMERA_VERBOSE_LOG_ENABLE
            if (xSemaphoreTake(s_capture.frame_mutex, pdMS_TO_TICKS(USB_THERMAL_CAMERA_FRAME_WAIT_MS)) == pdTRUE) {
                if ((end_us - s_capture.last_video_log_us) >= (int64_t)USB_THERMAL_CAMERA_CONVERT_LOG_US) {
                    s_capture.last_video_log_us = end_us;
                    ESP_LOGI(TAG,
                             "USB 热像仪成像转换完成 | O_UYY_E_VYY | 效果=%s | %"PRIu32"x%"PRIu32" | 耗时 %.2f ms | 输出=%zu 字节",
                             usb_thermal_camera_get_effect_name(effect),
                             width,
                             height,
                             (double)(end_us - start_us) / 1000.0,
                             expected_len);
                }
                xSemaphoreGive(s_capture.frame_mutex);
            }
#endif
        }
    }

    xSemaphoreGive(s_capture.convert_mutex);
    return ret;
#endif
}
