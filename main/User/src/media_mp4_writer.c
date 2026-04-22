/**
 * @file media_mp4_writer.c
 * @brief 基于 esp_muxer 的 H.264 单路 MP4 写入器
 *
 * 设计约束：
 *   - 输入为 Annex-B 格式的 H.264 帧
 *   - 仅封装单路视频，不包含音频
 *   - 由调用方提供 SPS/PPS，不在本模块内自行缓存完整 MP4 索引
 *   - 外部接口保持不变，便于最小代价替换现有录像流程
 */

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_muxer.h"
#include "impl/mp4_muxer.h"

#include "media_mp4_writer.h"

static const char *TAG = "media_mp4";

#define MP4_MUXER_RAM_CACHE_SIZE (64U * 1024U)
#define H264_START_CODE_LEN      4U

static const uint8_t s_h264_start_code[H264_START_CODE_LEN] = {0x00, 0x00, 0x00, 0x01};

static void media_mp4_writer_release_buffers(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    free(writer->path);
    free(writer->codec_spec_info);

    writer->path = NULL;
    writer->codec_spec_info = NULL;
    writer->codec_spec_info_len = 0;
}

static void media_mp4_writer_reset(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    media_mp4_writer_release_buffers(writer);
    writer->muxer = NULL;
    writer->video_stream_index = -1;
    writer->width = 0;
    writer->height = 0;
    writer->timescale = 0;
    writer->default_sample_duration = 0;
    writer->fps = 0;
}

static uint8_t media_mp4_writer_calc_fps(uint32_t timescale,
                                         uint32_t default_sample_duration)
{
    uint64_t fps = 0;

    if (timescale == 0U || default_sample_duration == 0U) {
        return 1U;
    }

    fps = ((uint64_t)timescale + ((uint64_t)default_sample_duration / 2U)) /
          (uint64_t)default_sample_duration;
    if (fps == 0U) {
        fps = 1U;
    }
    if (fps > UINT8_MAX) {
        fps = UINT8_MAX;
    }
    return (uint8_t)fps;
}

static uint32_t media_mp4_writer_duration_to_ms(uint32_t timescale,
                                                uint32_t duration)
{
    uint64_t duration_ms = 0;

    if (timescale == 0U || duration == 0U) {
        return 1U;
    }

    duration_ms = ((uint64_t)duration * 1000U + (uint64_t)timescale - 1U) /
                  (uint64_t)timescale;
    if (duration_ms == 0U) {
        duration_ms = 1U;
    }
    if (duration_ms > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)duration_ms;
}

static uint32_t media_mp4_writer_pts_to_ms(const media_mp4_writer_t *writer,
                                           uint32_t pts)
{
    uint64_t pts_ms = 0;

    if (!writer || writer->timescale == 0U) {
        return 0U;
    }

    pts_ms = ((uint64_t)pts * 1000U + ((uint64_t)writer->timescale / 2U)) /
             (uint64_t)writer->timescale;
    if (pts_ms > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)pts_ms;
}

static esp_err_t media_mp4_writer_copy_path(media_mp4_writer_t *writer,
                                            const char *path)
{
    size_t path_len = 0;
    char *path_copy = NULL;

    if (!writer || !path) {
        return ESP_ERR_INVALID_ARG;
    }

    path_len = strlen(path) + 1U;
    path_copy = (char *)malloc(path_len);
    if (!path_copy) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(path_copy, path, path_len);
    writer->path = path_copy;
    return ESP_OK;
}

static esp_err_t media_mp4_writer_build_codec_spec_info(
    media_mp4_writer_t *writer,
    const media_mp4_writer_config_t *config)
{
    size_t total_len = 0;
    uint8_t *buf = NULL;
    size_t offset = 0;

    if (!writer || !config || !config->sps || !config->pps) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->sps_len > (SIZE_MAX - (2U * H264_START_CODE_LEN)) ||
        config->pps_len > (SIZE_MAX - config->sps_len - (2U * H264_START_CODE_LEN))) {
        return ESP_ERR_INVALID_ARG;
    }

    total_len = (2U * H264_START_CODE_LEN) + config->sps_len + config->pps_len;
    if (total_len > (size_t)INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    buf = (uint8_t *)malloc(total_len);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(buf + offset, s_h264_start_code, H264_START_CODE_LEN);
    offset += H264_START_CODE_LEN;
    memcpy(buf + offset, config->sps, config->sps_len);
    offset += config->sps_len;
    memcpy(buf + offset, s_h264_start_code, H264_START_CODE_LEN);
    offset += H264_START_CODE_LEN;
    memcpy(buf + offset, config->pps, config->pps_len);

    writer->codec_spec_info = buf;
    writer->codec_spec_info_len = total_len;
    return ESP_OK;
}

static int media_mp4_writer_url_pattern(esp_muxer_slice_info_t *info, void *ctx)
{
    const media_mp4_writer_t *writer = (const media_mp4_writer_t *)ctx;
    int written = 0;

    if (!info || !writer || !writer->path || !info->file_path || info->len <= 0) {
        return -1;
    }

    if (info->slice_index != 0) {
        ESP_LOGW(TAG, "当前封装层只支持单文件输出，拒绝额外切片: %d",
                 info->slice_index);
        return -1;
    }

    written = snprintf(info->file_path, info->len, "%s", writer->path);
    if (written < 0 || written >= info->len) {
        return -1;
    }

    return 0;
}

static esp_err_t media_mp4_writer_close_muxer(media_mp4_writer_t *writer)
{
    esp_muxer_err_t muxer_ret = ESP_MUXER_ERR_OK;

    if (!writer || !writer->muxer) {
        return ESP_ERR_INVALID_STATE;
    }

    muxer_ret = esp_muxer_close((esp_muxer_handle_t)writer->muxer);
    writer->muxer = NULL;
    if (muxer_ret != ESP_MUXER_ERR_OK) {
        ESP_LOGE(TAG, "关闭 MP4 muxer 失败: %d", (int)muxer_ret);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void media_mp4_writer_init(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    memset(writer, 0, sizeof(*writer));
    writer->video_stream_index = -1;
}

void media_mp4_writer_deinit(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    if (writer->muxer) {
        esp_err_t ret = media_mp4_writer_close_muxer(writer);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "异常释放 MP4 写入器时关闭 muxer 失败");
        }
    }

    media_mp4_writer_reset(writer);
}

bool media_mp4_writer_is_open(const media_mp4_writer_t *writer)
{
    return writer && writer->muxer != NULL;
}

esp_err_t media_mp4_writer_open(media_mp4_writer_t *writer, const char *path,
                                const media_mp4_writer_config_t *config)
{
    mp4_muxer_config_t muxer_cfg;
    esp_muxer_video_stream_info_t video_info;
    esp_muxer_err_t muxer_ret = ESP_MUXER_ERR_OK;
    esp_err_t ret = ESP_OK;

    if (!writer || !path || !config ||
        config->width == 0U || config->height == 0U ||
        config->width > UINT16_MAX || config->height > UINT16_MAX ||
        config->timescale == 0U || config->default_sample_duration == 0U ||
        !config->sps || config->sps_len < 4U ||
        !config->pps || config->pps_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (media_mp4_writer_is_open(writer)) {
        return ESP_ERR_INVALID_STATE;
    }

    media_mp4_writer_reset(writer);

    ret = media_mp4_writer_copy_path(writer, path);
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = media_mp4_writer_build_codec_spec_info(writer, config);
    if (ret != ESP_OK) {
        goto fail;
    }

    writer->width = config->width;
    writer->height = config->height;
    writer->timescale = config->timescale;
    writer->default_sample_duration = config->default_sample_duration;
    writer->fps = media_mp4_writer_calc_fps(config->timescale,
                                            config->default_sample_duration);

    memset(&muxer_cfg, 0, sizeof(muxer_cfg));
    muxer_cfg.base_config.muxer_type = ESP_MUXER_TYPE_MP4;
    muxer_cfg.base_config.slice_duration = ESP_MUXER_MAX_SLICE_DURATION;
    muxer_cfg.base_config.url_pattern_ex = media_mp4_writer_url_pattern;
    muxer_cfg.base_config.ctx = writer;
    muxer_cfg.base_config.ram_cache_size = MP4_MUXER_RAM_CACHE_SIZE;
    muxer_cfg.base_config.no_key_frame_verify = true;
    muxer_cfg.display_in_order = true;
    muxer_cfg.moov_before_mdat = false;

    muxer_ret = mp4_muxer_register();
    if (muxer_ret != ESP_MUXER_ERR_OK) {
        ESP_LOGE(TAG, "注册 MP4 muxer 失败: %d", (int)muxer_ret);
        ret = ESP_FAIL;
        goto fail;
    }

    writer->muxer = esp_muxer_open(&muxer_cfg.base_config, sizeof(muxer_cfg));
    if (!writer->muxer) {
        ESP_LOGE(TAG, "打开 MP4 muxer 失败: %s", path);
        ret = ESP_FAIL;
        goto fail;
    }

    memset(&video_info, 0, sizeof(video_info));
    video_info.codec = ESP_MUXER_VDEC_H264;
    video_info.width = (uint16_t)config->width;
    video_info.height = (uint16_t)config->height;
    video_info.fps = writer->fps;
    video_info.min_packet_duration = media_mp4_writer_duration_to_ms(
        config->timescale, config->default_sample_duration);
    video_info.codec_spec_info = writer->codec_spec_info;
    video_info.spec_info_len = (int)writer->codec_spec_info_len;

    writer->video_stream_index = -1;
    muxer_ret = esp_muxer_add_video_stream((esp_muxer_handle_t)writer->muxer,
                                           &video_info,
                                           &writer->video_stream_index);
    if (muxer_ret != ESP_MUXER_ERR_OK) {
        ESP_LOGE(TAG, "添加 MP4 视频轨失败: %d", (int)muxer_ret);
        ret = ESP_FAIL;
        goto fail;
    }

    return ESP_OK;

fail:
    if (writer->muxer) {
        media_mp4_writer_close_muxer(writer);
    }
    media_mp4_writer_reset(writer);
    return ret;
}

esp_err_t media_mp4_writer_write_frame(media_mp4_writer_t *writer,
                                       const uint8_t *h264_buf, size_t h264_len,
                                       uint32_t pts, bool is_sync)
{
    esp_muxer_video_packet_t packet;
    esp_muxer_err_t muxer_ret = ESP_MUXER_ERR_OK;

    if (!writer || !h264_buf || h264_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!writer->muxer || writer->video_stream_index < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (h264_len > (size_t)INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&packet, 0, sizeof(packet));
    packet.data = (void *)h264_buf;
    packet.len = (int)h264_len;
    packet.pts = media_mp4_writer_pts_to_ms(writer, pts);
    packet.dts = packet.pts;
    packet.key_frame = is_sync;

    muxer_ret = esp_muxer_add_video_packet((esp_muxer_handle_t)writer->muxer,
                                           writer->video_stream_index,
                                           &packet);
    if (muxer_ret != ESP_MUXER_ERR_OK) {
        ESP_LOGE(TAG, "写入 MP4 视频包失败: %d | len=%d | key=%d",
                 (int)muxer_ret, packet.len, is_sync ? 1 : 0);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t media_mp4_writer_close(media_mp4_writer_t *writer)
{
    esp_err_t ret = ESP_OK;

    if (!writer) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!writer->muxer) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = media_mp4_writer_close_muxer(writer);
    media_mp4_writer_reset(writer);
    return ret;
}
