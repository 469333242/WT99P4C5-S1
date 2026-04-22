/**
 * @file media_mp4_writer.c
 * @brief H.264 单路 MP4 文件封装辅助模块
 *
 * 设计约束：
 *   - 输入为 Annex-B 格式的 H.264 压缩帧
 *   - 仅封装单视频轨，不含音频
 *   - 假定码流无 B 帧，因此不生成 ctts
 *   - 采用一个 sample 对应一个 chunk，简化 stsc/stco 组织
 *   - 仅在 close 时补写 moov，因此异常掉电时当前段可能不可播放
 */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "media_mp4_writer.h"

static const char *TAG = "media_mp4";

#define MP4_NALU_TYPE_MASK 0x1F
#define MP4_NALU_TYPE_SPS  7
#define MP4_NALU_TYPE_PPS  8
#define MP4_TRACK_ID       1
#define MP4_HANDLER_NAME   "VideoHandler"
#define MP4_LANGUAGE_UND   0x55C4
#define MP4_FILE_IO_BUF_SIZE (512U * 1024U)

typedef struct {
    size_t offset;
    size_t len;
    uint8_t type;
} mp4_nalu_info_t;

static FILE *media_mp4_writer_file(const media_mp4_writer_t *writer)
{
    return writer ? (FILE *)writer->fp : NULL;
}

static void media_mp4_writer_reset_metadata(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    free(writer->sps);
    free(writer->pps);
    free(writer->sample_sizes);
    free(writer->sample_offsets);
    free(writer->sample_durations);
    free(writer->sync_samples);
    free(writer->sample_buf);
    free(writer->io_buf);

    writer->sps = NULL;
    writer->pps = NULL;
    writer->sample_sizes = NULL;
    writer->sample_offsets = NULL;
    writer->sample_durations = NULL;
    writer->sync_samples = NULL;
    writer->sample_buf = NULL;
    writer->io_buf = NULL;
    writer->sps_len = 0;
    writer->pps_len = 0;
    writer->sample_count = 0;
    writer->sample_capacity = 0;
    writer->sync_count = 0;
    writer->sync_capacity = 0;
    writer->sample_buf_capacity = 0;
    writer->io_buf_size = 0;
    writer->last_pts = 0;
    writer->last_duration = 0;
    writer->has_last_pts = false;
    writer->mdat_size_offset = 0;
}

static esp_err_t media_mp4_writer_copy_nalu(uint8_t **dst, size_t *dst_len,
                                            const uint8_t *src, size_t src_len)
{
    uint8_t *buf = NULL;

    if (!dst || !dst_len || !src || src_len == 0) {
        return ESP_ERR_INVALID_ARG;
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

static esp_err_t media_mp4_writer_grow_u32_array(uint32_t **array,
                                                 uint32_t *capacity,
                                                 uint32_t required)
{
    uint32_t new_capacity;
    uint32_t *new_array;

    if (!array || !capacity) {
        return ESP_ERR_INVALID_ARG;
    }

    if (*capacity >= required) {
        return ESP_OK;
    }

    new_capacity = (*capacity == 0U) ? 256U : *capacity;
    while (new_capacity < required) {
        if (new_capacity > (UINT32_MAX / 2U)) {
            return ESP_ERR_NO_MEM;
        }
        new_capacity *= 2U;
    }

    new_array = (uint32_t *)realloc(*array, (size_t)new_capacity * sizeof(uint32_t));
    if (!new_array) {
        return ESP_ERR_NO_MEM;
    }

    *array = new_array;
    *capacity = new_capacity;
    return ESP_OK;
}

static esp_err_t media_mp4_writer_ensure_sample_capacity(media_mp4_writer_t *writer,
                                                         uint32_t required)
{
    uint32_t new_capacity;
    uint32_t *new_sizes = NULL;
    uint32_t *new_offsets = NULL;
    uint32_t *new_durations = NULL;

    if (!writer) {
        return ESP_ERR_INVALID_ARG;
    }

    if (writer->sample_capacity >= required) {
        return ESP_OK;
    }

    new_capacity = (writer->sample_capacity == 0U) ? 256U : writer->sample_capacity;
    while (new_capacity < required) {
        if (new_capacity > (UINT32_MAX / 2U)) {
            return ESP_ERR_NO_MEM;
        }
        new_capacity *= 2U;
    }

    new_sizes = (uint32_t *)realloc(writer->sample_sizes,
                                    (size_t)new_capacity * sizeof(uint32_t));
    if (!new_sizes) {
        return ESP_ERR_NO_MEM;
    }
    writer->sample_sizes = new_sizes;

    new_offsets = (uint32_t *)realloc(writer->sample_offsets,
                                      (size_t)new_capacity * sizeof(uint32_t));
    if (!new_offsets) {
        return ESP_ERR_NO_MEM;
    }
    writer->sample_offsets = new_offsets;

    new_durations = (uint32_t *)realloc(writer->sample_durations,
                                        (size_t)new_capacity * sizeof(uint32_t));
    if (!new_durations) {
        return ESP_ERR_NO_MEM;
    }
    writer->sample_durations = new_durations;
    writer->sample_capacity = new_capacity;
    return ESP_OK;
}

static esp_err_t media_mp4_writer_ensure_sample_buf(media_mp4_writer_t *writer,
                                                    size_t required)
{
    size_t new_capacity;
    uint8_t *new_buf;

    if (!writer) {
        return ESP_ERR_INVALID_ARG;
    }

    if (writer->sample_buf_capacity >= required) {
        return ESP_OK;
    }

    new_capacity = (writer->sample_buf_capacity == 0U) ? (64U * 1024U)
                                                       : writer->sample_buf_capacity;
    while (new_capacity < required) {
        if (new_capacity > (SIZE_MAX / 2U)) {
            return ESP_ERR_NO_MEM;
        }
        new_capacity *= 2U;
    }

    new_buf = (uint8_t *)realloc(writer->sample_buf, new_capacity);
    if (!new_buf) {
        return ESP_ERR_NO_MEM;
    }

    writer->sample_buf = new_buf;
    writer->sample_buf_capacity = new_capacity;
    return ESP_OK;
}

static esp_err_t media_mp4_writer_write_bytes(FILE *fp, const void *data, size_t len)
{
    if (!fp || (!data && len != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0) {
        return ESP_OK;
    }

    if (fwrite(data, 1, len, fp) != len) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t media_mp4_writer_write_u8(FILE *fp, uint8_t value)
{
    return media_mp4_writer_write_bytes(fp, &value, sizeof(value));
}

static esp_err_t media_mp4_writer_write_u16(FILE *fp, uint16_t value)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)((value >> 8) & 0xFFU);
    buf[1] = (uint8_t)(value & 0xFFU);
    return media_mp4_writer_write_bytes(fp, buf, sizeof(buf));
}

static esp_err_t media_mp4_writer_write_u24(FILE *fp, uint32_t value)
{
    uint8_t buf[3];

    buf[0] = (uint8_t)((value >> 16) & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
    buf[2] = (uint8_t)(value & 0xFFU);
    return media_mp4_writer_write_bytes(fp, buf, sizeof(buf));
}

static esp_err_t media_mp4_writer_write_u32(FILE *fp, uint32_t value)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)((value >> 24) & 0xFFU);
    buf[1] = (uint8_t)((value >> 16) & 0xFFU);
    buf[2] = (uint8_t)((value >> 8) & 0xFFU);
    buf[3] = (uint8_t)(value & 0xFFU);
    return media_mp4_writer_write_bytes(fp, buf, sizeof(buf));
}

static esp_err_t media_mp4_writer_write_u32_at(FILE *fp, long offset, uint32_t value)
{
    long cur_pos;
    esp_err_t ret;

    if (!fp || offset < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cur_pos = ftell(fp);
    if (cur_pos < 0) {
        return ESP_FAIL;
    }

    if (fseek(fp, offset, SEEK_SET) != 0) {
        return ESP_FAIL;
    }

    ret = media_mp4_writer_write_u32(fp, value);
    if (ret != ESP_OK) {
        return ret;
    }

    if (fseek(fp, cur_pos, SEEK_SET) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static long media_mp4_writer_box_begin(FILE *fp, const char box_type[4])
{
    long start = 0;

    if (!fp || !box_type) {
        return -1;
    }

    start = ftell(fp);
    if (start < 0) {
        return -1;
    }

    if (media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, box_type, 4) != ESP_OK) {
        return -1;
    }

    return start;
}

static esp_err_t media_mp4_writer_box_end(FILE *fp, long start)
{
    long end = 0;
    long size = 0;

    if (!fp || start < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    end = ftell(fp);
    if (end < 0) {
        return ESP_FAIL;
    }

    size = end - start;
    if (size <= 0 || size > INT32_MAX) {
        return ESP_FAIL;
    }

    return media_mp4_writer_write_u32_at(fp, start, (uint32_t)size);
}

static int media_mp4_writer_parse_nalus(const uint8_t *h264_data, size_t h264_len,
                                        mp4_nalu_info_t *nalus, int max_nalus)
{
    int count = 0;
    size_t i = 0;

    if (!h264_data || h264_len == 0 || !nalus || max_nalus <= 0) {
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
                size_t next = 0;
                bool found_next = false;

                if (nalu_start >= h264_len) {
                    break;
                }

                next = nalu_start + 1U;
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
                nalus[count].type = h264_data[nalu_start] & MP4_NALU_TYPE_MASK;
                count++;

                i = next;
                continue;
            }
        }

        i++;
    }

    return count;
}

static esp_err_t media_mp4_writer_write_ftyp(FILE *fp)
{
    static const char major_brand[4] = {'i', 's', 'o', 'm'};
    static const char compat_0[4] = {'i', 's', 'o', 'm'};
    static const char compat_1[4] = {'a', 'v', 'c', '1'};
    static const char compat_2[4] = {'m', 'p', '4', '1'};
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "ftyp");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_bytes(fp, major_brand, sizeof(major_brand)) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0x00000200U) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, compat_0, sizeof(compat_0)) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, compat_1, sizeof(compat_1)) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, compat_2, sizeof(compat_2)) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_mvhd(FILE *fp, const media_mp4_writer_t *writer,
                                             uint32_t duration)
{
    long box = 0;
    uint32_t matrix[9] = {
        0x00010000U, 0, 0,
        0, 0x00010000U, 0,
        0, 0, 0x40000000U,
    };

    box = media_mp4_writer_box_begin(fp, "mvhd");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->timescale) != ESP_OK ||
        media_mp4_writer_write_u32(fp, duration) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0x00010000U) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0x0100U) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    for (size_t i = 0; i < 9U; i++) {
        if (media_mp4_writer_write_u32(fp, matrix[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    for (size_t i = 0; i < 6U; i++) {
        if (media_mp4_writer_write_u32(fp, 0) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    if (media_mp4_writer_write_u32(fp, MP4_TRACK_ID + 1U) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_tkhd(FILE *fp, const media_mp4_writer_t *writer,
                                             uint32_t duration)
{
    long box = 0;
    uint32_t matrix[9] = {
        0x00010000U, 0, 0,
        0, 0x00010000U, 0,
        0, 0, 0x40000000U,
    };

    box = media_mp4_writer_box_begin(fp, "tkhd");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0x000007U) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, MP4_TRACK_ID) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, duration) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    for (size_t i = 0; i < 9U; i++) {
        if (media_mp4_writer_write_u32(fp, matrix[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    if (media_mp4_writer_write_u32(fp, writer->width << 16) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->height << 16) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_mdhd(FILE *fp, const media_mp4_writer_t *writer,
                                             uint32_t duration)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "mdhd");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->timescale) != ESP_OK ||
        media_mp4_writer_write_u32(fp, duration) != ESP_OK ||
        media_mp4_writer_write_u16(fp, MP4_LANGUAGE_UND) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_hdlr(FILE *fp)
{
    static const char handler_type[4] = {'v', 'i', 'd', 'e'};
    long box = 0;
    size_t name_len = strlen(MP4_HANDLER_NAME) + 1U;

    box = media_mp4_writer_box_begin(fp, "hdlr");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, handler_type, sizeof(handler_type)) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, MP4_HANDLER_NAME, name_len) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_vmhd(FILE *fp)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "vmhd");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0x000001U) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_dinf(FILE *fp)
{
    long dinf = 0;
    long dref = 0;
    long url = 0;

    dinf = media_mp4_writer_box_begin(fp, "dinf");
    if (dinf < 0) {
        return ESP_FAIL;
    }

    dref = media_mp4_writer_box_begin(fp, "dref");
    if (dref < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    url = media_mp4_writer_box_begin(fp, "url ");
    if (url < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0x000001U) != ESP_OK) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_box_end(fp, url) != ESP_OK ||
        media_mp4_writer_box_end(fp, dref) != ESP_OK ||
        media_mp4_writer_box_end(fp, dinf) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t media_mp4_writer_write_avcc(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    if (!writer->sps || writer->sps_len < 4U || !writer->pps || writer->pps_len == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    box = media_mp4_writer_box_begin(fp, "avcC");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u8(fp, writer->sps[1]) != ESP_OK ||
        media_mp4_writer_write_u8(fp, writer->sps[2]) != ESP_OK ||
        media_mp4_writer_write_u8(fp, writer->sps[3]) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0xFFU) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0xE1U) != ESP_OK ||
        media_mp4_writer_write_u16(fp, (uint16_t)writer->sps_len) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, writer->sps, writer->sps_len) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u16(fp, (uint16_t)writer->pps_len) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, writer->pps, writer->pps_len) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stsd(FILE *fp, const media_mp4_writer_t *writer)
{
    long stsd = 0;
    long avc1 = 0;
    uint8_t compressor_name[32] = {0};

    stsd = media_mp4_writer_box_begin(fp, "stsd");
    if (stsd < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    avc1 = media_mp4_writer_box_begin(fp, "avc1");
    if (avc1 < 0) {
        return ESP_FAIL;
    }

    compressor_name[0] = 0;

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, (uint16_t)writer->width) != ESP_OK ||
        media_mp4_writer_write_u16(fp, (uint16_t)writer->height) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0x00480000U) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0x00480000U) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 1) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, compressor_name, sizeof(compressor_name)) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0x0018U) != ESP_OK ||
        media_mp4_writer_write_u16(fp, 0xFFFFU) != ESP_OK) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_avcc(fp, writer) != ESP_OK ||
        media_mp4_writer_box_end(fp, avc1) != ESP_OK ||
        media_mp4_writer_box_end(fp, stsd) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static uint32_t media_mp4_writer_calc_duration(const media_mp4_writer_t *writer)
{
    uint64_t total = 0;

    for (uint32_t i = 0; i < writer->sample_count; i++) {
        total += writer->sample_durations[i];
    }

    if (total > UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)total;
}

static uint32_t media_mp4_writer_count_stts_entries(const media_mp4_writer_t *writer)
{
    uint32_t entry_count = 0;
    uint32_t last_delta = 0;

    for (uint32_t i = 0; i < writer->sample_count; i++) {
        if (i == 0U || writer->sample_durations[i] != last_delta) {
            entry_count++;
            last_delta = writer->sample_durations[i];
        }
    }

    return entry_count;
}

static esp_err_t media_mp4_writer_write_stts(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;
    uint32_t entry_count = 0;
    uint32_t run_count = 0;
    uint32_t run_delta = 0;

    box = media_mp4_writer_box_begin(fp, "stts");
    if (box < 0) {
        return ESP_FAIL;
    }

    entry_count = media_mp4_writer_count_stts_entries(writer);
    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, entry_count) != ESP_OK) {
        return ESP_FAIL;
    }

    for (uint32_t i = 0; i < writer->sample_count; i++) {
        uint32_t delta = writer->sample_durations[i];

        if (run_count == 0U) {
            run_count = 1U;
            run_delta = delta;
            continue;
        }

        if (delta == run_delta) {
            run_count++;
            continue;
        }

        if (media_mp4_writer_write_u32(fp, run_count) != ESP_OK ||
            media_mp4_writer_write_u32(fp, run_delta) != ESP_OK) {
            return ESP_FAIL;
        }

        run_count = 1U;
        run_delta = delta;
    }

    if (writer->sample_count > 0U) {
        if (media_mp4_writer_write_u32(fp, run_count) != ESP_OK ||
            media_mp4_writer_write_u32(fp, run_delta) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stss(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "stss");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->sync_count) != ESP_OK) {
        return ESP_FAIL;
    }

    for (uint32_t i = 0; i < writer->sync_count; i++) {
        if (media_mp4_writer_write_u32(fp, writer->sync_samples[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stsc(FILE *fp)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "stsc");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stsz(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "stsz");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->sample_count) != ESP_OK) {
        return ESP_FAIL;
    }

    for (uint32_t i = 0; i < writer->sample_count; i++) {
        if (media_mp4_writer_write_u32(fp, writer->sample_sizes[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stco(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "stco");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_u8(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u24(fp, 0) != ESP_OK ||
        media_mp4_writer_write_u32(fp, writer->sample_count) != ESP_OK) {
        return ESP_FAIL;
    }

    for (uint32_t i = 0; i < writer->sample_count; i++) {
        if (media_mp4_writer_write_u32(fp, writer->sample_offsets[i]) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_stbl(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "stbl");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_stsd(fp, writer) != ESP_OK ||
        media_mp4_writer_write_stts(fp, writer) != ESP_OK) {
        return ESP_FAIL;
    }

    if (writer->sync_count > 0U && media_mp4_writer_write_stss(fp, writer) != ESP_OK) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_stsc(fp) != ESP_OK ||
        media_mp4_writer_write_stsz(fp, writer) != ESP_OK ||
        media_mp4_writer_write_stco(fp, writer) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_minf(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "minf");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_vmhd(fp) != ESP_OK ||
        media_mp4_writer_write_dinf(fp) != ESP_OK ||
        media_mp4_writer_write_stbl(fp, writer) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_mdia(FILE *fp, const media_mp4_writer_t *writer,
                                             uint32_t duration)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "mdia");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_mdhd(fp, writer, duration) != ESP_OK ||
        media_mp4_writer_write_hdlr(fp) != ESP_OK ||
        media_mp4_writer_write_minf(fp, writer) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_trak(FILE *fp, const media_mp4_writer_t *writer,
                                             uint32_t duration)
{
    long box = 0;

    box = media_mp4_writer_box_begin(fp, "trak");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_tkhd(fp, writer, duration) != ESP_OK ||
        media_mp4_writer_write_mdia(fp, writer, duration) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

static esp_err_t media_mp4_writer_write_moov(FILE *fp, const media_mp4_writer_t *writer)
{
    long box = 0;
    uint32_t duration = 0;

    duration = media_mp4_writer_calc_duration(writer);

    box = media_mp4_writer_box_begin(fp, "moov");
    if (box < 0) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_mvhd(fp, writer, duration) != ESP_OK ||
        media_mp4_writer_write_trak(fp, writer, duration) != ESP_OK) {
        return ESP_FAIL;
    }

    return media_mp4_writer_box_end(fp, box);
}

void media_mp4_writer_init(media_mp4_writer_t *writer)
{
    if (!writer) {
        return;
    }

    memset(writer, 0, sizeof(*writer));
}

void media_mp4_writer_deinit(media_mp4_writer_t *writer)
{
    FILE *fp = media_mp4_writer_file(writer);

    if (!writer) {
        return;
    }

    if (fp) {
        fclose(fp);
        writer->fp = NULL;
    }

    media_mp4_writer_reset_metadata(writer);
    writer->width = 0;
    writer->height = 0;
    writer->timescale = 0;
    writer->default_sample_duration = 0;
}

bool media_mp4_writer_is_open(const media_mp4_writer_t *writer)
{
    return media_mp4_writer_file(writer) != NULL;
}

esp_err_t media_mp4_writer_open(media_mp4_writer_t *writer, const char *path,
                                const media_mp4_writer_config_t *config)
{
    FILE *fp = NULL;
    esp_err_t ret = ESP_OK;

    if (!writer || !path || !config ||
        config->width == 0U || config->height == 0U ||
        config->timescale == 0U || config->default_sample_duration == 0U ||
        !config->sps || config->sps_len < 4U ||
        !config->pps || config->pps_len == 0U ||
        config->sps_len > UINT16_MAX || config->pps_len > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (media_mp4_writer_is_open(writer)) {
        return ESP_ERR_INVALID_STATE;
    }

    media_mp4_writer_reset_metadata(writer);

    ret = media_mp4_writer_copy_nalu(&writer->sps, &writer->sps_len,
                                     config->sps, config->sps_len);
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = media_mp4_writer_copy_nalu(&writer->pps, &writer->pps_len,
                                     config->pps, config->pps_len);
    if (ret != ESP_OK) {
        goto fail;
    }

    fp = fopen(path, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "打开 MP4 文件失败: %s, errno=%d", path, errno);
        ret = ESP_FAIL;
        goto fail;
    }

    writer->io_buf = (uint8_t *)heap_caps_malloc(MP4_FILE_IO_BUF_SIZE,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (writer->io_buf) {
        if (setvbuf(fp, (char *)writer->io_buf, _IOFBF, MP4_FILE_IO_BUF_SIZE) == 0) {
            writer->io_buf_size = MP4_FILE_IO_BUF_SIZE;
        } else {
            ESP_LOGW(TAG, "设置 MP4 文件写入缓冲失败，将使用默认缓冲");
            free(writer->io_buf);
            writer->io_buf = NULL;
            writer->io_buf_size = 0;
        }
    } else {
        ESP_LOGW(TAG, "申请 MP4 文件写入缓冲失败，将使用默认缓冲");
    }

    writer->fp = fp;
    writer->width = config->width;
    writer->height = config->height;
    writer->timescale = config->timescale;
    writer->default_sample_duration = config->default_sample_duration;
    writer->last_duration = config->default_sample_duration;

    ret = media_mp4_writer_write_ftyp(fp);
    if (ret != ESP_OK) {
        goto fail;
    }

    writer->mdat_size_offset = ftell(fp);
    if (writer->mdat_size_offset < 0) {
        ret = ESP_FAIL;
        goto fail;
    }

    if (media_mp4_writer_write_u32(fp, 0) != ESP_OK ||
        media_mp4_writer_write_bytes(fp, "mdat", 4) != ESP_OK) {
        ret = ESP_FAIL;
        goto fail;
    }

    return ESP_OK;

fail:
    if (fp) {
        fclose(fp);
        writer->fp = NULL;
    }
    media_mp4_writer_reset_metadata(writer);
    writer->width = 0;
    writer->height = 0;
    writer->timescale = 0;
    writer->default_sample_duration = 0;
    return ret;
}

esp_err_t media_mp4_writer_write_frame(media_mp4_writer_t *writer,
                                       const uint8_t *h264_buf, size_t h264_len,
                                       uint32_t pts, bool is_sync)
{
    FILE *fp = NULL;
    mp4_nalu_info_t nalus[32];
    int nalu_count = 0;
    uint32_t sample_index = 0;
    uint32_t sample_size = 0;
    size_t sample_pos = 0;
    long sample_offset = 0;
    esp_err_t ret = ESP_OK;

    if (!writer || !h264_buf || h264_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    fp = media_mp4_writer_file(writer);
    if (!fp) {
        return ESP_ERR_INVALID_STATE;
    }

    nalu_count = media_mp4_writer_parse_nalus(h264_buf, h264_len, nalus,
                                              (int)(sizeof(nalus) / sizeof(nalus[0])));
    if (nalu_count <= 0) {
        return ESP_FAIL;
    }

    ret = media_mp4_writer_ensure_sample_capacity(writer, writer->sample_count + 1U);
    if (ret != ESP_OK) {
        return ret;
    }

    if (is_sync) {
        ret = media_mp4_writer_grow_u32_array(&writer->sync_samples,
                                              &writer->sync_capacity,
                                              writer->sync_count + 1U);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    for (int i = 0; i < nalu_count; i++) {
        if (nalus[i].type == MP4_NALU_TYPE_SPS || nalus[i].type == MP4_NALU_TYPE_PPS) {
            continue;
        }
        if (nalus[i].len > UINT32_MAX - 4U ||
            sample_size > UINT32_MAX - 4U - (uint32_t)nalus[i].len) {
            return ESP_FAIL;
        }

        sample_size += 4U + (uint32_t)nalus[i].len;
    }

    if (sample_size == 0U) {
        return ESP_FAIL;
    }

    ret = media_mp4_writer_ensure_sample_buf(writer, sample_size);
    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < nalu_count; i++) {
        uint32_t nalu_len;
        const uint8_t *nalu_ptr = &h264_buf[nalus[i].offset];

        if (nalus[i].type == MP4_NALU_TYPE_SPS || nalus[i].type == MP4_NALU_TYPE_PPS) {
            continue;
        }

        nalu_len = (uint32_t)nalus[i].len;
        writer->sample_buf[sample_pos++] = (uint8_t)((nalu_len >> 24) & 0xFFU);
        writer->sample_buf[sample_pos++] = (uint8_t)((nalu_len >> 16) & 0xFFU);
        writer->sample_buf[sample_pos++] = (uint8_t)((nalu_len >> 8) & 0xFFU);
        writer->sample_buf[sample_pos++] = (uint8_t)(nalu_len & 0xFFU);
        memcpy(writer->sample_buf + sample_pos, nalu_ptr, nalus[i].len);
        sample_pos += nalus[i].len;
    }

    sample_offset = ftell(fp);
    if (sample_offset < 0) {
        return ESP_FAIL;
    }
    if ((unsigned long)sample_offset > UINT32_MAX) {
        return ESP_FAIL;
    }

    if (sample_pos != sample_size) {
        return ESP_FAIL;
    }

    if (media_mp4_writer_write_bytes(fp, writer->sample_buf, sample_size) != ESP_OK) {
        return ESP_FAIL;
    }

    sample_index = writer->sample_count;
    writer->sample_sizes[sample_index] = sample_size;
    writer->sample_offsets[sample_index] = (uint32_t)sample_offset;
    writer->sample_durations[sample_index] = writer->default_sample_duration;

    if (writer->has_last_pts && writer->sample_count > 0U) {
        uint32_t delta = (pts > writer->last_pts) ? (pts - writer->last_pts)
                                                  : writer->default_sample_duration;

        if (delta == 0U) {
            delta = writer->default_sample_duration;
        }

        writer->sample_durations[writer->sample_count - 1U] = delta;
        writer->last_duration = delta;
    }

    if (is_sync) {
        writer->sync_samples[writer->sync_count] = writer->sample_count + 1U;
        writer->sync_count++;
    }

    writer->sample_count++;
    writer->last_pts = pts;
    writer->has_last_pts = true;
    return ESP_OK;
}

esp_err_t media_mp4_writer_close(media_mp4_writer_t *writer)
{
    FILE *fp = NULL;
    esp_err_t ret = ESP_OK;
    long media_end = 0;

    if (!writer) {
        return ESP_ERR_INVALID_ARG;
    }

    fp = media_mp4_writer_file(writer);
    if (!fp) {
        return ESP_ERR_INVALID_STATE;
    }

    if (writer->sample_count > 0U) {
        uint32_t final_duration = writer->last_duration;

        if (final_duration == 0U) {
            final_duration = writer->default_sample_duration;
        }
        writer->sample_durations[writer->sample_count - 1U] = final_duration;
    }

    media_end = ftell(fp);
    if (media_end < 0) {
        ret = ESP_FAIL;
        goto done;
    }

    if ((media_end - writer->mdat_size_offset) > INT32_MAX) {
        ret = ESP_FAIL;
        goto done;
    }

    ret = media_mp4_writer_write_u32_at(fp, writer->mdat_size_offset,
                                        (uint32_t)(media_end - writer->mdat_size_offset));
    if (ret != ESP_OK) {
        goto done;
    }

    ret = media_mp4_writer_write_moov(fp, writer);
    if (ret != ESP_OK) {
        goto done;
    }

    if (fflush(fp) != 0 || fsync(fileno(fp)) != 0) {
        ret = ESP_FAIL;
    }

done:
    fclose(fp);
    writer->fp = NULL;
    media_mp4_writer_reset_metadata(writer);
    writer->width = 0;
    writer->height = 0;
    writer->timescale = 0;
    writer->default_sample_duration = 0;
    return ret;
}
