/**
 * @file media_mp4_writer.h
 * @brief H.264 单路 MP4 文件写入辅助模块
 *
 * 该模块只负责把 Annex-B 格式 H.264 压缩帧封装为 MP4 文件，
 * 不涉及采集、编码、分段策略和队列调度。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t width;                    /* 视频宽度 */
    uint32_t height;                   /* 视频高度 */
    uint32_t timescale;                /* MP4 时间基 */
    uint32_t default_sample_duration;  /* 默认帧时长 */
    const uint8_t *sps;                /* SPS NALU，不含起始码 */
    size_t sps_len;                    /* SPS 长度 */
    const uint8_t *pps;                /* PPS NALU，不含起始码 */
    size_t pps_len;                    /* PPS 长度 */
} media_mp4_writer_config_t;

typedef struct {
    void *fp;                          /* FILE*，对外保持不透明 */
    uint32_t width;
    uint32_t height;
    uint32_t timescale;
    uint32_t default_sample_duration;

    uint8_t *sps;
    size_t sps_len;
    uint8_t *pps;
    size_t pps_len;

    uint32_t *sample_sizes;
    uint32_t *sample_offsets;
    uint32_t *sample_durations;
    uint32_t *sync_samples;
    uint32_t sample_count;
    uint32_t sample_capacity;
    uint32_t sync_count;
    uint32_t sync_capacity;

    uint32_t last_pts;
    uint32_t last_duration;
    bool has_last_pts;

    uint8_t *sample_buf;
    size_t sample_buf_capacity;
    uint8_t *io_buf;
    size_t io_buf_size;

    long mdat_size_offset;
} media_mp4_writer_t;

/**
 * @brief 初始化 MP4 写入器结构体
 */
void media_mp4_writer_init(media_mp4_writer_t *writer);

/**
 * @brief 释放 MP4 写入器占用的资源
 *
 * 若文件尚未正常 close，本接口只负责关闭文件句柄和释放内存，
 * 不保证输出文件可播放。
 */
void media_mp4_writer_deinit(media_mp4_writer_t *writer);

/**
 * @brief 判断 MP4 文件是否已打开
 */
bool media_mp4_writer_is_open(const media_mp4_writer_t *writer);

/**
 * @brief 打开一个新的 MP4 文件并写入文件头
 *
 * @param writer  写入器
 * @param path    目标文件路径
 * @param config  视频参数
 *
 * @return ESP_OK 成功；其它值表示失败
 */
esp_err_t media_mp4_writer_open(media_mp4_writer_t *writer, const char *path,
                                const media_mp4_writer_config_t *config);

/**
 * @brief 写入一帧 H.264 压缩帧
 *
 * 输入为 Annex-B 码流，内部会转换为 MP4 所需的长度前缀格式。
 * SPS/PPS 不重复写入 sample，而是通过 avcC 保存。
 *
 * @param writer    写入器
 * @param h264_buf  Annex-B H.264 帧数据
 * @param h264_len  数据长度
 * @param pts       90kHz 时基下的显示时间戳
 * @param is_sync   是否为关键帧
 *
 * @return ESP_OK 成功；其它值表示失败
 */
esp_err_t media_mp4_writer_write_frame(media_mp4_writer_t *writer,
                                       const uint8_t *h264_buf, size_t h264_len,
                                       uint32_t pts, bool is_sync);

/**
 * @brief 结束写入并补齐 moov 索引
 *
 * @return ESP_OK 成功；其它值表示失败
 */
esp_err_t media_mp4_writer_close(media_mp4_writer_t *writer);

#ifdef __cplusplus
}
#endif
