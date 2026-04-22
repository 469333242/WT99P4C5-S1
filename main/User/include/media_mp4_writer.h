/**
 * @file media_mp4_writer.h
 * @brief H.264 单路 MP4 封装辅助模块
 *
 * 该模块只负责把 Annex-B 格式的 H.264 码流写入 MP4 文件，
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

/**
 * @brief MP4 写入参数
 */
typedef struct {
    uint32_t width;                    /* 视频宽度 */
    uint32_t height;                   /* 视频高度 */
    uint32_t timescale;                /* 输入 PTS 的时间基 */
    uint32_t default_sample_duration;  /* 默认帧时长，单位与 timescale 一致 */
    const uint8_t *sps;                /* SPS NALU，不含起始码 */
    size_t sps_len;                    /* SPS 长度 */
    const uint8_t *pps;                /* PPS NALU，不含起始码 */
    size_t pps_len;                    /* PPS 长度 */
} media_mp4_writer_config_t;

/**
 * @brief MP4 写入器状态
 *
 * 对外仍然保持为一个普通结构体，便于静态分配；
 * 内部句柄与缓存字段由实现文件负责解释。
 */
typedef struct {
    void *muxer;               /* esp_muxer_handle_t，对外保持不透明 */
    int video_stream_index;    /* muxer 返回的视频流索引 */
    uint32_t width;
    uint32_t height;
    uint32_t timescale;
    uint32_t default_sample_duration;
    uint8_t fps;

    char *path;                /* 当前输出文件路径副本 */
    uint8_t *codec_spec_info;  /* StartCode + SPS + StartCode + PPS */
    size_t codec_spec_info_len;
} media_mp4_writer_t;

/**
 * @brief 初始化 MP4 写入器结构体
 */
void media_mp4_writer_init(media_mp4_writer_t *writer);

/**
 * @brief 释放 MP4 写入器占用的资源
 *
 * 如果文件尚未正常 close，本接口会尽力关闭内部句柄并释放内存。
 */
void media_mp4_writer_deinit(media_mp4_writer_t *writer);

/**
 * @brief 判断 MP4 文件是否已打开
 */
bool media_mp4_writer_is_open(const media_mp4_writer_t *writer);

/**
 * @brief 打开一个新的 MP4 文件并创建视频轨道
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
 * @brief 写入一帧 H.264 码流
 *
 * 输入为 Annex-B 格式帧数据，内部会交给官方 muxer 封装为 MP4。
 *
 * @param writer    写入器
 * @param h264_buf  Annex-B H.264 帧数据
 * @param h264_len  数据长度
 * @param pts       输入帧 PTS，单位由 timescale 决定
 * @param is_sync   是否为关键帧
 *
 * @return ESP_OK 成功；其它值表示失败
 */
esp_err_t media_mp4_writer_write_frame(media_mp4_writer_t *writer,
                                       const uint8_t *h264_buf, size_t h264_len,
                                       uint32_t pts, bool is_sync);

/**
 * @brief 结束写入并完成 MP4 封装
 *
 * @return ESP_OK 成功；其它值表示失败
 */
esp_err_t media_mp4_writer_close(media_mp4_writer_t *writer);

#ifdef __cplusplus
}
#endif
