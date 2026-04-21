# WT99P4C5-S1 视频与媒体存储工程

## 1. 项目概述

本工程运行在 `WT99P4C5-S1` 开发板上，核心目标是把 OV5647 MIPI 摄像头采集、硬件 ISP、H.264 编码、RTSP 推流、TF 卡媒体存储和网页浏览管理串起来。

当前工程的主链路是：

```text
OV5647 MIPI-CSI
  -> V4L2 采集
  -> 硬件 ISP 输出 YUV420
  -> H.264 硬件编码
  -> RTSP 推流
  -> 复用同一份 H.264 码流旁路封装 MP4
```

照片链路与视频链路共用同一次上电会话目录，统一落到 TF 卡。

## 2. 当前已实现功能

### 2.1 网络接入

- 支持通过 `ESP-Hosted + ESP32-C5 SDIO` 方式联网。
- 工程保留了以太网初始化入口，当前默认走 Wi-Fi。
- 上电后会等待网络连通，并尝试同步系统时间。

### 2.2 实时视频

- 支持 OV5647 MIPI-CSI 采集。
- 支持硬件 ISP 输出 `YUV420`。
- 支持 H.264 硬件编码。
- 支持 RTSP/RTP 推流，默认地址：

```text
rtsp://<设备IP>:8554/stream
```

### 2.3 TF 卡媒体存储

- 支持 TF 卡挂载与状态检测。
- 支持自动拍照保存 JPEG。
- 支持自动录像保存 MP4。
- 录像直接复用主链路已经编码完成的 H.264 码流，不做第二路视频编码。
- MP4 默认按 `120 秒` 分段。

### 2.4 媒体网页

- 启动 HTTP 服务，默认端口 `80`。
- 支持浏览照片和视频。
- 支持播放 TF 卡中保存的 MP4 文件。
- 支持同步设备时间。
- 支持删除照片和视频文件。

访问地址：

```text
http://<设备IP>/
```

### 2.5 TCP-UART 透传

- `tcp_uart_server` 模块已经存在。
- 当前 `hello_world_main.c` 中启动代码默认注释，属于“代码已接入、默认未启用”状态。
- 对应端口预留为：

```text
TCP <设备IP>:8880
TCP <设备IP>:8881
```

## 3. 当前行为与触发方式

### 3.1 RTSP 播放触发

当 RTSP 客户端从 `0 -> 1` 连接时，`camera` 模块会：

1. 启动相机采集任务。
2. 请求一次自动拍照。
3. 打开录像请求标志。

当 RTSP 客户端全部断开时，`camera` 模块会：

1. 停止相机采集。
2. 请求停止录像。

也就是说，当前版本的录像不是通过网页按钮或独立控制接口启动，而是跟随 RTSP 会话自动开始和停止。

对应入口在：

- `main/User/camera.c`
- `main/User/media_storage.c`

## 4. MP4 封装是怎么进行的

### 4.1 总体思路

工程里的 MP4 封装不是重新编码视频，而是把主链路已经产出的 `Annex-B H.264` 码流旁路复制出来，在后台任务中封装成单路视频 MP4。

设计约束如下：

- 只封装视频，不包含音频。
- 只处理 H.264。
- 假定码流无 B 帧，因此不生成 `ctts`。
- 一个 sample 对应一个 chunk，简化 `stsc/stco` 组织。
- `moov` 只在正常关闭文件时补写，因此异常掉电时当前段可能不可播放。

这些设计都在 `main/User/media_mp4_writer.c` 中实现。

### 4.2 录像触发与旁路入队

相机采集任务完成一帧 H.264 编码后，会同时做两件事：

1. 继续把 H.264 推给 RTSP。
2. 调用 `media_storage_process_h264_frame()` 把同一帧提交给录像旁路。

录像旁路只做非阻塞复制和入队：

- 从空闲槽位里拿一个缓冲。
- 把 H.264 数据拷贝进去。
- 把槽位索引投递给后台录像任务。

如果没有空闲槽位，当前帧直接丢弃，避免反向卡住 RTSP 主链路。

对应位置：

- `main/User/camera.c`
- `main/User/media_storage.c`

### 4.3 后台录像任务何时真正开段

后台录像任务拿到 H.264 帧后，不是立刻建 MP4 文件，而是先满足以下条件：

- 已经有录像请求。
- 已经从码流中提取到 `SPS/PPS`。
- 当前帧是 `IDR`。

只有这样才会创建新的 MP4 分段，这样可以保证新文件从关键帧开始，播放器更容易正常解码。

如果中间发生丢帧，代码会先跳过后续非 IDR 帧，等到下一帧 IDR 再继续向当前分段写入，避免直接把半个 GOP 拼进 MP4。

### 4.4 打开 MP4 分段时做了什么

当满足开段条件后，`media_storage_video_open_segment()` 会：

1. 生成临时文件路径和最终文件路径。
2. 构造 `media_mp4_writer_config_t`，填入：
   - 宽高
   - timescale
   - 默认 sample duration
   - SPS/PPS
3. 调用 `media_mp4_writer_open()` 打开 `.tmp` 文件。

`media_mp4_writer_open()` 具体会：

1. 复制 SPS/PPS 到 writer 内部状态。
2. `fopen(path, "wb")` 打开文件。
3. 尝试申请 `512 KB` 的文件 IO 缓冲，并通过 `setvbuf()` 绑定给 `FILE*`。
4. 先写 `ftyp` box。
5. 再写一个占位大小的 `mdat` 头，真正视频数据后续直接追加进去。

### 4.5 每帧写入 MP4 时做了什么

`media_mp4_writer_write_frame()` 的处理流程如下：

1. 解析输入 H.264 中的 `Annex-B NALU`。
2. 过滤掉 `SPS/PPS`，因为参数集不重复写进 sample。
3. 把剩余 NALU 从 `Annex-B 起始码格式` 转成 `4 字节长度 + NALU 数据` 的 MP4/AVCC sample 格式。
4. 把 sample 直接写到 `mdat`。
5. 同时记录每个 sample 的元数据：
   - `sample size`
   - `sample offset`
   - `sample duration`
   - `sync sample` 索引

这里记录下来的元数据，会在收尾阶段用于生成 `moov` 里的各种索引表。

### 4.6 关闭 MP4 分段时做了什么

`media_mp4_writer_close()` 在关闭文件时会：

1. 计算最后一个 sample 的 duration。
2. 回写 `mdat` 的实际大小。
3. 生成并写入 `moov`。
4. `fflush()` + `fsync()`，确保数据刷盘。
5. 关闭文件。

`moov` 中实际写入的结构包括：

- `mvhd`
- `trak`
- `tkhd`
- `mdia`
- `mdhd`
- `hdlr`
- `minf`
- `vmhd`
- `dinf`
- `stbl`
- `stsd`
- `avc1`
- `avcC`
- `stts`
- `stss`
- `stsc`
- `stsz`
- `stco`

其中：

- `avcC` 中保存的是 `SPS/PPS`
- `stts` 保存时间戳信息
- `stss` 保存关键帧索引
- `stsz` 保存每帧大小
- `stco` 保存每帧偏移

完成后，外层逻辑会把 `.tmp` 重命名为最终 `.mp4`。

### 4.7 MP4 封装相关源码位置

- `main/User/media_storage.c`
  - 负责录像请求、H.264 旁路入队、分段开关、异常丢帧续写策略
- `main/User/media_mp4_writer.c`
  - 负责 MP4 box 组织、sample 写入、索引表生成、文件收尾
- `main/User/media_mp4_writer.h`
  - MP4 writer 配置与对外接口

## 5. 照片保存链路

### 5.1 触发方式

当前版本在 RTSP 客户端开始播放时，会触发一次自动拍照请求。

### 5.2 数据处理流程

照片链路如下：

```text
YUV420 相机帧
  -> 拷贝到独立拍照缓冲
  -> PPA 硬件转换为 RGB565
  -> JPEG 硬件编码
  -> 写入 TF 卡
```

这里的目标是把耗时的颜色转换、JPEG 编码、文件写入放到后台任务中做，减少对 RTSP 主链路的影响。

主要代码位置：

- `main/User/media_storage.c`

## 6. 媒体网页接口

当前网页模块位于：

- `main/User/photo_web_server.c`
- `main/User/photo_web_server.h`

已实现接口如下：

```text
GET  /             媒体首页
GET  /api/photos   照片列表 JSON
GET  /api/videos   视频列表 JSON
GET  /api/time     同步设备时间
POST /api/delete   删除照片或视频
GET  /photo/*      返回 JPEG 原图
GET  /video/*      返回 MP4 文件，支持 Range 请求
HEAD /video/*      返回 MP4 头信息，支持视频在线播放
```

网页当前支持：

- 浏览照片
- 浏览视频
- 在线播放 MP4
- 同步设备时间
- 选择并删除媒体文件

## 7. 目录结构与命名规则

照片和视频共用同一次上电会话目录，典型结构如下：

```text
/sdcard/<session>/photo/<timestamp>.jpeg
/sdcard/<session>/video/<timestamp>_<segment>.mp4
```

例如：

```text
/sdcard/001_20260421/photo/2026-04-21T16-20-15-123.jpeg
/sdcard/001_20260421/video/2026-04-21T16-20-15-123_0001.mp4
```

说明：

- `<session>` 为本次上电会话目录。
- 会话目录只有在本次上电第一次真正保存媒体时才会创建。
- 照片和视频都写入同一个会话目录下的不同子目录。
- 系统时间未同步时，会退回默认日期命名。

## 8. 主要源码文件

- `main/hello_world_main.c`
  - 系统启动入口，负责初始化网络、TF 卡、媒体模块、网页和 RTSP。
- `main/User/camera.c`
  - 摄像头采集、ISP 格式切换、H.264 编码、RTSP 推流、媒体旁路触发。
- `main/User/media_storage.c`
  - 照片/视频存储主逻辑，后台任务、缓冲管理、分段控制、文件路径生成。
- `main/User/media_mp4_writer.c`
  - MP4 封装实现。
- `main/User/photo_web_server.c`
  - TF 卡媒体网页和接口。
- `main/User/tf_card.c`
  - TF 卡挂载与状态检测。
- `main/User/rtsp_server.c`
  - RTSP/RTP 服务。
- `main/User/tcp_uart_server.c`
  - TCP-UART 透传。

## 9. 启动顺序

当前 `app_main()` 的实际启动顺序是：

1. 初始化 NVS。
2. 初始化网络接口和事件循环。
3. 建立 ESP-Hosted SDIO 链路。
4. 连接 Wi-Fi，并等待获取 IP。
5. 尝试同步时间。
6. 初始化 TF 卡。
7. 初始化 `media_storage`。
8. 启动媒体网页服务。
9. 启动 RTSP 服务。
10. 初始化摄像头。

## 10. 分辨率与配置

`main/User/camera.c` 里当前保留了多组 profile，包含：

- `1280x960`
- `1920x1080`
- `800x800`
- `800x640`

当前通过 `H264_PROFILE` 在编译期选择目标 profile，并根据分辨率切换 OV5647 的 sensor mode。

另外，录像旁路的缓冲槽位数量会随分辨率动态调整，1080p 和 1280x960 使用的录像缓冲深度不同。

## 11. 当前限制与说明

- MP4 只在正常 close 时补写 `moov`，异常掉电时当前段可能不可播放。
- 当前录像与 RTSP 会话绑定，默认不是常驻录像。
- 当前网页主要用于媒体浏览、删除和时间同步，不负责控制录像启停。
- `tcp_uart_server` 默认未在 `app_main()` 中启动。
- 高分辨率下，照片和录像都会明显增加内存占用，需要结合实际 SPIRAM 余量调整。

## 12. 使用示例

### 12.1 RTSP 播放

```text
rtsp://<设备IP>:8554/stream
```

例如：

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -rtsp_transport tcp "rtsp://192.168.137.200:8554/stream"
```

### 12.2 媒体网页

```text
http://<设备IP>/
```

### 12.3 获取照片列表

```text
http://<设备IP>/api/photos
```

### 12.4 获取视频列表

```text
http://<设备IP>/api/videos
```
