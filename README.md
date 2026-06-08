# WT99P4C5-S1 视频与媒体存储工程

本工程运行在 WT99P4C5-S1 开发板上，面向实时视频、热成像、RTSP 推流、TF 卡媒体存储、网页配置管理、TCP-UART 透传以及 Z-1mini 吊舱网络透传场景。

如果只想了解系统整体怎么工作，可以先阅读 [SYSTEM_IMPLEMENTATION.md](SYSTEM_IMPLEMENTATION.md)。该文档面向不熟悉代码的读者，解释系统采用的方法和数据流，不展开具体函数实现。

## 当前默认运行方式

当前代码默认启用 Z-1mini 网口透传模式：

```text
Z1MINI_BRIDGE_ENABLE = 1
USE_ETHERNET = 0
```

默认网络关系如下：

```text
电脑/手机 Wi-Fi
  -> WT99 Wi-Fi AP: 192.168.144.2
  -> WT99 板载以太网原始帧转发
  -> Z-1mini 吊舱: 192.168.144.108
```

常用默认地址：

| 用途 | 地址 |
| --- | --- |
| WT99 网页 | `http://192.168.144.2/` |
| WT99 本机 RTSP | `rtsp://192.168.144.2` |
| WT99 A3 兼容热成像 RTSP | `rtsp://192.168.144.2:580/live/6` |
| Z-1mini 吊舱 RTSP | `rtsp://192.168.144.108` |
| TCP-UART0 | `192.168.144.2:8880` |

说明：

- WT99 本机服务和 Z-1mini 吊舱服务是两个不同目标。
- WT99 默认 AP 地址是 `192.168.144.2`。
- Z-1mini 吊舱默认地址是 `192.168.144.108`。
- 当前 RTSP 主端口是 `554`，所以 WT99 本机 RTSP 地址可以写成 `rtsp://<设备IP>`。

## 主要能力

- OV5647 MIPI-CSI 摄像头采集
- USB UVC 热像仪 Y16 原始热数据采集
- USB 热像仪本地白热、黑热、铁红成像效果
- 硬件 ISP、硬件 H.264 编码、硬件 JPEG 编码
- RTSP over TCP 实时推流
- TF 卡 JPEG 照片保存
- TF 卡 MP4 分段录像保存
- TF 卡容量、剩余录像时长、可拍照数量和读写测速状态查看
- 网页端状态查看、参数配置、拍照、媒体浏览、MP4 播放、批量删除
- A3 兼容接口：SD 卡状态查询、录像/拍照控制、时间同步
- TCP-UART0 透传，UART1 透传代码已接入但默认未启动
- Wi-Fi AP/STA 配置和 STA 失联自动回退 AP
- Z-1mini 吊舱以太网到 Wi-Fi AP 原始帧透传

## 硬件与工程配置

当前工程关键配置：

| 项目 | 当前值 |
| --- | --- |
| ESP-IDF target | `esp32p4` |
| Flash | `16MB` |
| PSRAM | 已启用，`200MHz`，可用于 malloc |
| 分区表 | 自定义 `partitions.csv` |
| Wi-Fi 协处理器 | ESP32-C5，ESP-Hosted over SDIO Slot1 |
| TF 卡 | SDMMC Slot0，4-bit，高速模式，挂载点 `/sdcard` |
| LWIP socket 数 | `20` |

TF 卡引脚：

| 信号 | GPIO |
| --- | --- |
| CLK | 43 |
| CMD | 44 |
| D0 | 39 |
| D1 | 40 |
| D2 | 41 |
| D3 | 42 |

ESP-Hosted SDIO Slot1 引脚由 `sdkconfig` 配置，当前使用 CMD 19、CLK 18、D0-D3 14-17、C5 reset 54。

## 视频链路

### MIPI 摄像头

MIPI 路径用于 OV5647 摄像头：

```text
OV5647
  -> MIPI-CSI
  -> ISP
  -> YUV420
  -> H.264 编码
  -> RTSP 推流
  -> 可旁路保存 MP4
```

当前固件支持的 MIPI 档位：

| 档位 | 分辨率 | 帧率 |
| --- | --- | --- |
| 1 | `1280 x 960` | 45 fps |
| 2 | `1920 x 1080` | 30 fps |
| 3 | `800 x 800` | 50 fps |
| 4 | `800 x 640` | 50 fps |

### USB 热像仪

USB 热像仪路径用于 UVC Y16 热数据：

```text
USB UVC 热像仪
  -> Y16 原始热数据
  -> 按当前帧最小/最大热值做灰度拉伸
  -> 白热/黑热/铁红本地成像
  -> H.264 编码
  -> RTSP 推流
  -> 可旁路保存 MP4
```

当前热像仪输入按 `512 x 390` Y16 处理。H.264 编码时按 `512 x 400` 对齐，底部补黑只用于编码宏块对齐；照片仍按实际 `512 x 390` 图像区域保存。

热像仪成像效果：

| 值 | 效果 |
| --- | --- |
| `0` | 白热 |
| `1` | 黑热 |
| `2` | 铁红 |

热像仪成像效果是本机对 Y16 数据进行转换，实时视频、录像和照片会保持一致。厂商私有 USB 成像命令当前默认关闭。

## RTSP 推流

当前 RTSP 服务监听两个端口：

| 端口 | 用途 |
| --- | --- |
| `554` | WT99 本机主 RTSP |
| `580` | A3 兼容热成像 RTSP |

RTSP 采用 TCP 承载，RTP 数据通过 RTSP 连接交织发送，适合弱网络和低延迟场景。

当前策略：

- 最大 RTSP 客户端数为 `2`。
- 客户端播放时激活视频采集，没有客户端且没有外部控制时暂停采集，降低负载。
- 每个客户端只保留很短的发送队列，网络拥塞时优先丢弃旧帧，等待新的关键帧恢复画面。
- RTSP 服务保存最新 SPS/PPS，用于客户端重新建立解码。

## TF 卡媒体存储

TF 卡挂载点：

```text
/sdcard
```

媒体目录结构：

```text
/sdcard/<session>/photo/<time>-<photo_index>.jpeg
/sdcard/<session>/video/<timestamp>_<segment>.mp4
```

示例：

```text
/sdcard/001_20260608/photo/2026-06-08T10-20-15-001.jpeg
/sdcard/001_20260608/video/2026-06-08T10-20-15-123_002.mp4
```

存储策略：

- 拍照前检查 TF 卡是否挂载和剩余空间是否足够。
- 启动新录像前检查 TF 卡剩余空间是否足够。
- 录像已经开始后，如果切分新 MP4 时空间不足，允许删除最早会话中的旧 MP4 后继续录像。
- 自动覆盖只删除旧视频，不删除照片，不删除 TF 卡其他无关文件。
- 默认 MP4 分段时长为 `120` 秒；A3 控制接口可按兼容协议设置 `60` 秒分段。
- TF 卡测速只在手动读取状态时触发；录像中会跳过测速。

## 网页功能

网页默认监听 `80` 端口：

```text
http://<设备IP>/
```

当前网页支持：

- 查看当前时间、当前 IP、RTSP 地址、TCP-UART 地址
- 查看 TF 卡挂载状态、容量、剩余空间、预计可拍照数量、预计可录像时长
- 手动触发 TF 卡测速
- 查看 RTSP 客户端数和 AP 客户端数
- 配置 UART0/UART1 波特率
- 切换视频源：USB 热像仪或 MIPI 摄像头，保存后重启生效
- 配置 MIPI 分辨率档位
- USB 热像仪模式下即时切换白热、黑热、铁红
- 配置 AP 或 STA 网络参数
- STA 模式网页访问确认，避免自动回退 AP
- 保存配置、恢复默认配置、重启设备
- 浏览 TF 卡照片和录像
- 在线播放 MP4
- 请求拍照
- 批量删除照片或录像

网页时间显示采用“设备返回基准时间，浏览器本地走时，页面可见时低频校准”的方式，避免频繁请求设备。

## HTTP API

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/` | 网页首页 |
| GET | `/api/status` | 当前设备、网络、TF 卡和 RTSP 状态 |
| GET | `/api/status?run_speed_test=1` | 读取状态并手动触发 TF 卡测速 |
| GET | `/api/config` | 当前已保存配置 |
| POST | `/api/config` | 保存网页配置，通常需要重启生效 |
| GET | `/api/time?unix_ms=<ms>` | 使用 Unix 毫秒时间戳同步设备时间 |
| POST | `/api/thermal_effect` | 即时设置 USB 热像仪成像效果 |
| POST | `/api/sta_confirm` | 确认 STA 网页可访问，取消自动回退 |
| POST | `/api/factory_reset` | 恢复网页可配置参数默认值 |
| POST | `/api/reboot` | 重启设备 |
| GET | `/api/photos` | 照片列表 |
| GET | `/api/videos` | 录像列表 |
| POST | `/api/capture` | 请求拍照一次 |
| POST | `/api/delete` | 删除一个或多个媒体文件 |
| GET | `/api/a3/sdcard-param` | A3 兼容 SD 卡状态 |
| POST | `/api/a3/record-param` | A3 兼容录像/拍照控制 |
| GET | `/photo/<path>` | 访问 JPEG 原图 |
| GET | `/video/<path>` | 访问 MP4，支持 Range |
| HEAD | `/video/<path>` | 获取 MP4 头信息 |

接口补充：

- `/api/config` 和 `/api/thermal_effect` 使用 `application/x-www-form-urlencoded`。
- `/api/delete` 使用换行分隔的媒体相对路径列表。
- `/api/photos` 和 `/api/videos` 返回 `name`、`session`、`path`、`url`、`size_bytes`。
- `/api/a3/record-param` 使用 JSON，当前兼容 `channel=1` 和 `file_switch_interval=60`。
- 网页普通界面当前没有录像启停按钮；录像主要由 A3 兼容接口控制。

## 参数持久化

网页可配置参数保存在 NVS 中，并带版本迁移和合法性校验。

当前持久化参数包括：

- 视频源
- MIPI 分辨率档位
- UART0/UART1 波特率
- Wi-Fi AP 名称、密码、IP、网关、子网掩码
- Wi-Fi STA SSID、密码、DHCP/固定 IP、网关、子网掩码
- STA 待确认标记

默认值：

| 项目 | 默认值 |
| --- | --- |
| 视频源 | USB 热像仪 |
| AP SSID | `WT99P4C5` |
| AP 密码 | `12345678` |
| AP IP | `192.168.144.2` |
| STA SSID | `CEEWA` |
| STA 密码 | `52285509` |
| UART0/UART1 | `115200` |

从 AP 切换到 STA 后，设备会进入待确认状态。重启进入 STA 后必须在新地址网页点击确认；如果约 `180` 秒内未确认，设备会自动回退 AP 并重启。

## 启动顺序

当前启动路径按以下顺序执行：

1. 初始化 NVS。
2. 初始化并加载网页配置。
3. 初始化网络栈和默认事件循环。
4. 初始化 ESP-Hosted，等待 ESP32-C5 SDIO transport ready。
5. 默认进入 Z-1mini 桥接模式，创建 Wi-Fi AP 和以太网原始帧转发。
6. 等待 AP 就绪，短暂检查以太网链路；即使吊舱以太网未连接，本机网页、RTSP 和 UART 服务仍继续启动。
7. 初始化 TF 卡。
8. 初始化媒体存储；TF 卡不可用时记录失败，但网页、RTSP 和 UART 服务仍可运行。
9. 启动网页服务。
10. 启动 RTSP 服务。
11. 启动 TCP-UART0 服务。
12. 按保存配置启动 MIPI 摄像头或 USB 热像仪。

如果关闭 Z-1mini 桥接，可切换到普通 Wi-Fi AP/STA 分支或以太网分支。

## 目录结构

```text
main/
  hello_world_main.c
  CMakeLists.txt
  idf_component.yml
  User/
    include/
    src/
partitions.csv
sdkconfig
README.md
SYSTEM_IMPLEMENTATION.md
```

关键模块：

| 模块 | 职责 |
| --- | --- |
| `hello_world_main.c` | 系统启动编排 |
| `device_web_config` | 网页配置持久化和校验 |
| `z1mini_bridge` | Z-1mini AP 与以太网原始帧转发 |
| `wifi_connect` | 普通 Wi-Fi AP/STA 分支 |
| `eth_connect` | 普通以太网分支 |
| `camera` | MIPI 摄像头采集、编码和推流 |
| `usb_thermal_camera` | USB 热像仪采集、本地成像、编码和推流 |
| `rtsp_server` | RTSP/RTP 服务 |
| `media_storage` | 照片和录像后台存储 |
| `media_mp4_writer` | MP4 封装 |
| `tf_card` | TF 卡挂载、容量信息和测速 |
| `photo_web_server` | 网页和 HTTP API |
| `tcp_uart_server` | TCP-UART 透传 |

## 构建与烧录

在 ESP-IDF 环境中执行：

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

如果只修改应用代码，通常不需要重新设置 target。

## 当前限制与注意事项

- 当前默认启用 Z-1mini 桥接，普通以太网分支不会同时启动。
- AP 和 STA 不会同时启用。
- 网页配置保存后，多数参数需要重启才能完全生效。
- 网页普通界面目前不提供录像启停按钮；录像由 A3 兼容接口控制。
- 网页拍照按钮当前要求 RTSP 有播放客户端时才启用；底层拍照接口本身会临时保持采集以等待下一帧。
- USB 热像仪厂商私有调色命令默认关闭，当前采用本机 Y16 转换成像。
- `UART1` TCP 透传服务默认未在启动流程中启用。
- RTSP 客户端数、HTTP 连接和 TCP-UART 连接共享 socket 资源，当前 LWIP 最大 socket 数为 `20`。
