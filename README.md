# WT99P4C5-S1 视频与媒体存储工程

## 1. 项目概述

本工程运行在 `WT99P4C5-S1` 开发板上，围绕以下能力进行集成：

- OV5647 MIPI 摄像头采集
- 硬件 ISP 图像处理
- H.264 硬件编码
- RTSP 实时推流
- TF 卡照片与 MP4 存储
- 网页端设备状态查看、参数配置与媒体管理
- TCP-UART 透传

当前代码结构遵循以下约定：

- `main/hello_world_main.c` 只负责系统初始化与模块调用
- 具体功能模块统一放在 `main/User/src`
- 对应头文件统一放在 `main/User/include`

核心数据链路如下：

```text
OV5647 MIPI-CSI
  -> V4L2 采集
  -> 硬件 ISP 输出 YUV420
  -> H.264 硬件编码
  -> RTSP 推流
  -> 复用同一路 H.264 码流后台封装 MP4

YUV420 图像
  -> PPA 色彩转换
  -> JPEG 硬件编码
  -> TF 卡保存照片
```

### 1.1 代码设计思路

本工程在代码设计上重点遵循“主流程只编排、具体能力模块化、耗时任务后台化、网页只做控制与展示”这几个原则：

- `main/hello_world_main.c` 只负责系统初始化、网络选择、模块启动顺序和回调接线，不堆叠具体业务逻辑
- 可持久化的网页参数统一放到 `device_web_config` 中管理，避免网页层、网络层、摄像头层分别维护一份配置
- `media_storage` 统一负责照片与录像存储，外部模块只提交“拍照请求”或“编码帧”，不直接操作 TF 卡文件
- `photo_web_server` 只负责 HTTP 接口、网页页面和参数解析，真正的媒体保存、配置落盘、重启、时间同步分别交给对应模块处理
- 与 TF 卡、JPEG 编码、MP4 封装相关的耗时路径统一放到后台任务中，避免把摄像头采集和 RTSP 推流线程拖慢

### 1.2 关键模块协作思路

#### 1.2.1 配置管理

`device_web_config` 的职责是维护“网页可修改且需要持久化”的参数集合，例如波特率、视频分辨率、静态 IP 开关和静态 IP 三元组。

这样做的目的有两个：

- 所有配置的合法性校验集中在一个模块中，避免多处重复校验
- 运行期模块只读取配置结果，不关心 NVS 读写细节，降低模块耦合

#### 1.2.2 照片链路

照片链路采用“前台取帧，后台编码和写卡”的设计：

1. 网页点击拍照后，`media_storage_request_photo()` 只登记一次拍照请求
2. 摄像头线程在下一帧 `YUV420` 到达时复制所需图像数据
3. 后台照片任务完成 `YUV420 -> RGB565 -> JPEG -> TF 卡写入`

这样设计的核心原因是：

- 拍照不会在网页接口线程里直接阻塞
- 摄像头线程不直接做文件写入，减少对实时视频链路的影响
- 如果 TF 卡未挂载或空间不足，可以在进入耗时编码前先拒绝本次拍照

#### 1.2.3 录像链路

录像链路采用“复用现有 H.264 编码结果，后台封装 MP4”的设计，而不是再开一路独立编码。

当前思路是：

1. 摄像头线程继续以 RTSP 实时性优先
2. `media_storage` 只旁路拷贝已经硬件编码好的 `H.264` 帧
3. 后台录像任务在合适时机创建 MP4 分段并持续写入
4. 当旁路队列压力升高时，允许丢帧或等待下一帧 `IDR` 恢复，避免反向拖慢 RTSP

这样做可以把“录像保存”和“实时推流”解耦，确保 TF 卡变慢或 MP4 封装耗时增加时，优先保证前台视频链路可用。

#### 1.2.4 TF 卡空间策略

TF 卡策略区分“新操作启动前”和“录像进行中”两种场景：

- 拍照前先检查 TF 卡剩余空间，不足则直接拒绝，不做拍照
- 启动新录像前先检查剩余空间，不足则直接拒绝，不创建新录像
- 录像已经开始并且后续切新分段时空间不足，允许删除最小序号目录中的旧 `MP4` 后继续录像
- 自动覆盖只删除旧视频，不删除照片，也不删除 TF 卡其他功能相关内容

这里刻意不对照片做自动覆盖，原因是录像文件体积大、可连续产生、最容易占满空间；而照片通常需要保留，不应被后台静默删除。

#### 1.2.5 网页状态与时间刷新

网页状态读取采用“状态汇总接口 + 前端低频刷新”的方式：

- `/api/status` 统一返回设备时间、网络信息、TF 卡状态、容量、预估录像时长、预估拍照数量和测速结果
- TF 卡测速只在手动读取状态时触发，自动刷新只读取缓存状态
- 网页当前时间使用设备返回的 `current_unix_ms` 作为基准，在浏览器端本地走时
- 页面可见时再以低频方式重新读取设备状态做校准，页面隐藏时停止自动状态轮询

这样做的目的是在保证网页时间和设备状态看起来“实时”的同时，避免每秒请求设备接口，降低 HTTP 轮询对整体程序的影响。

## 2. 当前功能状态

### 2.1 网络接入

- 默认使用 `ESP-Hosted + ESP32-C5 SDIO` 方式进行 Wi-Fi STA 联网
- 代码保留以太网接入分支，可通过 `main/hello_world_main.c` 中的宏切换
- Wi-Fi 联网后会尝试通过 SNTP 校时
- 如果 SNTP 未及时完成，可通过网页接口从电脑同步时间
- 网页保存的静态 IP 配置会覆盖 `wifi_connect.h` 中的默认 Profile 静态 IP 参数

### 2.2 实时视频

- 支持 OV5647 MIPI-CSI 采集
- 支持硬件 ISP 输出 `YUV420`
- 支持 H.264 硬件编码
- 支持 RTSP over TCP 推流
- 当前 RTSP 最大客户端数为 `2`

访问地址：

```text
rtsp://<设备IP>:8554/stream
```

### 2.3 TF 卡媒体存储

- TF 卡挂载点为 `/sdcard`
- 照片和视频写入同一次上电会话目录
- 网页拍照请求到来后，会在下一帧图像到达时保存一张 JPEG
- 录像复用已有 H.264 编码帧，由后台通过 `esp_muxer` 封装为 MP4
- MP4 默认按 `120` 秒切段
- 录像开始/停止跟随 RTSP 播放状态变化
- 拍照和“启动新录像”前都会先检查 TF 卡剩余空间，空间不足时直接拒绝执行
- 正在录像过程中如果 TF 卡写满，会从最小序号上电目录开始删除旧 `MP4` 后继续录像
- 满卡覆盖策略只删除旧视频，不删除照片，也不会影响 TF 卡原有浏览与删除功能
- 可读取 TF 卡总容量、剩余容量、已用容量、预估剩余录像时间、预估可拍照数量
- TF 卡测速仅在网页手动“读取状态”时触发；录像中会跳过测速并返回“录像中，已跳过测速”

### 2.4 网页端

网页默认监听 `80` 端口，访问地址：

```text
http://<设备IP>/
```

当前网页支持：

- 查看设备当前时间、IP、网关、掩码、RTSP 地址、TF 挂载状态、RTSP 客户端数
- 查看 TF 卡总容量、剩余容量、已用容量、预估录像时长、预估拍照数量、读写测速结果
- 配置 UART0/UART1 波特率
- 配置视频分辨率档位
- 配置 Wi-Fi 静态 IP、网关、掩码
- 保存配置并提示重启生效
- 恢复网页可配置参数的默认值
- 重启设备
- 浏览 TF 卡中的照片和视频
- 在线播放 MP4
- 在照片栏手动拍照
- 照片栏和视频栏分别进入选择模式
- 照片栏和视频栏分别独立全选
- 批量删除照片或视频
- 当前时间采用“前端本地走时 + 页面可见时低频读取设备状态校准”的方式显示，避免频繁请求设备

### 2.5 TCP-UART 与网口 TCP

- `UART0` 对应 TCP 端口 `8880`
- `UART1` 对应 TCP 端口 `8881`
- 当前 `app_main()` 默认启动 `UART0` 透传服务
- `UART1` 服务代码已接入，但默认未在 `app_main()` 中启用
- 以太网 TCP 服务端口为 `9000`
- 当前 `ETH_ENABLE_TCP_SERVER` 默认关闭

## 3. 网页配置模块

网页设备配置由 `device_web_config` 模块统一管理：

- 头文件：`main/User/include/device_web_config.h`
- 源文件：`main/User/src/device_web_config.c`

该模块负责把网页可修改参数持久化保存到 NVS，当前包含以下配置项：

- `UART0` 波特率
- `UART1` 波特率
- 视频分辨率档位
- Wi-Fi 静态 IP 开关
- Wi-Fi 静态 IP
- Wi-Fi 网关
- Wi-Fi 子网掩码

当前网页预置的波特率选项为：

```text
9600
19200
38400
57600
115200
230400
460800
921600
1500000
```

后端允许的波特率范围为：

```text
1200 ~ 2000000
```

当前网页提供的分辨率档位为：

```text
1 -> 1280 x 960
2 -> 1920 x 1080
3 -> 800 x 800
4 -> 800 x 640
```

说明：

- 实际是否支持某个分辨率，取决于当前固件编译时启用的摄像头 profile
- 若保存了当前固件未启用的分辨率，后端会拒绝该请求
- 配置保存后需要重启设备生效
- 恢复默认配置只重置网页可配置参数
- 恢复默认配置不会删除 TF 卡中的照片和视频

## 4. 默认端口与访问方式

| 功能 | 端口 | 说明 |
| --- | --- | --- |
| HTTP 网页 | 80 | 媒体浏览、设备状态、参数配置 |
| RTSP | 8554 | `rtsp://<设备IP>:8554/stream` |
| TCP-UART0 | 8880 | 默认启用 |
| TCP-UART1 | 8881 | 代码已接入，默认未启用 |
| 以太网 TCP | 9000 | 仅以太网模式且显式开启时启用 |

## 5. 网页 API

网页模块位于：

- `main/User/src/photo_web_server.c`
- `main/User/include/photo_web_server.h`

当前已实现接口如下：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/` | 网页首页 |
| GET | `/api/photos` | 照片列表 JSON |
| GET | `/api/videos` | 视频列表 JSON |
| GET | `/api/status` | 当前设备与 TF 卡状态，支持 `run_speed_test=1` 手动测速 |
| GET | `/api/config` | 当前已保存配置 |
| POST | `/api/config` | 保存设备配置，返回 `reboot_required=true` |
| GET | `/api/time` | 通过 `unix_ms` 参数同步设备时间 |
| POST | `/api/capture` | 请求拍照一次 |
| POST | `/api/delete` | 删除一个或多个媒体文件 |
| POST | `/api/factory_reset` | 恢复网页默认配置 |
| POST | `/api/reboot` | 重启设备 |
| GET | `/photo/<path>` | 访问 JPEG 原图 |
| GET | `/video/<path>` | 访问 MP4 文件，支持 Range |
| HEAD | `/video/<path>` | 获取 MP4 头信息 |

补充说明：

- `/api/config` 使用 `application/x-www-form-urlencoded` 提交
- `/api/time` 需要传入 `unix_ms=<毫秒时间戳>`
- `/api/status?run_speed_test=1` 会触发一次 TF 卡测速；不带参数时只返回缓存状态
- `/api/delete` 的请求体为换行分隔的相对路径列表
- `/api/photos` 和 `/api/videos` 返回的条目包含 `name`、`session`、`path`、`url`、`size_bytes`
- `/api/capture` 只有在 TF 卡已挂载、当前存在 RTSP 播放客户端且剩余空间允许拍照时才允许执行

`/api/config` 相关参数如下：

```text
video_profile
uart0_baud_rate
uart1_baud_rate
wifi_use_static_ip
wifi_static_ip
wifi_static_gw
wifi_static_mask
```

## 6. 启动顺序

当前 `app_main()` 采用“公共初始化 + 网络分支 + 业务模块启动”的顺序：

1. 初始化 NVS
2. 初始化 `device_web_config`
3. 初始化 `esp_netif` 与默认事件循环
4. 进入网络分支

当前默认分支为 Wi-Fi，`USE_ETHERNET=0`：

1. 初始化 ESP-Hosted
2. 等待 C5 SDIO transport ready
3. 初始化 Wi-Fi 并等待获取 IP
4. 尝试等待 SNTP 校时

如果切换为以太网分支：

1. 初始化以太网并等待链路获取 IP
2. 按需启动以太网 TCP 服务

网络就绪后继续执行：

1. 初始化 TF 卡
2. 初始化 `media_storage`
3. 启动网页服务
4. 启动 RTSP 服务
5. 启动 `UART0` TCP 透传服务
6. 初始化摄像头模块

说明：

- `camera_init()` 会完成摄像头相关初始化
- 实际采集开始与停止由 RTSP 播放状态回调控制

## 7. 目录结构与关键文件

```text
main/
  hello_world_main.c
  User/
    include/
    src/
```

关键文件说明：

- `main/hello_world_main.c`
  负责系统初始化与各模块启动调用
- `main/User/src/device_web_config.c`
  网页设备配置持久化
- `main/User/src/wifi_connect.c`
  Wi-Fi 连接、静态 IP 覆盖与 SNTP 校时
- `main/User/src/camera.c`
  摄像头采集、ISP、H.264 编码、RTSP 推送
- `main/User/src/rtsp_server.c`
  RTSP/RTP 服务
- `main/User/src/media_storage.c`
  照片与录像后台存储
- `main/User/src/media_mp4_writer.c`
  基于 `esp_muxer` 的 MP4 封装
- `main/User/src/photo_web_server.c`
  网页与 HTTP API
- `main/User/src/tcp_uart_server.c`
  TCP-UART 透传
- `main/User/src/eth_tcp_server.c`
  开发板网口 TCP 服务
- `main/User/src/tf_card.c`
  TF 卡驱动与挂载

## 8. TF 卡目录结构

照片与视频写入同一次上电会话目录，典型结构如下：

```text
/sdcard/<session>/photo/<timestamp>.jpeg
/sdcard/<session>/video/<timestamp>_<segment>.mp4
```

示例：

```text
/sdcard/001_20260421/photo/2026-04-21T16-20-15-123.jpeg
/sdcard/001_20260421/video/2026-04-21T16-20-15-123_0001.mp4
```

说明：

- `<session>` 为本次上电会话目录
- 只有在本次上电首次真正写入媒体文件时才会创建会话目录
- 照片与视频共用同一会话目录，不同子目录分别存放

## 9. 当前行为说明

### 9.1 录像触发

当前录像不是网页独立启停控制，而是跟随 RTSP 播放状态：

- 第一个 RTSP 客户端进入播放后，请求开始录像
- 最后一个 RTSP 客户端断开后，请求停止录像

### 9.2 拍照触发

当前拍照由网页按钮触发：

- 网页“照片”栏提供“立即拍照”按钮
- 点击后会在下一帧图像到达时保存照片
- 必须满足 TF 卡已挂载且当前 RTSP 正在播放

### 9.3 参数生效时机

- 波特率、分辨率、静态 IP 保存后重启生效
- 网页恢复默认配置同样需要重启后完全生效
- 时间同步成功后，后续新保存文件会使用真实时间命名

### 9.4 TF 卡空间与覆盖策略

- 拍照和“启动新录像”前都会先检查 TF 卡可用空间
- 如果卡已满后再发起拍照或启动新录像，请求会直接失败，并通过网页返回“TF 卡剩余空间不足”
- 只有“当前录像已经真正开始写分段”后，后续因空间不足切新分段时，才允许删除旧视频继续录像
- 自动覆盖时按最小上电目录序号优先，只删除旧 `MP4`
- 手动读取状态时如果设备正在录像，会跳过 TF 卡测速，并返回“录像中，已跳过测速”

### 9.5 网页时间与状态刷新策略

- 网页初次加载时会先同步一次设备时间
- 设备状态接口返回 `current_unix_ms` 后，网页本地每秒递增显示当前时间
- 页面可见时才进行低频状态刷新校准，避免频繁请求设备
- 页面切到后台后停止自动状态轮询，减少对 socket 和 CPU 的占用

## 10. 当前限制与注意事项

- 录像与 RTSP 会话绑定，当前不是常驻独立录像模式
- 网页端目前不负责手动启动或停止录像
- `UART1` TCP 透传服务默认未在 `app_main()` 中启用
- 以太网模式与网口 TCP 服务默认关闭，需要手动改宏
- 网页显示了多种分辨率选项，但最终是否可保存，仍由当前固件编译配置决定
- 当前 `sdkconfig` 中 `CONFIG_LWIP_MAX_SOCKETS=10`，同时开启较多 HTTP、RTSP、TCP 连接时，可能出现 socket 资源紧张

## 11. 使用示例

### 11.1 打开网页

```text
http://<设备IP>/
```

### 11.2 播放 RTSP

```text
rtsp://<设备IP>:8554/stream
```

示例：

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -rtsp_transport tcp "rtsp://192.168.0.200:8554/stream"
```

### 11.3 获取设备状态

```text
http://<设备IP>/api/status
```

手动触发一次 TF 卡测速：

```text
http://<设备IP>/api/status?run_speed_test=1
```

### 11.4 获取当前配置

```text
http://<设备IP>/api/config
```

### 11.5 从电脑同步时间

```text
http://<设备IP>/api/time?unix_ms=<当前Unix毫秒时间戳>
```
