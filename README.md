# WT99P4C5-S1 视频与照片存储项目

## 1. 项目概述

本工程运行在 `WT99P4C5-S1` 开发板上，当前已经实现以下功能：

- WiFi 联网
- RTSP 实时视频推流
- TF 卡挂载与容量检测
- 开始 RTSP 推流后自动抓拍并保存 JPEG 照片
- 通过网页查看 TF 卡中的已保存照片

当前照片相关功能拆分为两个模块：

- `media_storage`：负责照片保存
- `photo_web_server`：负责网页浏览照片

`hello_world_main.c` 只负责系统初始化和模块调用，具体业务逻辑放在 `main/User/` 目录下。

## 2. 当前已实现功能

### 2.1 RTSP 视频链路

- 摄像头输出走当前已有的 `YUV420 -> H.264` 硬编码链路
- RTSP 服务默认端口为 `8554`
- 访问地址：

```text
rtsp://<设备IP>:8554/stream
```

### 2.2 TF 卡照片保存

- 当 RTSP 客户端开始播放时，`camera` 模块会调用 `media_storage_request_auto_photo()`
- 摄像头采集到下一帧 `YUV420` 图像后，`camera` 模块会调用 `media_storage_process_camera_frame()`
- `media_storage` 只在需要抓拍时拷贝一帧到独立缓冲区，避免阻塞实时推流主链路
- 后台任务使用 `PPA` 硬件完成 `YUV420 -> RGB565`
- 然后调用官方 `JPEG` 硬件编码 API 将 `RGB565` 编码为 `JPEG`
- 编码完成后写入 TF 卡

### 2.3 网页浏览照片

- 启动后会开启 HTTP 服务，默认端口 `80`
- 浏览器访问：

```text
http://<设备IP>/
```

- 已实现接口：

```text
GET /                 相册首页
GET /api/photos       返回照片列表 JSON
GET /photo/<path>     返回 JPEG 原图
```

网页会扫描 TF 卡中所有符合规则的照片目录，并以卡片形式展示缩略图，点击后可查看原图。

## 3. 照片保存是怎么实现的

### 3.1 触发时机

当前版本不需要额外拍照指令。

流程如下：

1. RTSP 客户端连上后，`on_rtsp_playing(true)` 被调用
2. `camera` 模块设置自动拍照请求
3. 摄像头采集线程在处理视频帧时，把当前帧同时送往：
   - H.264 编码与 RTSP 推流
   - `media_storage` 抓拍旁路
4. `media_storage` 后台任务完成颜色转换、JPEG 编码和写卡

这样实现后，RTSP 主链路仍然保持原来的 `YUV420/H.264` 流程不变，抓拍走独立后台流程，对实时视频影响较小。

### 3.2 数据处理链路

照片保存的数据流如下：

```text
摄像头当前 YUV420 帧
    -> 拷贝到抓拍专用缓冲区
    -> PPA 硬件转换为 RGB565
    -> 官方 JPEG 硬件编码器压缩
    -> 保存到 /sdcard
```

实现位置：

- `main/User/camera.c`
- `main/User/media_storage.c`

### 3.3 目录与命名规则

照片目录规则如下：

```text
/sdcard/***_YYYYMMDD/photo/YYYY-MM-DDTHH-MM-SS-mmm.jpeg
```

例如：

```text
/sdcard/001_19800106/photo/1980-01-06T07-59-42-000.jpeg
```

规则说明：

- `***` 为“历史上电且实际发生过照片保存”的次序号
- 某次上电如果没有真正保存照片，不占用次序号
- 同一次上电内，只在第一张照片保存成功时创建一次目录
- 后续照片继续写入同一个 `***_YYYYMMDD/photo/` 目录
- 如果系统时间还未同步，则日期回退为 `19800106`
- 一旦本次上电第一次照片确定了目录日期，后续照片继续沿用该目录

### 3.4 历史次序号的保存方式

- 使用 NVS 命名空间 `media_storage`
- 键名为 `boot_seq`
- 只有在本次上电第一次真正准备落盘时，才递增并写回 NVS

因此满足“中间通电但未拍照不计入”的要求。

### 3.5 时间来源

- 已同步时间时：使用系统本地时间生成目录日期和文件名时间戳
- 未同步时间时：回退到 `1980-01-06` + 上电运行时间

当前代码中时区设置为：

```text
CST-8
```

实际时间同步由联网后的系统时间来源决定，你当前方案中可通过 WiFi 从电脑侧同步时间。

## 4. 网页模块是怎么实现的

网页模块基于 `esp_http_server` 实现，核心文件为：

- `main/User/photo_web_server.c`
- `main/User/photo_web_server.h`

### 4.1 页面接口

#### `GET /`

- 返回内嵌 HTML 页面
- 页面使用浏览器 `fetch('/api/photos')` 拉取照片列表
- 前端按照片路径倒序显示，最新照片通常排在前面

#### `GET /api/photos`

- 扫描 TF 卡根目录下所有会话目录
- 匹配形如：

```text
/sdcard/<session>/photo/*.jpg
/sdcard/<session>/photo/*.jpeg
```

- 返回 JSON 字段包括：
  - `name`
  - `session`
  - `path`
  - `url`
  - `size_bytes`

#### `GET /photo/<path>`

- 读取 TF 卡上的 JPEG 文件并分块发送给浏览器
- 返回类型为 `image/jpeg`

### 4.2 安全处理

为避免非法路径访问，网页模块对图片路径做了限制：

- 禁止 `..`
- 禁止反斜杠 `\`
- 只允许数字、大小写字母、`/`、`_`、`-`、`.`
- 必须包含 `/photo/`
- 只允许 `.jpg` 或 `.jpeg`

## 5. 代码结构

### 5.1 关键文件

- `main/hello_world_main.c`
  - 负责初始化和启动各模块
- `main/User/camera.c`
  - 负责摄像头采集、H.264 编码、RTSP 推流，以及抓拍触发
- `main/User/media_storage.c`
  - 负责照片缓冲、PPA 转换、JPEG 硬编码、目录创建、文件写入
- `main/User/photo_web_server.c`
  - 负责 HTTP 服务、照片列表接口、原图访问接口、网页页面
- `main/User/tf_card.c`
  - 负责 TF 卡挂载与状态查询

### 5.2 主流程启动顺序

当前 `app_main()` 中的相关顺序为：

1. 网络初始化
2. WiFi 获取 IP
3. TF 卡初始化
4. `media_storage_init()`
5. `photo_web_server_start()`
6. `rtsp_server_start()`
7. `camera_init()`

## 6. 构建相关配置调整

为支持当前功能，工程已做以下配置调整：

- `main/CMakeLists.txt`
  - 新增 `User/media_storage.c`
  - 新增 `User/photo_web_server.c`
  - 增加依赖：
    - `esp_http_server`
    - `esp_driver_jpeg`
    - `esp_driver_ppa`
    - `fatfs`
    - `sdmmc`

- `sdkconfig`
  - 提高 `CONFIG_VFS_MAX_COUNT`
  - 开启 FATFS 长文件名支持

- `partitions.csv`
  - 调整应用分区大小，避免固件超出默认 `factory` 分区

## 7. 当前状态与后续计划

当前已经完成：

- 照片自动保存
- 照片网页浏览

后续可继续扩展：

- 手动拍照接口
- SD 卡录像保存
- 网页端视频在线播放或文件管理
- 通过网页删除、下载、筛选图片

## 8. 使用说明

### 8.1 查看视频流

```text
rtsp://<设备IP>:8554/stream
```

例如：

```text
ffplay -fflags nobuffer -flags low_delay -framedrop -rtsp_transport tcp "rtsp://192.168.137.200:8554/stream"
```

### 8.2 查看照片网页

```text
http://<设备IP>/
```

### 8.3 查看照片列表接口

```text
http://<设备IP>/api/photos
```

