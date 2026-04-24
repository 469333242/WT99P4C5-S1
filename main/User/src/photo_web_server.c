/**
 * @file photo_web_server.c
 * @brief SD 卡媒体网页浏览模块实现
 *
 * 模块职责：
 *   - 通过 HTTP 服务提供照片和视频浏览网页
 *   - 提供网页设备状态读取、波特率/分辨率/静态 IP 配置、恢复默认配置和重启接口
 *   - 提供网页拍照接口，请求在推流过程中抓拍一张照片
 *   - 扫描 TF 卡中各次上电目录下的 photo 和 video 子目录
 *   - 将 JPEG 照片和 MP4 录像提供给浏览器访问
 *   - 网页加载时接收浏览器/电脑时间，用于校正后续媒体文件命名
 *
 * 目录约定：
 *   /sdcard/002_19800106/photo/1980-01-06T00-01-10-134.jpeg
 *   /sdcard/002_19800106/video/1980-01-06T00-01-10-134_0001.mp4
 */

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"

#include "device_web_config.h"
#include "media_storage.h"
#include "photo_web_server.h"
#include "rtsp_server.h"
#include "tf_card.h"

static const char *TAG = "photo_web";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define PHOTO_WEB_MAX_PATH_LEN 256
#define PHOTO_WEB_FILE_CHUNK_SIZE 2048
#define PHOTO_WEB_RANGE_HEADER_LEN 96
#define PHOTO_WEB_HTTP_HEADER_LEN 512
#define PHOTO_WEB_TIME_QUERY_LEN 96
#define PHOTO_WEB_TIME_VALUE_LEN 24
#define PHOTO_WEB_DELETE_BODY_MAX_LEN 16384
#define PHOTO_WEB_CONFIG_BODY_MAX_LEN 512
#define PHOTO_WEB_STATUS_TEXT_LEN 64
#define PHOTO_WEB_VALID_UNIX_SEC 1704067200LL
#define PHOTO_WEB_SERVER_STACK_SIZE (8 * 1024)

typedef struct
{
    httpd_handle_t server;
} photo_web_ctx_t;

typedef struct
{
    bool partial;
    uint64_t start;
    uint64_t end;
    uint64_t length;
} photo_web_range_t;

typedef bool (*photo_web_suffix_check_t)(const char *name);

static photo_web_ctx_t s_photo_web;

static esp_err_t photo_web_read_request_body(httpd_req_t *req, char **body_out);

/* ------------------------------------------------------------------ */
/* 网页内容                                                            */
/* ------------------------------------------------------------------ */
static const char *s_photo_index_html_v6[] = {
    "<!doctype html>\n",
    "<html lang='zh-CN'>\n",
    "<head>\n",
    "  <meta charset='utf-8'>\n",
    "  <meta name='viewport' content='width=device-width,initial-scale=1'>\n",
    "  <title>SD 卡媒体浏览</title>\n",
    "  <style>\n",
    "    :root {\n",
    "      --bg: #f4efe7;\n",
    "      --panel: #fffaf2;\n",
    "      --card: #ffffff;\n",
    "      --ink: #1b1b1b;\n",
    "      --muted: #6e6457;\n",
    "      --line: #e7d8c8;\n",
    "      --accent: #c65a1e;\n",
    "      --danger: #b3312d;\n",
    "    }\n",
    "    * { box-sizing: border-box; }\n",
    "    body {\n",
    "      margin: 0;\n",
    "      font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;\n",
    "      color: var(--ink);\n",
    "      background: radial-gradient(circle at top, #fff8ef 0, #f4efe7 42%, #ece2d6 100%);\n",
    "    }\n",
    "    a { color: inherit; }\n",
    "    .shell {\n",
    "      max-width: 1260px;\n",
    "      margin: 0 auto;\n",
    "      padding: 24px 16px 40px;\n",
    "    }\n",
    "    .hero {\n",
    "      display: flex;\n",
    "      flex-wrap: wrap;\n",
    "      align-items: flex-end;\n",
    "      justify-content: space-between;\n",
    "      gap: 16px;\n",
    "      margin-bottom: 18px;\n",
    "    }\n",
    "    .title {\n",
    "      margin: 0;\n",
    "      font-size: clamp(28px, 5vw, 44px);\n",
    "      letter-spacing: .04em;\n",
    "    }\n",
    "    .desc {\n",
    "      max-width: 780px;\n",
    "      margin: 8px 0 0;\n",
    "      color: var(--muted);\n",
    "      font-size: 14px;\n",
    "      line-height: 1.7;\n",
    "    }\n",
    "    .toolbar,\n",
    "    .sectionTools,\n",
    "    .sectionTitleWrap {\n",
    "      display: flex;\n",
    "      flex-wrap: wrap;\n",
    "      align-items: center;\n",
    "      gap: 10px;\n",
    "    }\n",
    "    .badge,\n",
    "    .count,\n",
    "    .miniBadge {\n",
    "      padding: 10px 14px;\n",
    "      border: 1px solid var(--line);\n",
    "      border-radius: 999px;\n",
    "      background: rgba(255, 250, 242, .92);\n",
    "      color: var(--muted);\n",
    "      font-size: 13px;\n",
    "    }\n",
    "    .badge.error {\n",
    "      color: #8f201b;\n",
    "      border-color: #e7b0ad;\n",
    "      background: #fff1f0;\n",
    "    }\n",
    "    button {\n",
    "      padding: 11px 16px;\n",
    "      border: 0;\n",
    "      border-radius: 999px;\n",
    "      font-size: 14px;\n",
    "      font-weight: 700;\n",
    "      cursor: pointer;\n",
    "      transition: transform .18s ease, box-shadow .18s ease, opacity .18s ease;\n",
    "    }\n",
    "    button:hover:not(:disabled) { transform: translateY(-1px); }\n",
    "    button:disabled {\n",
    "      opacity: .55;\n",
    "      cursor: default;\n",
    "      transform: none;\n",
    "      box-shadow: none;\n",
    "    }\n",
    "    .primaryBtn {\n",
    "      color: #fff;\n",
    "      background: linear-gradient(135deg, #db6d2e, #b94a14);\n",
    "      box-shadow: 0 14px 30px rgba(185, 74, 20, .18);\n",
    "    }\n",
    "    .ghostBtn,\n",
    "    .foldBtn {\n",
    "      color: var(--ink);\n",
    "      border: 1px solid var(--line);\n",
    "      background: #fff;\n",
    "      box-shadow: none;\n",
    "    }\n",
    "    .dangerBtn,\n",
    "    .miniDanger {\n",
    "      color: #fff;\n",
    "      background: linear-gradient(135deg, #cb5141, #a92a24);\n",
    "      box-shadow: 0 14px 26px rgba(169, 42, 36, .16);\n",
    "    }\n",
    "    .foldBtn {\n",
    "      padding: 8px 14px;\n",
    "      font-size: 13px;\n",
    "    }\n",
    "    .miniDanger {\n",
    "      padding: 7px 12px;\n",
    "      font-size: 12px;\n",
    "    }\n",
    "    .settingsGrid {\n",
    "      display: grid;\n",
    "      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));\n",
    "      gap: 18px;\n",
    "      margin-bottom: 18px;\n",
    "    }\n",
    "    .settingCard {\n",
    "      padding: 18px;\n",
    "      border: 1px solid rgba(231, 216, 200, .92);\n",
    "      border-radius: 24px;\n",
    "      background: rgba(255, 250, 242, .88);\n",
    "      backdrop-filter: blur(8px);\n",
    "      box-shadow: 0 18px 42px rgba(74, 39, 16, .08);\n",
    "    }\n",
    "    .field {\n",
    "      display: grid;\n",
    "      gap: 6px;\n",
    "      margin-bottom: 12px;\n",
    "    }\n",
    "    .field label {\n",
    "      color: var(--muted);\n",
    "      font-size: 13px;\n",
    "      font-weight: 700;\n",
    "    }\n",
    "    .field input,\n",
    "    .field select {\n",
    "      width: 100%;\n",
    "      padding: 11px 12px;\n",
    "      border: 1px solid var(--line);\n",
    "      border-radius: 14px;\n",
    "      background: #fff;\n",
    "      color: var(--ink);\n",
    "      font-size: 14px;\n",
    "    }\n",
    "    .field input:disabled,\n",
    "    .field select:disabled {\n",
    "      background: #efe7dd;\n",
    "      color: #8f7f70;\n",
    "    }\n",
    "    .fieldRow {\n",
    "      display: grid;\n",
    "      grid-template-columns: repeat(2, minmax(0, 1fr));\n",
    "      gap: 12px;\n",
    "    }\n",
    "    .checkboxLine {\n",
    "      display: flex;\n",
    "      align-items: center;\n",
    "      gap: 10px;\n",
    "      margin: 8px 0 12px;\n",
    "      color: var(--ink);\n",
    "      font-size: 14px;\n",
    "      font-weight: 700;\n",
    "    }\n",
    "    .checkboxLine input {\n",
    "      width: 16px;\n",
    "      height: 16px;\n",
    "      margin: 0;\n",
    "      accent-color: #c65a1e;\n",
    "    }\n",
    "    .settingHint {\n",
    "      margin-top: 4px;\n",
    "      color: var(--muted);\n",
    "      font-size: 12px;\n",
    "      line-height: 1.7;\n",
    "    }\n",
    "    .settingActions {\n",
    "      display: flex;\n",
    "      flex-wrap: wrap;\n",
    "      gap: 10px;\n",
    "      margin-top: 14px;\n",
    "    }\n",
    "    .statusList {\n",
    "      display: grid;\n",
    "      gap: 10px;\n",
    "    }\n",
    "    .statusRow {\n",
    "      display: flex;\n",
    "      align-items: flex-start;\n",
    "      justify-content: space-between;\n",
    "      gap: 12px;\n",
    "      padding: 10px 12px;\n",
    "      border: 1px solid var(--line);\n",
    "      border-radius: 16px;\n",
    "      background: rgba(255, 255, 255, .8);\n",
    "    }\n",
    "    .statusKey {\n",
    "      color: var(--muted);\n",
    "      font-size: 13px;\n",
    "      font-weight: 700;\n",
    "      white-space: nowrap;\n",
    "    }\n",
    "    .statusValue {\n",
    "      font-size: 13px;\n",
    "      font-weight: 700;\n",
    "      text-align: right;\n",
    "      word-break: break-all;\n",
    "    }\n",
    "    .media {\n",
    "      display: grid;\n",
    "      gap: 18px;\n",
    "    }\n",
    "    .panel {\n",
    "      padding: 18px;\n",
    "      border: 1px solid rgba(231, 216, 200, .92);\n",
    "      border-radius: 24px;\n",
    "      background: rgba(255, 250, 242, .86);\n",
    "      backdrop-filter: blur(8px);\n",
    "      box-shadow: 0 18px 42px rgba(74, 39, 16, .08);\n",
    "    }\n",
    "    .sectionHead {\n",
    "      display: flex;\n",
    "      flex-wrap: wrap;\n",
    "      align-items: center;\n",
    "      justify-content: space-between;\n",
    "      gap: 14px;\n",
    "      margin: 0 0 14px;\n",
    "    }\n",
    "    .sectionBody.hidden { display: none; }\n",
    "    .grid {\n",
    "      display: grid;\n",
    "      grid-template-columns: repeat(auto-fill, minmax(230px, 1fr));\n",
    "      gap: 18px;\n",
    "    }\n",
    "    .card {\n",
    "      position: relative;\n",
    "      display: flex;\n",
    "      flex-direction: column;\n",
    "      min-height: 310px;\n",
    "      overflow: hidden;\n",
    "      border: 1px solid #eadccc;\n",
    "      border-radius: 20px;\n",
    "      background: var(--card);\n",
    "      box-shadow: 0 12px 24px rgba(74, 39, 16, .06);\n",
    "    }\n",
    "    .card.selected {\n",
    "      border-color: #d56a2f;\n",
    "      box-shadow: 0 0 0 2px rgba(213, 106, 47, .16), 0 16px 28px rgba(74, 39, 16, .08);\n",
    "    }\n",
    "    .cardTop {\n",
    "      position: absolute;\n",
    "      top: 12px;\n",
    "      right: 12px;\n",
    "      z-index: 2;\n",
    "      display: flex;\n",
    "      align-items: center;\n",
    "      pointer-events: none;\n",
    "    }\n",
    "    .pick {\n",
    "      display: inline-flex;\n",
    "      align-items: center;\n",
    "      justify-content: center;\n",
    "      width: 34px;\n",
    "      height: 34px;\n",
    "      border: 1px solid rgba(231, 216, 200, .95);\n",
    "      border-radius: 50%;\n",
    "      background: rgba(255, 255, 255, .94);\n",
    "      pointer-events: auto;\n",
    "      box-shadow: 0 10px 20px rgba(74, 39, 16, .10);\n",
    "    }\n",
    "    .pick input {\n",
    "      width: 16px;\n",
    "      height: 16px;\n",
    "      margin: 0;\n",
    "      accent-color: #c65a1e;\n",
    "    }\n",
    "    .thumb {\n",
    "      display: block;\n",
    "      aspect-ratio: 4 / 3;\n",
    "      overflow: hidden;\n",
    "      background: linear-gradient(135deg, #f9dcc0, #f3b183);\n",
    "    }\n",
    "    .thumb img,\n",
    "    .thumb video {\n",
    "      display: block;\n",
    "      width: 100%;\n",
    "      height: 100%;\n",
    "      object-fit: cover;\n",
    "      background: #14110e;\n",
    "      transition: transform .25s ease;\n",
    "    }\n",
    "    .card:hover .thumb img,\n",
    "    .card:hover .thumb video {\n",
    "      transform: scale(1.02);\n",
    "    }\n",
    "    .videoGrid .thumb {\n",
    "      background: linear-gradient(135deg, #2d2720, #7f4b26);\n",
    "    }\n",
    "    .meta {\n",
    "      display: grid;\n",
    "      gap: 8px;\n",
    "      padding: 14px 14px 16px;\n",
    "    }\n",
    "    .name {\n",
    "      font-size: 13px;\n",
    "      font-weight: 700;\n",
    "      line-height: 1.6;\n",
    "      word-break: break-all;\n",
    "    }\n",
    "    .sub {\n",
    "      color: var(--muted);\n",
    "      font-size: 12px;\n",
    "      line-height: 1.6;\n",
    "      word-break: break-all;\n",
    "    }\n",
    "    .path {\n",
    "      color: #8d7e6f;\n",
    "      font-size: 12px;\n",
    "      line-height: 1.6;\n",
    "      word-break: break-all;\n",
    "    }\n",
    "    .metaActions {\n",
    "      display: flex;\n",
    "      align-items: center;\n",
    "      justify-content: space-between;\n",
    "      gap: 12px;\n",
    "    }\n",
    "    .open {\n",
    "      color: var(--accent);\n",
    "      font-size: 12px;\n",
    "      font-weight: 700;\n",
    "      text-decoration: none;\n",
    "    }\n",
    "    .empty {\n",
    "      grid-column: 1 / -1;\n",
    "      padding: 42px 18px;\n",
    "      border: 1px dashed var(--line);\n",
    "      border-radius: 18px;\n",
    "      background: rgba(255, 255, 255, .65);\n",
    "      color: var(--muted);\n",
    "      text-align: center;\n",
    "    }\n",
    "    @media (max-width: 760px) {\n",
    "      .shell { padding: 18px 12px 32px; }\n",
    "      .panel { padding: 14px; }\n",
    "      .grid {\n",
    "        grid-template-columns: repeat(2, minmax(0, 1fr));\n",
    "        gap: 12px;\n",
    "      }\n",
    "      .card { min-height: 0; }\n",
    "      .sectionTools { width: 100%; }\n",
    "    }\n",
    "    @media (max-width: 480px) {\n",
    "      .fieldRow { grid-template-columns: 1fr; }\n",
    "      .grid { grid-template-columns: 1fr; }\n",
    "      .toolbar,\n",
    "      .sectionTools,\n",
    "      .settingActions { align-items: stretch; }\n",
    "      .toolbar button,\n",
    "      .sectionTools button,\n",
    "      .settingActions button,\n",
    "      .badge,\n",
    "      .count,\n",
    "      .miniBadge {\n",
    "        width: 100%;\n",
    "        text-align: center;\n",
    "      }\n",
    "    }\n",
    "  </style>\n",
    "</head>\n",
    "<body>\n",
    "  <main class='shell'>\n",
    "    <header class='hero'>\n",
    "      <div>\n",
    "        <h1 class='title'>SD 卡媒体浏览</h1>\n",
    "        <p class='desc'>浏览 SD 卡中的照片和视频，支持网页拍照、设备状态查看、波特率与分辨率配置、静态 IP 设置以及批量删除操作。</p>\n",
    "      </div>\n",
    "      <div class='toolbar'>\n",
    "        <div id='status' class='badge' role='status' aria-live='polite'>正在读取媒体列表...</div>\n",
    "        <div id='overview' class='badge' aria-live='polite'>照片 0 张 | 视频 0 段 | 已选 0 项</div>\n",
    "        <button id='reload' class='ghostBtn' type='button'>刷新列表</button>\n",
    "        <button id='toggleFoldAll' class='ghostBtn' type='button'>全部折叠</button>\n",
    "        <button id='deleteSelection' class='dangerBtn' type='button'>删除选中</button>\n",
    "      </div>\n",
    "    </header>\n",
    "    <section class='settingsGrid'>\n",
    "      <section class='settingCard' aria-labelledby='deviceStatusHeading'>\n",
    "        <div class='sectionHead'>\n",
    "          <div class='sectionTitleWrap'>\n",
    "            <h2 id='deviceStatusHeading'>设备状态</h2>\n",
    "            <div class='count'>实时读取</div>\n",
    "          </div>\n",
    "        </div>\n",
    "        <div class='statusList'>\n",
    "          <div class='statusRow'><div class='statusKey'>当前时间</div><div id='deviceCurrentTime' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>当前 IP</div><div id='deviceCurrentIp' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>当前网关</div><div id='deviceCurrentGw' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>子网掩码</div><div id='deviceCurrentMask' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>RTSP 地址</div><div id='deviceRtspUrl' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>TF 卡状态</div><div id='deviceTfStatus' class='statusValue'>--</div></div>\n",
    "          <div class='statusRow'><div class='statusKey'>RTSP 客户端</div><div id='deviceRtspClients' class='statusValue'>0</div></div>\n",
    "        </div>\n",
    "        <div class='settingActions'>\n",
    "          <button id='refreshStatusBtn' class='ghostBtn' type='button'>读取状态</button>\n",
    "          <button id='syncTimeBtn' class='ghostBtn' type='button'>同步时间</button>\n",
    "          <button id='rebootBtn' class='ghostBtn' type='button'>重启设备</button>\n",
    "        </div>\n",
    "      </section>\n",
    "      <section class='settingCard' aria-labelledby='deviceConfigHeading'>\n",
    "        <div class='sectionHead'>\n",
    "          <div class='sectionTitleWrap'>\n",
    "            <h2 id='deviceConfigHeading'>设备设置</h2>\n",
    "            <div class='count'>保存后重启生效</div>\n",
    "          </div>\n",
    "        </div>\n",
    "        <div class='field'>\n",
    "          <label for='videoProfile'>视频分辨率</label>\n",
    "          <select id='videoProfile'>\n",
    "            <option value='1'>1280 x 960</option>\n",
    "            <option value='2'>1920 x 1080</option>\n",
    "            <option value='3'>800 x 800</option>\n",
    "            <option value='4'>800 x 640</option>\n",
    "          </select>\n",
    "        </div>\n",
    "        <div class='fieldRow'>\n",
    "          <div class='field'>\n",
    "            <label for='uart0Baud'>UART0 波特率</label>\n",
    "            <select id='uart0Baud'>\n",
    "              <option value='9600'>9600</option>\n",
    "              <option value='19200'>19200</option>\n",
    "              <option value='38400'>38400</option>\n",
    "              <option value='57600'>57600</option>\n",
    "              <option value='115200'>115200</option>\n",
    "              <option value='230400'>230400</option>\n",
    "              <option value='460800'>460800</option>\n",
    "              <option value='921600'>921600</option>\n",
    "              <option value='1500000'>1500000</option>\n",
    "            </select>\n",
    "          </div>\n",
    "          <div class='field'>\n",
    "            <label for='uart1Baud'>UART1 波特率</label>\n",
    "            <select id='uart1Baud'>\n",
    "              <option value='9600'>9600</option>\n",
    "              <option value='19200'>19200</option>\n",
    "              <option value='38400'>38400</option>\n",
    "              <option value='57600'>57600</option>\n",
    "              <option value='115200'>115200</option>\n",
    "              <option value='230400'>230400</option>\n",
    "              <option value='460800'>460800</option>\n",
    "              <option value='921600'>921600</option>\n",
    "              <option value='1500000'>1500000</option>\n",
    "            </select>\n",
    "          </div>\n",
    "        </div>\n",
    "        <label class='checkboxLine'>\n",
    "          <input id='wifiUseStaticIp' type='checkbox'>\n",
    "          使用静态 IP\n",
    "        </label>\n",
    "        <div class='fieldRow'>\n",
    "          <div class='field'>\n",
    "            <label for='wifiStaticIp'>静态 IP</label>\n",
    "            <input id='wifiStaticIp' type='text' inputmode='decimal' placeholder='192.168.0.200'>\n",
    "          </div>\n",
    "          <div class='field'>\n",
    "            <label for='wifiStaticGw'>网关</label>\n",
    "            <input id='wifiStaticGw' type='text' inputmode='decimal' placeholder='192.168.0.1'>\n",
    "          </div>\n",
    "        </div>\n",
    "        <div class='field'>\n",
    "          <label for='wifiStaticMask'>子网掩码</label>\n",
    "          <input id='wifiStaticMask' type='text' inputmode='decimal' placeholder='255.255.255.0'>\n",
    "        </div>\n",
    "        <div class='settingHint'>静态 IP、波特率和视频分辨率在保存后需要重启设备生效。恢复默认配置仅重置网页可配置参数，不会删除 TF 卡中的照片和视频。</div>\n",
    "        <div class='settingActions'>\n",
    "          <button id='saveConfigBtn' class='primaryBtn' type='button'>保存配置</button>\n",
    "          <button id='factoryResetBtn' class='dangerBtn' type='button'>恢复默认配置</button>\n",
    "        </div>\n",
    "      </section>\n",
    "    </section>\n",
    "    <section class='media'>\n",
    "      <section class='panel' aria-labelledby='photoHeading'>\n",
    "        <div class='sectionHead'>\n",
    "          <div class='sectionTitleWrap'>\n",
    "            <button id='photoFoldBtn' class='foldBtn' type='button' aria-controls='photoBody' aria-expanded='true'>收起</button>\n",
    "            <h2 id='photoHeading'>照片</h2>\n",
    "            <div id='photoCount' class='count'>0 张</div>\n",
    "            <div id='photoSelection' class='miniBadge'>未进入选择</div>\n",
    "            <button id='photoSelectAllBtn' class='ghostBtn' type='button'>全选</button>\n",
    "          </div>\n",
    "          <div class='sectionTools'>\n",
    "            <button id='captureBtn' class='primaryBtn' type='button'>立即拍照</button>\n",
    "            <button id='photoToggleAllBtn' class='ghostBtn' type='button'>选择照片</button>\n",
    "            <button id='photoDeleteSelectedBtn' class='dangerBtn' type='button'>删除选中</button>\n",
    "          </div>\n",
    "        </div>\n",
    "        <div id='photoBody' class='sectionBody'>\n",
    "          <div id='photoGrid' class='grid'></div>\n",
    "        </div>\n",
    "      </section>\n",
    "      <section class='panel' aria-labelledby='videoHeading'>\n",
    "        <div class='sectionHead'>\n",
    "          <div class='sectionTitleWrap'>\n",
    "            <button id='videoFoldBtn' class='foldBtn' type='button' aria-controls='videoBody' aria-expanded='true'>收起</button>\n",
    "            <h2 id='videoHeading'>视频</h2>\n",
    "            <div id='videoCount' class='count'>0 段</div>\n",
    "            <div id='videoSelection' class='miniBadge'>未进入选择</div>\n",
    "            <button id='videoSelectAllBtn' class='ghostBtn' type='button'>全选</button>\n",
    "          </div>\n",
    "          <div class='sectionTools'>\n",
    "            <button id='videoToggleAllBtn' class='ghostBtn' type='button'>选择视频</button>\n",
    "            <button id='videoDeleteSelectedBtn' class='dangerBtn' type='button'>删除选中</button>\n",
    "          </div>\n",
    "        </div>\n",
    "        <div id='videoBody' class='sectionBody'>\n",
    "          <div id='videoGrid' class='grid videoGrid'></div>\n",
    "        </div>\n",
    "      </section>\n",
    "    </section>\n",
    "  </main>\n",
    "  <script>\n",
    "    const SECTION_KEYS = ['photo', 'video'];\n",
    "    const CAPTURE_RETRY_COUNT = 6;\n",
    "    const CAPTURE_RETRY_DELAY_MS = 500;\n",
    "    const state = {\n",
    "      busy: false,\n",
    "      deviceConfig: null,\n",
    "      deviceStatus: null,\n",
    "      photo: { title: '照片', unit: '张', empty: 'SD 卡中暂无照片', items: [], selected: new Set(), collapsed: false, selecting: false },\n",
    "      video: { title: '视频', unit: '段', empty: 'SD 卡中暂无视频', items: [], selected: new Set(), collapsed: false, selecting: false }\n",
    "    };\n",
    "    const refs = {\n",
    "      status: document.getElementById('status'),\n",
    "      overview: document.getElementById('overview'),\n",
    "      captureBtn: document.getElementById('captureBtn'),\n",
    "      reloadBtn: document.getElementById('reload'),\n",
    "      toggleFoldAllBtn: document.getElementById('toggleFoldAll'),\n",
    "      deleteSelectionBtn: document.getElementById('deleteSelection'),\n",
    "      device: {\n",
    "        currentTime: document.getElementById('deviceCurrentTime'),\n",
    "        currentIp: document.getElementById('deviceCurrentIp'),\n",
    "        currentGw: document.getElementById('deviceCurrentGw'),\n",
    "        currentMask: document.getElementById('deviceCurrentMask'),\n",
    "        rtspUrl: document.getElementById('deviceRtspUrl'),\n",
    "        tfStatus: document.getElementById('deviceTfStatus'),\n",
    "        rtspClients: document.getElementById('deviceRtspClients'),\n",
    "        refreshStatusBtn: document.getElementById('refreshStatusBtn'),\n",
    "        syncTimeBtn: document.getElementById('syncTimeBtn'),\n",
    "        rebootBtn: document.getElementById('rebootBtn'),\n",
    "        videoProfile: document.getElementById('videoProfile'),\n",
    "        uart0Baud: document.getElementById('uart0Baud'),\n",
    "        uart1Baud: document.getElementById('uart1Baud'),\n",
    "        wifiUseStaticIp: document.getElementById('wifiUseStaticIp'),\n",
    "        wifiStaticIp: document.getElementById('wifiStaticIp'),\n",
    "        wifiStaticGw: document.getElementById('wifiStaticGw'),\n",
    "        wifiStaticMask: document.getElementById('wifiStaticMask'),\n",
    "        saveConfigBtn: document.getElementById('saveConfigBtn'),\n",
    "        factoryResetBtn: document.getElementById('factoryResetBtn')\n",
    "      },\n",
    "      photo: {\n",
    "        body: document.getElementById('photoBody'),\n",
    "        grid: document.getElementById('photoGrid'),\n",
    "        count: document.getElementById('photoCount'),\n",
    "        selection: document.getElementById('photoSelection'),\n",
    "        selectAllBtn: document.getElementById('photoSelectAllBtn'),\n",
    "        toggleAllBtn: document.getElementById('photoToggleAllBtn'),\n",
    "        deleteBtn: document.getElementById('photoDeleteSelectedBtn'),\n",
    "        foldBtn: document.getElementById('photoFoldBtn')\n",
    "      },\n",
    "      video: {\n",
    "        body: document.getElementById('videoBody'),\n",
    "        grid: document.getElementById('videoGrid'),\n",
    "        count: document.getElementById('videoCount'),\n",
    "        selection: document.getElementById('videoSelection'),\n",
    "        selectAllBtn: document.getElementById('videoSelectAllBtn'),\n",
    "        toggleAllBtn: document.getElementById('videoToggleAllBtn'),\n",
    "        deleteBtn: document.getElementById('videoDeleteSelectedBtn'),\n",
    "        foldBtn: document.getElementById('videoFoldBtn')\n",
    "      }\n",
    "    };\n",
    "    function formatSize(size) {\n",
    "      if (!Number.isFinite(size) || size <= 0) {\n",
    "        return '--';\n",
    "      }\n",
    "      if (size < 1024) {\n",
    "        return size + ' B';\n",
    "      }\n",
    "      if (size < 1024 * 1024) {\n",
    "        return (size / 1024).toFixed(1) + ' KB';\n",
    "      }\n",
    "      return (size / 1024 / 1024).toFixed(2) + ' MB';\n",
    "    }\n",
    "    function sortItems(items) {\n",
    "      return items.slice().sort((a, b) => String(b.path || '').localeCompare(String(a.path || '')));\n",
    "    }\n",
    "    function totalItemCount() {\n",
    "      return state.photo.items.length + state.video.items.length;\n",
    "    }\n",
    "    function totalSelectedCount() {\n",
    "      return state.photo.selected.size + state.video.selected.size;\n",
    "    }\n",
    "    function hasSelectionMode() {\n",
    "      return SECTION_KEYS.some((kind) => state[kind].selecting);\n",
    "    }\n",
    "    function allPanelsCollapsed() {\n",
    "      return SECTION_KEYS.every((kind) => state[kind].items.length === 0 || state[kind].collapsed);\n",
    "    }\n",
    "    function sectionAllSelected(kind) {\n",
    "      const info = state[kind];\n",
    "      return info.selecting && info.items.length > 0 && info.selected.size === info.items.length;\n",
    "    }\n",
    "    function setStatus(text, isError) {\n",
    "      refs.status.textContent = text;\n",
    "      refs.status.classList.toggle('error', !!isError);\n",
    "    }\n",
    "    function createEmptyNode(message) {\n",
    "      const box = document.createElement('div');\n",
    "      box.className = 'empty';\n",
    "      box.textContent = message;\n",
    "      return box;\n",
    "    }\n",
    "    function appendText(parent, className, text) {\n",
    "      const node = document.createElement('div');\n",
    "      node.className = className;\n",
    "      node.textContent = text;\n",
    "      parent.appendChild(node);\n",
    "    }\n",
    "    function waitMs(delay) {\n",
    "      return new Promise((resolve) => window.setTimeout(resolve, delay));\n",
    "    }\n",
    "    function updateWifiFieldState() {\n",
    "      const disabled = state.busy || !refs.device.wifiUseStaticIp.checked;\n",
    "      refs.device.wifiStaticIp.disabled = disabled;\n",
    "      refs.device.wifiStaticGw.disabled = disabled;\n",
    "      refs.device.wifiStaticMask.disabled = disabled;\n",
    "    }\n",
    "    function renderDeviceStatus() {\n",
    "      const info = state.deviceStatus || {};\n",
    "      refs.device.currentTime.textContent = info.current_time || '--';\n",
    "      refs.device.currentIp.textContent = info.current_ip || '--';\n",
    "      refs.device.currentGw.textContent = info.current_gw || '--';\n",
    "      refs.device.currentMask.textContent = info.current_mask || '--';\n",
    "      refs.device.rtspUrl.textContent = info.rtsp_url || '--';\n",
    "      refs.device.tfStatus.textContent = info.tf_mounted ? '已挂载' : '未挂载';\n",
    "      refs.device.rtspClients.textContent = String(Number(info.active_clients) || 0);\n",
    "    }\n",
    "    function renderDeviceConfig() {\n",
    "      const config = state.deviceConfig;\n",
    "      if (!config) {\n",
    "        return;\n",
    "      }\n",
    "      refs.device.videoProfile.value = String(config.video_profile || '1');\n",
    "      refs.device.uart0Baud.value = String(config.uart0_baud_rate || '115200');\n",
    "      refs.device.uart1Baud.value = String(config.uart1_baud_rate || '115200');\n",
    "      refs.device.wifiUseStaticIp.checked = !!config.wifi_use_static_ip;\n",
    "      refs.device.wifiStaticIp.value = String(config.wifi_static_ip || '');\n",
    "      refs.device.wifiStaticGw.value = String(config.wifi_static_gw || '');\n",
    "      refs.device.wifiStaticMask.value = String(config.wifi_static_mask || '');\n",
    "      updateWifiFieldState();\n",
    "    }\n",
    "    function updateDeviceActions() {\n",
    "      refs.device.refreshStatusBtn.disabled = state.busy;\n",
    "      refs.device.syncTimeBtn.disabled = state.busy;\n",
    "      refs.device.rebootBtn.disabled = state.busy;\n",
    "      refs.device.videoProfile.disabled = state.busy;\n",
    "      refs.device.uart0Baud.disabled = state.busy;\n",
    "      refs.device.uart1Baud.disabled = state.busy;\n",
    "      refs.device.wifiUseStaticIp.disabled = state.busy;\n",
    "      refs.device.saveConfigBtn.disabled = state.busy;\n",
    "      refs.device.factoryResetBtn.disabled = state.busy;\n",
    "      updateWifiFieldState();\n",
    "    }\n",
    "    function updateOverview() {\n",
    "      refs.overview.textContent = '照片 ' + state.photo.items.length + ' 张 | 视频 ' + state.video.items.length + ' 段 | 已选 ' + totalSelectedCount() + ' 项';\n",
    "    }\n",
    "    function updateSectionHeader(kind) {\n",
    "      const info = state[kind];\n",
    "      const ref = refs[kind];\n",
    "      const total = info.items.length;\n",
    "      const selected = info.selected.size;\n",
    "      ref.count.textContent = total + ' ' + info.unit;\n",
    "      ref.selection.textContent = info.selecting ? ('已选 ' + selected + ' 项') : '未进入选择';\n",
    "      ref.selectAllBtn.textContent = sectionAllSelected(kind) ? '取消全选' : '全选';\n",
    "      ref.selectAllBtn.disabled = state.busy || total === 0;\n",
    "      ref.toggleAllBtn.textContent = info.selecting ? ('取消选择' + info.title) : ('选择' + info.title);\n",
    "      ref.toggleAllBtn.disabled = state.busy || total === 0;\n",
    "      ref.deleteBtn.disabled = state.busy || !info.selecting || selected === 0;\n",
    "      ref.foldBtn.disabled = state.busy || total === 0;\n",
    "      ref.foldBtn.textContent = info.collapsed ? '展开' : '收起';\n",
    "      ref.foldBtn.setAttribute('aria-expanded', info.collapsed ? 'false' : 'true');\n",
    "      ref.body.classList.toggle('hidden', info.collapsed);\n",
    "    }\n",
    "    function updateToolbar() {\n",
    "      const total = totalItemCount();\n",
    "      const selected = totalSelectedCount();\n",
    "      refs.captureBtn.disabled = state.busy;\n",
    "      refs.reloadBtn.disabled = state.busy;\n",
    "      refs.toggleFoldAllBtn.disabled = state.busy || total === 0;\n",
    "      refs.toggleFoldAllBtn.textContent = allPanelsCollapsed() ? '全部展开' : '全部折叠';\n",
    "      refs.deleteSelectionBtn.disabled = state.busy || selected === 0 || !hasSelectionMode();\n",
    "    }\n",
    "    function updateActionStates() {\n",
    "      updateOverview();\n",
    "      updateSectionHeader('photo');\n",
    "      updateSectionHeader('video');\n",
    "      updateToolbar();\n",
    "      updateDeviceActions();\n",
    "    }\n",
    "    function setBusy(busy) {\n",
    "      state.busy = busy;\n",
    "      updateActionStates();\n",
    "    }\n",
    "    function clampSelection(kind) {\n",
    "      const validPaths = new Set(state[kind].items.map((item) => item.path));\n",
    "      state[kind].selected = new Set(Array.from(state[kind].selected).filter((path) => validPaths.has(path)));\n",
    "      if (state[kind].items.length === 0) {\n",
    "        state[kind].collapsed = false;\n",
    "        state[kind].selecting = false;\n",
    "        state[kind].selected.clear();\n",
    "      }\n",
    "    }\n",
    "    function pauseOtherVideos(current) {\n",
    "      document.querySelectorAll('video').forEach((video) => {\n",
    "        if (video !== current) {\n",
    "          video.pause();\n",
    "        }\n",
    "      });\n",
    "    }\n",
    "    function createCard(kind, item) {\n",
    "      const info = state[kind];\n",
    "      const selected = info.selected.has(item.path);\n",
    "      const card = document.createElement('article');\n",
    "      let thumb = null;\n",
    "      const meta = document.createElement('div');\n",
    "      const actions = document.createElement('div');\n",
    "      const open = document.createElement('a');\n",
    "      card.className = 'card' + (selected ? ' selected' : '');\n",
    "      if (info.selecting) {\n",
    "        const top = document.createElement('div');\n",
    "        const pick = document.createElement('label');\n",
    "        const checkbox = document.createElement('input');\n",
    "        top.className = 'cardTop';\n",
    "        pick.className = 'pick';\n",
    "        checkbox.type = 'checkbox';\n",
    "        checkbox.checked = selected;\n",
    "        checkbox.disabled = state.busy;\n",
    "        checkbox.setAttribute('aria-label', '选择媒体文件');\n",
    "        checkbox.addEventListener('change', () => {\n",
    "          if (checkbox.checked) {\n",
    "            info.selected.add(item.path);\n",
    "          } else {\n",
    "            info.selected.delete(item.path);\n",
    "          }\n",
    "          card.classList.toggle('selected', checkbox.checked);\n",
    "          updateActionStates();\n",
    "        });\n",
    "        pick.appendChild(checkbox);\n",
    "        top.appendChild(pick);\n",
    "        card.appendChild(top);\n",
    "      }\n",
    "      if (kind === 'photo') {\n",
    "        const img = document.createElement('img');\n",
    "        thumb = document.createElement('a');\n",
    "        thumb.className = 'thumb';\n",
    "        thumb.href = item.url;\n",
    "        thumb.target = '_blank';\n",
    "        thumb.rel = 'noreferrer';\n",
    "        img.loading = 'lazy';\n",
    "        img.decoding = 'async';\n",
    "        img.src = item.url;\n",
    "        img.alt = item.name || 'photo';\n",
    "        thumb.appendChild(img);\n",
    "      } else {\n",
    "        const video = document.createElement('video');\n",
    "        thumb = document.createElement('div');\n",
    "        thumb.className = 'thumb';\n",
    "        video.controls = true;\n",
    "        video.preload = 'none';\n",
    "        video.src = item.url;\n",
    "        video.setAttribute('playsinline', '');\n",
    "        video.addEventListener('play', () => pauseOtherVideos(video));\n",
    "        thumb.appendChild(video);\n",
    "      }\n",
    "      meta.className = 'meta';\n",
    "      appendText(meta, 'name', item.name || '');\n",
    "      appendText(meta, 'sub', '目录：' + (item.session || ''));\n",
    "      appendText(meta, 'sub', '大小：' + formatSize(Number(item.size_bytes)));\n",
    "      appendText(meta, 'path', item.path || '');\n",
    "      actions.className = 'metaActions';\n",
    "      open.className = 'open';\n",
    "      open.href = item.url;\n",
    "      open.target = '_blank';\n",
    "      open.rel = 'noreferrer';\n",
    "      open.textContent = kind === 'photo' ? '打开图片' : '打开视频';\n",
    "      actions.appendChild(open);\n",
    "      meta.appendChild(actions);\n",
    "      card.appendChild(thumb);\n",
    "      card.appendChild(meta);\n",
    "      return card;\n",
    "    }\n",
    "    function renderSection(kind) {\n",
    "      const info = state[kind];\n",
    "      const ref = refs[kind];\n",
    "      ref.grid.innerHTML = '';\n",
    "      if (!info.items.length) {\n",
    "        ref.grid.appendChild(createEmptyNode(info.empty));\n",
    "        return;\n",
    "      }\n",
    "      info.items.forEach((item) => {\n",
    "        ref.grid.appendChild(createCard(kind, item));\n",
    "      });\n",
    "    }\n",
    "    function renderAll() {\n",
    "      renderSection('photo');\n",
    "      renderSection('video');\n",
    "      updateActionStates();\n",
    "    }\n",
    "    function toggleSectionSelection(kind) {\n",
    "      const info = state[kind];\n",
    "      if (state.busy || !info.items.length) {\n",
    "        return;\n",
    "      }\n",
    "      info.selecting = !info.selecting;\n",
    "      if (!info.selecting) {\n",
    "        info.selected.clear();\n",
    "      }\n",
    "      renderAll();\n",
    "    }\n",
    "    function toggleSectionSelectAll(kind) {\n",
    "      const info = state[kind];\n",
    "      if (state.busy || !info.items.length) {\n",
    "        return;\n",
    "      }\n",
    "      if (!info.selecting) {\n",
    "        info.selecting = true;\n",
    "      }\n",
    "      if (sectionAllSelected(kind)) {\n",
    "        info.selected.clear();\n",
    "      } else {\n",
    "        info.selected = new Set(info.items.map((item) => item.path));\n",
    "      }\n",
    "      renderAll();\n",
    "    }\n",
    "    function toggleSectionFold(kind) {\n",
    "      if (state.busy || !state[kind].items.length) {\n",
    "        return;\n",
    "      }\n",
    "      state[kind].collapsed = !state[kind].collapsed;\n",
    "      updateActionStates();\n",
    "    }\n",
    "    function toggleAllFolds() {\n",
    "      const collapse = !allPanelsCollapsed();\n",
    "      if (state.busy || totalItemCount() === 0) {\n",
    "        return;\n",
    "      }\n",
    "      SECTION_KEYS.forEach((kind) => {\n",
    "        if (state[kind].items.length > 0) {\n",
    "          state[kind].collapsed = collapse;\n",
    "        }\n",
    "      });\n",
    "      updateActionStates();\n",
    "    }\n",
    "    // 统一读取 JSON 响应，复用网页接口的错误处理逻辑。\n",
    "    async function fetchJson(url, options) {\n",
    "      const request = Object.assign({ cache: 'no-store' }, options || {});\n",
    "      const response = await fetch(url, request);\n",
    "      let data = {};\n",
    "      try {\n",
    "        data = await response.json();\n",
    "      } catch (error) {\n",
    "        data = {};\n",
    "      }\n",
    "      if (!response.ok) {\n",
    "        throw new Error(data.error || ('HTTP ' + response.status));\n",
    "      }\n",
    "      return data;\n",
    "    }\n",
    "    async function fetchList(url) {\n",
    "      const data = await fetchJson(url);\n",
    "      return Array.isArray(data.items) ? data.items : [];\n",
    "    }\n",
    "    async function fetchDeviceStatus() {\n",
    "      return fetchJson('/api/status');\n",
    "    }\n",
    "    async function fetchDeviceConfig() {\n",
    "      return fetchJson('/api/config');\n",
    "    }\n",
    "    async function syncDeviceTime() {\n",
    "      return fetchJson('/api/time?unix_ms=' + Date.now());\n",
    "    }\n",
    "    async function requestSaveDeviceConfig() {\n",
    "      const params = new URLSearchParams();\n",
    "      params.set('video_profile', refs.device.videoProfile.value);\n",
    "      params.set('uart0_baud_rate', refs.device.uart0Baud.value);\n",
    "      params.set('uart1_baud_rate', refs.device.uart1Baud.value);\n",
    "      params.set('wifi_use_static_ip', refs.device.wifiUseStaticIp.checked ? '1' : '0');\n",
    "      params.set('wifi_static_ip', refs.device.wifiStaticIp.value.trim());\n",
    "      params.set('wifi_static_gw', refs.device.wifiStaticGw.value.trim());\n",
    "      params.set('wifi_static_mask', refs.device.wifiStaticMask.value.trim());\n",
    "      return fetchJson('/api/config', {\n",
    "        method: 'POST',\n",
    "        headers: { 'Content-Type': 'application/x-www-form-urlencoded; charset=utf-8' },\n",
    "        body: params.toString()\n",
    "      });\n",
    "    }\n",
    "    async function requestFactoryReset() {\n",
    "      return fetchJson('/api/factory_reset', { method: 'POST' });\n",
    "    }\n",
    "    async function requestDeviceReboot() {\n",
    "      return fetchJson('/api/reboot', { method: 'POST' });\n",
    "    }\n",
    "    async function refreshDevicePanel() {\n",
    "      const results = await Promise.all([fetchDeviceStatus(), fetchDeviceConfig()]);\n",
    "      state.deviceStatus = results[0];\n",
    "      state.deviceConfig = results[1];\n",
    "      renderDeviceStatus();\n",
    "      renderDeviceConfig();\n",
    "    }\n",
    "    async function refreshMediaList() {\n",
    "      const results = await Promise.all([fetchList('/api/photos'), fetchList('/api/videos')]);\n",
    "      state.photo.items = sortItems(results[0]);\n",
    "      state.video.items = sortItems(results[1]);\n",
    "      clampSelection('photo');\n",
    "      clampSelection('video');\n",
    "      renderAll();\n",
    "    }\n",
    "    async function requestCapture() {\n",
    "      return fetchJson('/api/capture', { method: 'POST' });\n",
    "    }\n",
    "    async function requestDelete(paths) {\n",
    "      return fetchJson('/api/delete', {\n",
    "        method: 'POST',\n",
    "        headers: { 'Content-Type': 'text/plain; charset=utf-8' },\n",
    "        body: paths.join('\\n')\n",
    "      });\n",
    "    }\n",
    "    async function loadDeviceStatusOnly(doneText) {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在读取设备状态...', false);\n",
    "      try {\n",
    "        await refreshDevicePanel();\n",
    "        setStatus(doneText || '设备状态已更新', false);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '读取设备状态失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function syncDeviceClockAndRefresh() {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在同步设备时间...', false);\n",
    "      try {\n",
    "        await syncDeviceTime();\n",
    "        await refreshDevicePanel();\n",
    "        setStatus('设备时间已同步', false);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '同步设备时间失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function saveDeviceConfig() {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在保存设备配置...', false);\n",
    "      try {\n",
    "        await requestSaveDeviceConfig();\n",
    "        await refreshDevicePanel();\n",
    "        setStatus('配置已保存，请重启设备使新配置生效', false);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '保存设备配置失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function restoreFactoryConfig() {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      if (!window.confirm('确定恢复网页可配置参数的默认值吗？此操作不会删除 TF 卡文件。')) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在恢复默认配置...', false);\n",
    "      try {\n",
    "        await requestFactoryReset();\n",
    "        await refreshDevicePanel();\n",
    "        setStatus('默认配置已恢复，请重启设备使默认配置生效', false);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '恢复默认配置失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function rebootDevice() {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      if (!window.confirm('确定立即重启设备吗？当前页面连接将会中断。')) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在发送重启指令...', false);\n",
    "      try {\n",
    "        await requestDeviceReboot();\n",
    "        setStatus('设备正在重启，请稍后手动刷新页面', false);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '发送重启指令失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    function buildDeleteBatches(paths) {\n",
    "      const batches = [];\n",
    "      let current = [];\n",
    "      let currentLen = 0;\n",
    "      const maxLen = 3000;\n",
    "      paths.forEach((path) => {\n",
    "        const lineLen = (path ? path.length : 0) + 1;\n",
    "        if (current.length > 0 && currentLen + lineLen > maxLen) {\n",
    "          batches.push(current);\n",
    "          current = [];\n",
    "          currentLen = 0;\n",
    "        }\n",
    "        current.push(path);\n",
    "        currentLen += lineLen;\n",
    "      });\n",
    "      if (current.length > 0) {\n",
    "        batches.push(current);\n",
    "      }\n",
    "      return batches;\n",
    "    }\n",
    "    // 拍照请求提交后，短轮询几次照片列表，等待新文件落盘。\n",
    "    async function capturePhoto() {\n",
    "      const beforeCount = state.photo.items.length;\n",
    "      const beforeFirstPath = state.photo.items.length > 0 ? String(state.photo.items[0].path || '') : '';\n",
    "      let captured = false;\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在提交拍照请求...', false);\n",
    "      try {\n",
    "        await requestCapture();\n",
    "        for (let i = 0; i < CAPTURE_RETRY_COUNT; i += 1) {\n",
    "          setStatus('正在等待照片保存...', false);\n",
    "          await waitMs(CAPTURE_RETRY_DELAY_MS);\n",
    "          await refreshMediaList();\n",
    "          const currentFirstPath = state.photo.items.length > 0 ? String(state.photo.items[0].path || '') : '';\n",
    "          if (state.photo.items.length > beforeCount || currentFirstPath !== beforeFirstPath) {\n",
    "            captured = true;\n",
    "            break;\n",
    "          }\n",
    "        }\n",
    "        if (captured) {\n",
    "          setStatus('拍照完成，照片列表已更新', false);\n",
    "        } else {\n",
    "          setStatus('拍照请求已提交，请稍后刷新查看结果', false);\n",
    "        }\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '拍照失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function loadMedia(doneText, doneIsError) {\n",
    "      if (state.busy) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      setStatus('正在同步设备时间...', false);\n",
    "      try {\n",
    "        await syncDeviceTime();\n",
    "        setStatus('正在读取设备状态...', false);\n",
    "        await refreshDevicePanel();\n",
    "        setStatus('正在读取媒体列表...', false);\n",
    "        await refreshMediaList();\n",
    "        if (doneText) {\n",
    "          setStatus(doneText, !!doneIsError);\n",
    "        } else {\n",
    "          setStatus('照片 ' + state.photo.items.length + ' 张，视频 ' + state.video.items.length + ' 段', false);\n",
    "        }\n",
    "      } catch (error) {\n",
    "        state.photo.items = [];\n",
    "        state.video.items = [];\n",
    "        state.photo.selected.clear();\n",
    "        state.video.selected.clear();\n",
    "        state.photo.selecting = false;\n",
    "        state.video.selecting = false;\n",
    "        renderAll();\n",
    "        setStatus(error.message || '读取媒体列表失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    async function deletePaths(paths) {\n",
    "      const uniquePaths = Array.from(new Set((paths || []).filter(Boolean)));\n",
    "      const tips = uniquePaths.length === 1 ? '确定删除当前文件吗？' : ('确定删除选中的 ' + uniquePaths.length + ' 个文件吗？');\n",
    "      let summary = { requested: 0, deleted: 0, failed: 0 };\n",
    "      if (state.busy || uniquePaths.length === 0) {\n",
    "        return;\n",
    "      }\n",
    "      if (!window.confirm(tips)) {\n",
    "        return;\n",
    "      }\n",
    "      setBusy(true);\n",
    "      try {\n",
    "        const batches = buildDeleteBatches(uniquePaths);\n",
    "        for (let i = 0; i < batches.length; i += 1) {\n",
    "          setStatus('正在删除第 ' + (i + 1) + ' / ' + batches.length + ' 批...', false);\n",
    "          const result = await requestDelete(batches[i]);\n",
    "          summary.requested += Number(result.requested) || batches[i].length;\n",
    "          summary.deleted += Number(result.deleted) || 0;\n",
    "          summary.failed += Number(result.failed) || 0;\n",
    "        }\n",
    "        uniquePaths.forEach((path) => {\n",
    "          state.photo.selected.delete(path);\n",
    "          state.video.selected.delete(path);\n",
    "        });\n",
    "        setStatus('正在刷新媒体列表...', false);\n",
    "        await refreshMediaList();\n",
    "        setStatus('删除完成：成功 ' + summary.deleted + '，失败 ' + summary.failed, summary.failed > 0);\n",
    "      } catch (error) {\n",
    "        setStatus(error.message || '删除失败', true);\n",
    "      } finally {\n",
    "        setBusy(false);\n",
    "      }\n",
    "    }\n",
    "    refs.captureBtn.addEventListener('click', capturePhoto);\n",
    "    refs.reloadBtn.addEventListener('click', () => loadMedia());\n",
    "    refs.toggleFoldAllBtn.addEventListener('click', toggleAllFolds);\n",
    "    refs.deleteSelectionBtn.addEventListener('click', () => deletePaths(Array.from(state.photo.selected).concat(Array.from(state.video.selected))));\n",
    "    refs.device.refreshStatusBtn.addEventListener('click', () => loadDeviceStatusOnly());\n",
    "    refs.device.syncTimeBtn.addEventListener('click', syncDeviceClockAndRefresh);\n",
    "    refs.device.rebootBtn.addEventListener('click', rebootDevice);\n",
    "    refs.device.saveConfigBtn.addEventListener('click', saveDeviceConfig);\n",
    "    refs.device.factoryResetBtn.addEventListener('click', restoreFactoryConfig);\n",
    "    refs.device.wifiUseStaticIp.addEventListener('change', updateWifiFieldState);\n",
    "    refs.photo.selectAllBtn.addEventListener('click', () => toggleSectionSelectAll('photo'));\n",
    "    refs.photo.toggleAllBtn.addEventListener('click', () => toggleSectionSelection('photo'));\n",
    "    refs.video.selectAllBtn.addEventListener('click', () => toggleSectionSelectAll('video'));\n",
    "    refs.video.toggleAllBtn.addEventListener('click', () => toggleSectionSelection('video'));\n",
    "    refs.photo.deleteBtn.addEventListener('click', () => deletePaths(Array.from(state.photo.selected)));\n",
    "    refs.video.deleteBtn.addEventListener('click', () => deletePaths(Array.from(state.video.selected)));\n",
    "    refs.photo.foldBtn.addEventListener('click', () => toggleSectionFold('photo'));\n",
    "    refs.video.foldBtn.addEventListener('click', () => toggleSectionFold('video'));\n",
    "    window.addEventListener('DOMContentLoaded', () => {\n",
    "      loadMedia();\n",
    "    });\n",
    "  </script>\n",
    "</body>\n",
    "</html>\n",
    NULL,
};

/* ------------------------------------------------------------------ */
/* 内部工具函数                                                        */
/* ------------------------------------------------------------------ */
static esp_err_t photo_web_append_text(char *dst, size_t dst_size,
                                       size_t *offset, const char *text)
{
    size_t text_len;

    if (!dst || dst_size == 0 || !offset || !text)
    {
        return ESP_ERR_INVALID_ARG;
    }

    text_len = strlen(text);
    if (*offset + text_len >= dst_size)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(dst + *offset, text, text_len);
    *offset += text_len;
    dst[*offset] = '\0';
    return ESP_OK;
}

static esp_err_t photo_web_join_path(char *dst, size_t dst_size,
                                     const char *dir, const char *name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !dir || !name)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, dir);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, name);
    }

    return ret;
}

static esp_err_t photo_web_build_relative_path(char *dst, size_t dst_size,
                                               const char *session, const char *subdir,
                                               const char *file_name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !session || !subdir || !file_name)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, session);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, subdir);
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, file_name);
    }

    return ret;
}

static esp_err_t photo_web_build_media_uri(char *dst, size_t dst_size,
                                           const char *uri_prefix,
                                           const char *relative_path)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !uri_prefix || !relative_path)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, uri_prefix);
    if (ret == ESP_OK)
    {
        ret = photo_web_append_text(dst, dst_size, &offset, relative_path);
    }

    return ret;
}

static bool photo_web_has_jpeg_suffix(const char *name)
{
    size_t len;

    if (!name)
    {
        return false;
    }

    len = strlen(name);
    if (len >= 5 && strcasecmp(name + len - 5, ".jpeg") == 0)
    {
        return true;
    }
    if (len >= 4 && strcasecmp(name + len - 4, ".jpg") == 0)
    {
        return true;
    }

    return false;
}

static bool photo_web_has_mp4_suffix(const char *name)
{
    size_t len;

    if (!name)
    {
        return false;
    }

    len = strlen(name);
    return len >= 4 && strcasecmp(name + len - 4, ".mp4") == 0;
}

static bool photo_web_path_is_safe(const char *subpath)
{
    const char *p;

    if (!subpath || subpath[0] != '/' || subpath[1] == '\0')
    {
        return false;
    }

    if (strstr(subpath, "..") || strchr(subpath, '\\'))
    {
        return false;
    }

    /* 当前媒体命名规则只需要这些字符，拒绝其它字符可避免路径穿越和 URL 编码歧义。 */
    for (p = subpath + 1; *p; p++)
    {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'Z') ||
            (*p >= 'a' && *p <= 'z') ||
            *p == '/' || *p == '_' || *p == '-' || *p == '.')
        {
            continue;
        }
        return false;
    }

    return true;
}

static bool photo_web_relative_path_is_safe(const char *relative_path)
{
    const char *p;

    if (!relative_path || relative_path[0] == '\0' || relative_path[0] == '/')
    {
        return false;
    }

    if (strstr(relative_path, "..") || strchr(relative_path, '\\'))
    {
        return false;
    }

    for (p = relative_path; *p; p++)
    {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'Z') ||
            (*p >= 'a' && *p <= 'z') ||
            *p == '/' || *p == '_' || *p == '-' || *p == '.')
        {
            continue;
        }
        return false;
    }

    return true;
}

static bool photo_web_is_dir(const char *path)
{
    struct stat st = {0};

    return (stat(path, &st) == 0) && S_ISDIR(st.st_mode);
}

static bool photo_web_is_regular_file(const char *path, struct stat *out_st)
{
    struct stat st = {0};

    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode))
    {
        return false;
    }

    if (out_st)
    {
        *out_st = st;
    }
    return true;
}

static bool photo_web_dir_is_empty(const char *path)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;
    bool is_empty = true;

    if (!path || !photo_web_is_dir(path))
    {
        return false;
    }

    dir = opendir(path);
    if (!dir)
    {
        return false;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }
        is_empty = false;
        break;
    }

    closedir(dir);
    return is_empty;
}

static void photo_web_try_remove_empty_dir(const char *path)
{
    if (!path || !photo_web_dir_is_empty(path))
    {
        return;
    }

    if (remove(path) != 0 && errno != ENOENT)
    {
        ESP_LOGW(TAG, "删除空目录失败: %s, errno=%d", path, errno);
    }
}

static void photo_web_cleanup_empty_session_dirs(const char *relative_path, const char *subdir)
{
    char session_name[64] = {0};
    char session_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char media_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *slash = NULL;
    size_t session_len;

    if (!relative_path || !subdir)
    {
        return;
    }

    slash = strchr(relative_path, '/');
    if (!slash)
    {
        return;
    }

    session_len = (size_t)(slash - relative_path);
    if (session_len == 0U || session_len >= sizeof(session_name))
    {
        return;
    }

    memcpy(session_name, relative_path, session_len);
    session_name[session_len] = '\0';

    if (photo_web_join_path(session_dir, sizeof(session_dir),
                            tf_card_get_mount_point(), session_name) != ESP_OK)
    {
        return;
    }
    if (photo_web_join_path(media_dir, sizeof(media_dir), session_dir, subdir) != ESP_OK)
    {
        return;
    }

    photo_web_try_remove_empty_dir(media_dir);
    photo_web_try_remove_empty_dir(session_dir);
}

static void photo_web_set_no_cache(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
}

static esp_err_t photo_web_send_json_escaped(httpd_req_t *req, const char *text)
{
    char one_char[1];

    if (!text)
    {
        return ESP_OK;
    }

    for (const char *p = text; *p; p++)
    {
        switch (*p)
        {
        case '\\':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\\"), TAG, "发送 JSON 转义失败");
            break;
        case '"':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\""), TAG, "发送 JSON 转义失败");
            break;
        case '\n':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\n"), TAG, "发送 JSON 转义失败");
            break;
        case '\r':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\r"), TAG, "发送 JSON 转义失败");
            break;
        case '\t':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\t"), TAG, "发送 JSON 转义失败");
            break;
        default:
            one_char[0] = *p;
            ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(req, one_char, sizeof(one_char)),
                                TAG, "发送 JSON 字符失败");
            break;
        }
    }

    return ESP_OK;
}

static esp_err_t photo_web_send_json_error(httpd_req_t *req,
                                           const char *status, const char *message)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"error\":\""), TAG, "发送错误 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, message), TAG, "发送错误信息失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\"}"), TAG, "发送错误 JSON 失败");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_send_delete_result(httpd_req_t *req,
                                              uint32_t requested,
                                              uint32_t deleted,
                                              uint32_t failed)
{
    char resp[128];
    int resp_len;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    resp_len = snprintf(resp, sizeof(resp),
                        "{\"ok\":%s,\"requested\":%" PRIu32 ",\"deleted\":%" PRIu32 ",\"failed\":%" PRIu32 "}",
                        (failed == 0U) ? "true" : "false",
                        requested, deleted, failed);
    if (resp_len < 0 || resp_len >= (int)sizeof(resp))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return httpd_resp_sendstr(req, resp);
}

static bool photo_web_parse_bool_text(const char *text, bool *value)
{
    if (!text || !value) {
        return false;
    }

    if (strcmp(text, "1") == 0 || strcasecmp(text, "true") == 0) {
        *value = true;
        return true;
    }

    if (strcmp(text, "0") == 0 || strcasecmp(text, "false") == 0) {
        *value = false;
        return true;
    }

    return false;
}

static esp_err_t photo_web_form_get_text(const char *body, const char *key,
                                         char *value, size_t value_size)
{
    if (!body || !key || !value || value_size == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    value[0] = '\0';
    return (httpd_query_key_value(body, key, value, value_size) == ESP_OK) ?
           ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t photo_web_form_get_u32(const char *body, const char *key,
                                        uint32_t *value)
{
    char text[24] = {0};
    char *end = NULL;
    unsigned long parsed;

    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_form_get_text(body, key, text, sizeof(text)),
                        TAG, "读取表单参数失败");

    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    *value = (uint32_t)parsed;
    return ESP_OK;
}

static esp_netif_t *photo_web_get_active_netif(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (netif) {
        return netif;
    }

    return esp_netif_get_handle_from_ifkey("ETH_DEF");
}

static void photo_web_copy_text_or_default(char *dst, size_t dst_size, const char *text)
{
    if (!dst || dst_size == 0U) {
        return;
    }

    if (!text || text[0] == '\0') {
        snprintf(dst, dst_size, "--");
        return;
    }

    snprintf(dst, dst_size, "%s", text);
}

static void photo_web_fill_ip_texts(char *ip_text, size_t ip_text_size,
                                    char *gw_text, size_t gw_text_size,
                                    char *mask_text, size_t mask_text_size,
                                    char *rtsp_url, size_t rtsp_url_size)
{
    esp_netif_t *netif = photo_web_get_active_netif();
    esp_netif_ip_info_t ip_info = {0};

    photo_web_copy_text_or_default(ip_text, ip_text_size, NULL);
    photo_web_copy_text_or_default(gw_text, gw_text_size, NULL);
    photo_web_copy_text_or_default(mask_text, mask_text_size, NULL);
    photo_web_copy_text_or_default(rtsp_url, rtsp_url_size, NULL);

    if (!netif || esp_netif_get_ip_info(netif, &ip_info) != ESP_OK ||
        ip_info.ip.addr == 0U) {
        return;
    }

    snprintf(ip_text, ip_text_size, IPSTR, IP2STR(&ip_info.ip));
    snprintf(gw_text, gw_text_size, IPSTR, IP2STR(&ip_info.gw));
    snprintf(mask_text, mask_text_size, IPSTR, IP2STR(&ip_info.netmask));
    snprintf(rtsp_url, rtsp_url_size, "rtsp://" IPSTR ":%d/stream",
             IP2STR(&ip_info.ip), RTSP_PORT);
}

static bool photo_web_is_time_valid(time_t unix_sec)
{
    return unix_sec >= PHOTO_WEB_VALID_UNIX_SEC;
}

static void photo_web_build_current_time_text(char *time_text, size_t time_text_size,
                                              int64_t *unix_ms, bool *time_valid)
{
    struct timeval tv = {0};

    if (!time_text || time_text_size == 0U) {
        return;
    }

    gettimeofday(&tv, NULL);

    if (unix_ms) {
        *unix_ms = (int64_t)tv.tv_sec * 1000LL + (int64_t)(tv.tv_usec / 1000);
    }
    if (time_valid) {
        *time_valid = photo_web_is_time_valid((time_t)tv.tv_sec);
    }

    if (photo_web_is_time_valid((time_t)tv.tv_sec)) {
        struct tm tm_info = {0};
        time_t sec = (time_t)tv.tv_sec;

        localtime_r(&sec, &tm_info);
        snprintf(time_text, time_text_size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
                 (int)(tv.tv_usec / 1000));
    } else {
        snprintf(time_text, time_text_size, "时间未同步");
    }
}

static char *photo_web_trim_text(char *text)
{
    char *start = text;
    char *end;

    if (!text)
    {
        return NULL;
    }

    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n')
    {
        start++;
    }

    end = start + strlen(start);
    while (end > start &&
           (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
    {
        end--;
    }
    *end = '\0';
    return start;
}

static esp_err_t photo_web_read_request_body(httpd_req_t *req, char **body_out)
{
    char *body = NULL;
    int total_len;
    int recv_total = 0;

    if (!req || !body_out)
    {
        return ESP_ERR_INVALID_ARG;
    }

    total_len = req->content_len;
    if (total_len <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (total_len > PHOTO_WEB_DELETE_BODY_MAX_LEN)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    body = (char *)malloc((size_t)total_len + 1U);
    if (!body)
    {
        return ESP_ERR_NO_MEM;
    }

    while (recv_total < total_len)
    {
        int recv_len = httpd_req_recv(req, body + recv_total, (size_t)(total_len - recv_total));

        if (recv_len <= 0)
        {
            free(body);
            return ESP_FAIL;
        }
        recv_total += recv_len;
    }

    body[recv_total] = '\0';
    *body_out = body;
    return ESP_OK;
}

static esp_err_t photo_web_delete_media_file(const char *relative_path)
{
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *subdir = NULL;

    if (!relative_path || !photo_web_relative_path_is_safe(relative_path))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (strstr(relative_path, "/photo/") != NULL && photo_web_has_jpeg_suffix(relative_path))
    {
        subdir = "photo";
    }
    else if (strstr(relative_path, "/video/") != NULL && photo_web_has_mp4_suffix(relative_path))
    {
        subdir = "video";
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(file_path, sizeof(file_path),
                                            tf_card_get_mount_point(), relative_path),
                        TAG, "构建删除路径失败");

    if (!photo_web_is_regular_file(file_path, NULL))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (remove(file_path) != 0)
    {
        ESP_LOGW(TAG, "删除媒体文件失败: %s, errno=%d", file_path, errno);
        return ESP_FAIL;
    }

    photo_web_cleanup_empty_session_dirs(relative_path, subdir);
    return ESP_OK;
}

static esp_err_t photo_web_send_media_json(httpd_req_t *req, const char *session,
                                           const char *subdir, const char *file_name,
                                           const struct stat *file_st,
                                           const char *uri_prefix, bool *first_item)
{
    char relative_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char public_uri[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char size_text[32] = {0};
    uint64_t file_size = 0;

    if (!req || !session || !subdir || !file_name || !file_st || !uri_prefix || !first_item)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_build_relative_path(relative_path, sizeof(relative_path),
                                                      session, subdir, file_name),
                        TAG, "构建媒体相对路径失败");
    ESP_RETURN_ON_ERROR(photo_web_build_media_uri(public_uri, sizeof(public_uri),
                                                  uri_prefix, relative_path),
                        TAG, "构建媒体访问地址失败");

    if (file_st->st_size > 0)
    {
        file_size = (uint64_t)file_st->st_size;
    }
    snprintf(size_text, sizeof(size_text), "%" PRIu64, file_size);

    if (!*first_item)
    {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, ","), TAG, "发送 JSON 分隔符失败");
    }
    *first_item = false;

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"name\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, file_name), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"session\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, session), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"path\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, relative_path), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"url\":\""), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, public_uri), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"size_bytes\":"), TAG, "发送 JSON 失败");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, size_text), TAG, "发送 JSON 失败");
    return httpd_resp_sendstr_chunk(req, "}");
}

static esp_err_t photo_web_scan_media_session_dir(httpd_req_t *req, const char *session,
                                                  const char *subdir,
                                                  const char *media_name,
                                                  photo_web_suffix_check_t suffix_check,
                                                  const char *uri_prefix,
                                                  bool *first_item)
{
    esp_err_t ret;
    DIR *media_dir_handle = NULL;
    struct dirent *media_entry = NULL;
    char session_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char media_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};

    if (!req || !session || !subdir || !media_name || !suffix_check || !uri_prefix || !first_item)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(session_dir, sizeof(session_dir),
                                            tf_card_get_mount_point(), session),
                        TAG, "构建上电会话目录失败");
    if (!photo_web_is_dir(session_dir))
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(media_dir, sizeof(media_dir), session_dir, subdir),
                        TAG, "构建媒体目录失败");
    if (!photo_web_is_dir(media_dir))
    {
        return ESP_OK;
    }

    media_dir_handle = opendir(media_dir);
    if (!media_dir_handle)
    {
        ESP_LOGW(TAG, "打开%s目录失败: %s, errno=%d", media_name, media_dir, errno);
        return ESP_OK;
    }

    while ((media_entry = readdir(media_dir_handle)) != NULL)
    {
        struct stat file_st = {0};
        char media_path[PHOTO_WEB_MAX_PATH_LEN] = {0};

        if (!suffix_check(media_entry->d_name))
        {
            continue;
        }

        if (photo_web_join_path(media_path, sizeof(media_path),
                                media_dir, media_entry->d_name) != ESP_OK)
        {
            ESP_LOGW(TAG, "%s路径过长，跳过: %s/%s", media_name, media_dir, media_entry->d_name);
            continue;
        }

        if (!photo_web_is_regular_file(media_path, &file_st))
        {
            continue;
        }

        ret = photo_web_send_media_json(req, session, subdir, media_entry->d_name,
                                        &file_st, uri_prefix, first_item);
        if (ret != ESP_OK)
        {
            closedir(media_dir_handle);
            return ret;
        }
    }

    closedir(media_dir_handle);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* HTTP 处理函数                                                       */
/* ------------------------------------------------------------------ */
static esp_err_t photo_web_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    photo_web_set_no_cache(req);

    for (size_t i = 0; s_photo_index_html_v6[i] != NULL; i++)
    {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, s_photo_index_html_v6[i]),
                            TAG, "发送媒体网页失败");
    }

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_api_media_handler(httpd_req_t *req,
                                             const char *subdir,
                                             const char *media_name,
                                             photo_web_suffix_check_t suffix_check,
                                             const char *uri_prefix)
{
    esp_err_t ret;
    DIR *root_dir = NULL;
    struct dirent *entry = NULL;
    bool first_item = true;

    if (!req || !subdir || !media_name || !suffix_check || !uri_prefix)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted())
    {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    root_dir = opendir(tf_card_get_mount_point());
    if (!root_dir)
    {
        ESP_LOGE(TAG, "打开 TF 卡根目录失败: %s, errno=%d", tf_card_get_mount_point(), errno);
        return photo_web_send_json_error(req, "500 Internal Server Error", "无法读取 TF 卡目录");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    ret = httpd_resp_sendstr_chunk(req, "{\"items\":[");
    if (ret != ESP_OK)
    {
        closedir(root_dir);
        return ret;
    }

    while ((entry = readdir(root_dir)) != NULL)
    {
        if (entry->d_name[0] == '.')
        {
            continue;
        }

        ret = photo_web_scan_media_session_dir(req, entry->d_name, subdir, media_name,
                                               suffix_check, uri_prefix, &first_item);
        if (ret != ESP_OK)
        {
            closedir(root_dir);
            return ret;
        }
    }

    closedir(root_dir);
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "]}"), TAG, "发送 JSON 结束失败");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_api_photos_handler(httpd_req_t *req)
{
    return photo_web_api_media_handler(req, "photo", "照片",
                                       photo_web_has_jpeg_suffix, "/photo/");
}

static esp_err_t photo_web_api_videos_handler(httpd_req_t *req)
{
    return photo_web_api_media_handler(req, "video", "视频",
                                       photo_web_has_mp4_suffix, "/video/");
}

static esp_err_t photo_web_api_status_handler(httpd_req_t *req)
{
    char ip_text[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN] = {0};
    char gw_text[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN] = {0};
    char mask_text[DEVICE_WEB_CONFIG_IPV4_TEXT_LEN] = {0};
    char rtsp_url[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char time_text[PHOTO_WEB_STATUS_TEXT_LEN] = {0};
    char resp[640] = {0};
    int resp_len;
    int64_t unix_ms = 0;
    bool time_valid = false;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    photo_web_fill_ip_texts(ip_text, sizeof(ip_text),
                            gw_text, sizeof(gw_text),
                            mask_text, sizeof(mask_text),
                            rtsp_url, sizeof(rtsp_url));
    photo_web_build_current_time_text(time_text, sizeof(time_text), &unix_ms, &time_valid);

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    resp_len = snprintf(resp, sizeof(resp),
                        "{\"current_time\":\"%s\",\"current_unix_ms\":%" PRId64
                        ",\"time_valid\":%s,\"current_ip\":\"%s\",\"current_gw\":\"%s\""
                        ",\"current_mask\":\"%s\",\"rtsp_url\":\"%s\",\"tf_mounted\":%s"
                        ",\"active_clients\":%" PRIu32 "}",
                        time_text, unix_ms, time_valid ? "true" : "false",
                        ip_text, gw_text, mask_text, rtsp_url,
                        tf_card_is_mounted() ? "true" : "false",
                        rtsp_get_active_client_count());
    if (resp_len < 0 || resp_len >= (int)sizeof(resp)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return httpd_resp_send(req, resp, resp_len);
}

static esp_err_t photo_web_api_config_get_handler(httpd_req_t *req)
{
    device_web_config_t config = {0};
    char resp[512] = {0};
    int resp_len;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    device_web_config_get(&config);

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);

    resp_len = snprintf(resp, sizeof(resp),
                        "{\"uart0_baud_rate\":%" PRIu32 ",\"uart1_baud_rate\":%" PRIu32
                        ",\"video_profile\":%" PRIu32 ",\"video_profile_name\":\"%s\""
                        ",\"wifi_use_static_ip\":%s,\"wifi_static_ip\":\"%s\""
                        ",\"wifi_static_gw\":\"%s\",\"wifi_static_mask\":\"%s\"}",
                        config.uart0_baud_rate, config.uart1_baud_rate,
                        config.video_profile,
                        device_web_config_get_video_profile_name(config.video_profile),
                        config.wifi_use_static_ip ? "true" : "false",
                        config.wifi_static_ip, config.wifi_static_gw, config.wifi_static_mask);
    if (resp_len < 0 || resp_len >= (int)sizeof(resp)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return httpd_resp_send(req, resp, resp_len);
}

static esp_err_t photo_web_api_config_post_handler(httpd_req_t *req)
{
    char *body = NULL;
    char bool_text[8] = {0};
    bool wifi_use_static_ip = false;
    device_web_config_t config = {0};
    esp_err_t ret;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    if (req->content_len > PHOTO_WEB_CONFIG_BODY_MAX_LEN) {
        return photo_web_send_json_error(req, "413 Payload Too Large", "配置请求过大");
    }

    ret = photo_web_read_request_body(req, &body);
    if (ret == ESP_ERR_INVALID_ARG) {
        return photo_web_send_json_error(req, "400 Bad Request", "配置请求为空");
    }
    if (ret == ESP_ERR_INVALID_SIZE) {
        return photo_web_send_json_error(req, "413 Payload Too Large", "配置请求过大");
    }
    if (ret == ESP_ERR_NO_MEM) {
        return photo_web_send_json_error(req, "500 Internal Server Error", "内存不足");
    }
    if (ret != ESP_OK) {
        return photo_web_send_json_error(req, "400 Bad Request", "配置请求读取失败");
    }

    ret = photo_web_form_get_u32(body, "video_profile", &config.video_profile);
    if (ret == ESP_OK) {
        ret = photo_web_form_get_u32(body, "uart0_baud_rate", &config.uart0_baud_rate);
    }
    if (ret == ESP_OK) {
        ret = photo_web_form_get_u32(body, "uart1_baud_rate", &config.uart1_baud_rate);
    }
    if (ret == ESP_OK) {
        ret = photo_web_form_get_text(body, "wifi_use_static_ip", bool_text, sizeof(bool_text));
    }
    if (ret == ESP_OK && !photo_web_parse_bool_text(bool_text, &wifi_use_static_ip)) {
        ret = ESP_ERR_INVALID_ARG;
    }
    config.wifi_use_static_ip = wifi_use_static_ip;
    if (ret == ESP_OK) {
        ret = photo_web_form_get_text(body, "wifi_static_ip",
                                      config.wifi_static_ip, sizeof(config.wifi_static_ip));
    }
    if (ret == ESP_OK) {
        ret = photo_web_form_get_text(body, "wifi_static_gw",
                                      config.wifi_static_gw, sizeof(config.wifi_static_gw));
    }
    if (ret == ESP_OK) {
        ret = photo_web_form_get_text(body, "wifi_static_mask",
                                      config.wifi_static_mask, sizeof(config.wifi_static_mask));
    }

    free(body);

    if (ret != ESP_OK) {
        return photo_web_send_json_error(req, "400 Bad Request", "配置参数不完整或格式错误");
    }

    ret = device_web_config_save(&config);
    if (ret == ESP_ERR_INVALID_ARG) {
        return photo_web_send_json_error(req, "400 Bad Request", "配置参数非法或当前固件不支持该分辨率");
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存设备配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return photo_web_send_json_error(req, "500 Internal Server Error", "保存设备配置失败");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    return httpd_resp_sendstr(req, "{\"ok\":true,\"reboot_required\":true}");
}

static esp_err_t photo_web_api_time_handler(httpd_req_t *req)
{
    char query[PHOTO_WEB_TIME_QUERY_LEN] = {0};
    char unix_ms_text[PHOTO_WEB_TIME_VALUE_LEN] = {0};
    char *end = NULL;
    long long unix_ms = 0;
    size_t query_len;
    esp_err_t ret;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0U || query_len >= sizeof(query))
    {
        return photo_web_send_json_error(req, "400 Bad Request", "缺少时间参数");
    }

    ret = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (ret != ESP_OK ||
        httpd_query_key_value(query, "unix_ms", unix_ms_text, sizeof(unix_ms_text)) != ESP_OK)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数错误");
    }

    errno = 0;
    unix_ms = strtoll(unix_ms_text, &end, 10);
    if (errno != 0 || end == unix_ms_text || *end != '\0' || unix_ms <= 0)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数非法");
    }

    ret = media_storage_sync_time_from_unix_ms((int64_t)unix_ms);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "网页时间同步失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return photo_web_send_json_error(req, "400 Bad Request", "设备拒绝该时间");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t photo_web_api_factory_reset_handler(httpd_req_t *req)
{
    esp_err_t ret;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = device_web_config_reset_to_factory();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "恢复默认配置失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return photo_web_send_json_error(req, "500 Internal Server Error", "恢复默认配置失败");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    return httpd_resp_sendstr(req, "{\"ok\":true,\"reboot_required\":true}");
}

static esp_err_t photo_web_api_reboot_handler(httpd_req_t *req)
{
    esp_err_t ret;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    ret = httpd_resp_sendstr(req, "{\"ok\":true}");
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "收到网页重启请求，设备即将重启");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

static esp_err_t photo_web_api_capture_handler(httpd_req_t *req)
{
    esp_err_t ret;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted())
    {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    if (rtsp_get_active_client_count() == 0U)
    {
        return photo_web_send_json_error(req, "409 Conflict", "当前未在推流，无法拍照");
    }
    ret = media_storage_request_photo();
    if (ret == ESP_OK)
    {
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        photo_web_set_no_cache(req);
        ESP_LOGI(TAG, "网页拍照请求已受理");
        return httpd_resp_sendstr(req, "{\"ok\":true}");
    }

    if (ret == ESP_ERR_INVALID_STATE)
    {
        return photo_web_send_json_error(req, "409 Conflict", "当前已有拍照任务正在处理中");
    }

    if (ret == ESP_ERR_NO_MEM)
    {
        return photo_web_send_json_error(req, "500 Internal Server Error", "拍照后台任务初始化失败");
    }

    ESP_LOGW(TAG, "网页拍照请求提交失败: 0x%x (%s)", ret, esp_err_to_name(ret));
    return photo_web_send_json_error(req, "500 Internal Server Error", "拍照请求提交失败");
}

static esp_err_t photo_web_api_delete_handler(httpd_req_t *req)
{
    char *body = NULL;
    char *cursor = NULL;
    uint32_t requested = 0;
    uint32_t deleted = 0;
    uint32_t failed = 0;
    esp_err_t ret;

    if (!req)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted())
    {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    ret = photo_web_read_request_body(req, &body);
    if (ret == ESP_ERR_INVALID_ARG)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "删除请求为空");
    }
    if (ret == ESP_ERR_INVALID_SIZE)
    {
        return photo_web_send_json_error(req, "413 Payload Too Large", "删除请求过大");
    }
    if (ret == ESP_ERR_NO_MEM)
    {
        return photo_web_send_json_error(req, "500 Internal Server Error", "内存不足");
    }
    if (ret != ESP_OK)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "删除请求读取失败");
    }

    cursor = body;
    while (cursor && *cursor != '\0')
    {
        char *line_end = strpbrk(cursor, "\r\n");
        char *line = cursor;

        if (line_end)
        {
            *line_end = '\0';
            cursor = line_end + 1;
            while (*cursor == '\r' || *cursor == '\n')
            {
                cursor++;
            }
        }
        else
        {
            cursor = NULL;
        }

        line = photo_web_trim_text(line);
        if (!line || line[0] == '\0')
        {
            continue;
        }

        requested++;
        ret = photo_web_delete_media_file(line);
        if (ret == ESP_OK)
        {
            deleted++;
        }
        else
        {
            failed++;
            ESP_LOGW(TAG, "网页删除失败 | 路径=%s | ret=0x%x (%s)",
                     line, ret, esp_err_to_name(ret));
        }
    }

    free(body);

    if (requested == 0U)
    {
        return photo_web_send_json_error(req, "400 Bad Request", "未提供待删除路径");
    }

    ESP_LOGI(TAG, "网页删除完成 | 请求=%" PRIu32 " | 成功=%" PRIu32 " | 失败=%" PRIu32,
             requested, deleted, failed);
    return photo_web_send_delete_result(req, requested, deleted, failed);
}

static bool photo_web_parse_u64_value(const char **cursor, uint64_t *value)
{
    uint64_t result = 0;
    const char *p;
    bool has_digit = false;

    if (!cursor || !*cursor || !value)
    {
        return false;
    }

    p = *cursor;
    while (*p >= '0' && *p <= '9')
    {
        uint64_t digit = (uint64_t)(*p - '0');

        if (result > (UINT64_MAX - digit) / 10U)
        {
            return false;
        }
        result = result * 10U + digit;
        has_digit = true;
        p++;
    }

    if (!has_digit)
    {
        return false;
    }

    *cursor = p;
    *value = result;
    return true;
}

static esp_err_t photo_web_parse_range_request(httpd_req_t *req, uint64_t file_size,
                                               photo_web_range_t *range)
{
    char range_header[PHOTO_WEB_RANGE_HEADER_LEN] = {0};
    size_t header_len;
    const char *p;

    if (!req || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    range->partial = false;
    range->start = 0;
    range->end = (file_size > 0U) ? (file_size - 1U) : 0U;
    range->length = file_size;

    header_len = httpd_req_get_hdr_value_len(req, "Range");
    if (header_len == 0U)
    {
        return ESP_OK;
    }
    if (header_len >= sizeof(range_header))
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (httpd_req_get_hdr_value_str(req, "Range", range_header, sizeof(range_header)) != ESP_OK)
    {
        return ESP_ERR_INVALID_ARG;
    }

    p = range_header;
    if (strncmp(p, "bytes=", strlen("bytes=")) != 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    p += strlen("bytes=");

    if (*p == '-')
    {
        uint64_t suffix_len = 0;

        p++;
        if (!photo_web_parse_u64_value(&p, &suffix_len) || *p != '\0' ||
            suffix_len == 0U || file_size == 0U)
        {
            return ESP_ERR_INVALID_ARG;
        }

        range->start = (suffix_len >= file_size) ? 0U : (file_size - suffix_len);
        range->end = file_size - 1U;
    }
    else
    {
        uint64_t start = 0;
        uint64_t end = 0;

        if (!photo_web_parse_u64_value(&p, &start) || *p != '-')
        {
            return ESP_ERR_INVALID_ARG;
        }
        p++;

        if (*p == '\0')
        {
            if (file_size == 0U || start >= file_size)
            {
                return ESP_ERR_INVALID_ARG;
            }
            end = file_size - 1U;
        }
        else
        {
            if (!photo_web_parse_u64_value(&p, &end) || *p != '\0' ||
                file_size == 0U || start >= file_size || end < start)
            {
                return ESP_ERR_INVALID_ARG;
            }
            if (end >= file_size)
            {
                end = file_size - 1U;
            }
        }

        range->start = start;
        range->end = end;
    }

    range->partial = true;
    range->length = range->end - range->start + 1U;
    return ESP_OK;
}

static void photo_web_close_request_session(httpd_req_t *req)
{
    int sockfd;

    if (!req)
    {
        return;
    }

    sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0)
    {
        httpd_sess_trigger_close(req->handle, sockfd);
    }
}

static esp_err_t photo_web_raw_send_all(httpd_req_t *req, const char *buf, size_t len)
{
    size_t sent_total = 0;

    if (!req || (!buf && len > 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    while (sent_total < len)
    {
        int sent = httpd_send(req, buf + sent_total, len - sent_total);
        if (sent <= 0)
        {
            photo_web_close_request_session(req);
            return ESP_ERR_HTTPD_RESP_SEND;
        }
        sent_total += (size_t)sent;
    }

    return ESP_OK;
}

static esp_err_t photo_web_send_range_error(httpd_req_t *req, uint64_t file_size)
{
    char header[PHOTO_WEB_HTTP_HEADER_LEN];
    int header_len;

    header_len = snprintf(header, sizeof(header),
                          "HTTP/1.1 416 Range Not Satisfiable\r\n"
                          "Content-Type: text/plain\r\n"
                          "Content-Length: 0\r\n"
                          "Accept-Ranges: bytes\r\n"
                          "Content-Range: bytes */%" PRIu64 "\r\n"
                          "Cache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n",
                          file_size);
    if (header_len < 0 || header_len >= (int)sizeof(header))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return photo_web_raw_send_all(req, header, (size_t)header_len);
}

static esp_err_t photo_web_send_video_header(httpd_req_t *req, const photo_web_range_t *range,
                                             uint64_t file_size)
{
    char header[PHOTO_WEB_HTTP_HEADER_LEN];
    int header_len;

    if (!req || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->partial)
    {
        header_len = snprintf(header, sizeof(header),
                              "HTTP/1.1 206 Partial Content\r\n"
                              "Content-Type: video/mp4\r\n"
                              "Content-Length: %" PRIu64 "\r\n"
                              "Accept-Ranges: bytes\r\n"
                              "Content-Range: bytes %" PRIu64 "-%" PRIu64 "/%" PRIu64 "\r\n"
                              "Cache-Control: no-store\r\n"
                              "Connection: close\r\n"
                              "Content-Disposition: inline\r\n\r\n",
                              range->length, range->start, range->end, file_size);
    }
    else
    {
        header_len = snprintf(header, sizeof(header),
                              "HTTP/1.1 200 OK\r\n"
                              "Content-Type: video/mp4\r\n"
                              "Content-Length: %" PRIu64 "\r\n"
                              "Accept-Ranges: bytes\r\n"
                              "Cache-Control: no-store\r\n"
                              "Connection: close\r\n"
                              "Content-Disposition: inline\r\n\r\n",
                              range->length);
    }

    if (header_len < 0 || header_len >= (int)sizeof(header))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return photo_web_raw_send_all(req, header, (size_t)header_len);
}

static esp_err_t photo_web_send_video_body(httpd_req_t *req, FILE *fp,
                                           const char *file_path,
                                           const photo_web_range_t *range)
{
    char file_buf[PHOTO_WEB_FILE_CHUNK_SIZE];
    uint64_t remain;

    if (!req || !fp || !file_path || !range)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->start > 0U)
    {
        if (range->start > (uint64_t)LONG_MAX ||
            fseek(fp, (long)range->start, SEEK_SET) != 0)
        {
            ESP_LOGE(TAG, "定位视频文件失败: %s, offset=%" PRIu64 ", errno=%d",
                     file_path, range->start, errno);
            return ESP_FAIL;
        }
    }

    remain = range->length;
    while (remain > 0U)
    {
        size_t want_len = (remain > sizeof(file_buf)) ? sizeof(file_buf) : (size_t)remain;
        size_t read_len = fread(file_buf, 1, want_len, fp);

        if (read_len == 0U)
        {
            ESP_LOGE(TAG, "读取视频文件失败: %s", file_path);
            return ESP_FAIL;
        }

        esp_err_t ret = photo_web_raw_send_all(req, file_buf, read_len);
        if (ret != ESP_OK)
        {
            return ret;
        }
        remain -= (uint64_t)read_len;
    }

    return ESP_OK;
}

static esp_err_t photo_web_photo_handler(httpd_req_t *req)
{
    FILE *fp = NULL;
    struct stat file_st = {0};
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char file_buf[PHOTO_WEB_FILE_CHUNK_SIZE];
    const char *subpath = req->uri + strlen("/photo");
    size_t offset = 0;
    esp_err_t ret = ESP_OK;

    if (!tf_card_is_mounted())
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/photo/") == NULL ||
        !photo_web_has_jpeg_suffix(subpath))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "照片路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                              &offset, tf_card_get_mount_point()),
                        TAG, "构建本地照片路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地照片路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st))
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "照片不存在");
    }

    fp = fopen(file_path, "rb");
    if (!fp)
    {
        ESP_LOGE(TAG, "打开照片文件失败: %s, errno=%d", file_path, errno);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开照片失败");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline");

    while (1)
    {
        size_t read_len = fread(file_buf, 1, sizeof(file_buf), fp);
        if (read_len > 0)
        {
            ret = httpd_resp_send_chunk(req, file_buf, read_len);
            if (ret != ESP_OK)
            {
                fclose(fp);
                photo_web_close_request_session(req);
                return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
            }
        }

        if (read_len < sizeof(file_buf))
        {
            if (ferror(fp))
            {
                ESP_LOGE(TAG, "读取照片文件失败: %s", file_path);
                fclose(fp);
                return ESP_FAIL;
            }
            break;
        }
    }

    fclose(fp);
    ret = httpd_resp_send_chunk(req, NULL, 0);
    if (ret != ESP_OK)
    {
        photo_web_close_request_session(req);
    }
    return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
}

static esp_err_t photo_web_video_handler(httpd_req_t *req)
{
    FILE *fp = NULL;
    struct stat file_st = {0};
    char file_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    const char *subpath = req->uri + strlen("/video");
    size_t offset = 0;
    uint64_t file_size = 0;
    photo_web_range_t range = {0};
    esp_err_t ret;
    bool send_body;

    if (!tf_card_is_mounted())
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/video/") == NULL ||
        !photo_web_has_mp4_suffix(subpath))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "视频路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                              &offset, tf_card_get_mount_point()),
                        TAG, "构建本地视频路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地视频路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st))
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "视频不存在");
    }

    if (file_st.st_size > 0)
    {
        file_size = (uint64_t)file_st.st_size;
    }

    ret = photo_web_parse_range_request(req, file_size, &range);
    if (ret != ESP_OK)
    {
        ret = photo_web_send_range_error(req, file_size);
        photo_web_close_request_session(req);
        return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
    }

    send_body = (req->method != HTTP_HEAD);
    if (send_body)
    {
        fp = fopen(file_path, "rb");
        if (!fp)
        {
            ESP_LOGE(TAG, "打开视频文件失败: %s, errno=%d", file_path, errno);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开视频失败");
        }
    }

    ret = photo_web_send_video_header(req, &range, file_size);
    if (ret == ESP_OK && send_body)
    {
        ret = photo_web_send_video_body(req, fp, file_path, &range);
    }

    if (fp)
    {
        fclose(fp);
    }

    photo_web_close_request_session(req);
    return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
}

static esp_err_t photo_web_favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
esp_err_t photo_web_server_start(void)
{
    esp_err_t ret;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (s_photo_web.server)
    {
        return ESP_OK;
    }

    config.server_port = PHOTO_WEB_SERVER_PORT;
    config.stack_size = PHOTO_WEB_SERVER_STACK_SIZE;
    config.max_open_sockets = 6;
    config.max_uri_handlers = 16;
    config.backlog_conn = 4;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ret = httpd_start(&s_photo_web.server, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "启动媒体网页服务失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return ret;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = photo_web_index_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_photos_uri = {
        .uri = "/api/photos",
        .method = HTTP_GET,
        .handler = photo_web_api_photos_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_videos_uri = {
        .uri = "/api/videos",
        .method = HTTP_GET,
        .handler = photo_web_api_videos_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = photo_web_api_status_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_config_get_uri = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = photo_web_api_config_get_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_config_post_uri = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = photo_web_api_config_post_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_time_uri = {
        .uri = "/api/time",
        .method = HTTP_GET,
        .handler = photo_web_api_time_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_capture_uri = {
        .uri = "/api/capture",
        .method = HTTP_POST,
        .handler = photo_web_api_capture_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_delete_uri = {
        .uri = "/api/delete",
        .method = HTTP_POST,
        .handler = photo_web_api_delete_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_factory_reset_uri = {
        .uri = "/api/factory_reset",
        .method = HTTP_POST,
        .handler = photo_web_api_factory_reset_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t api_reboot_uri = {
        .uri = "/api/reboot",
        .method = HTTP_POST,
        .handler = photo_web_api_reboot_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t photo_uri = {
        .uri = "/photo/*",
        .method = HTTP_GET,
        .handler = photo_web_photo_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t video_uri = {
        .uri = "/video/*",
        .method = HTTP_GET,
        .handler = photo_web_video_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t video_head_uri = {
        .uri = "/video/*",
        .method = HTTP_HEAD,
        .handler = photo_web_video_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = photo_web_favicon_handler,
        .user_ctx = NULL,
    };

    ret = httpd_register_uri_handler(s_photo_web.server, &index_uri);
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_photos_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_videos_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_status_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_config_get_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_config_post_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_time_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_factory_reset_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_reboot_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_capture_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_delete_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &photo_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_head_uri);
    }
    if (ret == ESP_OK)
    {
        ret = httpd_register_uri_handler(s_photo_web.server, &favicon_uri);
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "注册媒体网页 URI 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        photo_web_server_stop();
        return ret;
    }

    ESP_LOGI(TAG, "媒体网页服务已启动，端口 %d", PHOTO_WEB_SERVER_PORT);
    return ESP_OK;
}

void photo_web_server_stop(void)
{
    if (!s_photo_web.server)
    {
        return;
    }

    httpd_stop(s_photo_web.server);
    s_photo_web.server = NULL;
    ESP_LOGI(TAG, "媒体网页服务已停止");
}
