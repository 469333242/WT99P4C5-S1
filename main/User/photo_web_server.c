/**
 * @file photo_web_server.c
 * @brief SD 卡媒体网页浏览模块实现
 *
 * 模块职责：
 *   - 通过 HTTP 服务提供照片和视频浏览网页
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

#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "media_storage.h"
#include "photo_web_server.h"
#include "tf_card.h"

static const char *TAG = "photo_web";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define PHOTO_WEB_MAX_PATH_LEN       256
#define PHOTO_WEB_FILE_CHUNK_SIZE    2048
#define PHOTO_WEB_RANGE_HEADER_LEN   96
#define PHOTO_WEB_HTTP_HEADER_LEN    512
#define PHOTO_WEB_TIME_QUERY_LEN     96
#define PHOTO_WEB_TIME_VALUE_LEN     24
#define PHOTO_WEB_SERVER_STACK_SIZE  (8 * 1024)

typedef struct {
    httpd_handle_t server;
} photo_web_ctx_t;

typedef struct {
    bool partial;
    uint64_t start;
    uint64_t end;
    uint64_t length;
} photo_web_range_t;

typedef bool (*photo_web_suffix_check_t)(const char *name);

static photo_web_ctx_t s_photo_web;

/* ------------------------------------------------------------------ */
/* 网页内容                                                            */
/* ------------------------------------------------------------------ */
static const char *s_photo_index_html[] = {
    "<!doctype html><html lang=\"zh-CN\"><head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>SD 卡媒体浏览</title>"
    "<style>"
    ":root{--bg:#f4efe7;--panel:#fffaf2;--ink:#1b1b1b;--muted:#72685b;--accent:#c65a1e;--line:#e7d8c8;}"
    "*{box-sizing:border-box}body{margin:0;font-family:'Segoe UI','Microsoft YaHei',sans-serif;"
    "background:radial-gradient(circle at top,#fff7ec 0,#f4efe7 42%,#efe6db 100%);color:var(--ink);}"
    ".shell{max-width:1200px;margin:0 auto;padding:24px 16px 40px;}"
    ".hero{display:flex;flex-wrap:wrap;gap:16px;align-items:flex-end;justify-content:space-between;margin-bottom:18px;}"
    ".title{margin:0;font-size:clamp(28px,5vw,44px);letter-spacing:.04em;}"
    ".desc{margin:8px 0 0;color:var(--muted);font-size:14px;}"
    ".toolbar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;}"
    ".badge{padding:10px 14px;border:1px solid var(--line);border-radius:999px;background:rgba(255,250,242,.88);font-size:13px;color:var(--muted);}"
    "button{border:0;border-radius:999px;background:linear-gradient(135deg,#db6d2e,#b94a14);color:#fff;padding:12px 18px;font-size:14px;font-weight:600;cursor:pointer;box-shadow:0 14px 30px rgba(185,74,20,.18);}"
    "button:disabled{opacity:.6;cursor:default}"
    ".media{display:grid;gap:18px}.panel{background:rgba(255,250,242,.84);border:1px solid rgba(231,216,200,.9);border-radius:24px;padding:18px;backdrop-filter:blur(8px);box-shadow:0 18px 42px rgba(74,39,16,.08);}"
    ".sectionHead{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:0 0 14px}.sectionHead h2{margin:0;font-size:20px}.count{font-size:12px;color:var(--muted)}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:18px;}"
    ".card{overflow:hidden;border-radius:20px;background:#fff;border:1px solid #eadccc;display:flex;flex-direction:column;min-height:280px;}"
    ".thumb{display:block;aspect-ratio:4/3;background:linear-gradient(135deg,#f9dcc0,#f3b183);overflow:hidden;}"
    ".thumb img,.thumb video{display:block;width:100%;height:100%;object-fit:cover;transition:transform .25s ease;background:#14110e;}"
    ".card:hover .thumb img{transform:scale(1.03)}"
    ".videoGrid .card{min-height:0}.videoGrid .thumb{background:linear-gradient(135deg,#2d2720,#7f4b26)}"
    ".meta{padding:14px 14px 16px;display:grid;gap:8px;}"
    ".name{font-size:13px;font-weight:700;word-break:break-all;}"
    ".sub{font-size:12px;color:var(--muted);word-break:break-all;}"
    ".empty{grid-column:1/-1;padding:42px 18px;text-align:center;color:var(--muted);border:1px dashed var(--line);border-radius:18px;background:rgba(255,255,255,.65);}"
    "a.open{color:var(--accent);text-decoration:none;font-size:12px;font-weight:700;}"
    "@media (max-width:640px){.shell{padding:18px 12px 32px}.panel{padding:14px}.grid{grid-template-columns:1fr 1fr;gap:12px}.card{min-height:0}}"
    "@media (max-width:460px){.grid{grid-template-columns:1fr}}"
    "</style></head><body><main class=\"shell\">"
    "<section class=\"hero\"><div><h1 class=\"title\">SD 卡媒体浏览</h1>"
    "<p class=\"desc\">实时读取设备 TF 卡中已保存的照片和录像，视频点击后才开始加载播放。</p></div>"
    "<div class=\"toolbar\"><div id=\"status\" class=\"badge\">正在读取媒体列表...</div>"
    "<button id=\"reload\" type=\"button\">刷新列表</button></div></section>"
    "<section class=\"media\">"
    "<section class=\"panel\"><div class=\"sectionHead\"><h2>照片</h2><div id=\"photoCount\" class=\"count\">0 张</div></div><div id=\"photoGrid\" class=\"grid\"></div></section>"
    "<section class=\"panel\"><div class=\"sectionHead\"><h2>视频</h2><div id=\"videoCount\" class=\"count\">0 段</div></div><div id=\"videoGrid\" class=\"grid videoGrid\"></div></section>"
    "</section></main>",

    "<script>"
    "const statusEl=document.getElementById('status');"
    "const photoGridEl=document.getElementById('photoGrid');"
    "const videoGridEl=document.getElementById('videoGrid');"
    "const photoCountEl=document.getElementById('photoCount');"
    "const videoCountEl=document.getElementById('videoCount');"
    "const reloadBtn=document.getElementById('reload');"
    "function formatSize(size){if(!Number.isFinite(size)||size<=0)return '--';if(size<1024)return size+' B';if(size<1024*1024)return (size/1024).toFixed(1)+' KB';return (size/1024/1024).toFixed(2)+' MB';}"
    "function setEmpty(grid,message){grid.innerHTML='';const box=document.createElement('div');box.className='empty';box.textContent=message;grid.appendChild(box);}"
    "function appendText(cls,text,parent){const node=document.createElement('div');node.className=cls;node.textContent=text;parent.appendChild(node);}"
    "function sortItems(items){items.sort((a,b)=>String(b.path).localeCompare(String(a.path)));return items;}"
    "function renderPhotos(items){photoGridEl.innerHTML='';photoCountEl.textContent=items.length+' 张';if(!items.length){setEmpty(photoGridEl,'TF 卡中暂无照片');return;}sortItems(items);"
    "for(const item of items){const card=document.createElement('article');card.className='card';"
    "const thumb=document.createElement('a');thumb.className='thumb';thumb.href=item.url;thumb.target='_blank';thumb.rel='noreferrer';"
    "const img=document.createElement('img');img.loading='lazy';img.src=item.url;img.alt=item.name||'photo';thumb.appendChild(img);"
    "const meta=document.createElement('div');meta.className='meta';"
    "appendText('name',item.name||'',meta);appendText('sub','目录：'+(item.session||''),meta);appendText('sub','大小：'+formatSize(Number(item.size_bytes)),meta);"
    "const open=document.createElement('a');open.className='open';open.href=item.url;open.target='_blank';open.rel='noreferrer';open.textContent='打开原图';meta.appendChild(open);"
    "card.appendChild(thumb);card.appendChild(meta);photoGridEl.appendChild(card);}}"
    "function renderVideos(items){videoGridEl.innerHTML='';videoCountEl.textContent=items.length+' 段';if(!items.length){setEmpty(videoGridEl,'TF 卡中暂无视频');return;}sortItems(items);"
    "for(const item of items){const card=document.createElement('article');card.className='card';"
    "const thumb=document.createElement('div');thumb.className='thumb';const video=document.createElement('video');video.controls=true;video.preload='none';video.src=item.url;video.setAttribute('playsinline','');thumb.appendChild(video);"
    "const meta=document.createElement('div');meta.className='meta';appendText('name',item.name||'',meta);appendText('sub','目录：'+(item.session||''),meta);appendText('sub','大小：'+formatSize(Number(item.size_bytes)),meta);"
    "const open=document.createElement('a');open.className='open';open.href=item.url;open.target='_blank';open.rel='noreferrer';open.textContent='打开视频';meta.appendChild(open);"
    "card.appendChild(thumb);card.appendChild(meta);videoGridEl.appendChild(card);}}"
    "async function fetchList(url){const response=await fetch(url,{cache:'no-store'});let data={};try{data=await response.json();}catch(e){}if(!response.ok){throw new Error(data.error||('HTTP '+response.status));}return Array.isArray(data.items)?data.items:[];}"
    "async function syncDeviceTime(){try{await fetch('/api/time?unix_ms='+Date.now(),{cache:'no-store'});}catch(e){}}"
    "async function loadMedia(){reloadBtn.disabled=true;statusEl.textContent='正在同步设备时间...';"
    "try{await syncDeviceTime();statusEl.textContent='正在读取媒体列表...';const result=await Promise.all([fetchList('/api/photos'),fetchList('/api/videos')]);renderPhotos(result[0]);renderVideos(result[1]);statusEl.textContent='照片 '+result[0].length+' 张，视频 '+result[1].length+' 段';}"
    "catch(error){setEmpty(photoGridEl,error.message||'读取失败');setEmpty(videoGridEl,'请刷新重试');statusEl.textContent='媒体列表读取失败';}"
    "finally{reloadBtn.disabled=false;}}"
    "reloadBtn.addEventListener('click',loadMedia);window.addEventListener('DOMContentLoaded',loadMedia);"
    "</script></body></html>",
    NULL,
};

/* ------------------------------------------------------------------ */
/* 内部工具函数                                                        */
/* ------------------------------------------------------------------ */
static esp_err_t photo_web_append_text(char *dst, size_t dst_size,
                                       size_t *offset, const char *text)
{
    size_t text_len;

    if (!dst || dst_size == 0 || !offset || !text) {
        return ESP_ERR_INVALID_ARG;
    }

    text_len = strlen(text);
    if (*offset + text_len >= dst_size) {
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

    if (!dst || dst_size == 0 || !dir || !name) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, dir);
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
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

    if (!dst || dst_size == 0 || !session || !subdir || !file_name) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, session);
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, subdir);
    }
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, "/");
    }
    if (ret == ESP_OK) {
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

    if (!dst || dst_size == 0 || !uri_prefix || !relative_path) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, uri_prefix);
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, relative_path);
    }

    return ret;
}

static bool photo_web_has_jpeg_suffix(const char *name)
{
    size_t len;

    if (!name) {
        return false;
    }

    len = strlen(name);
    if (len >= 5 && strcasecmp(name + len - 5, ".jpeg") == 0) {
        return true;
    }
    if (len >= 4 && strcasecmp(name + len - 4, ".jpg") == 0) {
        return true;
    }

    return false;
}

static bool photo_web_has_mp4_suffix(const char *name)
{
    size_t len;

    if (!name) {
        return false;
    }

    len = strlen(name);
    return len >= 4 && strcasecmp(name + len - 4, ".mp4") == 0;
}

static bool photo_web_path_is_safe(const char *subpath)
{
    const char *p;

    if (!subpath || subpath[0] != '/' || subpath[1] == '\0') {
        return false;
    }

    if (strstr(subpath, "..") || strchr(subpath, '\\')) {
        return false;
    }

    /* 当前媒体命名规则只需要这些字符，拒绝其它字符可避免路径穿越和 URL 编码歧义。 */
    for (p = subpath + 1; *p; p++) {
        if ((*p >= '0' && *p <= '9') ||
            (*p >= 'A' && *p <= 'Z') ||
            (*p >= 'a' && *p <= 'z') ||
            *p == '/' || *p == '_' || *p == '-' || *p == '.') {
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

    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        return false;
    }

    if (out_st) {
        *out_st = st;
    }
    return true;
}

static void photo_web_set_no_cache(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
}

static esp_err_t photo_web_send_json_escaped(httpd_req_t *req, const char *text)
{
    char one_char[1];

    if (!text) {
        return ESP_OK;
    }

    for (const char *p = text; *p; p++) {
        switch (*p) {
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

static esp_err_t photo_web_send_media_json(httpd_req_t *req, const char *session,
                                           const char *subdir, const char *file_name,
                                           const struct stat *file_st,
                                           const char *uri_prefix, bool *first_item)
{
    char relative_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char public_uri[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char size_text[32] = {0};
    uint64_t file_size = 0;

    if (!req || !session || !subdir || !file_name || !file_st || !uri_prefix || !first_item) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_build_relative_path(relative_path, sizeof(relative_path),
                                                     session, subdir, file_name),
                        TAG, "构建媒体相对路径失败");
    ESP_RETURN_ON_ERROR(photo_web_build_media_uri(public_uri, sizeof(public_uri),
                                                 uri_prefix, relative_path),
                        TAG, "构建媒体访问地址失败");

    if (file_st->st_size > 0) {
        file_size = (uint64_t)file_st->st_size;
    }
    snprintf(size_text, sizeof(size_text), "%" PRIu64, file_size);

    if (!*first_item) {
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

    if (!req || !session || !subdir || !media_name || !suffix_check || !uri_prefix || !first_item) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(session_dir, sizeof(session_dir),
                                           tf_card_get_mount_point(), session),
                        TAG, "构建上电会话目录失败");
    if (!photo_web_is_dir(session_dir)) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(media_dir, sizeof(media_dir), session_dir, subdir),
                        TAG, "构建媒体目录失败");
    if (!photo_web_is_dir(media_dir)) {
        return ESP_OK;
    }

    media_dir_handle = opendir(media_dir);
    if (!media_dir_handle) {
        ESP_LOGW(TAG, "打开%s目录失败: %s, errno=%d", media_name, media_dir, errno);
        return ESP_OK;
    }

    while ((media_entry = readdir(media_dir_handle)) != NULL) {
        struct stat file_st = {0};
        char media_path[PHOTO_WEB_MAX_PATH_LEN] = {0};

        if (!suffix_check(media_entry->d_name)) {
            continue;
        }

        if (photo_web_join_path(media_path, sizeof(media_path),
                                media_dir, media_entry->d_name) != ESP_OK) {
            ESP_LOGW(TAG, "%s路径过长，跳过: %s/%s", media_name, media_dir, media_entry->d_name);
            continue;
        }

        if (!photo_web_is_regular_file(media_path, &file_st)) {
            continue;
        }

        ret = photo_web_send_media_json(req, session, subdir, media_entry->d_name,
                                        &file_st, uri_prefix, first_item);
        if (ret != ESP_OK) {
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

    for (size_t i = 0; s_photo_index_html[i] != NULL; i++) {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, s_photo_index_html[i]),
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

    if (!req || !subdir || !media_name || !suffix_check || !uri_prefix) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tf_card_is_mounted()) {
        return photo_web_send_json_error(req, "503 Service Unavailable", "TF 卡未挂载");
    }

    root_dir = opendir(tf_card_get_mount_point());
    if (!root_dir) {
        ESP_LOGE(TAG, "打开 TF 卡根目录失败: %s, errno=%d", tf_card_get_mount_point(), errno);
        return photo_web_send_json_error(req, "500 Internal Server Error", "无法读取 TF 卡目录");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    ret = httpd_resp_sendstr_chunk(req, "{\"items\":[");
    if (ret != ESP_OK) {
        closedir(root_dir);
        return ret;
    }

    while ((entry = readdir(root_dir)) != NULL) {
        if (entry->d_name[0] == '.') {
            continue;
        }

        ret = photo_web_scan_media_session_dir(req, entry->d_name, subdir, media_name,
                                              suffix_check, uri_prefix, &first_item);
        if (ret != ESP_OK) {
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

static esp_err_t photo_web_api_time_handler(httpd_req_t *req)
{
    char query[PHOTO_WEB_TIME_QUERY_LEN] = {0};
    char unix_ms_text[PHOTO_WEB_TIME_VALUE_LEN] = {0};
    char *end = NULL;
    long long unix_ms = 0;
    size_t query_len;
    esp_err_t ret;

    if (!req) {
        return ESP_ERR_INVALID_ARG;
    }

    query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0U || query_len >= sizeof(query)) {
        return photo_web_send_json_error(req, "400 Bad Request", "缺少时间参数");
    }

    ret = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (ret != ESP_OK ||
        httpd_query_key_value(query, "unix_ms", unix_ms_text, sizeof(unix_ms_text)) != ESP_OK) {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数错误");
    }

    errno = 0;
    unix_ms = strtoll(unix_ms_text, &end, 10);
    if (errno != 0 || end == unix_ms_text || *end != '\0' || unix_ms <= 0) {
        return photo_web_send_json_error(req, "400 Bad Request", "时间参数非法");
    }

    ret = media_storage_sync_time_from_unix_ms((int64_t)unix_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "网页时间同步失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        return photo_web_send_json_error(req, "400 Bad Request", "设备拒绝该时间");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    photo_web_set_no_cache(req);
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static bool photo_web_parse_u64_value(const char **cursor, uint64_t *value)
{
    uint64_t result = 0;
    const char *p;
    bool has_digit = false;

    if (!cursor || !*cursor || !value) {
        return false;
    }

    p = *cursor;
    while (*p >= '0' && *p <= '9') {
        uint64_t digit = (uint64_t)(*p - '0');

        if (result > (UINT64_MAX - digit) / 10U) {
            return false;
        }
        result = result * 10U + digit;
        has_digit = true;
        p++;
    }

    if (!has_digit) {
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

    if (!req || !range) {
        return ESP_ERR_INVALID_ARG;
    }

    range->partial = false;
    range->start = 0;
    range->end = (file_size > 0U) ? (file_size - 1U) : 0U;
    range->length = file_size;

    header_len = httpd_req_get_hdr_value_len(req, "Range");
    if (header_len == 0U) {
        return ESP_OK;
    }
    if (header_len >= sizeof(range_header)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (httpd_req_get_hdr_value_str(req, "Range", range_header, sizeof(range_header)) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }

    p = range_header;
    if (strncmp(p, "bytes=", strlen("bytes=")) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    p += strlen("bytes=");

    if (*p == '-') {
        uint64_t suffix_len = 0;

        p++;
        if (!photo_web_parse_u64_value(&p, &suffix_len) || *p != '\0' ||
            suffix_len == 0U || file_size == 0U) {
            return ESP_ERR_INVALID_ARG;
        }

        range->start = (suffix_len >= file_size) ? 0U : (file_size - suffix_len);
        range->end = file_size - 1U;
    } else {
        uint64_t start = 0;
        uint64_t end = 0;

        if (!photo_web_parse_u64_value(&p, &start) || *p != '-') {
            return ESP_ERR_INVALID_ARG;
        }
        p++;

        if (*p == '\0') {
            if (file_size == 0U || start >= file_size) {
                return ESP_ERR_INVALID_ARG;
            }
            end = file_size - 1U;
        } else {
            if (!photo_web_parse_u64_value(&p, &end) || *p != '\0' ||
                file_size == 0U || start >= file_size || end < start) {
                return ESP_ERR_INVALID_ARG;
            }
            if (end >= file_size) {
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

    if (!req) {
        return;
    }

    sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        httpd_sess_trigger_close(req->handle, sockfd);
    }
}

static esp_err_t photo_web_raw_send_all(httpd_req_t *req, const char *buf, size_t len)
{
    size_t sent_total = 0;

    if (!req || (!buf && len > 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    while (sent_total < len) {
        int sent = httpd_send(req, buf + sent_total, len - sent_total);
        if (sent <= 0) {
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
    if (header_len < 0 || header_len >= (int)sizeof(header)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return photo_web_raw_send_all(req, header, (size_t)header_len);
}

static esp_err_t photo_web_send_video_header(httpd_req_t *req, const photo_web_range_t *range,
                                             uint64_t file_size)
{
    char header[PHOTO_WEB_HTTP_HEADER_LEN];
    int header_len;

    if (!req || !range) {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->partial) {
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
    } else {
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

    if (header_len < 0 || header_len >= (int)sizeof(header)) {
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

    if (!req || !fp || !file_path || !range) {
        return ESP_ERR_INVALID_ARG;
    }

    if (range->start > 0U) {
        if (range->start > (uint64_t)LONG_MAX ||
            fseek(fp, (long)range->start, SEEK_SET) != 0) {
            ESP_LOGE(TAG, "定位视频文件失败: %s, offset=%" PRIu64 ", errno=%d",
                     file_path, range->start, errno);
            return ESP_FAIL;
        }
    }

    remain = range->length;
    while (remain > 0U) {
        size_t want_len = (remain > sizeof(file_buf)) ? sizeof(file_buf) : (size_t)remain;
        size_t read_len = fread(file_buf, 1, want_len, fp);

        if (read_len == 0U) {
            ESP_LOGE(TAG, "读取视频文件失败: %s", file_path);
            return ESP_FAIL;
        }

        esp_err_t ret = photo_web_raw_send_all(req, file_buf, read_len);
        if (ret != ESP_OK) {
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

    if (!tf_card_is_mounted()) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/photo/") == NULL ||
        !photo_web_has_jpeg_suffix(subpath)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "照片路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                             &offset, tf_card_get_mount_point()),
                        TAG, "构建本地照片路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地照片路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "照片不存在");
    }

    fp = fopen(file_path, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "打开照片文件失败: %s, errno=%d", file_path, errno);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开照片失败");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline");

    while (1) {
        size_t read_len = fread(file_buf, 1, sizeof(file_buf), fp);
        if (read_len > 0) {
            esp_err_t ret = httpd_resp_send_chunk(req, file_buf, read_len);
            if (ret != ESP_OK) {
                fclose(fp);
                return ret;
            }
        }

        if (read_len < sizeof(file_buf)) {
            if (ferror(fp)) {
                ESP_LOGE(TAG, "读取照片文件失败: %s", file_path);
                fclose(fp);
                return ESP_FAIL;
            }
            break;
        }
    }

    fclose(fp);
    return httpd_resp_send_chunk(req, NULL, 0);
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

    if (!tf_card_is_mounted()) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF 卡未挂载");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/video/") == NULL ||
        !photo_web_has_mp4_suffix(subpath)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "视频路径非法");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                             &offset, tf_card_get_mount_point()),
                        TAG, "构建本地视频路径失败");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "构建本地视频路径失败");

    if (!photo_web_is_regular_file(file_path, &file_st)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "视频不存在");
    }

    if (file_st.st_size > 0) {
        file_size = (uint64_t)file_st.st_size;
    }

    ret = photo_web_parse_range_request(req, file_size, &range);
    if (ret != ESP_OK) {
        ret = photo_web_send_range_error(req, file_size);
        photo_web_close_request_session(req);
        return (ret == ESP_ERR_HTTPD_RESP_SEND) ? ESP_OK : ret;
    }

    send_body = (req->method != HTTP_HEAD);
    if (send_body) {
        fp = fopen(file_path, "rb");
        if (!fp) {
            ESP_LOGE(TAG, "打开视频文件失败: %s, errno=%d", file_path, errno);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "打开视频失败");
        }
    }

    ret = photo_web_send_video_header(req, &range, file_size);
    if (ret == ESP_OK && send_body) {
        ret = photo_web_send_video_body(req, fp, file_path, &range);
    }

    if (fp) {
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

    if (s_photo_web.server) {
        return ESP_OK;
    }

    config.server_port = PHOTO_WEB_SERVER_PORT;
    config.stack_size = PHOTO_WEB_SERVER_STACK_SIZE;
    config.max_open_sockets = 4;
    config.backlog_conn = 2;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ret = httpd_start(&s_photo_web.server, &config);
    if (ret != ESP_OK) {
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
    const httpd_uri_t api_time_uri = {
        .uri = "/api/time",
        .method = HTTP_GET,
        .handler = photo_web_api_time_handler,
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
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_photos_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_videos_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &api_time_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &photo_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &video_head_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &favicon_uri);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册媒体网页 URI 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        photo_web_server_stop();
        return ret;
    }

    ESP_LOGI(TAG, "媒体网页服务已启动，端口 %d", PHOTO_WEB_SERVER_PORT);
    return ESP_OK;
}

void photo_web_server_stop(void)
{
    if (!s_photo_web.server) {
        return;
    }

    httpd_stop(s_photo_web.server);
    s_photo_web.server = NULL;
    ESP_LOGI(TAG, "媒体网页服务已停止");
}
