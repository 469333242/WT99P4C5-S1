/**
 * @file photo_web_server.c
 * @brief SD 卡照片网页浏览模块实现
 *
 * 模块职责：
 *   - 通过 HTTP 服务提供相册网页
 *   - 扫描 TF 卡中各次上电目录下的 photo 子目录
 *   - 将 JPEG 文件以网页缩略图和原图方式提供给浏览器访问
 *
 * 目录约定：
 *   /sdcard/002_19800106/photo/1980-01-06T00-01-10-134.jpeg
 */

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "photo_web_server.h"
#include "tf_card.h"

static const char *TAG = "photo_web";

/* ------------------------------------------------------------------ */
/* 配置参数                                                            */
/* ------------------------------------------------------------------ */
#define PHOTO_WEB_MAX_PATH_LEN       256
#define PHOTO_WEB_FILE_CHUNK_SIZE    2048
#define PHOTO_WEB_SERVER_STACK_SIZE  (8 * 1024)

typedef struct {
    httpd_handle_t server;
} photo_web_ctx_t;

static photo_web_ctx_t s_photo_web;

/* ------------------------------------------------------------------ */
/* 网页内容                                                            */
/* ------------------------------------------------------------------ */
static const char *s_photo_index_html[] = {
    "<!doctype html><html lang=\"zh-CN\"><head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>SD 卡照片浏览</title>"
    "<style>"
    ":root{--bg:#f4efe7;--panel:#fffaf2;--ink:#1b1b1b;--muted:#72685b;--accent:#c65a1e;--line:#e7d8c8;}"
    "*{box-sizing:border-box}body{margin:0;font-family:'Segoe UI','Microsoft YaHei',sans-serif;"
    "background:radial-gradient(circle at top,#fff7ec 0,#f4efe7 42%,#efe6db 100%);color:var(--ink);}"
    ".shell{max-width:1200px;margin:0 auto;padding:24px 16px 40px;}"
    ".hero{display:flex;flex-wrap:wrap;gap:16px;align-items:flex-end;justify-content:space-between;margin-bottom:20px;}"
    ".title{margin:0;font-size:clamp(28px,5vw,44px);letter-spacing:.04em;}"
    ".desc{margin:8px 0 0;color:var(--muted);font-size:14px;}"
    ".toolbar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;}"
    ".badge{padding:10px 14px;border:1px solid var(--line);border-radius:999px;background:rgba(255,250,242,.88);font-size:13px;color:var(--muted);}"
    "button{border:0;border-radius:999px;background:linear-gradient(135deg,#db6d2e,#b94a14);color:#fff;padding:12px 18px;font-size:14px;font-weight:600;cursor:pointer;box-shadow:0 14px 30px rgba(185,74,20,.18);}"
    "button:disabled{opacity:.6;cursor:default}"
    ".panel{background:rgba(255,250,242,.84);border:1px solid rgba(231,216,200,.9);border-radius:24px;padding:18px;backdrop-filter:blur(8px);box-shadow:0 18px 42px rgba(74,39,16,.08);}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:18px;}"
    ".card{overflow:hidden;border-radius:20px;background:#fff;border:1px solid #eadccc;display:flex;flex-direction:column;min-height:280px;}"
    ".thumb{display:block;aspect-ratio:4/3;background:linear-gradient(135deg,#f9dcc0,#f3b183);overflow:hidden;}"
    ".thumb img{display:block;width:100%;height:100%;object-fit:cover;transition:transform .25s ease;}"
    ".card:hover .thumb img{transform:scale(1.03)}"
    ".meta{padding:14px 14px 16px;display:grid;gap:8px;}"
    ".name{font-size:13px;font-weight:700;word-break:break-all;}"
    ".sub{font-size:12px;color:var(--muted);word-break:break-all;}"
    ".empty{grid-column:1/-1;padding:42px 18px;text-align:center;color:var(--muted);border:1px dashed var(--line);border-radius:18px;background:rgba(255,255,255,.65);}"
    "a.open{color:var(--accent);text-decoration:none;font-size:12px;font-weight:700;}"
    "@media (max-width:640px){.shell{padding:18px 12px 32px}.panel{padding:14px}.grid{grid-template-columns:1fr 1fr;gap:12px}.card{min-height:0}}"
    "@media (max-width:460px){.grid{grid-template-columns:1fr}}"
    "</style></head><body><main class=\"shell\">"
    "<section class=\"hero\"><div><h1 class=\"title\">SD 卡照片浏览</h1>"
    "<p class=\"desc\">实时读取设备 TF 卡中已保存的照片，点击缩略图可打开原图。</p></div>"
    "<div class=\"toolbar\"><div id=\"status\" class=\"badge\">正在读取照片列表...</div>"
    "<button id=\"reload\" type=\"button\">刷新列表</button></div></section>"
    "<section class=\"panel\"><div id=\"grid\" class=\"grid\"></div></section></main>",

    "<script>"
    "const statusEl=document.getElementById('status');"
    "const gridEl=document.getElementById('grid');"
    "const reloadBtn=document.getElementById('reload');"
    "function formatSize(size){if(!Number.isFinite(size)||size<=0)return '--';if(size<1024)return size+' B';if(size<1024*1024)return (size/1024).toFixed(1)+' KB';return (size/1024/1024).toFixed(2)+' MB';}"
    "function setEmpty(message){gridEl.innerHTML='';const box=document.createElement('div');box.className='empty';box.textContent=message;gridEl.appendChild(box);}"
    "function appendText(cls,text,parent){const node=document.createElement('div');node.className=cls;node.textContent=text;parent.appendChild(node);}"
    "function render(items){gridEl.innerHTML='';if(!items.length){setEmpty('TF 卡中暂无照片');return;}items.sort((a,b)=>String(b.path).localeCompare(String(a.path)));"
    "for(const item of items){const card=document.createElement('article');card.className='card';"
    "const thumb=document.createElement('a');thumb.className='thumb';thumb.href=item.url;thumb.target='_blank';thumb.rel='noreferrer';"
    "const img=document.createElement('img');img.loading='lazy';img.src=item.url;img.alt=item.name||'photo';thumb.appendChild(img);"
    "const meta=document.createElement('div');meta.className='meta';"
    "appendText('name',item.name||'',meta);appendText('sub','目录：'+(item.session||''),meta);appendText('sub','大小：'+formatSize(Number(item.size_bytes)),meta);"
    "const open=document.createElement('a');open.className='open';open.href=item.url;open.target='_blank';open.rel='noreferrer';open.textContent='打开原图';meta.appendChild(open);"
    "card.appendChild(thumb);card.appendChild(meta);gridEl.appendChild(card);}}"
    "async function loadPhotos(){reloadBtn.disabled=true;statusEl.textContent='正在读取照片列表...';"
    "try{const response=await fetch('/api/photos',{cache:'no-store'});const data=await response.json();if(!response.ok){throw new Error(data.error||('HTTP '+response.status));}"
    "const items=Array.isArray(data.items)?data.items:[];render(items);statusEl.textContent=items.length?('共 '+items.length+' 张照片'):'TF 卡中暂无照片';}"
    "catch(error){setEmpty(error.message||'读取失败');statusEl.textContent='照片列表读取失败';}"
    "finally{reloadBtn.disabled=false;}}"
    "reloadBtn.addEventListener('click',loadPhotos);window.addEventListener('DOMContentLoaded',loadPhotos);"
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
                                               const char *session, const char *file_name)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !session || !file_name) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, session);
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, "/photo/");
    }
    if (ret == ESP_OK) {
        ret = photo_web_append_text(dst, dst_size, &offset, file_name);
    }

    return ret;
}

static esp_err_t photo_web_build_photo_uri(char *dst, size_t dst_size,
                                           const char *relative_path)
{
    esp_err_t ret;
    size_t offset = 0;

    if (!dst || dst_size == 0 || !relative_path) {
        return ESP_ERR_INVALID_ARG;
    }

    dst[0] = '\0';
    ret = photo_web_append_text(dst, dst_size, &offset, "/photo/");
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

static bool photo_web_path_is_safe(const char *subpath)
{
    const char *p;

    if (!subpath || subpath[0] != '/' || subpath[1] == '\0') {
        return false;
    }

    if (strstr(subpath, "..") || strchr(subpath, '\\')) {
        return false;
    }

    /* 当前照片命名规则只需要这些字符，拒绝其它字符可避免路径穿越和 URL 编码歧义。 */
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
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\\"), TAG, "send json escape failed");
            break;
        case '"':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\\""), TAG, "send json escape failed");
            break;
        case '\n':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\n"), TAG, "send json escape failed");
            break;
        case '\r':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\r"), TAG, "send json escape failed");
            break;
        case '\t':
            ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\\t"), TAG, "send json escape failed");
            break;
        default:
            one_char[0] = *p;
            ESP_RETURN_ON_ERROR(httpd_resp_send_chunk(req, one_char, sizeof(one_char)),
                                TAG, "send json char failed");
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

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"error\":\""), TAG, "send error json failed");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, message), TAG, "send error message failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\"}"), TAG, "send error json failed");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_send_photo_json(httpd_req_t *req,
                                           const char *session, const char *file_name,
                                           const struct stat *file_st, bool *first_item)
{
    char relative_path[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char public_uri[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char size_text[32] = {0};
    uint64_t file_size = 0;

    if (!req || !session || !file_name || !file_st || !first_item) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(photo_web_build_relative_path(relative_path, sizeof(relative_path),
                                                     session, file_name),
                        TAG, "build photo relative path failed");
    ESP_RETURN_ON_ERROR(photo_web_build_photo_uri(public_uri, sizeof(public_uri), relative_path),
                        TAG, "build photo uri failed");

    if (file_st->st_size > 0) {
        file_size = (uint64_t)file_st->st_size;
    }
    snprintf(size_text, sizeof(size_text), "%" PRIu64, file_size);

    if (!*first_item) {
        ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, ","), TAG, "send json comma failed");
    }
    *first_item = false;

    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "{\"name\":\""), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, file_name), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"session\":\""), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, session), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"path\":\""), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, relative_path), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"url\":\""), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(photo_web_send_json_escaped(req, public_uri), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "\",\"size_bytes\":"), TAG, "send json failed");
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, size_text), TAG, "send json failed");
    return httpd_resp_sendstr_chunk(req, "}");
}

static esp_err_t photo_web_scan_session_dir(httpd_req_t *req, const char *session,
                                            bool *first_item)
{
    esp_err_t ret;
    DIR *photo_dir_handle = NULL;
    struct dirent *photo_entry = NULL;
    char session_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};
    char photo_dir[PHOTO_WEB_MAX_PATH_LEN] = {0};

    ESP_RETURN_ON_ERROR(photo_web_join_path(session_dir, sizeof(session_dir),
                                           tf_card_get_mount_point(), session),
                        TAG, "build session dir failed");
    if (!photo_web_is_dir(session_dir)) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(photo_web_join_path(photo_dir, sizeof(photo_dir), session_dir, "photo"),
                        TAG, "build photo dir failed");
    if (!photo_web_is_dir(photo_dir)) {
        return ESP_OK;
    }

    photo_dir_handle = opendir(photo_dir);
    if (!photo_dir_handle) {
        ESP_LOGW(TAG, "打开照片目录失败: %s, errno=%d", photo_dir, errno);
        return ESP_OK;
    }

    while ((photo_entry = readdir(photo_dir_handle)) != NULL) {
        struct stat file_st = {0};
        char photo_path[PHOTO_WEB_MAX_PATH_LEN] = {0};

        if (!photo_web_has_jpeg_suffix(photo_entry->d_name)) {
            continue;
        }

        if (photo_web_join_path(photo_path, sizeof(photo_path),
                                photo_dir, photo_entry->d_name) != ESP_OK) {
            ESP_LOGW(TAG, "照片路径过长，跳过: %s/%s", photo_dir, photo_entry->d_name);
            continue;
        }

        if (!photo_web_is_regular_file(photo_path, &file_st)) {
            continue;
        }

        ret = photo_web_send_photo_json(req, session, photo_entry->d_name, &file_st, first_item);
        if (ret != ESP_OK) {
            closedir(photo_dir_handle);
            return ret;
        }
    }

    closedir(photo_dir_handle);
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
                            TAG, "send index html failed");
    }

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t photo_web_api_photos_handler(httpd_req_t *req)
{
    esp_err_t ret;
    DIR *root_dir = NULL;
    struct dirent *entry = NULL;
    bool first_item = true;

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

        ret = photo_web_scan_session_dir(req, entry->d_name, &first_item);
        if (ret != ESP_OK) {
            closedir(root_dir);
            return ret;
        }
    }

    closedir(root_dir);
    ESP_RETURN_ON_ERROR(httpd_resp_sendstr_chunk(req, "]}"), TAG, "send json end failed");
    return httpd_resp_sendstr_chunk(req, NULL);
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
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "TF card not mounted");
    }

    if (!photo_web_path_is_safe(subpath) ||
        strstr(subpath, "/photo/") == NULL ||
        !photo_web_has_jpeg_suffix(subpath)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid photo path");
    }

    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path),
                                             &offset, tf_card_get_mount_point()),
                        TAG, "build local photo path failed");
    ESP_RETURN_ON_ERROR(photo_web_append_text(file_path, sizeof(file_path), &offset, subpath),
                        TAG, "build local photo path failed");

    if (!photo_web_is_regular_file(file_path, &file_st)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "photo not found");
    }

    fp = fopen(file_path, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "打开照片文件失败: %s, errno=%d", file_path, errno);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open photo failed");
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
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ret = httpd_start(&s_photo_web.server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动照片网页服务失败: 0x%x (%s)", ret, esp_err_to_name(ret));
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
    const httpd_uri_t photo_uri = {
        .uri = "/photo/*",
        .method = HTTP_GET,
        .handler = photo_web_photo_handler,
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
        ret = httpd_register_uri_handler(s_photo_web.server, &photo_uri);
    }
    if (ret == ESP_OK) {
        ret = httpd_register_uri_handler(s_photo_web.server, &favicon_uri);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册照片网页 URI 失败: 0x%x (%s)", ret, esp_err_to_name(ret));
        photo_web_server_stop();
        return ret;
    }

    ESP_LOGI(TAG, "照片网页服务已启动，端口 %d", PHOTO_WEB_SERVER_PORT);
    return ESP_OK;
}

void photo_web_server_stop(void)
{
    if (!s_photo_web.server) {
        return;
    }

    httpd_stop(s_photo_web.server);
    s_photo_web.server = NULL;
    ESP_LOGI(TAG, "照片网页服务已停止");
}
