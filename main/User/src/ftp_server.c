/**
 * @file ftp_server.c
 * @brief TF 卡只读 FTP 文件获取服务
 */

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "ftp_server.h"
#include "tf_card.h"

static const char *TAG = "ftp_server";

#define FTP_SERVER_TASK_STACK_SIZE      (8 * 1024)
#define FTP_SERVER_SESSION_STACK_SIZE   (8 * 1024)
#define FTP_SERVER_TASK_PRIORITY        (tskIDLE_PRIORITY + 3)
#define FTP_SERVER_SESSION_PRIORITY     (tskIDLE_PRIORITY + 3)
#define FTP_SERVER_BACKLOG              2
#define FTP_SERVER_CTRL_LINE_LEN        512
#define FTP_SERVER_RESP_LEN             256
#define FTP_SERVER_PATH_LEN             256
#define FTP_SERVER_NAME_LEN             128
#define FTP_SERVER_DATA_CHUNK_SIZE      4096
#define FTP_SERVER_DATA_TIMEOUT_SEC     15
#define FTP_SERVER_DATA_PORT_MIN        21200
#define FTP_SERVER_DATA_PORT_MAX        21210

typedef struct {
    int ctrl_fd;
    int pasv_fd;
    uint16_t pasv_port;
    bool logged_in;
    bool user_ok;
    uint64_t restart_offset;
    char cwd[FTP_SERVER_PATH_LEN];
} ftp_session_t;

static TaskHandle_t s_ftp_task_handle;
static int s_ftp_listen_fd = -1;
static SemaphoreHandle_t s_ftp_active_mutex;
static bool s_ftp_session_active;
static uint16_t s_ftp_next_data_port = FTP_SERVER_DATA_PORT_MIN;

static void ftp_server_task(void *arg);
static void ftp_session_task(void *arg);

static void ftp_close_fd(int *fd)
{
    if (fd && *fd >= 0) {
        close(*fd);
        *fd = -1;
    }
}

static esp_err_t ftp_send_all(int fd, const char *buf, size_t len)
{
    size_t sent_total = 0;

    if (fd < 0 || (!buf && len > 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    while (sent_total < len) {
        int sent = send(fd, buf + sent_total, len - sent_total, 0);
        if (sent <= 0) {
            return ESP_FAIL;
        }
        sent_total += (size_t)sent;
    }

    return ESP_OK;
}

static esp_err_t ftp_sendf(int fd, const char *fmt, ...)
{
    char resp[FTP_SERVER_RESP_LEN];
    va_list ap;
    int len;

    if (fd < 0 || !fmt) {
        return ESP_ERR_INVALID_ARG;
    }

    va_start(ap, fmt);
    len = vsnprintf(resp, sizeof(resp), fmt, ap);
    va_end(ap);
    if (len < 0 || len >= (int)sizeof(resp)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ftp_send_all(fd, resp, (size_t)len);
}

static esp_err_t ftp_read_line(int fd, char *line, size_t line_size)
{
    size_t pos = 0;

    if (fd < 0 || !line || line_size == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    while (pos + 1U < line_size) {
        char ch;
        int ret = recv(fd, &ch, 1, 0);

        if (ret <= 0) {
            return ESP_FAIL;
        }
        if (ch == '\n') {
            break;
        }
        if (ch != '\r') {
            line[pos++] = ch;
        }
    }

    line[pos] = '\0';
    return ESP_OK;
}

static char *ftp_trim_text(char *text)
{
    char *start = text;
    char *end;

    if (!text) {
        return NULL;
    }

    while (*start == ' ' || *start == '\t') {
        start++;
    }

    end = start + strlen(start);
    while (end > start && (end[-1] == ' ' || end[-1] == '\t')) {
        end--;
    }
    *end = '\0';
    return start;
}

static void ftp_upper_command(char *cmd)
{
    if (!cmd) {
        return;
    }
    for (; *cmd; cmd++) {
        *cmd = (char)toupper((unsigned char)*cmd);
    }
}

static const char *ftp_strip_quotes(const char *arg, char *buf, size_t buf_size)
{
    size_t len;

    if (!arg) {
        return "";
    }

    len = strlen(arg);
    if (len >= 2U && arg[0] == '"' && arg[len - 1U] == '"' && len - 1U < buf_size) {
        memcpy(buf, arg + 1, len - 2U);
        buf[len - 2U] = '\0';
        return buf;
    }

    return arg;
}

static bool ftp_is_safe_component(const char *name, size_t len)
{
    if (!name || len == 0U || len >= FTP_SERVER_NAME_LEN) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        unsigned char ch = (unsigned char)name[i];
        if (ch < 0x20U || ch == 0x7FU || ch == '\\' || ch == ':') {
            return false;
        }
    }

    return true;
}

static esp_err_t ftp_append_path_component(char *path, size_t path_size,
                                           const char *name, size_t len)
{
    size_t path_len;

    if (!path || path_size == 0U || !ftp_is_safe_component(name, len)) {
        return ESP_ERR_INVALID_ARG;
    }

    path_len = strlen(path);
    if (path_len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (path_len > 1U) {
        if (path_len + 1U + len >= path_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        path[path_len++] = '/';
    } else {
        if (path_len + len >= path_size) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    memcpy(path + path_len, name, len);
    path[path_len + len] = '\0';
    return ESP_OK;
}

static void ftp_pop_path_component(char *path)
{
    char *slash;

    if (!path || strcmp(path, "/") == 0) {
        return;
    }

    slash = strrchr(path, '/');
    if (!slash || slash == path) {
        path[1] = '\0';
        return;
    }

    *slash = '\0';
}

static esp_err_t ftp_normalize_path(const char *cwd, const char *arg,
                                    char *out, size_t out_size)
{
    char quoted[FTP_SERVER_PATH_LEN] = {0};
    char combined[FTP_SERVER_PATH_LEN] = {0};
    const char *input;
    const char *p;
    int combined_len;

    if (!cwd || !out || out_size < 2U) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * FTP 客户端看到的是以 / 为根的虚拟路径。
     * 这里先合并 cwd 和参数，再消解 . / ..，防止客户端跳出 TF 卡挂载点。
     */
    input = ftp_strip_quotes(arg, quoted, sizeof(quoted));
    if (!input || input[0] == '\0') {
        input = cwd;
    }

    if (input[0] == '/') {
        combined_len = snprintf(combined, sizeof(combined), "%s", input);
    } else if (strcmp(cwd, "/") == 0) {
        combined_len = snprintf(combined, sizeof(combined), "/%s", input);
    } else {
        combined_len = snprintf(combined, sizeof(combined), "%s/%s", cwd, input);
    }
    if (combined_len < 0 || combined_len >= (int)sizeof(combined)) {
        return ESP_ERR_INVALID_SIZE;
    }

    snprintf(out, out_size, "/");
    p = combined;
    while (*p) {
        const char *start;
        size_t len;

        while (*p == '/') {
            p++;
        }
        start = p;
        while (*p && *p != '/') {
            p++;
        }
        len = (size_t)(p - start);
        if (len == 0U || (len == 1U && start[0] == '.')) {
            continue;
        }
        if (len == 2U && start[0] == '.' && start[1] == '.') {
            ftp_pop_path_component(out);
            continue;
        }

        ESP_RETURN_ON_ERROR(ftp_append_path_component(out, out_size, start, len),
                            TAG, "FTP 路径组件非法");
    }

    return ESP_OK;
}

static esp_err_t ftp_build_local_path(const char *virtual_path,
                                      char *local_path, size_t local_path_size)
{
    int len;

    if (!virtual_path || !local_path || local_path_size == 0U ||
        virtual_path[0] != '/') {
        return ESP_ERR_INVALID_ARG;
    }

    /* 虚拟根目录固定映射到 TF 卡挂载点，FTP 层不暴露设备上的其它路径。 */
    if (strcmp(virtual_path, "/") == 0) {
        len = snprintf(local_path, local_path_size, "%s", tf_card_get_mount_point());
    } else {
        len = snprintf(local_path, local_path_size, "%s%s",
                       tf_card_get_mount_point(), virtual_path);
    }

    if (len < 0 || len >= (int)local_path_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static bool ftp_is_tmp_name(const char *name)
{
    size_t len;

    if (!name) {
        return false;
    }
    len = strlen(name);
    /* .tmp 是正在写入的 MP4 分段，列表和下载都隐藏，避免客户端下载到未封口文件。 */
    return len >= 4U && strcasecmp(name + len - 4U, ".tmp") == 0;
}

static bool ftp_is_regular_file(const char *path, struct stat *out_st)
{
    struct stat st = {0};

    if (!path || stat(path, &st) != 0) {
        return false;
    }
    if (!S_ISREG(st.st_mode)) {
        return false;
    }
    if (out_st) {
        *out_st = st;
    }
    return true;
}

static bool ftp_is_dir(const char *path)
{
    struct stat st = {0};

    return path && stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

static esp_err_t ftp_require_sdcard(int ctrl_fd)
{
    if (!tf_card_is_mounted()) {
        ftp_sendf(ctrl_fd, "550 SD card not mounted.\r\n");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t ftp_create_passive_listener(ftp_session_t *session, bool extended)
{
    int fd = -1;
    int opt = 1;
    uint16_t selected_port = 0;
    struct sockaddr_in addr = {0};

    if (!session) {
        return ESP_ERR_INVALID_ARG;
    }

    ftp_close_fd(&session->pasv_fd);
    session->pasv_port = 0;

    for (uint16_t i = 0; i <= (FTP_SERVER_DATA_PORT_MAX - FTP_SERVER_DATA_PORT_MIN); i++) {
        uint16_t port = s_ftp_next_data_port++;
        if (s_ftp_next_data_port > FTP_SERVER_DATA_PORT_MAX) {
            s_ftp_next_data_port = FTP_SERVER_DATA_PORT_MIN;
        }

        fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (fd < 0) {
            return ESP_FAIL;
        }

        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0 &&
            listen(fd, 1) == 0) {
            selected_port = port;
            break;
        }

        close(fd);
        fd = -1;
    }

    if (fd < 0 || selected_port == 0U) {
        ftp_sendf(session->ctrl_fd, "425 Cannot open passive connection.\r\n");
        return ESP_FAIL;
    }

    session->pasv_fd = fd;
    session->pasv_port = selected_port;

    if (extended) {
        return ftp_sendf(session->ctrl_fd,
                         "229 Entering Extended Passive Mode (|||%" PRIu16 "|).\r\n",
                         selected_port);
    }

    struct sockaddr_in local_addr = {0};
    socklen_t local_len = sizeof(local_addr);
    uint8_t *ip;
    if (getsockname(session->ctrl_fd, (struct sockaddr *)&local_addr, &local_len) != 0 ||
        local_addr.sin_addr.s_addr == 0U) {
        local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    }
    ip = (uint8_t *)&local_addr.sin_addr.s_addr;

    return ftp_sendf(session->ctrl_fd,
                     "227 Entering Passive Mode (%u,%u,%u,%u,%u,%u).\r\n",
                     ip[0], ip[1], ip[2], ip[3],
                     selected_port / 256U, selected_port % 256U);
}

static int ftp_accept_data_connection(ftp_session_t *session)
{
    fd_set read_set;
    struct timeval timeout = {
        .tv_sec = FTP_SERVER_DATA_TIMEOUT_SEC,
        .tv_usec = 0,
    };
    int ret;
    int data_fd;

    if (!session || session->pasv_fd < 0) {
        return -1;
    }

    FD_ZERO(&read_set);
    FD_SET(session->pasv_fd, &read_set);
    ret = select(session->pasv_fd + 1, &read_set, NULL, NULL, &timeout);
    if (ret <= 0) {
        ftp_close_fd(&session->pasv_fd);
        session->pasv_port = 0;
        return -1;
    }

    data_fd = accept(session->pasv_fd, NULL, NULL);
    ftp_close_fd(&session->pasv_fd);
    session->pasv_port = 0;
    return data_fd;
}

static void ftp_format_list_time(time_t value, char *out, size_t out_size)
{
    static const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    struct tm tm_info = {0};

    if (!out || out_size == 0U) {
        return;
    }

    if (value <= 0 || localtime_r(&value, &tm_info) == NULL) {
        snprintf(out, out_size, "Jan 01 00:00");
        return;
    }

    snprintf(out, out_size, "%s %02d %02d:%02d",
             months[tm_info.tm_mon < 12 ? tm_info.tm_mon : 0],
             tm_info.tm_mday, tm_info.tm_hour, tm_info.tm_min);
}

static esp_err_t ftp_send_list_entry(int data_fd, const char *name,
                                     const struct stat *st)
{
    char line[FTP_SERVER_RESP_LEN + FTP_SERVER_NAME_LEN];
    char time_text[32] = {0};
    const bool is_dir = st && S_ISDIR(st->st_mode);
    int64_t size = 0;
    int len;

    if (!name || !st || name[0] == '\0' || ftp_is_tmp_name(name)) {
        return ESP_OK;
    }
    if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
        return ESP_OK;
    }

    if (!is_dir && st->st_size > 0) {
        size = (int64_t)st->st_size;
    }
    ftp_format_list_time(st->st_mtime, time_text, sizeof(time_text));

    len = snprintf(line, sizeof(line),
                   "%crw-r--r-- 1 ftp ftp %10" PRId64 " %s %s\r\n",
                   is_dir ? 'd' : '-', size, time_text, name);
    if (len < 0 || len >= (int)sizeof(line)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ftp_send_all(data_fd, line, (size_t)len);
}

static esp_err_t ftp_open_data_reply(ftp_session_t *session, const char *message,
                                     int *out_data_fd)
{
    int data_fd;

    if (!session || !out_data_fd) {
        return ESP_ERR_INVALID_ARG;
    }
    if (session->pasv_fd < 0) {
        ftp_sendf(session->ctrl_fd, "425 Use PASV or EPSV first.\r\n");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 仅支持 PASV/EPSV 被动数据连接。
     * 设备常运行在 AP 或 NAT 后面，主动模式需要设备反连客户端，兼容性和安全边界都更差。
     */
    ftp_sendf(session->ctrl_fd, "150 %s\r\n", message ? message : "Opening data connection.");
    data_fd = ftp_accept_data_connection(session);
    if (data_fd < 0) {
        ftp_sendf(session->ctrl_fd, "425 Data connection timed out.\r\n");
        return ESP_FAIL;
    }

    *out_data_fd = data_fd;
    return ESP_OK;
}

static esp_err_t ftp_handle_list(ftp_session_t *session, const char *arg, bool names_only)
{
    char virtual_path[FTP_SERVER_PATH_LEN] = {0};
    char local_path[FTP_SERVER_PATH_LEN] = {0};
    DIR *dir = NULL;
    struct stat st = {0};
    int data_fd = -1;
    esp_err_t ret;

    if (ftp_require_sdcard(session->ctrl_fd) != ESP_OK) {
        return ESP_OK;
    }

    if (arg && arg[0] == '-') {
        arg = "";
    }

    ret = ftp_normalize_path(session->cwd, arg, virtual_path, sizeof(virtual_path));
    if (ret == ESP_OK) {
        ret = ftp_build_local_path(virtual_path, local_path, sizeof(local_path));
    }
    if (ret != ESP_OK || stat(local_path, &st) != 0) {
        ftp_sendf(session->ctrl_fd, "550 Path not found.\r\n");
        return ESP_OK;
    }

    ret = ftp_open_data_reply(session,
                              names_only ? "Opening name list." : "Opening directory list.",
                              &data_fd);
    if (ret != ESP_OK) {
        return ESP_OK;
    }

    if (S_ISDIR(st.st_mode)) {
        struct dirent *entry;

        dir = opendir(local_path);
        if (!dir) {
            ftp_close_fd(&data_fd);
            ftp_sendf(session->ctrl_fd, "550 Cannot open directory.\r\n");
            return ESP_OK;
        }

        while ((entry = readdir(dir)) != NULL) {
            char entry_path[FTP_SERVER_PATH_LEN] = {0};
            struct stat entry_st = {0};
            int path_len;

            if (strcmp(entry->d_name, ".") == 0 ||
                strcmp(entry->d_name, "..") == 0 ||
                ftp_is_tmp_name(entry->d_name)) {
                continue;
            }

            path_len = snprintf(entry_path, sizeof(entry_path), "%s/%s",
                                local_path, entry->d_name);
            if (path_len < 0 || path_len >= (int)sizeof(entry_path) ||
                stat(entry_path, &entry_st) != 0) {
                continue;
            }

            if (names_only) {
                char line[FTP_SERVER_NAME_LEN + 4];
                int len = snprintf(line, sizeof(line), "%s\r\n", entry->d_name);
                if (len > 0 && len < (int)sizeof(line)) {
                    ftp_send_all(data_fd, line, (size_t)len);
                }
            } else {
                ftp_send_list_entry(data_fd, entry->d_name, &entry_st);
            }
        }
        closedir(dir);
    } else if (S_ISREG(st.st_mode) && !ftp_is_tmp_name(local_path)) {
        const char *name = strrchr(local_path, '/');
        name = name ? name + 1 : local_path;
        if (names_only) {
            char line[FTP_SERVER_NAME_LEN + 4];
            int len = snprintf(line, sizeof(line), "%s\r\n", name);
            if (len > 0 && len < (int)sizeof(line)) {
                ftp_send_all(data_fd, line, (size_t)len);
            }
        } else {
            ftp_send_list_entry(data_fd, name, &st);
        }
    }

    ftp_close_fd(&data_fd);
    ftp_sendf(session->ctrl_fd, "226 Transfer complete.\r\n");
    return ESP_OK;
}

static esp_err_t ftp_handle_retr(ftp_session_t *session, const char *arg)
{
    char virtual_path[FTP_SERVER_PATH_LEN] = {0};
    char local_path[FTP_SERVER_PATH_LEN] = {0};
    struct stat st = {0};
    FILE *fp = NULL;
    uint8_t *buf = NULL;
    int data_fd = -1;
    esp_err_t ret;

    if (ftp_require_sdcard(session->ctrl_fd) != ESP_OK) {
        return ESP_OK;
    }

    ret = ftp_normalize_path(session->cwd, arg, virtual_path, sizeof(virtual_path));
    if (ret == ESP_OK) {
        ret = ftp_build_local_path(virtual_path, local_path, sizeof(local_path));
    }
    if (ret != ESP_OK || ftp_is_tmp_name(local_path) ||
        !ftp_is_regular_file(local_path, &st)) {
        ftp_sendf(session->ctrl_fd, "550 File not found.\r\n");
        session->restart_offset = 0;
        return ESP_OK;
    }

    fp = fopen(local_path, "rb");
    if (!fp) {
        ftp_sendf(session->ctrl_fd, "550 Cannot open file.\r\n");
        session->restart_offset = 0;
        return ESP_OK;
    }

    if (session->restart_offset > 0U) {
        if (session->restart_offset > (uint64_t)LONG_MAX ||
            fseek(fp, (long)session->restart_offset, SEEK_SET) != 0) {
            fclose(fp);
            session->restart_offset = 0;
            ftp_sendf(session->ctrl_fd, "554 Invalid restart offset.\r\n");
            return ESP_OK;
        }
    }

    ret = ftp_open_data_reply(session, "Opening binary mode data connection.", &data_fd);
    if (ret != ESP_OK) {
        fclose(fp);
        session->restart_offset = 0;
        return ESP_OK;
    }

    buf = (uint8_t *)malloc(FTP_SERVER_DATA_CHUNK_SIZE);
    if (!buf) {
        ftp_close_fd(&data_fd);
        fclose(fp);
        session->restart_offset = 0;
        ftp_sendf(session->ctrl_fd, "451 Not enough memory.\r\n");
        return ESP_OK;
    }

    ret = ESP_OK;
    while (1) {
        size_t read_len = fread(buf, 1, FTP_SERVER_DATA_CHUNK_SIZE, fp);
        if (read_len > 0U && ftp_send_all(data_fd, (const char *)buf, read_len) != ESP_OK) {
            ret = ESP_FAIL;
            break;
        }
        if (read_len < FTP_SERVER_DATA_CHUNK_SIZE) {
            if (ferror(fp)) {
                ret = ESP_FAIL;
            }
            break;
        }
    }

    free(buf);
    ftp_close_fd(&data_fd);
    fclose(fp);
    session->restart_offset = 0;

    if (ret == ESP_OK) {
        ftp_sendf(session->ctrl_fd, "226 Transfer complete.\r\n");
    } else {
        ftp_sendf(session->ctrl_fd, "426 Transfer aborted.\r\n");
    }
    return ESP_OK;
}

static esp_err_t ftp_handle_size(ftp_session_t *session, const char *arg)
{
    char virtual_path[FTP_SERVER_PATH_LEN] = {0};
    char local_path[FTP_SERVER_PATH_LEN] = {0};
    struct stat st = {0};
    esp_err_t ret;

    if (ftp_require_sdcard(session->ctrl_fd) != ESP_OK) {
        return ESP_OK;
    }

    ret = ftp_normalize_path(session->cwd, arg, virtual_path, sizeof(virtual_path));
    if (ret == ESP_OK) {
        ret = ftp_build_local_path(virtual_path, local_path, sizeof(local_path));
    }
    if (ret != ESP_OK || ftp_is_tmp_name(local_path) ||
        !ftp_is_regular_file(local_path, &st)) {
        ftp_sendf(session->ctrl_fd, "550 File not found.\r\n");
        return ESP_OK;
    }

    ftp_sendf(session->ctrl_fd, "213 %" PRId64 "\r\n",
              st.st_size > 0 ? (int64_t)st.st_size : 0);
    return ESP_OK;
}

static esp_err_t ftp_handle_mdtm(ftp_session_t *session, const char *arg)
{
    char virtual_path[FTP_SERVER_PATH_LEN] = {0};
    char local_path[FTP_SERVER_PATH_LEN] = {0};
    struct stat st = {0};
    struct tm tm_info = {0};
    esp_err_t ret;

    if (ftp_require_sdcard(session->ctrl_fd) != ESP_OK) {
        return ESP_OK;
    }

    ret = ftp_normalize_path(session->cwd, arg, virtual_path, sizeof(virtual_path));
    if (ret == ESP_OK) {
        ret = ftp_build_local_path(virtual_path, local_path, sizeof(local_path));
    }
    if (ret != ESP_OK || ftp_is_tmp_name(local_path) ||
        !ftp_is_regular_file(local_path, &st) ||
        gmtime_r(&st.st_mtime, &tm_info) == NULL) {
        ftp_sendf(session->ctrl_fd, "550 File not found.\r\n");
        return ESP_OK;
    }

    ftp_sendf(session->ctrl_fd, "213 %04d%02d%02d%02d%02d%02d\r\n",
              tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
              tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
    return ESP_OK;
}

static esp_err_t ftp_handle_cwd(ftp_session_t *session, const char *arg)
{
    char virtual_path[FTP_SERVER_PATH_LEN] = {0};
    char local_path[FTP_SERVER_PATH_LEN] = {0};
    esp_err_t ret;

    if (ftp_require_sdcard(session->ctrl_fd) != ESP_OK) {
        return ESP_OK;
    }

    ret = ftp_normalize_path(session->cwd, arg, virtual_path, sizeof(virtual_path));
    if (ret == ESP_OK) {
        ret = ftp_build_local_path(virtual_path, local_path, sizeof(local_path));
    }
    if (ret != ESP_OK || !ftp_is_dir(local_path)) {
        ftp_sendf(session->ctrl_fd, "550 Directory not found.\r\n");
        return ESP_OK;
    }

    snprintf(session->cwd, sizeof(session->cwd), "%s", virtual_path);
    ftp_sendf(session->ctrl_fd, "250 Directory changed to \"%s\".\r\n", session->cwd);
    return ESP_OK;
}

static esp_err_t ftp_handle_rest(ftp_session_t *session, const char *arg)
{
    char *end = NULL;
    unsigned long long offset;

    if (!session || !arg || arg[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    errno = 0;
    offset = strtoull(arg, &end, 10);
    if (errno != 0 || end == arg || *end != '\0') {
        ftp_sendf(session->ctrl_fd, "501 Invalid restart offset.\r\n");
        return ESP_OK;
    }

    session->restart_offset = (uint64_t)offset;
    ftp_sendf(session->ctrl_fd, "350 Restart position accepted.\r\n");
    return ESP_OK;
}

static bool ftp_command_allowed_before_login(const char *cmd)
{
    return strcmp(cmd, "USER") == 0 ||
           strcmp(cmd, "PASS") == 0 ||
           strcmp(cmd, "QUIT") == 0 ||
           strcmp(cmd, "NOOP") == 0 ||
           strcmp(cmd, "SYST") == 0 ||
           strcmp(cmd, "FEAT") == 0 ||
           strcmp(cmd, "OPTS") == 0 ||
           strcmp(cmd, "AUTH") == 0;
}

static bool ftp_handle_command(ftp_session_t *session, char *line)
{
    char *arg;
    char *cmd = line;

    arg = strchr(line, ' ');
    if (arg) {
        *arg++ = '\0';
        arg = ftp_trim_text(arg);
    } else {
        arg = "";
    }
    ftp_upper_command(cmd);

    if (!session->logged_in && !ftp_command_allowed_before_login(cmd)) {
        ftp_sendf(session->ctrl_fd, "530 Please login with USER and PASS.\r\n");
        return true;
    }

    if (strcmp(cmd, "USER") == 0) {
        session->user_ok = (strcmp(arg, FTP_SERVER_USER) == 0);
        ftp_sendf(session->ctrl_fd, "331 User name okay, need password.\r\n");
    } else if (strcmp(cmd, "PASS") == 0) {
        session->logged_in = session->user_ok && (strcmp(arg, FTP_SERVER_PASSWORD) == 0);
        ftp_sendf(session->ctrl_fd,
                  session->logged_in ? "230 Login successful.\r\n" : "530 Login incorrect.\r\n");
    } else if (strcmp(cmd, "SYST") == 0) {
        ftp_sendf(session->ctrl_fd, "215 UNIX Type: L8\r\n");
    } else if (strcmp(cmd, "FEAT") == 0) {
        ftp_sendf(session->ctrl_fd,
                  "211-Features\r\n"
                  " UTF8\r\n"
                  " SIZE\r\n"
                  " MDTM\r\n"
                  " REST STREAM\r\n"
                  " EPSV\r\n"
                  " PASV\r\n"
                  "211 End\r\n");
    } else if (strcmp(cmd, "OPTS") == 0) {
        ftp_sendf(session->ctrl_fd, "200 Option accepted.\r\n");
    } else if (strcmp(cmd, "AUTH") == 0) {
        ftp_sendf(session->ctrl_fd, "502 TLS is not supported.\r\n");
    } else if (strcmp(cmd, "PWD") == 0 || strcmp(cmd, "XPWD") == 0) {
        ftp_sendf(session->ctrl_fd, "257 \"%s\" is current directory.\r\n", session->cwd);
    } else if (strcmp(cmd, "CWD") == 0) {
        ftp_handle_cwd(session, arg);
    } else if (strcmp(cmd, "CDUP") == 0) {
        ftp_handle_cwd(session, "..");
    } else if (strcmp(cmd, "TYPE") == 0) {
        ftp_sendf(session->ctrl_fd, "200 Type set to %s.\r\n", arg && arg[0] ? arg : "I");
    } else if (strcmp(cmd, "MODE") == 0 || strcmp(cmd, "STRU") == 0) {
        ftp_sendf(session->ctrl_fd, "200 Command okay.\r\n");
    } else if (strcmp(cmd, "PASV") == 0) {
        ftp_create_passive_listener(session, false);
    } else if (strcmp(cmd, "EPSV") == 0) {
        ftp_create_passive_listener(session, true);
    } else if (strcmp(cmd, "PORT") == 0 || strcmp(cmd, "EPRT") == 0) {
        /* 主动模式关闭，统一要求客户端走被动模式下载。 */
        ftp_sendf(session->ctrl_fd, "502 Active mode is disabled. Use PASV or EPSV.\r\n");
    } else if (strcmp(cmd, "LIST") == 0) {
        ftp_handle_list(session, arg, false);
    } else if (strcmp(cmd, "NLST") == 0) {
        ftp_handle_list(session, arg, true);
    } else if (strcmp(cmd, "RETR") == 0) {
        ftp_handle_retr(session, arg);
    } else if (strcmp(cmd, "SIZE") == 0) {
        ftp_handle_size(session, arg);
    } else if (strcmp(cmd, "MDTM") == 0) {
        ftp_handle_mdtm(session, arg);
    } else if (strcmp(cmd, "REST") == 0) {
        ftp_handle_rest(session, arg);
    } else if (strcmp(cmd, "NOOP") == 0) {
        ftp_sendf(session->ctrl_fd, "200 NOOP okay.\r\n");
    } else if (strcmp(cmd, "ABOR") == 0) {
        ftp_close_fd(&session->pasv_fd);
        session->restart_offset = 0;
        ftp_sendf(session->ctrl_fd, "226 Abort successful.\r\n");
    } else if (strcmp(cmd, "STOR") == 0 || strcmp(cmd, "APPE") == 0 ||
               strcmp(cmd, "DELE") == 0 || strcmp(cmd, "MKD") == 0 ||
               strcmp(cmd, "XMKD") == 0 || strcmp(cmd, "RMD") == 0 ||
               strcmp(cmd, "XRMD") == 0 || strcmp(cmd, "RNFR") == 0 ||
               strcmp(cmd, "RNTO") == 0 || strcmp(cmd, "SITE") == 0) {
        /* FTP 服务定位为远程取文件，上传、删除、改名和建目录全部拒绝。 */
        ftp_sendf(session->ctrl_fd, "550 Permission denied.\r\n");
    } else if (strcmp(cmd, "QUIT") == 0) {
        ftp_sendf(session->ctrl_fd, "221 Goodbye.\r\n");
        return false;
    } else {
        ftp_sendf(session->ctrl_fd, "502 Command not implemented.\r\n");
    }

    return true;
}

static void ftp_set_session_active(bool active)
{
    if (s_ftp_active_mutex &&
        xSemaphoreTake(s_ftp_active_mutex, portMAX_DELAY) == pdTRUE) {
        s_ftp_session_active = active;
        xSemaphoreGive(s_ftp_active_mutex);
    }
}

static bool ftp_try_take_session_slot(void)
{
    bool granted = false;

    if (s_ftp_active_mutex &&
        xSemaphoreTake(s_ftp_active_mutex, portMAX_DELAY) == pdTRUE) {
        if (!s_ftp_session_active) {
            s_ftp_session_active = true;
            granted = true;
        }
        xSemaphoreGive(s_ftp_active_mutex);
    }

    return granted;
}

static void ftp_session_task(void *arg)
{
    ftp_session_t session = {
        .ctrl_fd = (int)(intptr_t)arg,
        .pasv_fd = -1,
        .pasv_port = 0,
        .logged_in = false,
        .user_ok = false,
        .restart_offset = 0,
        .cwd = "/",
    };
    char line[FTP_SERVER_CTRL_LINE_LEN];

    ESP_LOGI(TAG, "FTP 客户端已连接");
    ftp_sendf(session.ctrl_fd, "220 WT99P4C5-S1 FTP ready.\r\n");

    while (ftp_read_line(session.ctrl_fd, line, sizeof(line)) == ESP_OK) {
        if (line[0] == '\0') {
            continue;
        }
        if (!ftp_handle_command(&session, line)) {
            break;
        }
    }

    ftp_close_fd(&session.pasv_fd);
    ftp_close_fd(&session.ctrl_fd);
    ftp_set_session_active(false);
    ESP_LOGI(TAG, "FTP 客户端已断开");
    vTaskDelete(NULL);
}

static void ftp_server_task(void *arg)
{
    (void)arg;

    s_ftp_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (s_ftp_listen_fd < 0) {
        ESP_LOGE(TAG, "创建 FTP 监听 socket 失败");
        s_ftp_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(s_ftp_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(FTP_SERVER_PORT),
    };
    if (bind(s_ftp_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "绑定 FTP 端口失败: %d, errno=%d", FTP_SERVER_PORT, errno);
        ftp_close_fd(&s_ftp_listen_fd);
        s_ftp_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (listen(s_ftp_listen_fd, FTP_SERVER_BACKLOG) != 0) {
        ESP_LOGE(TAG, "监听 FTP 端口失败: errno=%d", errno);
        ftp_close_fd(&s_ftp_listen_fd);
        s_ftp_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "FTP 服务已启动，端口 %d，用户 %s", FTP_SERVER_PORT, FTP_SERVER_USER);

    while (s_ftp_listen_fd >= 0) {
        int cli_fd = accept(s_ftp_listen_fd, NULL, NULL);
        if (cli_fd < 0) {
            if (errno != EBADF) {
                ESP_LOGW(TAG, "FTP accept 失败: errno=%d", errno);
            }
            continue;
        }

        if (!ftp_try_take_session_slot()) {
            ftp_sendf(cli_fd, "421 Too many FTP clients.\r\n");
            close(cli_fd);
            continue;
        }

        if (xTaskCreate(ftp_session_task, "ftp_session",
                        FTP_SERVER_SESSION_STACK_SIZE,
                        (void *)(intptr_t)cli_fd,
                        FTP_SERVER_SESSION_PRIORITY,
                        NULL) != pdPASS) {
            ESP_LOGE(TAG, "创建 FTP 会话任务失败");
            ftp_sendf(cli_fd, "421 Cannot create FTP session.\r\n");
            close(cli_fd);
            ftp_set_session_active(false);
        }
    }

    s_ftp_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ftp_server_start(void)
{
    if (s_ftp_task_handle) {
        return ESP_OK;
    }

    if (!s_ftp_active_mutex) {
        s_ftp_active_mutex = xSemaphoreCreateMutex();
        if (!s_ftp_active_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreate(ftp_server_task, "ftp_server",
                    FTP_SERVER_TASK_STACK_SIZE,
                    NULL,
                    FTP_SERVER_TASK_PRIORITY,
                    &s_ftp_task_handle) != pdPASS) {
        s_ftp_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void ftp_server_stop(void)
{
    ftp_close_fd(&s_ftp_listen_fd);
}
