/**
 * @file wife.c
 * @brief WiFi(AP+STA) + TCP 手机遥控模块实现
 *
 * 参照 github1/xunji1/main/duoji.c 的协议与结构，针对四足机器人做了两点改造：
 *   1. 纯 AP → AP+STA（同时连路由器上网 + 开热点给手机）；
 *   2. 遥控锁定 → 实时连接状态（断开后由 main.c 恢复语音路径）。
 *
 * 指令协议：F 前进 / B 后退 / L 左移 / R 右移 / S 站立(停止)。
 * 本模块只解析指令并记录状态，不直接驱动电机——电机由 main.c 的电机任务执行。
 */
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "wife.h"

static const char *TAG = "wife";

/* 手机连接状态 + 最近指令（跨任务共享，volatile 保证可见性） */
static volatile bool       s_connected = false;
static volatile wife_cmd_t s_cmd       = WIFE_CMD_NONE;
static volatile int        s_client_fd = -1;   /* 当前 TCP 客户端 fd，-1 = 无连接 */

/* ==================== WiFi(AP+STA) ==================== */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* 联网是尽力而为，断开后台重连即可，不阻塞 AP 功能 */
        ESP_LOGW(TAG, "STA 断线，重连中...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "已联网(STA)，IP: " IPSTR, IP2STR(&evt->ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "有设备连上热点");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        /* 手机断开热点：立即关掉 TCP 连接，让 recv 返回、电机停止 */
        ESP_LOGI(TAG, "手机断开热点，停止");
        s_connected = false;
        int fd = s_client_fd;
        if (fd >= 0) {
            shutdown(fd, SHUT_RDWR);
        }
    }
}

static void wifi_init_apsta(void)
{
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    /* STA：连路由器上网（尽力而为，失败不阻塞） */
    wifi_config_t sta_cfg = { 0 };
    strlcpy((char *)sta_cfg.sta.ssid, WIFE_STA_SSID, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, WIFE_STA_PASS, sizeof(sta_cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    /* AP：开热点给手机 */
    wifi_config_t ap_cfg = {
        .ap = {
            .max_connection = WIFE_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .channel        = WIFE_AP_CHANNEL,
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid, WIFE_AP_SSID, sizeof(ap_cfg.ap.ssid));
    strlcpy((char *)ap_cfg.ap.password, WIFE_AP_PASS, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = strlen(WIFE_AP_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "已开自身热点(AP): %s 密码 %s，手机连它后访问 192.168.4.1:%d；后台尝试连路由器(STA): %s",
             WIFE_AP_SSID, WIFE_AP_PASS, WIFE_TCP_PORT, WIFE_STA_SSID);
}

/* ==================== 指令解析 ==================== */

static void handle_command(const char *line)
{
    switch (line[0]) {
    case 'F': case 'f': s_cmd = WIFE_CMD_FORWARD;  break;
    case 'B': case 'b': s_cmd = WIFE_CMD_BACKWARD; break;
    case 'L': case 'l': s_cmd = WIFE_CMD_LEFT;     break;
    case 'R': case 'r': s_cmd = WIFE_CMD_RIGHT;    break;
    case 'S': case 's': s_cmd = WIFE_CMD_STAND;    break;
    default:
        ESP_LOGW(TAG, "未知指令: %s", line);
        break;
    }
}

/* ==================== 手机遥控任务 ==================== */

static void wife_tcp_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() 失败: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(WIFE_TCP_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind() 失败: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "listen() 失败: %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP 服务器监听 :%d", WIFE_TCP_PORT);

    char rx[256];
    char line_buf[128];
    int  line_len = 0;

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client < 0) {
            continue;
        }

        ESP_LOGI(TAG, "手机已连上，进入遥控模式");
        s_connected = true;
        s_client_fd = client;
        s_cmd       = WIFE_CMD_STAND;   /* 连上后立刻站立，防止沿用上一次动作 */

        /* 1s 接收超时，便于及时检测断连 */
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        /* TCP keepalive：手机异常断连(直接断WiFi/杀App)时几秒内能检测到 */
        int ka_on = 1, ka_idle = 3, ka_intvl = 1, ka_cnt = 3;
        setsockopt(client, SOL_SOCKET, SO_KEEPALIVE, &ka_on, sizeof(ka_on));
        setsockopt(client, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
        setsockopt(client, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(client, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));

        while (1) {
            int n = recv(client, rx, sizeof(rx) - 1, 0);
            if (n == 0) {
                break;                              /* 正常断开 */
            }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;                       /* 超时，继续等 */
                }
                break;                              /* 出错断开 */
            }
            rx[n] = '\0';
            for (int i = 0; i < n; i++) {
                char c = rx[i];
                if (c == '\n' || c == '\r') {
                    if (line_len > 0) {
                        line_buf[line_len] = '\0';
                        handle_command(line_buf);
                        line_len = 0;
                    }
                } else if (line_len < (int)sizeof(line_buf) - 1) {
                    line_buf[line_len++] = c;
                }
            }
        }

        ESP_LOGI(TAG, "手机断开，站立等待重连/语音...");
        s_connected = false;
        s_client_fd = -1;
        s_cmd       = WIFE_CMD_STAND;   /* 断开立即站立 */
        close(client);
        /* 回到 accept 等待重连；连接状态已实时清除，语音路径恢复 */
    }
}

/* ==================== 对外接口 ==================== */

esp_err_t wife_init(void)
{
    /* nvs：首次上电或分区表变化时擦除重建（重复调用安全） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_apsta();
    xTaskCreate(wife_tcp_task, "wife_tcp", 8192, NULL, 5, NULL);
    return ESP_OK;
}

bool wife_is_connected(void)
{
    return s_connected;
}

wife_cmd_t wife_get_cmd(void)
{
    return s_cmd;
}
