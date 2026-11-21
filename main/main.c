#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "microphone.h"
#include "speaker.h"
#include "cloud_ai.h"

static const char *TAG = "main";

/* ==================== WiFi 配置 ==================== */
#define WIFI_SSID   "1"
#define WIFI_PASS   "943816729"

/* ==================== 语音助手参数 ==================== */
#define FRAME_SAMPLES           320      /* 每帧 20 ms @ 16kHz */
#define RECORD_SECONDS          5        /* 录音时长：唤醒词 + 问句 */
#define RECORD_SAMPLES          (MIC_SAMPLE_RATE * RECORD_SECONDS)

/* ==================== 触发 / 休眠参数 ==================== */
#define TRIGGER_RMS_THRESHOLD   500      /* 检测到说话的阈值（RMS），可调 */
#define SILENCE_RMS_THRESHOLD   200      /* 静音阈值（RMS），低于它算静音，可调 */
#define SLEEP_AFTER_SILENCE_MS  10000    /* 连续 10 秒静音后进入休眠 */

/* ==================== 唤醒词（从长到短，避免前缀误匹配） ==================== */
static const char *g_wake_words[] = {
    "千问千问",
    "千千问问",
    "千千千问",
    "千问问",
    "千千问",
    "千问",
};
#define WAKE_WORD_COUNT  (sizeof(g_wake_words) / sizeof(g_wake_words[0]))

typedef enum {
    APP_SLEEP = 0,   /* 休眠：AI 关闭 */
    APP_ACTIVE,      /* 活跃：AI 开启 */
} app_state_t;

static app_state_t s_state     = APP_SLEEP;
static uint32_t    s_silent_ms = 0;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0

/* ==================== WiFi ==================== */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi 断线，重连中...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "已联网");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* 关闭省电模式，防止空闲掉线 */

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ==================== 工具函数 ==================== */

static float rms_of(const int16_t *buf, size_t samples)
{
    if (samples == 0) {
        return 0.0f;
    }
    double sum = 0.0;
    for (size_t i = 0; i < samples; i++) {
        double v = buf[i];
        sum += v * v;
    }
    return (float)sqrt(sum / samples);
}

/* 跳过开头的空白与中英文标点，返回第一个有效字符的指针 */
static const char *skip_punct(const char *p)
{
    while (*p) {
        unsigned char c = (unsigned char)*p;

        if (c == ' ' || c == '\t') { p++; continue; }

        if (c < 0x80) {
            if (c == ',' || c == '.' || c == '!' || c == '?' || c == ';' ||
                c == ':' || c == '\'' || c == '"' || c == '~' || c == '-') {
                p++;
                continue;
            }
            break;
        }

        /* 解一个 UTF-8 码点 */
        uint32_t cp = 0;
        int len = 0;
        if ((c & 0xE0) == 0xC0)      { cp = c & 0x1F; len = 2; }
        else if ((c & 0xF0) == 0xE0) { cp = c & 0x0F; len = 3; }
        else if ((c & 0xF8) == 0xF0) { cp = c & 0x07; len = 4; }
        else                         { p++; continue; }

        bool ok = true;
        for (int i = 1; i < len; i++) {
            if (((unsigned char)p[i] & 0xC0) != 0x80) { ok = false; break; }
            cp = (cp << 6) | ((unsigned char)p[i] & 0x3F);
        }
        if (!ok) { p++; continue; }

        bool punct = (cp >= 0x3000 && cp <= 0x303F) ||   /* 。、、「」〈〉… */
                     (cp >= 0xFF01 && cp <= 0xFF20) ||   /* ！？，：；（）全角符号 */
                     (cp >= 0x2010 && cp <= 0x2027) ||   /* — ‘ ’ “ ” … */
                     cp == 0x00B7 || cp == 0x3000;
        if (!punct) break;
        p += len;
    }
    return p;
}

/* 匹配唤醒词前缀，返回剥掉前缀和标点后的问句指针；不匹配返回 NULL */
static const char *match_wake_word(const char *text)
{
    for (size_t i = 0; i < WAKE_WORD_COUNT; i++) {
        size_t len = strlen(g_wake_words[i]);
        if (strncmp(text, g_wake_words[i], len) == 0) {
            return skip_punct(text + len);
        }
    }
    return NULL;
}

/* ==================== 语音处理 ==================== */

/* 录音 → ASR → 匹配唤醒词 → LLM → TTS → 播放。返回是否真正唤醒 */
static bool handle_speech(int16_t *pcm)
{
    /* 1. 录音 */
    size_t total = 0;
    microphone_read(pcm, RECORD_SAMPLES, &total,
                    pdMS_TO_TICKS(RECORD_SECONDS * 1000 + 500));
    if (total == 0) {
        return false;
    }

    /* 2. ASR 识别 */
    static char text[512];
    if (cloud_asr(pcm, total, text, sizeof(text)) != ESP_OK) {
        ESP_LOGE(TAG, "语音识别失败");
        return false;
    }
    ESP_LOGI(TAG, "识别: %s", text);

    /* 3. 匹配唤醒词 */
    const char *q = match_wake_word(text);
    if (q == NULL) {
        return false;
    }

    /* 4. 只说了唤醒词，没带问句 */
    if (*q == '\0') {
        ESP_LOGI(TAG, "唤醒");
        int16_t *a = NULL;
        size_t n = 0;
        if (cloud_tts("我在", &a, &n) == ESP_OK) {
            speaker_play(a, n, portMAX_DELAY);
            free(a);
        }
        return true;
    }
    ESP_LOGI(TAG, "问句: %s", q);

    /* 5. 大模型对话 */
    static char reply[1024];
    if (cloud_llm(q, reply, sizeof(reply)) != ESP_OK) {
        ESP_LOGE(TAG, "大模型对话失败");
        return true;   /* 已唤醒，只是问答失败 */
    }
    ESP_LOGI(TAG, "回答: %s", reply);

    /* 6. 语音合成 + 播放 */
    int16_t *audio = NULL;
    size_t audio_samples = 0;
    if (cloud_tts(reply, &audio, &audio_samples) == ESP_OK) {
        speaker_play(audio, audio_samples, portMAX_DELAY);
        free(audio);
    }
    return true;
}

/* ==================== 主程序 ==================== */

void app_main(void)
{
    /* 基础初始化 */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 连接 WiFi */
    wifi_init_sta();

    /* 初始化音频外设 */
    ESP_ERROR_CHECK(microphone_init());
    ESP_ERROR_CHECK(speaker_init());

    int16_t *frame = malloc(FRAME_SAMPLES * sizeof(int16_t));
    int16_t *pcm   = malloc(RECORD_SAMPLES * sizeof(int16_t));
    if (frame == NULL || pcm == NULL) {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }

    ESP_LOGI(TAG, "启动");

    while (1) {
        /* 读一帧音频（20 ms） */
        size_t got = 0;
        if (microphone_read(frame, FRAME_SAMPLES, &got, portMAX_DELAY) != ESP_OK || got == 0) {
            continue;
        }

        float rms = rms_of(frame, got);

        /* 检测到说话 → 录音并处理 */
        if (rms > TRIGGER_RMS_THRESHOLD) {
            ESP_LOGI(TAG, "收到语音");
            bool woke = handle_speech(pcm);
            if (woke) {
                s_state     = APP_ACTIVE;
                s_silent_ms = 0;
            }
            continue;
        }

        /* 活跃状态下检测静音，超时进入休眠 */
        if (s_state == APP_ACTIVE) {
            if (rms < SILENCE_RMS_THRESHOLD) {
                s_silent_ms += 20;      /* 每帧 20 ms */
                if (s_silent_ms >= SLEEP_AFTER_SILENCE_MS) {
                    s_state     = APP_SLEEP;
                    s_silent_ms = 0;
                    ESP_LOGI(TAG, "休眠");
                }
            } else {
                s_silent_ms = 0;
            }
        }
        /* APP_SLEEP：只等触发 */
    }
}
