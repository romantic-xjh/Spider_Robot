/**
 * @file voice.c
 * @brief 本地离线命令词识别（ESP-SR MultiNet）
 *
 * 识别四个中文命令词：前进 / 后退 / 向左走 / 向右走，识别结果锁存，
 * 由 main.c 的电机任务轮询消费。不直接驱动电机。
 *
 * 依赖（已在 sdkconfig / 分区表配好）：
 *   - ESP-SR 组件（components/esp-sr），模型存 SPIFFS 分区（label = "model"）
 *   - 命令词格式：汉语拼音，空格分隔（MultiNet4.5 用 pinyin）
 *
 * 注意：MultiNet 中文命令词推荐 4~6 个字，本项目的"前进/后退"只有 2 个字，
 * 误识别率会偏高；如实际误触发，可改成长一些的短语（见 s_commands 表）。
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "voice.h"
#include "microphone.h"
#include "speaker.h"
#include "wife.h"
#include "cloud_ai.h"

#include "esp_mn_iface.h"
#include "esp_tts.h"
#include "esp_tts_voice_xiaole.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"

static const char *TAG = "voice";

/* 音量门限：RMS 低于它算静音/噪声，不喂识别。值越小越灵敏（可实测调整） */
#define VOICE_RMS_THRESHOLD   1000

static esp_mn_iface_t    *s_multinet   = NULL;
static model_iface_data_t *s_model_data = NULL;

static volatile voice_cmd_t s_cmd = VOICE_CMD_NONE;

static esp_tts_handle_t s_tts = NULL;   /* 中文 TTS 句柄（唤醒后播放「我在」） */

/* 命令词表：拼音(空格分隔) -> 语音命令 */
static const struct {
    const char  *pinyin;
    voice_cmd_t  cmd;
} s_commands[] = {
    { "qian jin", VOICE_CMD_FORWARD },   /* 0: 前进 */
    { "xiao zhi", VOICE_CMD_NONE    },   /* 1: 小智（唤醒词，走 TTS） */
};
#define VOICE_CMD_COUNT (sizeof(s_commands) / sizeof(s_commands[0]))
#define CMD_ID_FORWARD   0
#define CMD_ID_WAKE      1

/* 计算一帧 PCM 的 RMS（均方根），用于音量门控 */
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

/* 用中文 TTS 合成文字并通过扬声器播放（阻塞，语音播报用） */
static void voice_play_text(const char *text)
{
    if (s_tts == NULL) {
        ESP_LOGE(TAG, "TTS 未初始化，无法播放");
        return;
    }
    if (!esp_tts_parse_chinese(s_tts, text)) {
        ESP_LOGE(TAG, "TTS 解析失败: %s", text);
        return;
    }
    int len = 0;
    short *data = esp_tts_stream_play(s_tts, &len, 3);
    while (len > 0) {
        speaker_play(data, len, portMAX_DELAY);
        data = esp_tts_stream_play(s_tts, &len, 3);
    }
    esp_tts_stream_reset(s_tts);
}

/* ==================== 联网对话：录音（静音检测切段） ==================== */

#define RECORD_SAMPLE_RATE  16000   /* 与麦克风/云端一致 */
#define RECORD_CHUNK        512     /* 每次读 512 样本 ≈ 32ms */
#define RECORD_SILENCE_MS   900     /* 连续静音 900ms 判定说话结束 */
#define RECORD_MAX_MS       10000   /* 最长录音 10 秒 */
#define RECORD_WAIT_MAX     50      /* 最多等 50 次(≈5s)开始说话 */

/**
 * @brief 录音直到静音切段：等用户开口，检测到连续静音即停。
 * @return ESP_OK 成功，*pcm_out 由调用者 free；ESP_FAIL 无有效语音输入
 */
static esp_err_t voice_record(int16_t **pcm_out, size_t *samples_out)
{
    const size_t cap = (size_t)RECORD_SAMPLE_RATE * RECORD_MAX_MS / 1000;
    int16_t *pcm = malloc(cap * sizeof(int16_t));
    int16_t *tmp = malloc(RECORD_CHUNK * sizeof(int16_t));
    if (pcm == NULL || tmp == NULL) {
        free(pcm);
        free(tmp);
        return ESP_ERR_NO_MEM;
    }

    size_t total = 0;
    int silent_ms = 0;
    int no_speech = 0;
    bool started = false;

    while (total < cap) {
        size_t got = 0;
        if (microphone_read(tmp, RECORD_CHUNK, &got, pdMS_TO_TICKS(100)) != ESP_OK || got == 0) {
            continue;
        }
        float rms = rms_of(tmp, got);
        bool voice = (rms >= VOICE_RMS_THRESHOLD);

        if (!started) {
            if (voice) {
                started = true;
            } else if (++no_speech >= RECORD_WAIT_MAX) {
                ESP_LOGW(TAG, "唤醒后 5 秒未检测到语音，取消录音");
                free(pcm);
                free(tmp);
                return ESP_FAIL;
            } else {
                continue;
            }
        }

        memcpy(pcm + total, tmp, got * sizeof(int16_t));
        total += got;

        if (voice) {
            silent_ms = 0;
        } else {
            silent_ms += got * 1000 / RECORD_SAMPLE_RATE;
            if (silent_ms >= RECORD_SILENCE_MS) {
                break;
            }
        }
    }

    free(tmp);
    if (total < RECORD_SAMPLE_RATE / 2) {   /* 不足 0.5 秒，当作没说话 */
        free(pcm);
        return ESP_FAIL;
    }
    *pcm_out = pcm;
    *samples_out = total;
    return ESP_OK;
}

/* ==================== 识别任务 ==================== */
static void voice_task(void *arg)
{
    int chunksize = s_multinet->get_samp_chunksize(s_model_data);
    int16_t *buff = (int16_t *)malloc(chunksize * sizeof(int16_t));
    if (buff == NULL) {
        ESP_LOGE(TAG, "音频缓冲分配失败");
        vTaskDelete(NULL);
        return;
    }

    bool speaking = false;

    while (1) {
        size_t got = 0;
        if (microphone_read(buff, chunksize, &got, portMAX_DELAY) != ESP_OK ||
            got != (size_t)chunksize) {
            continue;
        }

        /* 1. 音量门控：低于阈值当静音/噪声，不喂识别 */
        float rms = rms_of(buff, chunksize);
        if (rms < VOICE_RMS_THRESHOLD) {
            speaking = false;
            continue;
        }

        if (!speaking) {
            ESP_LOGI(TAG, "检测到语音，开始识别");
            speaking = true;
        }

        /* 2. 喂 MultiNet 识别命令词 */
        esp_mn_state_t st = s_multinet->detect(s_model_data, buff);
        if (st == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *res = s_multinet->get_results(s_model_data);
            int id = (res && res->num > 0) ? res->command_id[0] : -1;
            if (id == CMD_ID_FORWARD) {
                /* 手机连上时忽略语音命令（听手机），避免断联切回语音模式的
                 * 瞬间，消费到手机连接期间残留锁存的命令而误触发 */
                if (!wife_is_connected()) {
                    s_cmd = VOICE_CMD_FORWARD;
                    ESP_LOGI(TAG, "识别到命令: %s", s_commands[id].pinyin);
                }
            } else if (id == CMD_ID_WAKE) {
                ESP_LOGI(TAG, "识别到唤醒词: %s", s_commands[id].pinyin);
                /* 手机连上时忽略唤醒词（听手机），没连上才走联网交流 */
                if (!wife_is_connected()) {
                    voice_play_text("我在");

                    /* 联网交流：录音 → ASR → LLM → TTS → 播放 */
                    int16_t *pcm = NULL;
                    size_t samples = 0;
                    if (voice_record(&pcm, &samples) != ESP_OK) {
                        ESP_LOGW(TAG, "录音失败或无语音输入");
                        continue;
                    }

                    char text[256];
                    if (cloud_asr(pcm, samples, text, sizeof(text)) != ESP_OK) {
                        ESP_LOGE(TAG, "云端 ASR 失败");
                        free(pcm);
                        continue;
                    }
                    free(pcm);
                    ESP_LOGI(TAG, "识别文字: %s", text);

                    char reply[512];
                    if (cloud_llm(text, reply, sizeof(reply)) != ESP_OK) {
                        ESP_LOGE(TAG, "云端 LLM 失败");
                        continue;
                    }
                    ESP_LOGI(TAG, "LLM 回复: %s", reply);

                    int16_t *tts_pcm = NULL;
                    size_t tts_samples = 0;
                    if (cloud_tts(reply, &tts_pcm, &tts_samples) != ESP_OK) {
                        ESP_LOGE(TAG, "云端 TTS 失败");
                        continue;
                    }
                    speaker_play(tts_pcm, tts_samples, portMAX_DELAY);
                    free(tts_pcm);
                }
            }
        }
    }
}

/* ==================== 对外接口 ==================== */

esp_err_t voice_init(void)
{
    /* 1. 麦克风 */
    esp_err_t ret = microphone_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "麦克风初始化失败 0x%x", ret);
        return ret;
    }

    /* 1.5 扬声器 + 中文 TTS（用于唤醒后播放「我在」），失败不阻断语音识别 */
    if (speaker_init() != ESP_OK) {
        ESP_LOGW(TAG, "扬声器初始化失败，唤醒词将无语音回应");
    }
    esp_tts_voice_t *tts_voice = esp_tts_voice_set_init(&esp_tts_voice_xiaole, NULL);
    if (tts_voice == NULL) {
        ESP_LOGW(TAG, "TTS 音色初始化失败");
    } else {
        s_tts = esp_tts_create(tts_voice);
        if (s_tts == NULL) {
            ESP_LOGW(TAG, "TTS 创建失败");
        }
    }

    /* 2. 挂载模型 SPIFFS 分区（label = "model"） */
    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL) {
        ESP_LOGE(TAG, "模型 SPIFFS 分区挂载失败（是否已烧录 model 分区？）");
        return ESP_FAIL;
    }

    /* 3. 取中文 MultiNet 模型名 */
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (mn_name == NULL) {
        ESP_LOGE(TAG, "未找到中文 MultiNet 模型");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "使用模型: %s", mn_name);

    /* 4. 获取句柄并创建模型 */
    s_multinet = esp_mn_handle_from_name(mn_name);
    if (s_multinet == NULL) {
        ESP_LOGE(TAG, "MultiNet 句柄获取失败");
        return ESP_FAIL;
    }
    s_model_data = s_multinet->create(mn_name, 3000);
    if (s_model_data == NULL) {
        ESP_LOGE(TAG, "MultiNet 模型创建失败");
        return ESP_FAIL;
    }

    /* 5. 注册命令词（update 成功后返回 NULL；非 NULL 为错误） */
    esp_mn_commands_alloc();
    for (int i = 0; i < (int)VOICE_CMD_COUNT; i++) {
        esp_mn_commands_add(i, (char *)s_commands[i].pinyin);
    }
    esp_mn_error_t *err = esp_mn_commands_update(s_multinet, s_model_data);
    if (err != NULL) {
        ESP_LOGW(TAG, "有 %d 个命令词无法被模型解析", err->num);
    }
    /* 链表保留不 free，避免 set_speech_commands 内部仍引用 */

    /* 6. 启动识别任务 */
    xTaskCreate(voice_task, "voice_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "语音命令词识别已启动");
    return ESP_OK;
}

voice_cmd_t voice_get_cmd(void)
{
    return s_cmd;
}

void voice_clear_cmd(void)
{
    s_cmd = VOICE_CMD_NONE;
}
