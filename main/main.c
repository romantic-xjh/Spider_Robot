/**
 * @file main.c
 * @brief 四足机器人主程序：状态机 + 电机任务
 *
 * 状态机（满足需求）：
 *   - 上电默认静默（只建立站立姿态，不做任何动作）；
 *   - 手机连上（TCP）→ 只听手机，忽略语音；
 *   - 手机没连上 + 语音命令词 → 走 8 秒后站立，继续监听。
 *
 * 电机任务是所有 motor_* 调用的唯一执行者：四足步态是阻塞式插值驱动，
 * 若手机、语音两处都直接调 motor_*，会并发抢 PCA9685 的 I2C 总线，
 * 所以统一由本任务串行执行。
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "wife.h"
#include "voice.h"
#include "pca9685_servo.h"
#include "motor.h"

static const char *TAG = "main";

/* 语音命令行走时长：1 个步态周期 ≈ 1.6s，5 个周期 ≈ 8s */
#define VOICE_WALK_CYCLES   5

/* 舵机驱动是否可用（PCA9685 初始化失败时禁用所有动作） */
static volatile bool s_motor_ok = false;

/* 内部统一动作编号 */
enum {
    ACT_STAND = 0,
    ACT_FORWARD,
    ACT_BACKWARD,
    ACT_LEFT,
    ACT_RIGHT,
};

/* 走一个步态周期（≈1.6s），供连续遥控与 8s 语音复用 */
static void walk_one_cycle(int act)
{
    switch (act) {
    case ACT_FORWARD:  motor_walk_forward(1);  break;
    case ACT_BACKWARD: motor_walk_backward(1); break;
    case ACT_LEFT:     motor_walk_left(1);     break;
    case ACT_RIGHT:    motor_walk_right(1);    break;
    default: break;
    }
}

/* 手机指令 -> 动作编号 */
static int wife_cmd_to_act(wife_cmd_t c)
{
    switch (c) {
    case WIFE_CMD_FORWARD:  return ACT_FORWARD;
    case WIFE_CMD_BACKWARD: return ACT_BACKWARD;
    case WIFE_CMD_LEFT:     return ACT_LEFT;
    case WIFE_CMD_RIGHT:    return ACT_RIGHT;
    case WIFE_CMD_STAND:
    case WIFE_CMD_NONE:
    default:                return ACT_STAND;
    }
}

/* 语音命令 -> 动作编号 */
static int voice_cmd_to_act(voice_cmd_t c)
{
    switch (c) {
    case VOICE_CMD_FORWARD:  return ACT_FORWARD;
    case VOICE_CMD_BACKWARD: return ACT_BACKWARD;
    case VOICE_CMD_LEFT:     return ACT_LEFT;
    case VOICE_CMD_RIGHT:    return ACT_RIGHT;
    default:                 return ACT_STAND;
    }
}

/* ==================== 电机任务 ==================== */
static void motor_task(void *arg)
{
    int last_act = ACT_STAND;   /* 上次动作，避免重复 motor_stand */

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

/* ==================== 动作指令（语音 -> motor 动作映射） ==================== */

#define MOTOR_CMD_CYCLES   2     /* 语音指令默认步态周期数（1 周期 ≈ 1.6s） */
#define MOTOR_SETTLE_MS    500   /* 动作结束后等舵机噪声衰减，防止麦克风误触发 */

typedef enum {
    MOTOR_ACTION_NONE = -1,
    MOTOR_ACTION_STAND = 0,   /* 站立 / 回静止姿态 */
    MOTOR_ACTION_RELEASE,     /* 全部舵机失力 */
    MOTOR_ACTION_FORWARD,     /* 前进 */
    MOTOR_ACTION_BACKWARD,    /* 后退 */
    MOTOR_ACTION_LEFT,        /* 左移 */
    MOTOR_ACTION_RIGHT,       /* 右移 */
    MOTOR_ACTION_TURN_LEFT,   /* 左转向 */
    MOTOR_ACTION_TURN_RIGHT,  /* 右转向 */
} motor_action_t;

static const char *g_cmd_stand[]    = { "站立", "站好", "站起来", "起立", "停下", "停住" };
static const char *g_cmd_release[]  = { "放松", "休息", "趴下" };
static const char *g_cmd_forward[]  = { "前进", "向前走", "往前走" };
static const char *g_cmd_backward[] = { "后退", "向后走", "往后退" };
static const char *g_cmd_left[]     = { "左移", "向左走", "往左走" };
static const char *g_cmd_right[]    = { "右移", "向右走", "往右走" };
static const char *g_cmd_turn_left[]  = { "左转", "向左转", "往左转" };
static const char *g_cmd_turn_right[] = { "右转", "向右转", "往右转" };

typedef struct {
    const char   **words;
    size_t         count;
    motor_action_t action;
} motor_cmd_entry_t;

#define CMD_ENTRY(arr, act)  { arr, sizeof(arr) / sizeof(arr[0]), act }

static const motor_cmd_entry_t g_motor_cmds[] = {
    CMD_ENTRY(g_cmd_stand,    MOTOR_ACTION_STAND),
    CMD_ENTRY(g_cmd_release,  MOTOR_ACTION_RELEASE),
    CMD_ENTRY(g_cmd_forward,  MOTOR_ACTION_FORWARD),
    CMD_ENTRY(g_cmd_backward, MOTOR_ACTION_BACKWARD),
    CMD_ENTRY(g_cmd_left,     MOTOR_ACTION_LEFT),
    CMD_ENTRY(g_cmd_right,    MOTOR_ACTION_RIGHT),
    CMD_ENTRY(g_cmd_turn_left,  MOTOR_ACTION_TURN_LEFT),
    CMD_ENTRY(g_cmd_turn_right, MOTOR_ACTION_TURN_RIGHT),
};
#define MOTOR_CMD_COUNT  (sizeof(g_motor_cmds) / sizeof(g_motor_cmds[0]))

/* 在句子里找动作指令词（子串匹配），命中返回动作，否则 MOTOR_ACTION_NONE */
static motor_action_t match_motor_cmd(const char *text)
{
    for (size_t i = 0; i < MOTOR_CMD_COUNT; i++) {
        for (size_t j = 0; j < g_motor_cmds[i].count; j++) {
            if (strstr(text, g_motor_cmds[i].words[j]) != NULL) {
                return g_motor_cmds[i].action;
            }
        }
    }
    return MOTOR_ACTION_NONE;
}

/* 执行动作（阻塞式：动作期间不响应语音，走完才返回） */
static void run_motor_action(motor_action_t action)
{
    esp_err_t ret = ESP_OK;
    switch (action) {
    case MOTOR_ACTION_STAND:    ret = motor_stand();                       break;
    case MOTOR_ACTION_RELEASE:  ret = motor_release();                     break;
    case MOTOR_ACTION_FORWARD:  ret = motor_walk_forward(MOTOR_CMD_CYCLES);  break;
    case MOTOR_ACTION_BACKWARD: ret = motor_walk_backward(MOTOR_CMD_CYCLES); break;
    case MOTOR_ACTION_LEFT:     ret = motor_walk_left(MOTOR_CMD_CYCLES);     break;
    case MOTOR_ACTION_RIGHT:    ret = motor_walk_right(MOTOR_CMD_CYCLES);    break;
    case MOTOR_ACTION_TURN_LEFT:  ret = motor_turn_left(MOTOR_CMD_CYCLES);   break;
    case MOTOR_ACTION_TURN_RIGHT: ret = motor_turn_right(MOTOR_CMD_CYCLES);  break;
    default: break;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "动作执行失败: 0x%x", (int)ret);
    }
}

/* ==================== 语音处理 ==================== */

/* 录音 → ASR → 匹配唤醒词 → LLM → TTS → 播放。返回是否真正唤醒 */
static bool handle_speech(int16_t *pcm)
{
    /* 1~2. 录音 → ASR（失败播提示音重试，最多 ASR_RETRY_COUNT 次重试） */
    static char text[512];
    bool recognized = false;
    for (int attempt = 0; attempt <= ASR_RETRY_COUNT; attempt++) {
        if (attempt > 0) {
            /* 重试前播提示音，等 150ms 让回声衰减 */
            int16_t *a = NULL;
            size_t n = 0;
            if (cloud_tts("请再说一次", &a, &n) == ESP_OK) {
                speaker_play(a, n, portMAX_DELAY);
                free(a);
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        size_t total = 0;
        microphone_read(pcm, RECORD_SAMPLES, &total,
                        pdMS_TO_TICKS(RECORD_SECONDS * 1000 + 500));
        if (total == 0) {
            ESP_LOGW(TAG, "录音为空（第 %d/%d 次）", attempt + 1, ASR_RETRY_COUNT + 1);
            continue;
        }

        if (wife_is_connected()) {
            /* ---- 手机模式：连续走，指令锁存直到 S/换向/断开 ---- */
            /* 手机模式下持续清掉语音命令，保证断开切回语音模式时
             * voice_get_cmd() 一定是干净状态（不残留连接建立前锁存的命令） */
            voice_clear_cmd();
            int act = wife_cmd_to_act(wife_get_cmd());
            if (act == ACT_STAND) {
                if (last_act != ACT_STAND) {
                    motor_stand();
                    last_act = ACT_STAND;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            } else {
                walk_one_cycle(act);
                last_act = act;
            }
        } else {
            /* ---- 语音模式：识别到命令词走 8s，然后站立 ---- */
            voice_cmd_t vc = voice_get_cmd();
            if (vc == VOICE_CMD_NONE) {
                if (last_act != ACT_STAND) {
                    motor_stand();
                    last_act = ACT_STAND;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            } else {
                voice_clear_cmd();
                int act = voice_cmd_to_act(vc);
                ESP_LOGI(TAG, "语音命令 %d -> 走 %d 个周期(≈8s)", (int)vc, VOICE_WALK_CYCLES);
                for (int i = 0; i < VOICE_WALK_CYCLES; i++) {
                    walk_one_cycle(act);
                    if (wife_is_connected()) {
                        break;   /* 中途手机连上则立即让位给手机 */
                    }
                }
                motor_stand();
                last_act = ACT_STAND;
            }
        }
    }
}

/* ==================== 主程序 ==================== */
void app_main(void)
{
    ESP_ERROR_CHECK(pca9685_servo_init());

    motor_stand();
    vTaskDelay(pdMS_TO_TICKS(1000));
    uint8_t a = 0;

    while (1)
    {
        if(a == 0)
        {
            a = 1;
            TickType_t start = xTaskGetTickCount();

            while (xTaskGetTickCount() - start < pdMS_TO_TICKS(10000))
            {
                motor_walk_right(1); /* 实时时钟每次进循环时由函数内部读取 */
            }

            motor_stand();
        }
    }

    // /* 基础初始化 */
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // /* 连接 WiFi */
    // wifi_init_sta();

    // /* 同步系统时间（HTTPS 证书校验需要） */
    // sntp_sync_time();

    // /* 初始化音频外设 */
    // ESP_ERROR_CHECK(microphone_init());
    // ESP_ERROR_CHECK(speaker_init());

    // /* 初始化舵机驱动（PCA9685）：失败只禁用动作功能，不影响语音 */
    // esp_err_t motor_err = pca9685_servo_init();
    // s_motor_ok = (motor_err == ESP_OK);
    // if (s_motor_ok) {
    //     motor_stand();   /* 上电建立站立姿态 */
    // } else {
    //     ESP_LOGE(TAG, "PCA9685 初始化失败(0x%x)，动作功能禁用", motor_err);
    // }

    // int16_t *frame = malloc(FRAME_SAMPLES * sizeof(int16_t));
    // int16_t *pcm   = malloc(RECORD_SAMPLES * sizeof(int16_t));
    // if (frame == NULL || pcm == NULL) {
    //     ESP_LOGE(TAG, "内存分配失败");
    //     return;
    // }

    // ESP_LOGI(TAG, "启动");

    // while (1) {
    //     /* 读一帧音频（20 ms） */
    //     size_t got = 0;
    //     if (microphone_read(frame, FRAME_SAMPLES, &got, portMAX_DELAY) != ESP_OK || got == 0) {
    //         continue;
    //     }

    //     float rms = rms_of(frame, got);

    //     /* 检测到说话 → 录音并处理 */
    //     if (rms > TRIGGER_RMS_THRESHOLD) {
    //         ESP_LOGI(TAG, "收到语音");
    //         bool woke = handle_speech(pcm);
    //         if (woke) {
    //             s_state     = APP_ACTIVE;
    //             s_silent_ms = 0;
    //         }
    //         continue;
    //     }

    //     /* 活跃状态下检测静音，超时进入休眠 */
    //     if (s_state == APP_ACTIVE) {
    //         if (rms < SILENCE_RMS_THRESHOLD) {
    //             s_silent_ms += 20;      /* 每帧 20 ms */
    //             if (s_silent_ms >= SLEEP_AFTER_SILENCE_MS) {
    //                 s_state     = APP_SLEEP;
    //                 s_silent_ms = 0;
    //                 ESP_LOGI(TAG, "休眠");
    //             }
    //         } else {
    //             s_silent_ms = 0;
    //         }
    //     }
    //     /* APP_SLEEP：只等触发 */
    // }
}
