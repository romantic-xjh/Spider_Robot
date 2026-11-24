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

    while (1) {
        if (!s_motor_ok) {
            vTaskDelay(pdMS_TO_TICKS(100));
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
    /* 1. WiFi(AP+STA) + TCP 遥控（wife_init 内部完成 nvs/netif/event loop） */
    ESP_ERROR_CHECK(wife_init());

    /* 2. 舵机驱动 + 上电站立（失败只禁用动作，不影响联网/语音） */
    esp_err_t motor_err = pca9685_servo_init();
    s_motor_ok = (motor_err == ESP_OK);
    if (s_motor_ok) {
        motor_stand();
    } else {
        ESP_LOGE(TAG, "PCA9685 初始化失败(0x%x)，动作功能禁用", motor_err);
    }

    /* 3. 本地离线命令词识别（失败则仅手机遥控可用） */
    esp_err_t voice_err = voice_init();
    if (voice_err != ESP_OK) {
        ESP_LOGW(TAG, "语音识别初始化失败(0x%x)，仅手机遥控可用", voice_err);
    }

    /* 4. 电机任务（唯一 motor_* 执行者） */
    xTaskCreate(motor_task, "motor_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "启动完成：上电静默，等待手机连接或语音命令");
}
