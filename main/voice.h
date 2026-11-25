/**
 * @file voice.h
 * @brief 本地离线语音命令词识别接口
 *
 * 基于 ESP-SR 的 MultiNet 命令词模型，在本地（不联网）识别四足机器人的
 * 四个动作命令词：前进 / 后退 / 向左走 / 向右走。识别结果锁存，由
 * main.c 的电机任务轮询消费。
 *
 * 说明：voice.c 内部自行完成麦克风初始化 + 音频喂入 + 命令词检测，
 * 并在 voice_init() 里启动一个独立任务持续监听；不直接驱动电机。
 */
#ifndef VOICE_H
#define VOICE_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 语音命令词 */
typedef enum {
    VOICE_CMD_NONE = 0,   /* 无命令 */
    VOICE_CMD_FORWARD,    /* 前进 */
    VOICE_CMD_BACKWARD,   /* 后退 */
    VOICE_CMD_LEFT,       /* 向左走 */
    VOICE_CMD_RIGHT,      /* 向右走 */
} voice_cmd_t;

/**
 * @brief 初始化麦克风 + MultiNet 命令词识别，并启动监听任务
 *
 * @return ESP_OK 成功；其他：模型初始化失败 / 麦克风失败
 */
esp_err_t voice_init(void);

/**
 * @brief 读取最近识别到的命令词（锁存，直到 voice_clear_cmd）
 */
voice_cmd_t voice_get_cmd(void);

/**
 * @brief 清除命令词（消费后调用，防止重复触发）
 */
void voice_clear_cmd(void);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_H */
