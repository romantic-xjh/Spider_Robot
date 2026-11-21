#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MAX98357A 扬声器参数 */
#define SPK_SAMPLE_RATE     16000          // 播放采样率：16kHz（与麦克风/TTS 保持一致）

/* MAX98357A 引脚定义（接 ESP32-S3）
 * MAX98357A 是 I2S 从机，BCLK/LRC 由 ESP32 提供，DIN 接收数据
 */
#define SPK_I2S_BCLK_GPIO   GPIO_NUM_15    // BCLK（位时钟）
#define SPK_I2S_LRC_GPIO    GPIO_NUM_16    // LRC/WS（左右声道时钟）
#define SPK_I2S_DIN_GPIO    GPIO_NUM_17    // DIN（数据输出到功放）

/**
 * 初始化 I2S 扬声器（MAX98357A）
 */
esp_err_t speaker_init(void);

/**
 * 播放一段 PCM 数据（16bit 单声道）
 * @param buf      待播放的数据
 * @param samples  样本数
 * @param timeout  阻塞超时时间
 */
esp_err_t speaker_play(const int16_t *buf, size_t samples, TickType_t timeout);

/**
 * 释放 I2S 扬声器资源
 */
esp_err_t speaker_deinit(void);

#ifdef __cplusplus
}
#endif
