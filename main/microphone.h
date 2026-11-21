#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* INMP441 麦克风参数 */
#define MIC_SAMPLE_RATE     16000          // 采样率：16kHz（语音识别标准采样率）
#define MIC_CHANNELS        1              // 单声道

/* INMP441 引脚定义（接 ESP32-S3）
 * INMP441 是 I2S 从机，SCK/WS 由 ESP32 作为主机提供，SD 是麦克风数据输出
 */
#define MIC_I2S_SCK_GPIO    GPIO_NUM_6     // INMP441 SCK  -> I2S BCLK（位时钟）
#define MIC_I2S_WS_GPIO     GPIO_NUM_7     // INMP441 WS   -> I2S WS/LRC（左右声道时钟）
#define MIC_I2S_SD_GPIO     GPIO_NUM_8     // INMP441 SD   -> I2S DIN（数据输入到 ESP32）

/**
 * 初始化 I2S 麦克风（INMP441）
 */
esp_err_t microphone_init(void);

/**
 * 读取一帧 PCM 数据（16bit 单声道）
 * @param buf           输出缓冲区（int16_t 数组）
 * @param samples       期望读取的样本数
 * @param samples_read  实际读到的样本数（可为 NULL）
 * @param timeout       阻塞超时时间
 */
esp_err_t microphone_read(int16_t *buf, size_t samples, size_t *samples_read, TickType_t timeout);

/**
 * 释放 I2S 麦克风资源
 */
esp_err_t microphone_deinit(void);

#ifdef __cplusplus
}
#endif
