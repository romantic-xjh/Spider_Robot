#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 阿里云百炼（DashScope）配置 ==================== */
/* TODO: 填你自己的 API Key（在阿里云百炼控制台 https://bailian.console.aliyun.com 获取） */
#define DASHSCOPE_API_KEY   "sk-ws-H.XMDIXEX.50pl.MEQCIFFrdEGONxx_g7wxQsOccuMo1INONYemvIaSE53qkWLVAiBGKWjgnWS-AGQHsh3cTyDdDifcUONGjcHOiVoCSb09Uw"

/* 模型名：如果你的账号里没有下面这些模型，请换成控制台里可用的模型名 */
#define ASR_MODEL           "qwen-audio-3.1-asr-flash"   // 语音识别（语音转文字）
#define LLM_MODEL           "qwen-plus"                  // 对话大模型
#define TTS_MODEL           "cosyvoice-v3-plus"          // 语音合成
#define TTS_VOICE           "longanyang"                 // 音色（可在控制台试听其他音色）

/* 音频统一采样率：麦克风、扬声器、云端三方一致，都是 16kHz */
#define CLOUD_SAMPLE_RATE   16000

/* ==================== 接口 ==================== */

/**
 * 语音识别：把 PCM 音频（16bit/16kHz/单声道）转成文字
 * @param pcm           PCM 采样数据
 * @param samples       样本数
 * @param text_out      输出识别文字
 * @param text_out_len  输出缓冲区大小
 */
esp_err_t cloud_asr(const int16_t *pcm, size_t samples, char *text_out, size_t text_out_len);

/**
 * 大模型对话：把用户文字发给大模型，得到回复文字
 */
esp_err_t cloud_llm(const char *user_text, char *reply_out, size_t reply_out_len);

/**
 * 语音合成：把文字合成为 PCM 音频（16bit/16kHz/单声道）
 * @param text          待合成的文字
 * @param pcm_out       输出 PCM 数据（调用者负责 free）
 * @param samples_out   输出样本数
 */
esp_err_t cloud_tts(const char *text, int16_t **pcm_out, size_t *samples_out);

#ifdef __cplusplus
}
#endif
