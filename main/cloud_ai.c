#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "mbedtls/base64.h"
#include "cloud_ai.h"
#include "mp3dec.h"

static const char *TAG = "cloud_ai";

/* 阿里云百炼（DashScope）接口地址 */
#define DASHSCOPE_HOST  "https://dashscope.aliyuncs.com"
#define ASR_URL         DASHSCOPE_HOST "/api/v1/services/aigc/multimodal-generation/generation"
#define LLM_URL         DASHSCOPE_HOST "/compatible-mode/v1/chat/completions"
#define TTS_URL         DASHSCOPE_HOST "/api/v1/services/audio/tts/SpeechSynthesizer"

/* ==================== HTTP 通用工具 ==================== */

typedef struct {
    uint8_t *data;
    size_t   len;
    size_t   cap;
} http_buf_t;

static esp_err_t http_buf_append(http_buf_t *buf, const char *data, int len)
{
    if (len <= 0) {
        return ESP_OK;
    }
    if (buf->len + len + 1 > buf->cap) {
        size_t new_cap = (buf->cap == 0) ? 4096 : buf->cap;
        while (new_cap < buf->len + len + 1) {
            new_cap *= 2;
        }
        uint8_t *p = realloc(buf->data, new_cap);
        if (p == NULL) {
            return ESP_ERR_NO_MEM;
        }
        buf->data = p;
        buf->cap  = new_cap;
    }
    memcpy(buf->data + buf->len, data, len);
    buf->len += len;
    buf->data[buf->len] = '\0';
    return ESP_OK;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_buf_t *buf = (http_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        return http_buf_append(buf, evt->data, evt->data_len);
    }
    return ESP_OK;
}

/**
 * 通用 HTTP 请求。返回的 *resp_out 以 '\0' 结尾（*resp_len 不含 '\0'），
 * 既可当字符串解析 JSON，也可当二进制（配合 *resp_len）使用。调用者负责 free。
 */
static esp_err_t http_request(const char *url,
                              esp_http_client_method_t method,
                              const char *auth_header,
                              const char *body,
                              int body_len,
                              uint8_t **resp_out,
                              size_t *resp_len)
{
    http_buf_t buf = {0};

    esp_http_client_config_t cfg = {
        .url               = url,
        .method            = method,
        .event_handler     = http_event_handler,
        .user_data         = &buf,
        .crt_bundle_attach = esp_crt_bundle_attach,   // 使用系统证书 bundle，免去手动放证书
        .timeout_ms        = 30000,
        .buffer_size       = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return ESP_FAIL;
    }

    if (auth_header != NULL) {
        esp_http_client_set_header(client, "Authorization", auth_header);
    }
    /* 只有带 body 的 POST 才加 Content-Type；GET 下载音频时不能加，
     * 否则会破坏 OSS 签名 URL 的签名校验（403 SignatureDoesNotMatch） */
    if (body != NULL) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, body, body_len);
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status >= 200 && status < 300) {
            if (resp_out != NULL) {
                *resp_out = buf.data;
                buf.data  = NULL;   // 交给调用者
            } else {
                free(buf.data);
            }
            if (resp_len != NULL) {
                *resp_len = buf.len;
            }
            err = ESP_OK;
        } else {
            ESP_LOGE(TAG, "HTTP %d: %s", status, buf.len ? (char *)buf.data : "(no body)");
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    free(buf.data);
    esp_http_client_cleanup(client);
    return err;
}

/* ==================== 辅助函数 ==================== */

/* 写入 44 字节标准 PCM WAV 头 */
static void wav_header_write(uint8_t *buf, uint32_t sample_rate, uint32_t data_size)
{
    uint32_t byte_rate    = sample_rate * 2;      // 16bit 单声道
    uint16_t block_align  = 2;                    // channels * bits/8 = 1 * 2

    memcpy(buf + 0,  "RIFF", 4);
    uint32_t riff_size = 36 + data_size;
    memcpy(buf + 4,  &riff_size, 4);
    memcpy(buf + 8,  "WAVE", 4);
    memcpy(buf + 12, "fmt ", 4);
    uint32_t fmt_size = 16;
    memcpy(buf + 16, &fmt_size, 4);
    uint16_t audio_format = 1;                    // PCM
    memcpy(buf + 20, &audio_format, 2);
    uint16_t channels = 1;
    memcpy(buf + 22, &channels, 2);
    memcpy(buf + 24, &sample_rate, 4);
    memcpy(buf + 28, &byte_rate, 4);
    memcpy(buf + 32, &block_align, 2);
    uint16_t bits = 16;
    memcpy(buf + 34, &bits, 2);
    memcpy(buf + 36, "data", 4);
    memcpy(buf + 40, &data_size, 4);
}

/* base64 编码，out 由调用者 free */
static esp_err_t base64_encode(const uint8_t *src, size_t src_len, char **out, size_t *out_len)
{
    size_t olen = 0;
    mbedtls_base64_encode(NULL, 0, &olen, src, src_len);          // 计算长度
    char *dst = malloc(olen + 1);
    if (dst == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (mbedtls_base64_encode((unsigned char *)dst, olen + 1, &olen, src, src_len) != 0) {
        free(dst);
        return ESP_FAIL;
    }
    *out = dst;
    if (out_len != NULL) {
        *out_len = olen;
    }
    return ESP_OK;
}

/* 拼接鉴权头 "Bearer <key>" */
static void build_auth_header(char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "Bearer %s", DASHSCOPE_API_KEY);
}

/* ==================== 语音识别（ASR） ==================== */

esp_err_t cloud_asr(const int16_t *pcm, size_t samples, char *text_out, size_t text_out_len)
{
    esp_err_t err = ESP_FAIL;

    /* 1. 组 WAV：44 字节头 + PCM 数据 */
    size_t data_size = samples * sizeof(int16_t);
    size_t wav_size  = 44 + data_size;
    uint8_t *wav = malloc(wav_size);
    if (wav == NULL) {
        return ESP_ERR_NO_MEM;
    }
    wav_header_write(wav, CLOUD_SAMPLE_RATE, data_size);
    memcpy(wav + 44, pcm, data_size);

    /* 2. base64 编码 */
    char *b64 = NULL;
    size_t b64_len = 0;
    err = base64_encode(wav, wav_size, &b64, &b64_len);
    free(wav);
    if (err != ESP_OK) {
        return err;
    }

    /* 3. 构造请求 JSON（base64 data URI） */
    char *data_uri = malloc(b64_len + 32);
    if (data_uri == NULL) {
        free(b64);
        return ESP_ERR_NO_MEM;
    }
    snprintf(data_uri, b64_len + 32, "data:audio/wav;base64,%s", b64);
    free(b64);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", ASR_MODEL);

    cJSON *input = cJSON_CreateObject();
    cJSON *messages = cJSON_CreateArray();
    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "role", "user");
    cJSON *content = cJSON_CreateArray();
    cJSON *item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "type", "input_audio");
    cJSON *audio = cJSON_CreateObject();
    cJSON_AddStringToObject(audio, "data", data_uri);
    cJSON_AddItemToObject(item, "input_audio", audio);
    cJSON_AddItemToArray(content, item);
    cJSON_AddItemToObject(msg, "content", content);
    cJSON_AddItemToArray(messages, msg);
    cJSON_AddItemToObject(input, "messages", messages);
    cJSON_AddItemToObject(root, "input", input);

    cJSON *params = cJSON_CreateObject();
    cJSON_AddStringToObject(params, "format", "wav");
    cJSON_AddNumberToObject(params, "sample_rate", CLOUD_SAMPLE_RATE);
    cJSON_AddItemToObject(root, "parameters", params);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(data_uri);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 4. POST 请求 */
    char auth[128];
    build_auth_header(auth, sizeof(auth));
    uint8_t *resp = NULL;
    size_t resp_len = 0;
    err = http_request(ASR_URL, HTTP_METHOD_POST, auth, body, (int)strlen(body), &resp, &resp_len);
    free(body);
    if (err != ESP_OK) {
        return err;
    }

    /* 5. 解析 output.text */
    cJSON *jresp = cJSON_Parse((const char *)resp);
    free(resp);
    if (jresp == NULL) {
        return ESP_FAIL;
    }
    cJSON *output = cJSON_GetObjectItem(jresp, "output");
    cJSON *text   = output ? cJSON_GetObjectItem(output, "text") : NULL;
    if (cJSON_IsString(text)) {
        strncpy(text_out, text->valuestring, text_out_len - 1);
        text_out[text_out_len - 1] = '\0';
        err = ESP_OK;
    } else {
        ESP_LOGE(TAG, "ASR: no text in response");
        err = ESP_FAIL;
    }
    cJSON_Delete(jresp);
    return err;
}

/* ==================== 大模型对话（LLM） ==================== */

esp_err_t cloud_llm(const char *user_text, char *reply_out, size_t reply_out_len)
{
    esp_err_t err = ESP_FAIL;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", LLM_MODEL);

    cJSON *messages = cJSON_CreateArray();
    cJSON *sys = cJSON_CreateObject();
    cJSON_AddStringToObject(sys, "role", "system");
    cJSON_AddStringToObject(sys, "content", "你是一个友好的中文语音助手，回答简洁明了，不超过15个字。");
    cJSON_AddItemToArray(messages, sys);

    cJSON *user = cJSON_CreateObject();
    cJSON_AddStringToObject(user, "role", "user");
    cJSON_AddStringToObject(user, "content", user_text);
    cJSON_AddItemToArray(messages, user);

    cJSON_AddItemToObject(root, "messages", messages);
    cJSON_AddTrueToObject(root, "enable_search");   // 开启联网搜索能力

    /* 强制联网搜索：qwen-plus 默认会自行判断是否联网，问天气时经常判断失误不搜 */
    cJSON *search_options = cJSON_CreateObject();
    cJSON_AddTrueToObject(search_options, "forced_search");
    cJSON_AddItemToObject(root, "search_options", search_options);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "LLM 请求体: %s", body);

    char auth[128];
    build_auth_header(auth, sizeof(auth));
    uint8_t *resp = NULL;
    size_t resp_len = 0;
    err = http_request(LLM_URL, HTTP_METHOD_POST, auth, body, (int)strlen(body), &resp, &resp_len);
    free(body);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *jresp = cJSON_Parse((const char *)resp);
    free(resp);
    if (jresp == NULL) {
        return ESP_FAIL;
    }
    cJSON *choices = cJSON_GetObjectItem(jresp, "choices");
    cJSON *choice  = choices ? cJSON_GetArrayItem(choices, 0) : NULL;
    cJSON *message = choice ? cJSON_GetObjectItem(choice, "message") : NULL;
    cJSON *content = message ? cJSON_GetObjectItem(message, "content") : NULL;
    if (cJSON_IsString(content)) {
        strncpy(reply_out, content->valuestring, reply_out_len - 1);
        reply_out[reply_out_len - 1] = '\0';
        err = ESP_OK;
    } else {
        ESP_LOGE(TAG, "LLM: no content in response");
        err = ESP_FAIL;
    }
    cJSON_Delete(jresp);
    return err;
}

/* ==================== 语音合成（TTS） ==================== */

/* 跳过 ID3v2 标签，返回真正 mp3 帧的起始位置和长度 */
static const uint8_t *skip_id3v2(const uint8_t *data, size_t len, size_t *out_len)
{
    if (len > 10 && data[0] == 'I' && data[1] == 'D' && data[2] == '3') {
        /* ID3v2 头：10 字节 + synchsafe 大小 */
        uint32_t tag_size = ((uint32_t)data[6] << 21) |
                            ((uint32_t)data[7] << 14) |
                            ((uint32_t)data[8] << 7)  |
                            ((uint32_t)data[9]);
        size_t skip = 10 + tag_size;
        if (data[5] & 0x10) {   /* footer present (ID3v2.4) */
            skip += 10;
        }
        if (skip < len) {
            *out_len = len - skip;
            return data + skip;
        }
    }
    *out_len = len;
    return data;
}

/* 把 MP3 数据解码成 16bit PCM，*pcm_out 由调用者 free */
static esp_err_t mp3_to_pcm(const uint8_t *mp3_data, size_t mp3_len,
                            int16_t **pcm_out, size_t *samples_out)
{
    HMP3Decoder dec = MP3InitDecoder();
    if (dec == NULL) {
        return ESP_FAIL;
    }

    /* 诊断：打印 mp3 开头字节，确认是否从帧同步字（0xFF）开始 */
    ESP_LOGI(TAG, "mp3 头: %02X %02X %02X %02X %02X %02X %02X %02X",
             mp3_len > 0 ? mp3_data[0] : 0, mp3_len > 1 ? mp3_data[1] : 0,
             mp3_len > 2 ? mp3_data[2] : 0, mp3_len > 3 ? mp3_data[3] : 0,
             mp3_len > 4 ? mp3_data[4] : 0, mp3_len > 5 ? mp3_data[5] : 0,
             mp3_len > 6 ? mp3_data[6] : 0, mp3_len > 7 ? mp3_data[7] : 0);

    /* 跳过可能的 ID3v2 标签，从真正的 mp3 帧开始 */
    size_t body_len = 0;
    const uint8_t *body = skip_id3v2(mp3_data, mp3_len, &body_len);

    unsigned char *in = (unsigned char *)body;
    int bytes_left = (int)body_len;

    /* 输出缓冲：动态扩容 */
    size_t cap = 8192;
    int16_t *out = malloc(cap * sizeof(int16_t));
    if (out == NULL) {
        MP3FreeDecoder(dec);
        return ESP_ERR_NO_MEM;
    }
    size_t total = 0;

    /* 单帧解码缓冲：MAX_NCHAN(2) * MAX_NSAMP(576) * MAX_NGRAN(2) = 2304 个 short，
     * 用堆分配，避免 4608 字节爆掉 main task 的 3584 字节栈 */
    short *frame = malloc(MAX_NCHAN * MAX_NSAMP * MAX_NGRAN * sizeof(short));
    if (frame == NULL) {
        free(out);
        MP3FreeDecoder(dec);
        return ESP_ERR_NO_MEM;
    }

    MP3FrameInfo fi;

    while (bytes_left > 0) {
        /* MP3Decode 返回 0 = 成功解码一帧，负数 = 错误 */
        int err = MP3Decode(dec, &in, &bytes_left, frame, 0);
        if (err != 0) {
            ESP_LOGW(TAG, "MP3Decode 错误 %d（剩余 %d 字节，已解 %d 样本）", err, bytes_left, (int)total);
            break;
        }
        MP3GetLastFrameInfo(dec, &fi);
        int n = fi.outputSamps;
        if (n <= 0) {
            break;
        }
        if (total + (size_t)n > cap) {
            while (cap < total + (size_t)n) {
                cap *= 2;
            }
            int16_t *p = realloc(out, cap * sizeof(int16_t));
            if (p == NULL) {
                free(out);
                free(frame);
                MP3FreeDecoder(dec);
                return ESP_ERR_NO_MEM;
            }
            out = p;
        }
        memcpy(out + total, frame, n * sizeof(short));
        total += n;
    }

    free(frame);
    MP3FreeDecoder(dec);

    if (total == 0) {
        free(out);
        return ESP_FAIL;
    }
    *pcm_out = out;
    *samples_out = total;
    return ESP_OK;
}

esp_err_t cloud_tts(const char *text, int16_t **pcm_out, size_t *samples_out)
{
    esp_err_t err = ESP_FAIL;

    /* 1. 构造请求 */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", TTS_MODEL);
    cJSON *input = cJSON_CreateObject();
    cJSON_AddStringToObject(input, "text", text);
    cJSON_AddStringToObject(input, "voice", TTS_VOICE);
    cJSON_AddStringToObject(input, "format", "mp3");
    cJSON_AddNumberToObject(input, "sample_rate", CLOUD_SAMPLE_RATE);
    cJSON_AddItemToObject(root, "input", input);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 2. POST 请求，拿音频下载地址 */
    char auth[128];
    build_auth_header(auth, sizeof(auth));
    uint8_t *resp = NULL;
    size_t resp_len = 0;
    err = http_request(TTS_URL, HTTP_METHOD_POST, auth, body, (int)strlen(body), &resp, &resp_len);
    free(body);
    if (err != ESP_OK) {
        return err;
    }

    cJSON *jresp = cJSON_Parse((const char *)resp);
    free(resp);
    if (jresp == NULL) {
        return ESP_FAIL;
    }
    cJSON *output = cJSON_GetObjectItem(jresp, "output");
    cJSON *audio  = output ? cJSON_GetObjectItem(output, "audio") : NULL;
    cJSON *url    = audio ? cJSON_GetObjectItem(audio, "url") : NULL;
    static char audio_url[1024];
    if (!cJSON_IsString(url)) {
        ESP_LOGE(TAG, "TTS: no audio url in response");
        cJSON_Delete(jresp);
        return ESP_FAIL;
    }
    strncpy(audio_url, url->valuestring, sizeof(audio_url) - 1);
    audio_url[sizeof(audio_url) - 1] = '\0';
    cJSON_Delete(jresp);

    /* 3. 下载音频（mp3 压缩数据） */
    uint8_t *audio_data = NULL;
    size_t audio_len = 0;
    err = http_request(audio_url, HTTP_METHOD_GET, NULL, NULL, 0, &audio_data, &audio_len);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "下载音频 %d 字节（mp3）", (int)audio_len);

    /* 4. 解码 mp3 → pcm */
    int16_t *pcm = NULL;
    size_t pcm_samples = 0;
    esp_err_t dec_err = mp3_to_pcm(audio_data, audio_len, &pcm, &pcm_samples);
    free(audio_data);   /* mp3 原始数据用完即弃 */
    if (dec_err != ESP_OK) {
        ESP_LOGE(TAG, "mp3 解码失败");
        return ESP_FAIL;
    }

    *pcm_out     = pcm;
    *samples_out = pcm_samples;
    return ESP_OK;
}
