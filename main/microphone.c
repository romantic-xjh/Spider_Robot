#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_std.h"
#include "microphone.h"

static const char *TAG = "microphone";

static i2s_chan_handle_t s_rx_chan = NULL;

esp_err_t microphone_init(void)
{
    /* 使用 I2S0 作为主机接收（RX） */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "i2s_new_channel failed");

    /*
     * INMP441 输出 24 bit 数据，左对齐在 32 bit 帧内。
     * 所以 slot 必须用 32 bit，读出来再右移 16 位取高 16 bit。
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                       I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_I2S_SCK_GPIO,
            .ws   = MIC_I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_I2S_SD_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    /* INMP441 的 L/R 接 GND = 左声道，只取左槽 */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "i2s_channel_init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "i2s_channel_enable failed");

    ESP_LOGI(TAG, "init done, SCK=%d WS=%d SD=%d @%dHz (32bit slot)",
             MIC_I2S_SCK_GPIO, MIC_I2S_WS_GPIO, MIC_I2S_SD_GPIO, MIC_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t microphone_read(int16_t *buf, size_t samples, size_t *samples_read, TickType_t timeout)
{
    if (s_rx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t tmp[128];   /* 小块 32bit 缓冲，避免大栈占用 */
    size_t  total = 0;

    while (total < samples) {
        size_t chunk = (samples - total < 128) ? (samples - total) : 128;
        size_t bytes = 0;
        esp_err_t ret = i2s_channel_read(s_rx_chan, tmp, chunk * sizeof(int32_t), &bytes, timeout);
        if (ret != ESP_OK) {
            break;
        }
        size_t n = bytes / sizeof(int32_t);
        /* INMP441 24 bit 左对齐在 32 bit 帧内，取高 16 bit */
        for (size_t i = 0; i < n; i++) {
            buf[total + i] = (int16_t)(tmp[i] >> 16);
        }
        total += n;
        if (n < chunk) {
            break;
        }
    }

    if (samples_read != NULL) {
        *samples_read = total;
    }
    return ESP_OK;
}

esp_err_t microphone_deinit(void)
{
    if (s_rx_chan != NULL) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    return ESP_OK;
}
