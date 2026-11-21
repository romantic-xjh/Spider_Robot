#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_std.h"
#include "speaker.h"

static const char *TAG = "speaker";

static i2s_chan_handle_t s_tx_chan = NULL;

esp_err_t speaker_init(void)
{
    /* 使用 I2S1 作为主机发送（TX） */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL), TAG, "i2s_new_channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SPK_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_I2S_BCLK_GPIO,
            .ws   = SPK_I2S_LRC_GPIO,
            .dout = SPK_I2S_DIN_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &std_cfg), TAG, "i2s_channel_init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx_chan), TAG, "i2s_channel_enable failed");

    ESP_LOGI(TAG, "init done, BCLK=%d LRC=%d DIN=%d @%dHz", SPK_I2S_BCLK_GPIO, SPK_I2S_LRC_GPIO, SPK_I2S_DIN_GPIO, SPK_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t speaker_play(const int16_t *buf, size_t samples, TickType_t timeout)
{
    if (s_tx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(s_tx_chan, buf, samples * sizeof(int16_t), &bytes_written, timeout);
    ESP_LOGI(TAG, "I2S 写入 %d / %d 字节", (int)bytes_written, (int)(samples * sizeof(int16_t)));
    return ret;
}

esp_err_t speaker_deinit(void)
{
    if (s_tx_chan != NULL) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }
    return ESP_OK;
}
