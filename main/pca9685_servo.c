/**
 * @file pca9685_servo.c
 * @brief PCA9685 舵机驱动板驱动实现（ESP-IDF v5.1，ESP32-S3）
 *
 * PCA9685 关键寄存器（数据手册 PCA9685_-_ENG (1).pdf）：
 *   MODE1   (0x00)：bit4 SLEEP 置 1 才能写预分频寄存器；
 *                   bit5 AI 自动递增（一次事务写多字节）；
 *                   bit7 RESTART 重启 PWM 计数器
 *   MODE2   (0x01)：bit2 OUTDRV=1 推挽输出（驱动舵机必须，默认开漏不行）
 *   LED0_ON_L  (0x06)：通道 n 的 4 个寄存器 = 0x06 + n*4
 *                      （ON_L / ON_H / OFF_L / OFF_H）
 *   ALL_LED_OFF_H (0xFD)：写 bit4 可让全部 16 路同时停止输出
 *   PRE_SCALE (0xFE)：预分频 = round(25MHz / (4096 * 频率)) - 1，
 *                     50Hz 时 = round(122.07) - 1 = 121 = 0x7A
 */
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "pca9685_servo.h"

static const char *TAG = "pca9685";

/* ==================== 舵机编号 -> PCA9685 通道映射 ====================
 * 《四足机器人舵机布局.md》第 4 节默认顺序假设，实际通道【待确认】。
 * 插线与实物不符时，只改这张表，代码和动作逻辑不用动。
 */
static const uint8_t s_servo_channel[SERVO_NUM] = {
    [A_R] = 0,   /* 左前髋 */
    [A_B] = 1,   /* 左前膝 */
    [B_R] = 2,   /* 右前髋 */
    [B_B] = 3,   /* 右前膝 */
    [C_R] = 4,   /* 左后髋 */
    [C_B] = 5,   /* 左后膝 */
    [D_R] = 6,   /* 右后髋 */
    [D_B] = 7,   /* 右后膝 */
};

/* ==================== PCA9685 寄存器定义 ==================== */
#define PCA9685_REG_MODE1          0x00
#define PCA9685_REG_MODE2          0x01
#define PCA9685_REG_LED0_ON_L      0x06   /* 通道基址：0x06 + 通道号*4 */
#define PCA9685_REG_ALL_LED_ON_L   0xFA   /* 全 16 路的 ON/OFF 组寄存器 */
#define PCA9685_REG_ALL_LED_OFF_H  0xFD   /* 全 16 路同时停止输出      */
#define PCA9685_REG_PRE_SCALE      0xFE

/* MODE1 寄存器位定义 */
#define PCA9685_MODE1_RESTART      (1u << 7)
#define PCA9685_MODE1_AI           (1u << 5)   /* 地址自动递增 */
#define PCA9685_MODE1_SLEEP        (1u << 4)   /* 睡眠（写预分频前必须置 1） */

/* MODE2 寄存器位定义 */
#define PCA9685_MODE2_OUTDRV       (1u << 2)   /* 推挽输出 */

/* OFF_H 寄存器 bit4：完全停止输出，引脚保持低电平（无脉冲） */
#define PCA9685_LED_FULL_OFF       (1u << 4)

#define PCA9685_OSC_FREQ_HZ        25000000.0f /* 内部振荡器标称 25MHz */

/* 模块初始化标志 */
static bool s_initialized = false;

/* ==================== 底层读写 ==================== */

/* 写 1 个寄存器 */
static esp_err_t pca9685_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(PCA9685_SERVO_I2C_PORT,
                                      PCA9685_SERVO_I2C_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(PCA9685_SERVO_I2C_TIMEOUT_MS));
}

/* 从 reg 开始连续写 len 字节（需 MODE1 的 AI 位已置 1） */
static esp_err_t pca9685_write_buf(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + 32]; /* 1 字节寄存器地址 + 最多 32 字节数据 */
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(PCA9685_SERVO_I2C_PORT,
                                      PCA9685_SERVO_I2C_ADDR,
                                      buf, len + 1,
                                      pdMS_TO_TICKS(PCA9685_SERVO_I2C_TIMEOUT_MS));
}

/* 读 1 个寄存器（先写寄存器地址再读，标准 I2C 组合传输） */
static esp_err_t pca9685_read_byte(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(PCA9685_SERVO_I2C_PORT,
                                        PCA9685_SERVO_I2C_ADDR,
                                        &reg, 1, val, 1,
                                        pdMS_TO_TICKS(PCA9685_SERVO_I2C_TIMEOUT_MS));
}

/* ==================== 内部功能 ==================== */

/**
 * 计算 PWM 频率对应的预分频值（NXP 数据手册公式）
 * 预分频 = round(osc / (4096 * freq)) - 1，芯片合法范围 0x03~0xFF。
 * 频率超出可配置范围（约 24~1526Hz）返回 ESP_ERR_INVALID_ARG，
 * 不做钳位——钳位会让实际频率与换算用频率不一致，导致脉宽错误。
 */
static esp_err_t pca9685_calc_prescaler(uint32_t freq_hz, uint8_t *out)
{
    float val = roundf(PCA9685_OSC_FREQ_HZ / (4096.0f * freq_hz)) - 1.0f;
    if (val < 3.0f || val > 255.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = (uint8_t)val;
    return ESP_OK;
}

/**
 * 写一个通道的 ON/OFF 寄存器
 * ON 固定为 0（周期起点输出高电平），OFF = 脉冲宽度对应计数值
 */
static esp_err_t pca9685_set_channel(uint8_t ch, uint16_t off_counts)
{
    uint8_t reg = PCA9685_REG_LED0_ON_L + (uint8_t)(ch * 4);
    uint8_t buf[4] = {
        0x00,                                 /* ON_L  = 0   */
        0x00,                                 /* ON_H  = 0   */
        (uint8_t)(off_counts & 0xFF),         /* OFF_L       */
        (uint8_t)((off_counts >> 8) & 0x0F),  /* OFF_H，bit4~7 保留 0 */
    };
    return pca9685_write_buf(reg, buf, sizeof(buf));
}

/* ==================== 对外接口实现 ==================== */

esp_err_t pca9685_servo_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "PCA9685 已初始化，跳过重复初始化");
        return ESP_OK;
    }

    /* 1. 初始化 I2C 主机（SDA=GPIO5，SCL=GPIO4，400kHz，内部上拉开启）
     *    这两步失败时驱动尚未安装，直接返回即可 */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PCA9685_SERVO_SDA_GPIO,
        .scl_io_num = PCA9685_SERVO_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = PCA9685_SERVO_I2C_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t ret = i2c_param_config(PCA9685_SERVO_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(PCA9685_SERVO_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 探测芯片（读 MODE1，仅验证 I2C 有应答，不校验具体值——
     *    模块可能已被配置过，上电默认值不一定是 0x11） */
    uint8_t mode1 = 0;
    ESP_GOTO_ON_ERROR(pca9685_read_byte(PCA9685_REG_MODE1, &mode1),
                      fail, TAG,
                      "PCA9685 无应答(0x%02X)，检查 SDA=%d/SCL=%d 接线和 I2C 地址！",
                      PCA9685_SERVO_I2C_ADDR, PCA9685_SERVO_SDA_GPIO, PCA9685_SERVO_SCL_GPIO);
    ESP_LOGI(TAG, "PCA9685 应答正常，MODE1=0x%02X", mode1);

    /* 3. 计算预分频（50Hz -> 0x7A）；频率超出芯片可配置范围(约24~1526Hz)
     *    直接报错返回，避免实际频率和换算频率不一致导致脉宽错误 */
    uint8_t prescaler = 0;
    ESP_GOTO_ON_ERROR(pca9685_calc_prescaler(PCA9685_SERVO_PWM_FREQ_HZ, &prescaler),
                      fail, TAG,
                      "PWM 频率 %d Hz 超出芯片可配置范围(24~1526Hz)！",
                      PCA9685_SERVO_PWM_FREQ_HZ);

    /* 4. 复位 -> 睡眠 -> 设预分频 -> 唤醒 -> 推挽输出 -> 重启 PWM */
    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_MODE1, 0x00),
                      fail, TAG, "复位 MODE1 失败");
    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP),
                      fail, TAG, "进入睡眠失败");

    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_PRE_SCALE, prescaler),
                      fail, TAG, "写预分频失败");
    ESP_LOGI(TAG, "PWM 频率 %d Hz，预分频 0x%02X", PCA9685_SERVO_PWM_FREQ_HZ, prescaler);

    /* 唤醒芯片，开启自动递增；手册要求 SLEEP 清除后至少 500us 振荡器才稳定，
     * 必须用忙等延时（vTaskDelay 受 tick 相位影响，实际时长可能远小于 1ms） */
    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_MODE1, PCA9685_MODE1_AI),
                      fail, TAG, "唤醒失败");
    esp_rom_delay_us(500);

    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV),
                      fail, TAG, "配置推挽输出失败");
    ESP_GOTO_ON_ERROR(pca9685_write_byte(PCA9685_REG_MODE1,
                                         PCA9685_MODE1_AI | PCA9685_MODE1_RESTART),
                      fail, TAG, "重启 PWM 计数器失败");

    /* 5. 清空全部 16 路的 ON/OFF 寄存器（写 ALL_LED 组寄存器 0xFA~0xFD，
     *    需 AI 自动递增，前面已开启）：上电后所有通道无脉冲输出，
     *    舵机全部失力，防止开机乱动 */
    uint8_t all_off[4] = { 0x00, 0x00, 0x00, 0x00 };
    ESP_GOTO_ON_ERROR(pca9685_write_buf(PCA9685_REG_ALL_LED_ON_L, all_off, 4),
                      fail, TAG, "清空通道输出失败");

    s_initialized = true;
    ESP_LOGI(TAG, "PCA9685 初始化完成，%d 路舵机就绪", SERVO_NUM);
    return ESP_OK;

fail:
    /* 任一步失败都要卸载已安装的 I2C 驱动，否则重试 init 会因驱动
     * 已存在而失败，且 deinit 因未置 s_initialized 无法清理 */
    i2c_driver_delete(PCA9685_SERVO_I2C_PORT);
    return ret;
}

esp_err_t pca9685_servo_set_pulse_us(servo_id_t id, float pulse_us)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((int)id < 0 || (int)id >= SERVO_NUM) {
        return ESP_ERR_INVALID_ARG;
    }
    /* NaN 的任何比较都是 false，会绕过限幅并让 float->整数转换变成未定义行为 */
    if (!isfinite(pulse_us)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pulse_us < PCA9685_SERVO_PULSE_MIN_US) {
        pulse_us = PCA9685_SERVO_PULSE_MIN_US;
    } else if (pulse_us > PCA9685_SERVO_PULSE_MAX_US) {
        pulse_us = PCA9685_SERVO_PULSE_MAX_US;
    }

    /* 计数值 = 脉宽(us) / 周期(us) * 4096；50Hz 周期 20000us
     * 例：1500us -> 1500/20000*4096 = 307.2 -> 307（对应 90° 中位） */
    float counts_f = pulse_us * (4096.0f * PCA9685_SERVO_PWM_FREQ_HZ) / 1000000.0f;
    uint16_t counts = (uint16_t)(counts_f + 0.5f);
    if (counts > 4095) {
        counts = 4095;
    }

    return pca9685_set_channel(s_servo_channel[id], counts);
}

esp_err_t pca9685_servo_set_angle(servo_id_t id, float angle)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((int)id < 0 || (int)id >= SERVO_NUM) {
        return ESP_ERR_INVALID_ARG;
    }
    if (angle < PCA9685_SERVO_ANGLE_MIN) {
        angle = PCA9685_SERVO_ANGLE_MIN;
    } else if (angle > PCA9685_SERVO_ANGLE_MAX) {
        angle = PCA9685_SERVO_ANGLE_MAX;
    }

    /* 脉宽 = 500us + 角度/180 * 2000us（0°~180° 线性对应 0.5ms~2.5ms） */
    float pulse_us = PCA9685_SERVO_PULSE_MIN_US +
                     angle / (PCA9685_SERVO_ANGLE_MAX - PCA9685_SERVO_ANGLE_MIN) *
                     (PCA9685_SERVO_PULSE_MAX_US - PCA9685_SERVO_PULSE_MIN_US);
    return pca9685_servo_set_pulse_us(id, pulse_us);
}

esp_err_t pca9685_servo_set_angle_all(const float angles[SERVO_NUM])
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (angles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    for (int i = 0; i < SERVO_NUM; i++) {
        esp_err_t ret = pca9685_servo_set_angle((servo_id_t)i, angles[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "舵机 %d 置 %0.1f° 失败: %s", i, angles[i], esp_err_to_name(ret));
            err = ret;
        }
    }
    return err;
}

esp_err_t pca9685_servo_set_cmds(const servo_cmd_t *cmds, size_t count)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cmds == NULL || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = pca9685_servo_set_angle(cmds[i].id, cmds[i].angle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "指令[%u]：舵机 %d 置 %0.1f° 失败: %s",
                     (unsigned)i, (int)cmds[i].id, cmds[i].angle, esp_err_to_name(ret));
            err = ret;
        }
    }
    return err;
}

esp_err_t pca9685_servo_release(servo_id_t id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((int)id < 0 || (int)id >= SERVO_NUM) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ON=0、OFF=0 且 OFF_H 的 FULL_OFF 位置 1：
     * 该通道输出保持低电平，不再发脉冲，舵机失力 */
    uint8_t buf[4] = { 0x00, 0x00, 0x00, PCA9685_LED_FULL_OFF };
    uint8_t reg = PCA9685_REG_LED0_ON_L + (uint8_t)(s_servo_channel[id] * 4);
    return pca9685_write_buf(reg, buf, sizeof(buf));
}

esp_err_t pca9685_servo_release_all(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 写 ALL_LED_OFF_H 的 FULL_OFF 位，全部 16 路同时停止输出 */
    return pca9685_write_byte(PCA9685_REG_ALL_LED_OFF_H, PCA9685_LED_FULL_OFF);
}

esp_err_t pca9685_servo_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    esp_err_t err = i2c_driver_delete(PCA9685_SERVO_I2C_PORT);
    s_initialized = false;
    return err;
}
