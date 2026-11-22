/**
 * @file pca9685_servo.h
 * @brief PCA9685 舵机驱动板驱动（ESP-IDF v5.1，ESP32-S3）
 *
 * 通过 I2C 控制 PCA9685 16 路 PWM 驱动板，驱动四足机器人的 8 个舵机。
 * 只实现舵机驱动功能：初始化、设置角度、停止输出。
 *
 * 硬件对应关系见《四足机器人舵机布局.md》：
 *   - I2C 地址 0x40（板上 A0~A5 全部断开时的默认地址）
 *   - PWM 频率 50Hz（周期 20ms），预分频值 0x7A
 *   - 12 位分辨率（0~4095），脉宽 0.5ms~2.5ms 对应角度 0°~180°
 */
#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* size_t */
#include "esp_err.h"
#include "driver/i2c.h"

/* ========================= 用户可修改配置 ========================= */
#define PCA9685_SERVO_I2C_PORT       I2C_NUM_0    /* I2C 控制器编号   */
#define PCA9685_SERVO_SDA_GPIO       5            /* SDA -> GPIO5     */
#define PCA9685_SERVO_SCL_GPIO       4            /* SCL -> GPIO4     */
#define PCA9685_SERVO_I2C_FREQ_HZ    (400 * 1000) /* I2C 时钟 400kHz  */
#define PCA9685_SERVO_I2C_ADDR       0x40         /* 模块 I2C 地址    */
#define PCA9685_SERVO_PWM_FREQ_HZ    50           /* 舵机 PWM 频率    */

/* 舵机脉宽范围（标准 SG90 类舵机，实际限位以舵机手册和装配为准） */
#define PCA9685_SERVO_PULSE_MIN_US   500.0f       /* 0° 对应脉宽    */
#define PCA9685_SERVO_PULSE_MAX_US   2500.0f      /* 180° 对应脉宽  */
#define PCA9685_SERVO_ANGLE_MIN      0.0f
#define PCA9685_SERVO_ANGLE_MAX      180.0f

/* I2C 单次读写超时时间 */
#define PCA9685_SERVO_I2C_TIMEOUT_MS 20

/* ========================= 舵机编号定义 ========================= */
/**
 * 四足机器人 8 个舵机编号（4 腿 × 髋/膝 2 关节）。
 * 命名规则：A~D = 腿号（A 左前、B 右前、C 左后、D 右后）；R = 髋，B = 膝。
 * 编号 -> PCA9685 通道的映射表在 pca9685_servo.c 顶部的
 * s_servo_channel[] 数组中，实际插线与默认不同时只改那一处。
 */
typedef enum {
    A_R = 0,   /* 左前 / 髋 */
    A_B,       /* 左前 / 膝 */
    B_R,       /* 右前 / 髋 */
    B_B,       /* 右前 / 膝 */
    C_R,       /* 左后 / 髋 */
    C_B,       /* 左后 / 膝 */
    D_R,       /* 右后 / 髋 */
    D_B,       /* 右后 / 膝 */
    SERVO_NUM         /* 舵机总数（= 8），仅作数组长度用，勿当编号传参 */
} servo_id_t;

/* ========================= 对外接口 ========================= */

/**
 * @brief 初始化 I2C 总线和 PCA9685（配置 50Hz、推挽输出、清空所有通道输出）
 *
 * 上电后所有通道保持无脉冲输出（舵机处于失力状态），
 * 需要调用 servo_set_angle* 后舵机才有脉冲。
 *
 * @return ESP_OK 成功；ESP_FAIL/其他错误码：I2C 无应答或配置失败
 */
esp_err_t pca9685_servo_init(void);

/**
 * @brief 设置单个舵机角度
 *
 * @param id     舵机编号（servo_id_t，不含 SERVO_NUM）
 * @param angle  目标角度，0°~180°（超出范围自动截断到限位）
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 未初始化；ESP_ERR_INVALID_ARG 编号非法
 */
esp_err_t pca9685_servo_set_angle(servo_id_t id, float angle);

/**
 * @brief 一次性设置全部 8 个舵机角度（逐个通道写入）
 *
 * @param angles 角度数组，长度为 SERVO_NUM，下标对应 servo_id_t
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 未初始化；ESP_ERR_INVALID_ARG angles 为空
 */
esp_err_t pca9685_servo_set_angle_all(const float angles[SERVO_NUM]);

/**
 * @brief 舵机指令：舵机编号 + 目标角度
 */
typedef struct {
    servo_id_t id;    /* 舵机编号（servo_id_t，不含 SERVO_NUM） */
    float angle;      /* 目标角度 0°~180°，超出自动截断到限位 */
} servo_cmd_t;

/**
 * @brief 按指令列表设置舵机角度（角度换算、I2C 发送全部在本函数内部完成）
 *
 * 想让哪几个舵机动就列哪几条；要 8 个一起动就把 8 条都写进数组。
 *
 * @param cmds  指令数组，每项为 {舵机编号, 角度}
 * @param count 指令条数
 * @return ESP_OK 全部成功；否则为最后一条失败项的错误码（失败项有日志）
 */
esp_err_t pca9685_servo_set_cmds(const servo_cmd_t *cmds, size_t count);

/**
 * @brief 直接以脉宽（微秒）控制单个舵机（角度换算不够用时用这个）
 *
 * @param id       舵机编号
 * @param pulse_us 脉宽，PCA9685_SERVO_PULSE_MIN_US ~ PCA9685_SERVO_PULSE_MAX_US
 * @return ESP_OK 成功；其余错误码含义同上
 */
esp_err_t pca9685_servo_set_pulse_us(servo_id_t id, float pulse_us);

/**
 * @brief 停止单个舵机的 PWM 脉冲输出（舵机失力，可用手掰动）
 */
esp_err_t pca9685_servo_release(servo_id_t id);

/**
 * @brief 停止全部舵机的 PWM 脉冲输出（写 ALL_LED 寄存器，一次完成）
 */
esp_err_t pca9685_servo_release_all(void);

/**
 * @brief 反初始化：注销 I2C 驱动（不清 PCA9685 寄存器）
 */
esp_err_t pca9685_servo_deinit(void);

#endif /* PCA9685_SERVO_H */
