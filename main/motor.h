/**
 * @file motor.h
 * @brief 四足机器人动作组接口（步态按《舵机运动.md》规则实现）
 *
 * 前置条件：调用任何动作接口前，必须已成功执行 pca9685_servo_init()
 * （返回 ESP_OK），否则所有动作返回 ESP_ERR_INVALID_STATE。
 *
 * 平移方向与方案的对应关系（《舵机运动.md》）：
 *   前进 -> 方案A    后退 -> 方案B    右移 -> 方案C    左移 -> 方案D
 *
 * 每个步态周期按规则(4)分两个半周期交替：
 *   半周期1：A_R/D_R 向下（触地支撑蹬地）、B_R/C_R 向上（悬空摆动）
 *   半周期2：A_R/D_R 向上（悬空回摆）、  B_R/C_R 向下（触地支撑蹬地）
 *   （例外：右移的先后顺序由 motor.c 顶部 MOTOR_RIGHT_BC_FIRST 开关决定，
 *     默认 1 = CB 先往下；改 0 恢复 AD 先往下）
 *
 * 注意：所有动作组都是阻塞式实现（内部按小步插值平滑驱动舵机，
 * 时长可调，见 motor.c 顶部 MOTOR_*_SWEEP_MS / MOTOR_INTERP_STEP_MS），
 * 必须在任务上下文中调用（app_main 或自建任务均可，勿在中断里调用）。
 * 平移动作组结束时【不回站立】，停在当前步态相位上：连续调用（含换方向）
 * 会从当前姿态无缝衔接继续走；需要停回静止姿态时显式调用 motor_stand()。
 *
 * 错误返回：ESP_OK 全部指令下发成功；ESP_ERR_INVALID_STATE 驱动未初始化；
 * 其他错误码为 I2C/驱动错误——动作已中止并尝试回静止姿态（详见 motor.c）。
 *
 * 转向按《舵机运动.md》规则(5)实现（motor_turn_left / motor_turn_right）：
 * 触地分组为对角对"AD ↔ BC"交替（2026-11-25 用户更正，原文 AC 为笔误），
 * 髋关节按"A_R向上D_R向下，B_R向上C_R向下"剪刀式摆动、采用方案A端点；
 * 左右转向的髋关节轨迹相同，区别是每周期第一拍哪一对触地
 * （左转第一拍 AD 触地，右转第一拍 BC 触地）。转向时规则(3)不严格成立：
 * 触地由 md 括号标注决定，膝关节按对分配，与髋关节方向解耦。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 姿态指令数组条目数：POSE_COUNT(pose) */
#define POSE_COUNT(pose)    (sizeof(pose) / sizeof((pose)[0]))

/**
 * @brief 静止站立姿态
 *
 * 规则(2)：_B 全部 90°，_R = {A_R:135, B_R:45, C_R:45, D_R:135}。
 * 平移动作组不会自动调用本函数，需要停回站立时显式调用。
 *
 * @return ESP_OK 成功；错误码含义见文件头注释
 */
esp_err_t motor_stand(void);

/**
 * @brief 全部舵机失力（停止 PWM 脉冲输出，腿部可用手掰动）
 *
 * @return ESP_OK 成功；其他：驱动未初始化或 I2C 失败（失力未生效）
 */
esp_err_t motor_release(void);

/**
 * @brief 向前平移（方案A）：走 cycles 个完整步态周期，结束停在步态姿态
 *
 * 一个完整步态周期 = 两个半周期（A/D 对蹬地一次 + B/C 对蹬地一次），
 * 机身向前移动一个步距。结束不回站立（见文件头注释）。
 *
 * @param cycles 步态周期数（0 表示不动作，直接返回）
 * @return ESP_OK 成功；错误码含义见文件头注释
 */
esp_err_t motor_walk_forward(uint32_t cycles);

/**
 * @brief 向后平移（方案B），参数与返回值含义同 motor_walk_forward
 */
esp_err_t motor_walk_backward(uint32_t cycles);

/**
 * @brief 向右平移（方案C），参数与返回值含义同 motor_walk_forward
 */
esp_err_t motor_walk_right(uint32_t cycles);

/**
 * @brief 向左平移（方案D），参数与返回值含义同 motor_walk_forward
 */
esp_err_t motor_walk_left(uint32_t cycles);

/**
 * @brief 原地左转向（规则(5)）：走 cycles 个完整转向周期，结束停在步态相位
 *
 * 一个转向周期 = 两个半周期：第一拍对角对{A,D}触地、{B,C}悬空摆动
 * （"A_R向上D_R向下，B_R向上C_R向下"剪刀式），第二拍互换。
 * 结束不回站立（见文件头注释），连续调用可无缝衔接。
 * 实机若转向方向与预期相反，把本函数与 motor_turn_right 的触地对开关对调。
 *
 * @param cycles 转向周期数（0 表示不动作，直接返回）
 * @return ESP_OK 成功；错误码含义见文件头注释
 */
esp_err_t motor_turn_left(uint32_t cycles);

/**
 * @brief 原地右转向（规则(5)），参数与返回值含义同 motor_turn_left
 *
 * 与左转的区别：第一拍对角对{B,C}触地、{A,D}悬空（触地对调），
 * 髋关节轨迹与左转相同。
 */
esp_err_t motor_turn_right(uint32_t cycles);

esp_err_t motor_interp_move(const float *knee_target, const float *hip_target,
                            uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
