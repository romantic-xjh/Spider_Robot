/**
 * @file motor.c
 * @brief 四足机器人动作组实现：静止姿态 + 四方向平移步态
 *
 * 数值按《舵机运动.md》（2026-11-21 起 _R 每次角度变化幅度为 20°）：
 *   - 规则(2)：静止时 _B 全 90°，_R = {A_R:135, B_R:45, C_R:45, D_R:135}；
 *   - 第 5~8 行：膝关节触地角 A_B = 50°、B_B = 120°、C_B = 130°、D_B = 45°，
 *     悬空角 A_B = 30°、B_B/C_B = 150°、D_B = 20°；
 *   - 规则(3)：A_R 与 D_R 一对、B_R 与 C_R 一对；向下的 _R 对应的 _B 触地，
 *     向上的 _R 对应的 _B 悬空；
 *   - 规则(4)：平移时先 A_R/D_R 向下 + B_R/C_R 向上，然后交换；
 *     例外：右移的起步顺序由 MOTOR_RIGHT_BC_FIRST 开关决定（默认 CB 先往下）；
 *     四个方向逻辑相同，仅角度方案不同（方案A/B/C/D 对应前/后/右/左）。
 *   - 规则(5)：转向的触地分组为对角对"AD ↔ BC"交替（2026-11-25 用户更正：
 *     md 原文括号内 AC 为 AD 笔误）；髋关节采用方案A的行程端点，按
 *     "A_R向上D_R向下，B_R向上C_R向下"→反向剪刀式摆动（每对内两腿反向）。
 *     左右转向的髋关节轨迹相同（md 两段 _R 文字一致），区别只在每周期
 *     第一拍哪一对触地：左转第一拍 AD 触地、BC 悬空；右转第一拍 BC 触地、
 *     AD 悬空。注意：转向时规则(3)"向下_R↔触地"不再严格成立——触地对内
 *     髋关节一上一下（剪刀运动），触地状态由 md 括号标注决定，膝关节按对
 *     分配触地/悬空角、与髋关节方向解耦。实机若转向方向与预期相反，
 *     把 motor_turn_left / motor_turn_right 的触地对开关对调即可。
 *   - 髋关节行程幅度 20°：每腿行程中与静止姿态相同的那一端保持不变，
 *     另一端向它收缩（保住方案A/B、方案C/D 各自共用行程端点的镜像结构）；
 *     2026-11-24 起方案A/B 的 A_R 上端点 135→127，稳定行程 12°；
 *     2026-11-25 方案D 端点实机调整：A_R/B_R 行程 25°、D_R 行程 15°。
 *
 * 平滑运动：所有关节都做匀速插值——每 MOTOR_INTERP_STEP_MS 下发一小步，
 * 在过渡时长内走完全程，最后一步下发精确目标角保证到位。舵机跟随的是
 * 平滑小台阶轨迹，而不是"一次甩到位再干等"，消除步态一卡一卡。
 * 过渡时长取 SWEEP 宏与"跨度 × MS_PER_DEG"中的较大者：稳态小步幅由 SWEEP
 * 决定步频，静止↔步态这类大跨度过渡自动放慢，衔接不发猛。
 *
 * 半周期内的指令顺序：先摆膝关节（支撑对落触地角、摆动对抬悬空角），
 * 过渡完再摆髋关节（支撑对蹬地、摆动对空中回摆）。
 * 膝先动保证蹬地段足端已贴地、摆动段足端已离地，足端不拖地。
 *
 * 错误处理：任一次指令下发失败（如 I2C NACK）都中止当前半周期——
 * 膝关节失败时髋关节保持不动（足端支撑状态不可信，动髋会拖地或蹬空），
 * 已走的部分尝试回到静止姿态，错误码逐层返回给调用方。
 * 每条失败指令在 pca9685_servo 驱动内部都有逐条日志。
 */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pca9685_servo.h"
#include "motor.h"

static const char *TAG = "motor";

/* ==================== 可调参数（按实机舵机速度/电压调整） ==================== */
#define MOTOR_INTERP_STEP_MS   40    /* 插值下发间隔（≈1个PWM周期），越小轨迹越平滑 */
#define MOTOR_INTERP_MS_PER_DEG 5.0f /* 每 1° 跨度需要的过渡时间：大跨度自动放慢的下限 */
#define MOTOR_KNEE_SWEEP_MS    150   /* 膝关节一次过渡（抬起/落下）总时长：越长越柔，越短步频越快 */
#define MOTOR_HIP_SWEEP_MS     250   /* 髋关节一次摆动（蹬地/回摆）总时长：承重蹬地宁慢勿快 */
#define MOTOR_STAND_SWEEP_MS   300   /* 进出静止姿态的过渡总时长 */

/* 右平移（方案C）起步开关：每个周期第一拍哪对腿先往下支撑蹬地
 *   1 = CB 先往下（B/C 腿先触地蹬地，A/D 悬空回摆）
 *   0 = AD 先往下（原顺序，B/C 后往下）
 * 只影响右移，前/后/左移不受影响 */
#define MOTOR_RIGHT_BC_FIRST   1

/* ==================== 《舵机运动.md》规则数值 ==================== */

/* 规则(2)：静止姿态 */
#define KNEE_STAND_ANGLE      90.0f
static const float KNEE_STAND_ALL[4] = { 90.0f, 90.0f, 90.0f, 90.0f };
static const float HIP_STAND[4]      = { 135.0f, 45.0f, 45.0f, 135.0f }; /* A_R,B_R,C_R,D_R */

/* 第 5~8 行：膝关节触地角与悬空角（下标顺序 A_B,B_B,C_B,D_B）
 * A_B 触地 60 悬空 30（md 2026-11-24 改），其余不变 */
static const float KNEE_TOUCH[4] = { 60.0f, 118.0f, 126.0f, 50.0f };
static const float KNEE_AIR[4]   = { 30.0f, 156.0f, 150.0f, 18.0f };

/*
 * 四个方案每个半周期的髋关节目标角（下标顺序 A_R,B_R,C_R,D_R）。
 * half1 = A_R/D_R 向下（触地蹬地）、B_R/C_R 向上（悬空摆动）：
 *   A_R、D_R 取 md 各行"向下"转移的终点角，B_R、C_R 取"向上"终点角；
 * half2 = A_R/D_R 向上、B_R/C_R 向下，取角方式相反。
 *
 * 行程结构（md 2026-11-21 起，每次变化幅度 20°）：
 *   每腿行程中与静止姿态相同的那一端保持不变，另一端向它收缩：
 *   方案A/B 共用同一组行程端点 {A:115~127, B:45~65, C:45~65, D:115~135}，
 *   仅"向上/向下"（摆动/蹬地）角色互换 → 前进/后退方向相反；
 *   方案C/D 同理（角色互换），端点 2026-11-25 起按实机调整、不再共用：
 *   方案C {A:135~155, B:25~45, C:25~45, D:135~155}，
 *   方案D {A:135~160, B:20~45, C:25~45, D:135~150}。
 * 静止姿态 {135,45,45,135}：A_R 的行程端点 2026-11-24 起 135→127，
 * 静止值 135 不再落在端点上——从静止进入 half1 时 A_R 先走 135→115，
 * 之后稳定在 115~127 间摆 12°；其余腿静止值仍在端点上，按表运动即可。
 */

/* 方案A：向前平移（朝 AB 边运动） */
static const float HIP_PLAN_A_HALF1[4] = { 115.0f, 45.0f, 65.0f, 135.0f };
static const float HIP_PLAN_A_HALF2[4] = { 127.0f, 65.0f, 45.0f, 115.0f };

/* 方案B：向后平移（朝 CD 边运动），行程端点与方案A相同、角色互换 */
static const float HIP_PLAN_B_HALF1[4] = { 127.0f, 65.0f, 45.0f, 115.0f };
static const float HIP_PLAN_B_HALF2[4] = { 115.0f, 45.0f, 65.0f, 135.0f };

/* 方案C：右移（朝 AC 边运动） */
static const float HIP_PLAN_C_HALF2[4] = {135.0f, 53.0f, 21.0f, 151.0f};
static const float HIP_PLAN_C_HALF1[4] = {160.0f, 25.0f, 41.0f, 135.0f};

/* 方案D：左移（朝 BD 边运动），角色与方案C互换；端点按实机调整（2026-11-25），
 * 不再与方案C共用：A_R 摆动端 155→160、B_R 支撑端 25→20、D_R 支撑端 155→150 */
static const float HIP_PLAN_D_HALF1[4] = { 135.0f, 45.0f, 25.0f, 150.0f };
static const float HIP_PLAN_D_HALF2[4] = { 160.0f, 20.0f, 45.0f, 135.0f };

/*
 * 规则(5)：转向步态髋关节目标角（采用方案A行程端点，2026-11-25 指定）。
 * 髋关节按 md 规则(5)文字逐条取终点角，每对{A,D}/{B,C}内两腿反向（剪刀式）：
 *   half1：A_R向上(127) D_R向下(135)，B_R向上(45) C_R向下(45)；
 *   half2：A_R向下(115) D_R向上(115)，B_R向下(65) C_R向上(65)。
 * 左右转向的髋关节轨迹相同（md 两段 _R 文字一致），区别在触地对（见
 * motor_turn_half_cycle）：左转第一拍 AD 触地，右转第一拍 BC 触地。
 * 左右各留独立表格便于实机微调（参考方案C/D 后期分家的做法）。
 */
static const float HIP_TURN_L_HALF1[4] = { 127.0f, 45.0f, 45.0f, 135.0f };
static const float HIP_TURN_L_HALF2[4] = { 115.0f, 65.0f, 65.0f, 115.0f };
static const float HIP_TURN_R_HALF1[4] = { 127.0f, 45.0f, 45.0f, 135.0f };
static const float HIP_TURN_R_HALF2[4] = { 115.0f, 65.0f, 65.0f, 115.0f };

/* ==================== 内部工具 ==================== */

/**
 * @brief 一次性下发四个膝关节目标角
 *
 * @param angles 四个膝关节目标角数组，单位°（下标顺序 A_B,B_B,C_B,D_B）
 * @return ESP_OK 成功；其他：指令下发失败
 */
static esp_err_t motor_set_knees(const float angles[4])
{
    const servo_cmd_t cmds[] = {
        { A_B, angles[0] }, { B_B, angles[1] },
        { C_B, angles[2] }, { D_B, angles[3] },
    };
    return pca9685_servo_set_cmds(cmds, POSE_COUNT(cmds));
}

/**
 * @brief 一次性下发四个髋关节目标角
 *
 * @param angles 四个髋关节目标角数组，单位°（下标顺序 A_R,B_R,C_R,D_R）
 * @return ESP_OK 成功；其他：指令下发失败
 */
static esp_err_t motor_set_hips(const float angles[4])
{
    const servo_cmd_t cmds[] = {
        { A_R, angles[0] }, { B_R, angles[1] },
        { C_R, angles[2] }, { D_R, angles[3] },
    };
    return pca9685_servo_set_cmds(cmds, POSE_COUNT(cmds));
}

/* ==================== 平滑插值 ==================== */

/* 当前角度 = 上次成功下发的目标角。初始按静止姿态假设：
 * 上电先调用 motor_stand() 即可保证与舵机真实位置一致 */
static float s_knee_now[4] = { 90.0f, 90.0f, 90.0f, 90.0f };   /* A_B,B_B,C_B,D_B */
static float s_hip_now[4]  = { 135.0f, 45.0f, 45.0f, 135.0f }; /* A_R,B_R,C_R,D_R */

/**
 * @brief 在过渡时长内把膝/髋关节从当前角匀速插值到目标角（阻塞）
 *
 * 每 MOTOR_INTERP_STEP_MS 下发一小步，最后一步下发精确目标角保证到位。
 * 膝、髋两组可以同时过渡（如回站立时一起平滑回位），传 NULL 表示该组不动。
 * 过渡时长 = max(duration_ms, 最大角度跨度 × MOTOR_INTERP_MS_PER_DEG)：
 * 稳态步态的小步幅由 duration 决定步频；静止↔步态这类大跨度自动放慢。
 *
 * @param knee_target 四个膝关节目标角（A_B,B_B,C_B,D_B），NULL 保持不动
 * @param hip_target  四个髋关节目标角（A_R,B_R,C_R,D_R），NULL 保持不动
 * @return ESP_OK 成功；失败时 s_*_now 保存最后成功下发的角度，
 *         供恢复动作（motor_stand）从正确位置继续
 */
esp_err_t motor_interp_move(const float *knee_target, const float *hip_target,
                                   uint32_t duration_ms)
{
    float max_delta = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (knee_target) {
            max_delta = fmaxf(max_delta, fabsf(knee_target[i] - s_knee_now[i]));
        }
        if (hip_target) {
            max_delta = fmaxf(max_delta, fabsf(hip_target[i] - s_hip_now[i]));
        }
    }
    uint32_t need = (uint32_t)(max_delta * MOTOR_INTERP_MS_PER_DEG);
    if (duration_ms < need) {
        duration_ms = need;
    }
    if (duration_ms < MOTOR_INTERP_STEP_MS) {
        duration_ms = MOTOR_INTERP_STEP_MS;
    }

    esp_err_t ret;
    for (uint32_t elapsed = 0; elapsed < duration_ms; ) {
        uint32_t dt = (duration_ms - elapsed >= MOTOR_INTERP_STEP_MS)
                          ? MOTOR_INTERP_STEP_MS : (duration_ms - elapsed);
        elapsed += dt;
        bool last = (elapsed >= duration_ms);
        float t = (float)elapsed / (float)duration_ms;

        float knee_a[4], hip_a[4];
        for (int i = 0; i < 4; i++) {
            if (knee_target) {
                knee_a[i] = last ? knee_target[i]
                                 : s_knee_now[i] + (knee_target[i] - s_knee_now[i]) * t;
            }
            if (hip_target) {
                hip_a[i] = last ? hip_target[i]
                                : s_hip_now[i] + (hip_target[i] - s_hip_now[i]) * t;
            }
        }

        if (knee_target) {
            ret = motor_set_knees(knee_a);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "膝关节插值下发失败，中止本次过渡");
                return ret;
            }
            memcpy(s_knee_now, knee_a, sizeof(s_knee_now));
        }
        if (hip_target) {
            ret = motor_set_hips(hip_a);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "髋关节插值下发失败，中止本次过渡");
                return ret;
            }
            memcpy(s_hip_now, hip_a, sizeof(s_hip_now));
        }
        if (!last) {
            vTaskDelay(pdMS_TO_TICKS(dt));
        }
    }
    return ESP_OK;
}

/**
 * @brief 执行一个半周期（阻塞，先膝后髋，各自按 SWEEP 时长匀速过渡）
 *
 * @param hip_target 本半周期结束时四个髋关节的目标角（A_R,B_R,C_R,D_R）
 * @param ad_support true：A/D 为支撑对（触地蹬地）、B/C 为摆动对（悬空回摆）；
 *                   false：相反
 * @return ESP_OK 成功；其他：指令下发失败（膝关节失败时髋关节不会执行）
 */
static esp_err_t motor_half_cycle(const float hip_target[4], bool ad_support)
{
    /* 规则(3)：向下的 _R 对应的 _B 触地，向上的 _R 对应的 _B 悬空 */
    float knees[4];
    knees[0] = ad_support ? KNEE_TOUCH[0] : KNEE_AIR[0];   /* A_B */
    knees[1] = ad_support ? KNEE_AIR[1]   : KNEE_TOUCH[1]; /* B_B */
    knees[2] = ad_support ? KNEE_AIR[2]   : KNEE_TOUCH[2]; /* C_B */
    knees[3] = ad_support ? KNEE_TOUCH[3] : KNEE_AIR[3];   /* D_B */

    /* 1. 膝先动：支撑对落触地角、摆动对抬悬空角，匀速过渡不甩腿。
     *    膝指令失败时足端支撑状态不可信，直接中止：
     *    此时动髋会导致拖地或蹬空 */
    esp_err_t ret = motor_interp_move(knees, NULL, MOTOR_KNEE_SWEEP_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "膝关节指令下发失败，中止本半周期（髋关节保持不动）");
        return ret;
    }

    /* 2. 髋后动：支撑对蹬地，摆动对空中回摆，匀速摆动 */
    ret = motor_interp_move(NULL, hip_target, MOTOR_HIP_SWEEP_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "髋关节指令下发失败，中止本半周期");
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief 平移步态通用实现（规则(4)：两个半周期交替，逻辑四方向相同）
 *
 * 结束时不回站立——停在当前步态相位上，连续调用（含换方向）可无缝衔接；
 * 需要回到静止姿态时由调用方显式调用 motor_stand()。
 *
 * @param half1  A/D 为支撑对的半周期结束时四个髋关节的目标角，单位°
 *               （下标顺序 A_R,B_R,C_R,D_R；本半周期 B/C 为摆动对）
 * @param half2  B/C 为支撑对的半周期结束时四个髋关节的目标角，单位°
 *               （下标顺序 A_R,B_R,C_R,D_R；本半周期 A/D 为摆动对）
 * @param cycles 步态周期数，每个周期 = 两个半周期
 * @param bc_first true：每个周期先走 B/C 支撑拍（CB 先往下），A/D 拍在后；
 *                 false：先走 A/D 支撑拍（AD 先往下，原顺序）
 * @return ESP_OK 走完 cycles 个周期；其他：中途失败（已尽力回静止姿态
 *         恢复），错误码为首次失败原因
 */
static esp_err_t motor_walk(const float half1[4], const float half2[4],
                            uint32_t cycles, bool bc_first)
{
    /* 按开关决定两拍的顺序与各自支撑对：角度表和支撑标志要配套换 */
    const float *first  = bc_first ? half2 : half1;
    const float *second = bc_first ? half1 : half2;
    bool first_ad_support = !bc_first;  /* 第一拍是否 A/D 支撑（false 即 B/C 支撑） */

    for (uint32_t i = 0; i < cycles; i++) {
        esp_err_t ret = motor_half_cycle(first, first_ad_support);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "第 %u 周期第一拍执行失败，尝试回静止姿态", (unsigned)(i + 1));
            motor_stand();   /* 尽力恢复到已知姿态，其自身失败有日志 */
            return ret;
        }
        ret = motor_half_cycle(second, !first_ad_support);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "第 %u 周期第二拍执行失败，尝试回静止姿态", (unsigned)(i + 1));
            motor_stand();
            return ret;
        }
    }
    return ESP_OK;
}

/**
 * @brief 执行一个转向半周期（阻塞，先膝后髋，各自按 SWEEP 时长匀速过渡）
 *
 * 与 motor_half_cycle 的区别：转向的触地分组是"对角对 AD ↔ BC"
 * （2026-11-25 用户更正 md 括号标注），且规则(3)在转向中不严格成立——
 * 触地对内髋关节一上一下（剪刀运动），触地状态由 md 规则(5)括号标注
 * 决定，膝关节按对分配触地/悬空角，与髋关节上下方向解耦。
 *
 * @param hip_target 本半周期结束时四个髋关节的目标角（A_R,B_R,C_R,D_R）
 * @param ad_touch   true：对角对{A,D}触地、{B,C}悬空；false：相反
 * @return ESP_OK 成功；其他：指令下发失败（膝关节失败时髋关节不会执行）
 */
static esp_err_t motor_turn_half_cycle(const float hip_target[4], bool ad_touch)
{
    /* 触地对落触地角、悬空对抬悬空角（按对分配，不跟随髋关节方向） */
    float knees[4];
    knees[0] = ad_touch ? KNEE_TOUCH[0] : KNEE_AIR[0];   /* A_B */
    knees[1] = ad_touch ? KNEE_AIR[1]   : KNEE_TOUCH[1]; /* B_B */
    knees[2] = ad_touch ? KNEE_AIR[2]   : KNEE_TOUCH[2]; /* C_B */
    knees[3] = ad_touch ? KNEE_TOUCH[3] : KNEE_AIR[3];   /* D_B */

    /* 1. 膝先动：触地对落触地角、悬空对抬悬空角（同平移，足端不拖地） */
    esp_err_t ret = motor_interp_move(knees, NULL, MOTOR_KNEE_SWEEP_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "转向膝关节指令下发失败，中止本半周期（髋关节保持不动）");
        return ret;
    }

    /* 2. 髋后动：触地对蹬地转向，悬空对空中回摆 */
    ret = motor_interp_move(NULL, hip_target, MOTOR_HIP_SWEEP_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "转向髋关节指令下发失败，中止本半周期");
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief 转向步态通用实现（规则(5)：对角对 AD ↔ BC 交替触地/悬空）
 *
 * 结束时不回站立——停在当前步态相位上，连续调用可无缝衔接；
 * 需要回到静止姿态时由调用方显式调用 motor_stand()。
 *
 * @param half1   每周期第一拍结束时四个髋关节的目标角，单位°（A_R,B_R,C_R,D_R）
 * @param half2   每周期第二拍结束时四个髋关节的目标角，单位°（A_R,B_R,C_R,D_R）
 * @param cycles  转向周期数，每个周期 = 两个半周期
 * @param ad_touch_first true：第一拍对角对{A,D}触地（左转）；
 *                       false：第一拍对角对{B,C}触地（右转）
 * @return ESP_OK 转完 cycles 个周期；其他：中途失败（已尽力回静止姿态恢复），
 *         错误码为首次失败原因
 */
static esp_err_t motor_turn(const float half1[4], const float half2[4],
                            uint32_t cycles, bool ad_touch_first)
{
    for (uint32_t i = 0; i < cycles; i++) {
        esp_err_t ret = motor_turn_half_cycle(half1, ad_touch_first);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "转向第 %u 周期第一拍执行失败，尝试回静止姿态", (unsigned)(i + 1));
            motor_stand();   /* 尽力恢复到已知姿态，其自身失败有日志 */
            return ret;
        }
        ret = motor_turn_half_cycle(half2, !ad_touch_first);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "转向第 %u 周期第二拍执行失败，尝试回静止姿态", (unsigned)(i + 1));
            motor_stand();
            return ret;
        }
    }
    return ESP_OK;
}

/* ==================== 对外接口 ==================== */

/**
 * @brief 回到静止姿态：膝、髋一起匀速回位（不蹬地，顺序无硬性要求）
 *
 * 无参数。
 *
 * @return ESP_OK 成功；其他：指令下发失败
 */
esp_err_t motor_stand(void)
{
    /* 膝、髋一起匀速回静止姿态（不蹬地，顺序无硬性要求） */
    return motor_interp_move(KNEE_STAND_ALL, HIP_STAND, MOTOR_STAND_SWEEP_MS);
}

/**
 * @brief 按方案A向前平移（朝 AB 边运动）
 *
 * @param cycles 步态周期数，每个周期含两个半周期（A/D 支撑 ↔ B/C 支撑交替）
 * @return ESP_OK 走完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_walk_forward(uint32_t cycles)
{
    return motor_walk(HIP_PLAN_A_HALF1, HIP_PLAN_A_HALF2, cycles, false);
}

/**
 * @brief 按方案B向后平移（朝 CD 边运动）
 *
 * @param cycles 步态周期数，每个周期含两个半周期（A/D 支撑 ↔ B/C 支撑交替）
 * @return ESP_OK 走完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_walk_backward(uint32_t cycles)
{
    return motor_walk(HIP_PLAN_B_HALF1, HIP_PLAN_B_HALF2, cycles, false);
}

/**
 * @brief 按方案C向右平移（朝 AC 边运动）
 *
 * @param cycles 步态周期数，每个周期含两个半周期（A/D 支撑 ↔ B/C 支撑交替）
 * @return ESP_OK 走完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_walk_right(uint32_t cycles)
{
    /* 起步相位由 MOTOR_RIGHT_BC_FIRST 决定（见文件顶部可调参数区） */
    return motor_walk(HIP_PLAN_C_HALF1, HIP_PLAN_C_HALF2, cycles,
                      MOTOR_RIGHT_BC_FIRST != 0);
}

/**
 * @brief 按方案D向左平移（朝 BD 边运动）
 *
 * @param cycles 步态周期数，每个周期含两个半周期（A/D 支撑 ↔ B/C 支撑交替）
 * @return ESP_OK 走完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_walk_left(uint32_t cycles)
{
    return motor_walk(HIP_PLAN_D_HALF1, HIP_PLAN_D_HALF2, cycles, false);
}

/**
 * @brief 原地右转向（规则(5)，采用方案A端点）
 *
 * 每周期第一拍对角对{A,D}触地、{B,C}悬空，第二拍互换；髋关节按 md 左转
 * 文字"A_R向上D_R向下，B_R向上C_R向下"→反向剪刀式摆动。
 * 实机若转向方向与预期相反，把本函数与 motor_turn_right 的触地对开关对调。
 *
 * @param cycles 转向周期数，每个周期含两个半周期（触地对互换）
 * @return ESP_OK 转完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_turn_right(uint32_t cycles)
{
    return motor_turn(HIP_TURN_L_HALF1, HIP_TURN_L_HALF2, cycles, true);
}

/**
 * @brief 原地左转向（规则(5)，采用方案A端点）
 *
 * 每周期第一拍对角对{B,C}触地、{A,D}悬空，第二拍互换；髋关节轨迹与
 * 左转相同（md 两段 _R 文字一致），区别仅在触地对顺序。
 *
 * @param cycles 转向周期数，含义同 motor_turn_left
 * @return ESP_OK 转完 cycles 个周期；其他：中途失败（已尽力回静止姿态）
 */
esp_err_t motor_turn_left(uint32_t cycles)
{
    return motor_turn(HIP_TURN_R_HALF1, HIP_TURN_R_HALF2, cycles, false);
}

/**
 * @brief 释放全部舵机（断电卸力，腿会软掉）
 *
 * 无参数。
 *
 * @return ESP_OK 成功；其他：指令下发失败
 */
esp_err_t motor_release(void)
{
    return pca9685_servo_release_all();
}
