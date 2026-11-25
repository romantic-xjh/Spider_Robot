/**
 * @file wife.h
 * @brief WiFi(AP+STA) + TCP 手机遥控模块接口
 *
 * 职责：ESP32 同时「联网(STA)」并「开热点(AP)」；手机连上热点后通过
 * TCP 发文本指令操控四足机器人。本模块只负责网络与指令解析，并维护
 * 手机连接状态 + 最近一条动作指令；真正驱动舵机的电机任务在 main.c，
 * 由主状态机根据 wife_is_connected() 决定「听手机」还是「听语音」。
 *
 * 指令协议（与 github1/xunji1 的 duoji.c 一致，每条一行，可带 \n 或 \r\n）：
 *   F  前进        B  后退        L  左移        R  右移        S  站立/停止
 *
 * 说明：
 *   - duoji.c 的 M/A/G（轮式占空比 / 单舵机）不适用于四足，这里不实现。
 *   - 与 duoji.c 不同：连接状态是「实时」的（断开即恢复语音路径），
 *     不做「首次连上永久锁定」。
 */
#ifndef WIFE_H
#define WIFE_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== WiFi 配置（按实际修改） ==================== */
#define WIFE_AP_SSID       "SpiderRobot"   /* 手机要连的热点名 */
#define WIFE_AP_PASS       "12345678"      /* 热点密码（≥8 位） */
#define WIFE_AP_CHANNEL    6
#define WIFE_AP_MAX_CONN   1               /* 一次只连一个手机 */

#define WIFE_STA_SSID      "1"             /* ESP32 联网用的路由器 SSID */
#define WIFE_STA_PASS      "943816729"

#define WIFE_TCP_PORT      3333            /* 手机 TCP 指令端口 */

/* ==================== 手机动作指令 ==================== */
typedef enum {
    WIFE_CMD_NONE = 0,   /* 无有效指令 */
    WIFE_CMD_STAND,      /* S：站立（回静止姿态，即停止） */
    WIFE_CMD_FORWARD,    /* F：前进 */
    WIFE_CMD_BACKWARD,   /* B：后退 */
    WIFE_CMD_LEFT,       /* L：左移 */
    WIFE_CMD_RIGHT,      /* R：右移 */
} wife_cmd_t;

/**
 * @brief 初始化 WiFi(AP+STA) + TCP 服务器 + 手机遥控任务
 *
 * 内部完成 nvs / netif / event loop / wifi 初始化（main.c 不要再重复做这些）。
 * @return ESP_OK 成功；其他：初始化失败
 */
esp_err_t wife_init(void);

/**
 * @brief 手机当前是否已连上（TCP 有连接，实时状态）
 *
 * 主状态机据此判断：true = 只听手机、忽略语音；false = 语音路径可用。
 * @return true 手机已连上，false 未连上
 */
bool wife_is_connected(void);

/**
 * @brief 读取手机最近一次动作指令
 *
 * 指令「锁存」：发 F 后一直保持 FORWARD，直到手机再发 S 或换方向。
 * @return 当前动作指令（wife_cmd_t）
 */
wife_cmd_t wife_get_cmd(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFE_H */
