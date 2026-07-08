/*
 * app_config.h - 上位机可调参数的默认初值
 *
 * 这些宏是 Flash 烧写区为空或校验失败时的回退值。
 * 运行中由 LAB 模块把它们装载到 g_leftPi / g_rightPi / g_syncKpX1000 / g_lineKpX1000 等变量里，
 * 上位机可以在线修改并通过 SAVE 命令写回 Flash。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 这里只保存“首次启动或 Flash 无效”时使用的保底值。
 * 正常运行采用 Flash V2 中保存的实测参数；修改本文件不会覆盖已有 Flash。
 * 若要让新默认值生效，需要擦除参数区或由上位机重新 SET 后 SAVE。 */

/* ---------------------------------------------------------------
 * 车轮闭环 PI 控制器默认参数
 * --------------------------------------------------------------- */

/* 固定小数系数统一使用 x1000 缩放：例如 580 表示 0.580 */

/* 左/右轮闭环比例增益 Kp（无单位，按 x1000 缩放） */
#define LAB_DEFAULT_LEFT_KP_X1000       580
#define LAB_DEFAULT_RIGHT_KP_X1000      580

/* 左/右轮闭环积分增益 Ki（无单位，按 x1000 缩放；0 表示纯 P 控制） */
#define LAB_DEFAULT_LEFT_KI_X1000         0
#define LAB_DEFAULT_RIGHT_KI_X1000        0

/* 左/右轮前馈增益 FF（按 x1000 缩放），把目标速度的一部分直接叠加到 PWM */
#define LAB_DEFAULT_LEFT_FF_X1000       340
#define LAB_DEFAULT_RIGHT_FF_X1000      340

/* 左/右轮的最小有效 PWM：低于这个值电机可能因摩擦无法启动 */
#define LAB_DEFAULT_LEFT_MIN_PWM         45
#define LAB_DEFAULT_RIGHT_MIN_PWM        45

/* ---------------------------------------------------------------
 * 直线同步外环（两侧轮差速补偿）
 * --------------------------------------------------------------- */

/* 同步环 Kp：每 mm/s 的编码器计数偏差对应的速度修正量，按 x1000 缩放 */
#define LAB_DEFAULT_SYNC_KP_X1000      1000

/* 左右轮目标计数器差，单位 1/10000；用于在线校正机械偏置 */
#define LAB_DEFAULT_RIGHT_BIAS_MMPS      25

/* ---------------------------------------------------------------
 * 循迹 PD 外环（基于 8 路灰度加权误差）
 * --------------------------------------------------------------- */

/* 循迹 Kp：按 x1000 缩放的速度修正比例 */
#define LAB_DEFAULT_LINE_KP_X1000       6000

/* 循迹 Kd：按 x1000 缩放的速度修正微分项 */
#define LAB_DEFAULT_LINE_KD_X1000       2500

/* ---------------------------------------------------------------
 * 逆时针正方形跑图相关（方框模式）
 * 转向角使用 0.1° 单位（例如 900 = 90.0°）
 * --------------------------------------------------------------- */

/* 单次转向的目标偏航角，单位 0.1°（900 = 90°） */
#define LAB_DEFAULT_TURN_ANGLE_X10        900

/* 转向快速阶段的 PWM 占空比 */
#define LAB_DEFAULT_TURN_FAST_PWM         185

/* 转向接近完成时降速的 PWM 占空比 */
#define LAB_DEFAULT_TURN_SLOW_PWM         140

/* 切换到慢速阶段的偏航角裕量，单位 0.1° */
#define LAB_DEFAULT_TURN_SLOW_MARGIN_X10  180

/* 转弯完成、重新走直线时使用的目标速度，单位 mm/s */
#define LAB_DEFAULT_TURN_EXIT_MMPS         140

/* 转弯过程中按里程提前结束的阈值，单位 mm */
#define LAB_DEFAULT_TURN_DISTANCE_MM        85

#endif
