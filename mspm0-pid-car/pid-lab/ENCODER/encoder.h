/*
 * encoder.h - 霍尔编码器测速 / 计数接口
 *
 * 主要能力：
 *   - 通过 A 相上升沿 GPIO 中断累计编码器脉冲
 *   - 5 ms 周期性任务入口，内部按 2:1 分频，最终生成 10 ms 周期的速度采样
 *   - 提供脉冲计数、累计距离、当前速度（mm/s）等读取接口
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 初始化编码器 GPIO 中断、计数器清零 */
void ENCODER_Init(void);

/* 5 ms 周期任务：在内部按 2:1 分频后实际每 10 ms 计算一次速度 */
void ENCODER_Task5ms(void);

/* 复位所有累计量（计数、速度、滤波器状态等） */
void ENCODER_Reset(void);

/* 仅复位速度反馈相关变量（用于闭环模式切换时避免大跳变） */
void ENCODER_ResetSpeedFeedback(void);

/* 读取最近一次计算得到的两轮速度，单位 mm/s */
int16_t ENCODER_GetLeftSpeed(void);
int16_t ENCODER_GetRightSpeed(void);

/* 速度采样序列号：每完成一次速度计算该值 +1，
 * 闭环任务通过它判断“本周期内是否已有新速度可用” */
uint32_t ENCODER_GetSpeedSampleSequence(void);

/* 读取当前两轮累计脉冲数（带方向信息） */
int32_t ENCODER_GetLeftCount(void);
int32_t ENCODER_GetRightCount(void);

/* 读取两轮累计行驶距离，单位 mm */
int32_t ENCODER_GetLeftDistanceMm(void);
int32_t ENCODER_GetRightDistanceMm(void);

#endif
