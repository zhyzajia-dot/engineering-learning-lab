/*
 * 文件：encoder.h
 * 用途：声明左右轮脉冲计数、轮程和速度接口。
 * 单位：速度为mm/s，距离为mm，计数为A相上升沿脉冲数。
 * 注意：当前实现按绝对转动方向累计脉冲，适合比赛总里程和原地转弯
 *       轮程判断，不提供带正负方向的真实轮速。
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 配置GPIO边沿中断并清零全部编码器状态。 */
void ENCODER_Init(void);
/* 每5ms调用一次；内部每2次调用生成一帧10ms速度样本。 */
void ENCODER_Task5ms(void);
/* 清零累计计数、距离、速度和速度样本序号。 */
void ENCODER_Reset(void);
/* 只清速度反馈，不清比赛累计距离。 */
void ENCODER_ResetSpeedFeedback(void);

/* 最近一帧滤波轮速，单位mm/s。 */
int16_t ENCODER_GetLeftSpeed(void);
int16_t ENCODER_GetRightSpeed(void);
/* 每产生一帧新速度加1，控制器可据此避免重复计算。 */
uint32_t ENCODER_GetSpeedSampleSequence(void);

/* 累计A相脉冲计数。 */
int32_t ENCODER_GetLeftCount(void);
int32_t ENCODER_GetRightCount(void);

/* 由累计脉冲换算的绝对轮程，单位mm。 */
int32_t ENCODER_GetLeftDistanceMm(void);
int32_t ENCODER_GetRightDistanceMm(void);

#endif
