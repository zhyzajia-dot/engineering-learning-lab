/*
 * motor.h - TB6612 双路电机驱动接口
 *
 * 对上层只暴露“逻辑 PWM”：正数表示前进、负数表示后退，方向由本模块内部换算。
 * 这样上层做差速 / 转向时不用关心电机的实际安装方向。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 初始化 TB6612：拉高 STBY、清零 PWM、清零方向 */
void MOTOR_Init(void);

/* 立即让两个电机停转（方向线拉低、PWM 占空比 0） */
void MOTOR_Stop(void);

/*
 * 设置左右电机目标 PWM：
 *   leftPwm  > 0 左轮前进
 *   leftPwm  < 0 左轮后退
 *   rightPwm 同理
 * 内部已做限幅和方向符号修正。
 */
void MOTOR_SetPWM(int16_t leftPwm, int16_t rightPwm);

/* 获取最近一次设置的逻辑 PWM（用于日志/上位机显示） */
int16_t MOTOR_GetLeftPWM(void);
int16_t MOTOR_GetRightPWM(void);

#endif