/*
 * 文件：motor.h
 * 用途：声明TB6612左右电机驱动接口。
 * 参数：PWM为带符号逻辑值，正数前进、负数后退、0停止，模块内部限幅
 *       到 -620～620；所有接口始终按“左轮、右轮”顺序。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 初始化方向、STBY和PWM定时器；初始化后输出为0。 */
void MOTOR_Init(void);
/* 两轮PWM清零并把方向输入置为停止状态。 */
void MOTOR_Stop(void);
/* 同时设置左右轮逻辑PWM。 */
void MOTOR_SetPWM(int16_t leftPwm, int16_t rightPwm);

/* 返回最近一次保存的逻辑PWM值，供状态显示和诊断使用。 */
int16_t MOTOR_GetLeftPWM(void);
int16_t MOTOR_GetRightPWM(void);

#endif
