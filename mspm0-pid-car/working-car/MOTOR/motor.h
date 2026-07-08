#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void MOTOR_Init(void);
void MOTOR_Stop(void);
void MOTOR_SetPWM(int16_t leftPwm, int16_t rightPwm);

int16_t MOTOR_GetLeftPWM(void);
int16_t MOTOR_GetRightPWM(void);

#endif