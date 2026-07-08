#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void ENCODER_Init(void);
void ENCODER_Task5ms(void);
void ENCODER_Reset(void);
void ENCODER_ResetSpeedFeedback(void);

int16_t ENCODER_GetLeftSpeed(void);
int16_t ENCODER_GetRightSpeed(void);
uint32_t ENCODER_GetSpeedSampleSequence(void);

int32_t ENCODER_GetLeftCount(void);
int32_t ENCODER_GetRightCount(void);

int32_t ENCODER_GetLeftDistanceMm(void);
int32_t ENCODER_GetRightDistanceMm(void);

#endif
