#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void IMU_Init(void);
void IMU_Task5ms(void);
void IMU_BeginRelativeYaw(void);

uint8_t IMU_IsReady(void);
uint8_t IMU_IsActive(void);

uint8_t IMU_GetAddress(void);

uint32_t IMU_GetOkCount(void);
uint32_t IMU_GetErrorCount(void);

uint16_t IMU_GetVersionCode(void);

int16_t IMU_GetRollX10(void);
int16_t IMU_GetPitchX10(void);
int16_t IMU_GetYawX10(void);

int16_t IMU_GetGyroX10(void);
int16_t IMU_GetGyroY10(void);
int16_t IMU_GetGyroZ10(void);

int16_t IMU_GetRelativeYawX10(void);

#endif
