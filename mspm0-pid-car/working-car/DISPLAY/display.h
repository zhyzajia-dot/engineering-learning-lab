/*
 * 文件：display.h
 * 用途：预留OLED显示层接口，使传感器和比赛控制不依赖具体屏幕型号。
 * 当前状态：display.c为空实现，调用这些函数不会产生任何显示或I2C传输。
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "race_ctrl.h"

/* 预留初始化接口。 */
void DISPLAY_Init(void);
/* 以下接口分别预留给灰度、运行状态、I2C扫描和IMU页面。 */
void DISPLAY_ShowSensor(uint8_t sensorMask, uint8_t diagCode);
void DISPLAY_ShowDrive(const RACE_Status_t *status, uint8_t sensorMask, uint8_t diagCode);
void DISPLAY_ShowI2CScan(const uint8_t *addrs, uint8_t count, uint8_t lineAddr);
void DISPLAY_ShowIMUAngles(int16_t relYawX10,
                           int16_t rollX10,
                           int16_t pitchX10,
                           int16_t yawX10,
                           uint8_t ready);

#endif
