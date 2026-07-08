#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "race_ctrl.h"

void DISPLAY_Init(void);
void DISPLAY_ShowSensor(uint8_t sensorMask, uint8_t diagCode);
void DISPLAY_ShowDrive(const RACE_Status_t *status, uint8_t sensorMask, uint8_t diagCode);
void DISPLAY_ShowI2CScan(const uint8_t *addrs, uint8_t count, uint8_t lineAddr);
void DISPLAY_ShowIMUAngles(int16_t relYawX10,
                           int16_t rollX10,
                           int16_t pitchX10,
                           int16_t yawX10,
                           uint8_t ready);

#endif
