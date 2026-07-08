#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct {
    uint8_t mask;
    uint8_t valid;
} SENSOR_Data_t;

void SENSOR_Init(void);
void SENSOR_ReadData(SENSOR_Data_t *data);

uint8_t SENSOR_GetRawMask(void);
uint8_t SENSOR_GetRawByte(void);
uint8_t SENSOR_GetDiagCode(void);
uint8_t SENSOR_GetI2cOk(void);
uint8_t SENSOR_GetFoundAddr(void);
uint16_t SENSOR_GetBadFrameCount(void);

#endif
