#include "display.h"

void DISPLAY_Init(void)
{
}

void DISPLAY_ShowSensor(uint8_t sensorMask, uint8_t diagCode)
{
    (void)sensorMask;
    (void)diagCode;
}

void DISPLAY_ShowDrive(const RACE_Status_t *status,
                       uint8_t sensorMask,
                       uint8_t diagCode)
{
    (void)status;
    (void)sensorMask;
    (void)diagCode;
}

void DISPLAY_ShowI2CScan(const uint8_t *addrs, uint8_t count, uint8_t lineAddr)
{
    (void)addrs;
    (void)count;
    (void)lineAddr;
}

void DISPLAY_ShowIMUAngles(int16_t relYawX10,
                           int16_t rollX10,
                           int16_t pitchX10,
                           int16_t yawX10,
                           uint8_t ready)
{
    (void)relYawX10;
    (void)rollX10;
    (void)pitchX10;
    (void)yawX10;
    (void)ready;
}
