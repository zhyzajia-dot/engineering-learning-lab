/*
 * 文件：display.c
 * 用途：为OLED显示预留统一接口，避免比赛控制和具体屏幕驱动耦合。
 * 当前状态：所有函数都是空实现，仅消除未使用参数警告，不会访问I2C0，
 *           也不会在OLED上显示内容。
 * 硬件预留：I2C0 SDA=PA0、SCL=PA1。确认OLED控制器型号和地址后，可在
 *           本文件内实现显示，不需要修改比赛控制模块。
 */

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
