/*
 * 文件：sensor.h
 * 用途：声明数字灰度传感器读取结果和诊断接口。
 * 单位/格式：mask的bit0～bit7对应S0～S7；valid表示本次是否得到有效帧。
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct {
    uint8_t mask;   /* 当前灰度原始位图 */
    uint8_t valid;  /* 1=本次读取有效，0=通信或数据异常 */
} SENSOR_Data_t;

/* 清空缓存并让下一次读取重新确认固定地址0x5D。 */
void SENSOR_Init(void);
/* 每5ms调用一次；data必须指向有效结果结构，NULL时函数直接返回。 */
void SENSOR_ReadData(SENSOR_Data_t *data);

/* 最近一次有效帧的灰度位图。 */
uint8_t SENSOR_GetRawMask(void);
/* 最近一次从结果寄存器读到的原始字节。 */
uint8_t SENSOR_GetRawByte(void);
/* 诊断码：0正常，4无响应，5读取失败，6总线异常，7发现非0x5D设备。 */
uint8_t SENSOR_GetDiagCode(void);
/* 1表示最近一次通信正常。 */
uint8_t SENSOR_GetI2cOk(void);
/* 已确认的I2C地址；尚未确认时返回0。 */
uint8_t SENSOR_GetFoundAddr(void);
/* 启动以来累计的坏帧次数。 */
uint16_t SENSOR_GetBadFrameCount(void);

#endif
