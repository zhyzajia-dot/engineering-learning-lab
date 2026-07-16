/*
 * sensor.h - 8 路数字灰度传感器接口
 *
 * 上层用 SENSOR_Data_t 拿到“当前一帧”的传感器快照，
 * 通过 SENSOR_Get* 系列可以拿到诊断信息和原始字节。
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

/* 单帧传感器数据：mask 是 8 位有效位，valid 表示这一帧是否可信 */
typedef struct {
    uint8_t mask;
    uint8_t valid;
} SENSOR_Data_t;

/* 初始化 I2C 状态、清空诊断计数 */
void SENSOR_Init(void);

/* 读取一帧传感器数据（包含自动寻址和重试逻辑） */
void SENSOR_ReadData(SENSOR_Data_t *data);

/* 读最近一次的有效位掩码（bit0=S0, ..., bit7=S7） */
uint8_t SENSOR_GetRawMask(void);

/* 读最近一次 I2C 读到的原始字节（出错时也保存错误位置） */
uint8_t SENSOR_GetRawByte(void);

/* 诊断码：0=正常 4=无响应 5=ACK 但读失败 6=I2C 总线忙 7=地址异常 */
uint8_t SENSOR_GetDiagCode(void);

/* I2C 通信是否正常（最近一次帧是否成功） */
uint8_t SENSOR_GetI2cOk(void);

/* Line control may reuse the last good sample across a short I2C glitch. */
uint8_t SENSOR_IsLineDataUsable(void);

/* 当前识别到的传感器 I2C 地址（0 表示未识别） */
uint8_t SENSOR_GetFoundAddr(void);

/* 自上电以来累计的错误帧数 */
uint16_t SENSOR_GetBadFrameCount(void);

#endif
