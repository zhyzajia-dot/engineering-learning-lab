/*
 * 文件：imu.h
 * 用途：声明IMU欧拉角和相对yaw接口。
 * 单位：角度统一使用0.1°整数，900表示90.0°；相对yaw自动处理±180°
 *       附近的环绕，结果范围为(-1800, 1800]。
 * 注意：当前固件只解析欧拉yaw；Roll/Pitch、Gyro和Version接口为兼容
 *       旧界面而保留并固定返回0。
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/* 清空状态；不会修改IMU模块内部配置。 */
void IMU_Init(void);
/* 每5ms调用一次，读取0x26开始的12字节并更新yaw。 */
void IMU_Task5ms(void);
/* 把当前yaw记录为新的0°参考点，必须在开始转动车体前调用。 */
void IMU_BeginRelativeYaw(void);

/* 连续4帧失败后返回0；成功帧恢复后重新返回1。 */
uint8_t IMU_IsReady(void);
/* IMU_Init()后返回1，表示该模块功能已启用。 */
uint8_t IMU_IsActive(void);

/* 返回当前固定7位I2C地址0x23。 */
uint8_t IMU_GetAddress(void);

/* 启动以来的成功帧数和失败帧数。 */
uint32_t IMU_GetOkCount(void);
uint32_t IMU_GetErrorCount(void);

/* 兼容旧显示接口；当前未读取版本寄存器，固定返回0。 */
uint16_t IMU_GetVersionCode(void);

/* Roll/Pitch为兼容旧接口固定返回0；Yaw返回最近有效值，单位0.1°。 */
int16_t IMU_GetRollX10(void);
int16_t IMU_GetPitchX10(void);
int16_t IMU_GetYawX10(void);

/* 兼容旧显示接口；当前未读取角速度寄存器，固定返回0。 */
int16_t IMU_GetGyroX10(void);
int16_t IMU_GetGyroY10(void);
int16_t IMU_GetGyroZ10(void);

/* 当前yaw相对最近一次BeginRelativeYaw的短弧差值，单位0.1°。 */
int16_t IMU_GetRelativeYawX10(void);

#endif
