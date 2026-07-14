/*
 * imu.h - IMU 偏航角（yaw）接口
 *
 * 周期任务读取 IMU 内部欧拉角寄存器，把 yaw（弧度）转成 0.1° 整数。
 * 同时提供“相对偏航”：调用 IMU_BeginRelativeYaw() 后，
 * IMU_GetRelativeYawX10() 返回当前 yaw 与参考点的差值（绕短弧修正）。
 *
 * 主要用于方框模式下的转弯判定。
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/*
 * 容错说明：
 *   单次 I2C 失败不会立即判定 IMU 离线；
 *   连续 4 次失败才标记为不可用，下一次成功读取后自动恢复。
 *   IMU 只辅助转弯减速，编码器里程和出弯重新捕线仍是主要判据。
 */

/* 初始化内部状态、统计计数 */
void IMU_Init(void);

/* 10 ms 周期任务：读取一帧 IMU 欧拉角，更新 yaw */
void IMU_Task10ms(void);

/* 把当前 yaw 当作新参考点，IMU_GetRelativeYawX10() 后续返回相对值 */
void IMU_BeginRelativeYaw(void);

/* IMU 当前数据是否有效（最近一次任务是否成功） */
uint8_t IMU_IsReady(void);

/* 自上电以来成功读取次数 */
uint32_t IMU_GetOkCount(void);

/* 自上电以来读取失败次数 */
uint32_t IMU_GetErrorCount(void);

/* 当前 yaw 绝对值，单位 0.1° */
int16_t IMU_GetYawX10(void);

/* 相对 yaw（相对 BeginRelativeYaw 时的 yaw），单位 0.1°，取值范围 (-1800, 1800] */
int16_t IMU_GetRelativeYawX10(void);

#endif
