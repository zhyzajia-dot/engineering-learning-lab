#include "imu.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

/* IMU 7-bit I2C 地址 */
#define IMU_ADDR                 0x23U
/* 欧拉角寄存器起始地址（按小端 float 顺序：roll, pitch, yaw，各 4 字节） */
#define IMU_EULER_REG            0x26U
/* 一次性读取 12 字节：roll / pitch / yaw */
#define IMU_EULER_LEN            12U
/* 弧度转 0.1° 的系数：1800 / π */
#define IMU_RAD_TO_DEG_X10       572.957795f
/* I2C 等待/重试超时计数 */
#define I2C_TIMEOUT               5000U
/* 连续失败达到此次数才判定 IMU 离线，过滤偶发 I2C 干扰。 */
#define IMU_FAILURE_LIMIT            4U

/* 最近一次读取是否成功 */
static uint8_t g_ready = 0U;
/* 成功累计计数 */
static uint32_t g_okCount = 0U;
/* 失败累计计数 */
static uint32_t g_errorCount = 0U;
/* 连续失败计数；成功读取一帧后立即清零。 */
static uint8_t g_consecutiveErrors = 0U;
/* 当前 yaw，单位 0.1° */
static int16_t g_yawX10 = 0;
/* 相对参考点（IMU_BeginRelativeYaw() 时记录的 yaw） */
static int16_t g_yawZeroX10 = 0;

/* 把任意角度差值（0.1°）归到 (-1800, 1800] 区间 */
static int16_t clamp_angle_delta_x10(int16_t value)
{
    while (value > 1800) value = (int16_t)(value - 3600);
    while (value < -1800) value = (int16_t)(value + 3600);
    return value;
}

/* 软件复位 I2C 控制器、清空两个 FIFO */
static void i2c_clean(void)
{
    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
}

/* 等待 I2C 总线变为空闲；超时则复位 */
static uint8_t i2c_wait_bus_free(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_BUS_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout-- == 0U) {
            i2c_clean();
            return 0U;
        }
    }
    return 1U;
}

/* 等待上一次 I2C 传输结束；遇到错误或超时则复位 */
static uint8_t i2c_wait_done(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (1) {
        uint32_t status = DL_I2C_getControllerStatus(I2C_BUS_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            i2c_clean();
            return 0U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            return 1U;
        }
        if (timeout-- == 0U) {
            i2c_clean();
            return 0U;
        }
    }
}

/* 从 IMU 读 length 个字节到 buffer，先写寄存器地址再启动读传输 */
static uint8_t imu_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t index;

    if ((buffer == 0) || (length == 0U) ||
        (i2c_wait_bus_free() == 0U)) {
        return 0U;
    }

    /* 第一步：写寄存器地址（1 字节） */
    i2c_clean();
    DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(I2C_BUS_INST, IMU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (i2c_wait_done() == 0U) return 0U;

    /* 短延时，确保 IMU 已准备好返回数据 */
    for (volatile uint32_t delay = 0U; delay < 100U; delay++) {
    }

    /* 第二步：按 length 字节读回 */
    i2c_clean();
    DL_I2C_startControllerTransfer(I2C_BUS_INST, IMU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, length);
    for (index = 0U; index < length; index++) {
        uint32_t timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_BUS_INST) != false) {
            if ((DL_I2C_getControllerStatus(I2C_BUS_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                i2c_clean();
                return 0U;
            }
            if (timeout-- == 0U) {
                i2c_clean();
                return 0U;
            }
        }
        buffer[index] = DL_I2C_receiveControllerData(I2C_BUS_INST);
    }
    return i2c_wait_done();
}

/* 把 4 字节按 float 重新解释（IMU 数据是 IEEE754 小端） */
static float bytes_to_float(const uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

/* 弧度 -> 0.1° 整数，并做 int16 范围钳制和四舍五入 */
static int16_t rad_to_deg_x10(float radians)
{
    float value = radians * IMU_RAD_TO_DEG_X10;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

void IMU_Init(void)
{
    /* 清空所有统计与缓存 */
    g_ready = 0U;
    g_okCount = 0U;
    g_errorCount = 0U;
    g_consecutiveErrors = 0U;
    g_yawX10 = 0;
    g_yawZeroX10 = 0;
}

void IMU_Task5ms(void)
{
    uint8_t buffer[IMU_EULER_LEN];
    float yawRadians;

    /* 读取失败：标记无效，累加错误计数 */
    if (imu_read_bytes(IMU_EULER_REG, buffer, IMU_EULER_LEN) == 0U) {
        if (g_errorCount < 0xFFFFFFFFU) g_errorCount++;
        if (g_consecutiveErrors < IMU_FAILURE_LIMIT) g_consecutiveErrors++;
        if (g_consecutiveErrors >= IMU_FAILURE_LIMIT) g_ready = 0U;
        return;
    }

    /* yaw 在 buffer[8..11] */
    yawRadians = bytes_to_float(&buffer[8]);
    /* 拒绝 NaN 和明显超出物理意义的值（>±6.5 rad ≈ ±372°） */
    if ((yawRadians != yawRadians) ||
        (yawRadians < -6.5f) || (yawRadians > 6.5f)) {
        if (g_errorCount < 0xFFFFFFFFU) g_errorCount++;
        if (g_consecutiveErrors < IMU_FAILURE_LIMIT) g_consecutiveErrors++;
        if (g_consecutiveErrors >= IMU_FAILURE_LIMIT) g_ready = 0U;
        return;
    }

    g_yawX10 = rad_to_deg_x10(yawRadians);
    g_consecutiveErrors = 0U;
    g_ready = 1U;
    /* 第一帧成功时把“参考点”初始化为当前 yaw */
    if (g_okCount == 0U) g_yawZeroX10 = g_yawX10;
    if (g_okCount < 0xFFFFFFFFU) g_okCount++;
}

void IMU_BeginRelativeYaw(void)
{
    /* 重新记录参考点；之后相对值会自动从 0 开始累加 */
    g_yawZeroX10 = g_yawX10;
}

uint8_t IMU_IsReady(void)
{
    return g_ready;
}

uint32_t IMU_GetOkCount(void)
{
    return g_okCount;
}

uint32_t IMU_GetErrorCount(void)
{
    return g_errorCount;
}

int16_t IMU_GetYawX10(void)
{
    return g_yawX10;
}

int16_t IMU_GetRelativeYawX10(void)
{
    return clamp_angle_delta_x10((int16_t)(g_yawX10 - g_yawZeroX10));
}
