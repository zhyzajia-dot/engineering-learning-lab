/*
 * 文件：mpu.c
 * 用途：通过I2C读取IMU欧拉角，并提供绝对yaw和单次转弯相对yaw。
 * 总线：I2C1，SDA=PB3、SCL=PB2，与灰度传感器共用。
 * 协议：7位地址0x23，欧拉角起始寄存器0x26，连续读取roll/pitch/yaw
 *       三个小端IEEE754 float；当前控制只解析最后4字节的yaw。
 * 输出：yaw使用0.1°整数，例如900表示90.0°。
 * 容错：连续4帧读取或数据校验失败才把IMU判为不可用，偶发失败保留
 *       上一帧有效角度，避免转弯状态机因单次I2C干扰退化。
 * 注意：当前Roll/Pitch、Gyro和Version接口为兼容旧界面保留并固定返回0。
 */

#include "imu.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

#define IMU_ADDR                 0x23U
#define IMU_EULER_REG            0x26U
#define IMU_EULER_LEN            12U
#define IMU_RAD_TO_DEG_X10       572.957795f
#define I2C_TIMEOUT               5000U
#define IMU_FAILURE_LIMIT            4U

/* 就绪状态、通信统计和最近一帧有效yaw。 */
static uint8_t g_ready = 0U;
static uint8_t g_active = 0U;
static uint32_t g_okCount = 0U;
static uint32_t g_errorCount = 0U;
static uint8_t g_consecutiveErrors = 0U;
static int16_t g_rollX10 = 0;
static int16_t g_pitchX10 = 0;
static int16_t g_yawX10 = 0;
static int16_t g_yawZeroX10 = 0;

/* 把角度差归一化到(-180.0°, 180.0°]，避免跨±180°时突跳。 */
static int16_t clamp_angle_delta_x10(int16_t v)
{
    while (v > 1800) {
        v = (int16_t)(v - 3600);
    }
    while (v < -1800) {
        v = (int16_t)(v + 3600);
    }
    return v;
}

/* 出错、超时或开始下一笔传输前复位 I2C 控制器传输状态并清空 FIFO。 */
static void i2c_clean(void)
{
    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
}

/* 等待共享 I2C 总线空闲；超时视为总线异常并清理控制器。 */
static uint8_t i2c_wait_bus_free(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_BUS_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout == 0U) {
            i2c_clean();
            return 0U;
        }
        timeout--;
    }

    return 1U;
}

/* 等待当前传输完成，同时把 NACK、仲裁等控制器错误统一转为失败。 */
static uint8_t i2c_wait_done(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    uint32_t status;

    while (1) {
        status = DL_I2C_getControllerStatus(I2C_BUS_INST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            i2c_clean();
            return 0U;
        }

        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            break;
        }

        if (timeout == 0U) {
            i2c_clean();
            return 0U;
        }

        timeout--;
    }

    return 1U;
}

/* 读取 IMU 连续寄存器：先写起始寄存器地址，再发起接收传输。
 * 每个字节均检查 FIFO 和超时，失败时不修改上层已保存的有效 yaw。 */
static uint8_t imu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if ((buf == 0) || (len == 0U)) {
        return 0U;
    }

    if (i2c_wait_bus_free() == 0U) {
        return 0U;
    }

    /* 先写寄存器地址，再单独发起连续读。 */
    i2c_clean();
    DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(I2C_BUS_INST,
                                   IMU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);
    if (i2c_wait_done() == 0U) {
        return 0U;
    }

    for (volatile uint32_t d = 0U; d < 100U; d++) {
    }

    i2c_clean();
    DL_I2C_startControllerTransfer(I2C_BUS_INST,
                                   IMU_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   len);

    for (i = 0U; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_BUS_INST) != false) {
            if ((DL_I2C_getControllerStatus(I2C_BUS_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                i2c_clean();
                return 0U;
            }

            if (timeout == 0U) {
                i2c_clean();
                return 0U;
            }

            timeout--;
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_BUS_INST);
    }

    return i2c_wait_done();
}

/* 将设备返回的小端 IEEE754 四字节直接还原为 float。 */
static float bytes_to_float(const uint8_t *bytes)
{
    float value;

    memcpy(&value, bytes, sizeof(value));
    return value;
}

/* 弧度转为 0.1°整数，并在 int16_t 范围内饱和、四舍五入。 */
static int16_t rad_to_deg_x10(float rad)
{
    float degX10 = rad * IMU_RAD_TO_DEG_X10;

    if (degX10 > 32767.0f) {
        return 32767;
    }

    if (degX10 < -32768.0f) {
        return -32768;
    }

    if (degX10 >= 0.0f) {
        return (int16_t)(degX10 + 0.5f);
    }

    return (int16_t)(degX10 - 0.5f);
}

/* 拒绝 NaN 及超过约 ±2π 的异常欧拉角，避免坏帧驱动转弯状态机。 */
static uint8_t yaw_rad_is_valid(float yawRad)
{
    if (yawRad != yawRad) {
        return 0U;
    }

    return ((yawRad >= -6.5f) && (yawRad <= 6.5f)) ? 1U : 0U;
}

/* 初始化通信统计与航向参考点；实际 I2C 读数从后续 5 ms 任务开始。 */
void IMU_Init(void)
{
    g_ready = 0U;
    g_active = 1U;
    g_okCount = 0U;
    g_errorCount = 0U;
    g_consecutiveErrors = 0U;
    g_rollX10 = 0;
    g_pitchX10 = 0;
    g_yawX10 = 0;
    g_yawZeroX10 = 0;
}

/* IMU 周期采样任务。成功帧更新绝对 yaw；连续四帧读错或校验失败才撤销 ready，
 * 单帧干扰保留上一帧角度，避免正常转弯被偶发 I2C 抖动中断。 */
void IMU_Task5ms(void)
{
    uint8_t buf[IMU_EULER_LEN];
    float yawRad;

    if (imu_read_bytes(IMU_EULER_REG, buf, IMU_EULER_LEN) == 0U) {
        if (g_errorCount < 0xFFFFFFFFU) {
            g_errorCount++;
        }
        if (g_consecutiveErrors < IMU_FAILURE_LIMIT) {
            g_consecutiveErrors++;
        }
        if (g_consecutiveErrors >= IMU_FAILURE_LIMIT) {
            g_ready = 0U;
        }
        return;
    }

    /* 12字节布局为roll[0..3]、pitch[4..7]、yaw[8..11]。 */
    yawRad = bytes_to_float(&buf[8]);
    if (yaw_rad_is_valid(yawRad) == 0U) {
        if (g_errorCount < 0xFFFFFFFFU) {
            g_errorCount++;
        }
        if (g_consecutiveErrors < IMU_FAILURE_LIMIT) {
            g_consecutiveErrors++;
        }
        if (g_consecutiveErrors >= IMU_FAILURE_LIMIT) {
            g_ready = 0U;
        }
        return;
    }

    g_yawX10 = rad_to_deg_x10(yawRad);
    g_consecutiveErrors = 0U;
    g_ready = 1U;

    if (g_okCount == 0U) {
        g_yawZeroX10 = g_yawX10;
    }

    if (g_okCount < 0xFFFFFFFFU) {
        g_okCount++;
    }
}

/* 把当前绝对 yaw 设为相对转角零点；调用前应先确认 IMU_IsReady() 为真。 */
void IMU_BeginRelativeYaw(void)
{
    g_yawZeroX10 = g_yawX10;
}

uint8_t IMU_IsReady(void)
{
    return g_ready;
}

uint8_t IMU_IsActive(void)
{
    return g_active;
}

uint8_t IMU_GetAddress(void)
{
    return IMU_ADDR;
}

uint32_t IMU_GetOkCount(void)
{
    return g_okCount;
}

uint32_t IMU_GetErrorCount(void)
{
    return g_errorCount;
}

uint16_t IMU_GetVersionCode(void)
{
    return 0U;
}

int16_t IMU_GetRollX10(void)
{
    return g_rollX10;
}

int16_t IMU_GetPitchX10(void)
{
    return g_pitchX10;
}

int16_t IMU_GetYawX10(void)
{
    return g_yawX10;
}

int16_t IMU_GetGyroX10(void)
{
    return 0;
}

int16_t IMU_GetGyroY10(void)
{
    return 0;
}

int16_t IMU_GetGyroZ10(void)
{
    return 0;
}

int16_t IMU_GetRelativeYawX10(void)
{
    return clamp_angle_delta_x10((int16_t)(g_yawX10 - g_yawZeroX10));
}
