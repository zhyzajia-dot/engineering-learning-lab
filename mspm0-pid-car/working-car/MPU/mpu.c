#include "imu.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <string.h>

#define IMU_ADDR                 0x23U
#define IMU_EULER_REG            0x26U
#define IMU_EULER_LEN            12U
#define IMU_RAD_TO_DEG_X10       572.957795f
#define I2C_TIMEOUT               5000U

static uint8_t g_ready = 0U;
static uint8_t g_active = 0U;
static uint32_t g_okCount = 0U;
static uint32_t g_errorCount = 0U;
static int16_t g_rollX10 = 0;
static int16_t g_pitchX10 = 0;
static int16_t g_yawX10 = 0;
static int16_t g_yawZeroX10 = 0;

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

static void i2c_clean(void)
{
    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
}

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

static float bytes_to_float(const uint8_t *bytes)
{
    float value;

    memcpy(&value, bytes, sizeof(value));
    return value;
}

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

static uint8_t yaw_rad_is_valid(float yawRad)
{
    if (yawRad != yawRad) {
        return 0U;
    }

    return ((yawRad >= -6.5f) && (yawRad <= 6.5f)) ? 1U : 0U;
}

void IMU_Init(void)
{
    g_ready = 0U;
    g_active = 1U;
    g_okCount = 0U;
    g_errorCount = 0U;
    g_rollX10 = 0;
    g_pitchX10 = 0;
    g_yawX10 = 0;
    g_yawZeroX10 = 0;
}

void IMU_Task5ms(void)
{
    uint8_t buf[IMU_EULER_LEN];
    float yawRad;

    if (imu_read_bytes(IMU_EULER_REG, buf, IMU_EULER_LEN) == 0U) {
        g_ready = 0U;
        if (g_errorCount < 0xFFFFFFFFU) {
            g_errorCount++;
        }
        return;
    }

    yawRad = bytes_to_float(&buf[8]);
    if (yaw_rad_is_valid(yawRad) == 0U) {
        g_ready = 0U;
        if (g_errorCount < 0xFFFFFFFFU) {
            g_errorCount++;
        }
        return;
    }

    g_yawX10 = rad_to_deg_x10(yawRad);
    g_ready = 1U;

    if (g_okCount == 0U) {
        g_yawZeroX10 = g_yawX10;
    }

    if (g_okCount < 0xFFFFFFFFU) {
        g_okCount++;
    }
}

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
