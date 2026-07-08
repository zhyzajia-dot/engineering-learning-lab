#include "sensor.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * sensor.c - 幻尔 8 路数字灰度传感器
 *
 * I2C0: SDA PA10, SCL PA11, 地址 0x5D，结果寄存器 5。
 * bit0 到 bit7 从左向右对应 S0 到 S7。
 *
 * 诊断码：
 *   0 = 正常
 *   4 = 传感器无响应
 *   5 = 地址有响应，但读取结果失败
 *   6 = I2C 总线忙或卡死
 */

#define LINE_SENSOR_ADDR          0x5DU
#define LINE_SENSOR_RESULT_REG    5U
#define I2C_TIMEOUT                5000U

static uint8_t g_rawMask = 0U;
static uint8_t g_rawByte = 0U;
static uint8_t g_i2cOk = 0U;
static uint8_t g_diagCode = 4U;
static uint8_t g_foundAddr = 0U;
static uint16_t g_badFrameCount = 0U;

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

/*
 * 向某个 IIC 地址写 1 个字节。
 * 能完成说明该地址 ACK。
 */
static uint8_t i2c_probe_addr(uint8_t addr)
{
    uint8_t reg = LINE_SENSOR_RESULT_REG;

    if (i2c_wait_bus_free() == 0U) {
        g_diagCode = 6U;
        return 0U;
    }

    i2c_clean();

    DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, &reg, 1U);

    DL_I2C_startControllerTransfer(I2C_BUS_INST,
                                   addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);

    if (i2c_wait_done() == 0U) {
        return 0U;
    }

    return 1U;
}

/* 当前只接受固定地址 0x5D。 */
static uint8_t i2c_scan_first_addr(void)
{
    if (i2c_probe_addr(LINE_SENSOR_ADDR) != 0U) {
        return LINE_SENSOR_ADDR;
    }
    return 0U;
}

static uint8_t i2c_write_reg_then_read(uint8_t addr, uint8_t reg, uint8_t *value)
{
    uint32_t timeout = I2C_TIMEOUT;

    if (value == 0) {
        return 0U;
    }

    if (i2c_wait_bus_free() == 0U) {
        g_diagCode = 6U;
        return 0U;
    }

    i2c_clean();

    DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, &reg, 1U);

    DL_I2C_startControllerTransfer(I2C_BUS_INST,
                                   addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);

    if (i2c_wait_done() == 0U) {
        return 0U;
    }

    for (volatile uint32_t i = 0U; i < 100U; i++) {
    }

    i2c_clean();

    DL_I2C_startControllerTransfer(I2C_BUS_INST,
                                   addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   1U);

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

    *value = DL_I2C_receiveControllerData(I2C_BUS_INST);

    if (i2c_wait_done() == 0U) {
        return 0U;
    }

    return 1U;
}

static uint8_t line_sensor_read_result(uint8_t *value)
{
    return i2c_write_reg_then_read(LINE_SENSOR_ADDR, LINE_SENSOR_RESULT_REG, value);
}

void SENSOR_Init(void)
{
    g_rawMask = 0U;
    g_rawByte = 0U;
    g_i2cOk = 0U;
    g_diagCode = 4U;
    g_foundAddr = 0U;
    g_badFrameCount = 0U;

    i2c_clean();
}

void SENSOR_ReadData(SENSOR_Data_t *data)
{
    uint8_t value = 0U;

    if (data == 0) {
        return;
    }

    /*
     * 上一帧成功后直接读取结果。读取失败时，下一帧重新探测地址。
     * 正常运行时每 5 ms 只执行一次结果读取。
     */
    if ((g_foundAddr == LINE_SENSOR_ADDR) && (g_i2cOk != 0U)) {
        if (line_sensor_read_result(&value) != 0U) {
            g_rawByte = value;
            g_rawMask = value;
            g_i2cOk = 1U;
            g_diagCode = 0U;

            data->mask = g_rawMask;
            data->valid = 1U;
            return;
        }

        /*
         * 读失败，标记下来，下一周期重新扫描。
         */
        g_i2cOk = 0U;
    }

    g_foundAddr = i2c_scan_first_addr();

    if (g_foundAddr == 0U) {
        g_rawMask = 0U;
        g_rawByte = 0U;
        g_i2cOk = 0U;
        g_diagCode = 4U;

        if (g_badFrameCount < 60000U) {
            g_badFrameCount++;
        }

        data->mask = 0U;
        data->valid = 0U;
        return;
    }

    if (g_foundAddr != LINE_SENSOR_ADDR) {
        g_rawMask = 0U;
        g_rawByte = g_foundAddr;
        g_i2cOk = 0U;
        g_diagCode = 7U;

        data->mask = 0U;
        data->valid = 0U;
        return;
    }

    if (line_sensor_read_result(&value) != 0U) {
        g_rawByte = value;
        g_rawMask = value;
        g_i2cOk = 1U;
        g_diagCode = 0U;

        data->mask = g_rawMask;
        data->valid = 1U;
    } else {
        g_rawMask = 0U;
        g_rawByte = 0U;
        g_i2cOk = 0U;
        g_diagCode = 5U;

        if (g_badFrameCount < 60000U) {
            g_badFrameCount++;
        }

        data->mask = 0U;
        data->valid = 0U;
    }
}

uint8_t SENSOR_GetRawMask(void)
{
    return g_rawMask;
}

uint8_t SENSOR_GetRawByte(void)
{
    return g_rawByte;
}

uint8_t SENSOR_GetDiagCode(void)
{
    return g_diagCode;
}

uint8_t SENSOR_GetI2cOk(void)
{
    return g_i2cOk;
}

uint8_t SENSOR_GetFoundAddr(void)
{
    return g_foundAddr;
}

uint16_t SENSOR_GetBadFrameCount(void)
{
    return g_badFrameCount;
}

