/*
 * main.c - PID 学习小车工程主入口
 *
 * 整体任务编排：
 *   1. SysConfig 初始化（时钟、GPIO、UART、I2C、PWM、Timer）
 *   2. 依次使能电机、编码器、串口、灰度传感器、IMU、上位机协议模块
 *   3. 启动 1 ms 定时器作为整个工程的时间基准
 *   4. 主循环里高频处理串口与编码器/传感器/IMU，慢速执行上位机控制任务
 *
 * 安全约束：
 *   - 上电/复位后两个电机都保持停转
 *   - 调参模式由上位机显式开启；方框赛道可用 SW1/SW2 选圈、SW3 启停
 *   - 任何模式都受 LAB 模块自身的输出限幅与超时保护
 */

#include "ti_msp_dl_config.h"

#include "motor.h"
#include "encoder.h"
#include "serial.h"
#include "lab_ctrl.h"
#include "sensor.h"
#include "imu.h"

#include <stdint.h>

/* 编码器任务保持 5 ms；共享 I2C 的灰度和 IMU 与 10 ms 控制周期对齐。 */
#define ENCODER_TASK_PERIOD_MS  5U
#define I2C_TASK_PERIOD_MS     10U

/* Build-only diagnostic.  When enabled, the firmware scans all legal 7-bit
 * addresses once after the IMU startup delay and prints each address whose
 * address phase is ACKed.  Normal firmware keeps this disabled. */
#ifndef I2C_ADDRESS_SCAN_DIAG_ENABLE
#define I2C_ADDRESS_SCAN_DIAG_ENABLE 0
#endif

/*
 * 临时硬件定位开关。正常工程保持 0；单独的诊断 HEX 会临时改为 1。
 * 该测试只允许在灰度传感器和 IMU 已拔掉时使用：PB2/PB3 会被短暂
 * 配成推挽 GPIO，输出可被逻辑分析仪直接辨认的互补方波，随后恢复 I2C1。
 */
#ifndef I2C_PIN_SELFTEST_ENABLE
#define I2C_PIN_SELFTEST_ENABLE 0
#endif

#if (I2C_PIN_SELFTEST_ENABLE != 0)
static void i2c_pin_selftest(void)
{
    uint8_t cycle;

    /* 覆盖 SysConfig 的 I2C 复用，仅用于确认 PB2/PB3 的实际物理测点。 */
    DL_GPIO_initDigitalOutput(GPIO_I2C_BUS_IOMUX_SCL);
    DL_GPIO_initDigitalOutput(GPIO_I2C_BUS_IOMUX_SDA);
    DL_GPIO_enableOutput(GPIO_I2C_BUS_SCL_PORT, GPIO_I2C_BUS_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_BUS_SDA_PORT, GPIO_I2C_BUS_SDA_PIN);

    /* 约 5 秒的互补电平：CH0/CH1 必须交替高低，便于任意时刻开始采集。 */
    for (cycle = 0U; cycle < 50U; cycle++) {
        DL_GPIO_setPins(GPIO_I2C_BUS_SCL_PORT, GPIO_I2C_BUS_SCL_PIN);
        DL_GPIO_clearPins(GPIO_I2C_BUS_SDA_PORT, GPIO_I2C_BUS_SDA_PIN);
        delay_cycles(CPUCLK_FREQ / 20U);

        DL_GPIO_clearPins(GPIO_I2C_BUS_SCL_PORT, GPIO_I2C_BUS_SCL_PIN);
        DL_GPIO_setPins(GPIO_I2C_BUS_SDA_PORT, GPIO_I2C_BUS_SDA_PIN);
        delay_cycles(CPUCLK_FREQ / 20U);
    }

    /* 还原 PB2/PB3 的 I2C1 复用和控制器状态，之后程序按正常流程运行。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_BUS_IOMUX_SDA, GPIO_I2C_BUS_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_I2C_BUS_IOMUX_SCL, GPIO_I2C_BUS_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_BUS_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_BUS_IOMUX_SCL);
    SYSCFG_DL_I2C_BUS_init();
}
#endif

/* 独立看门狗约 1 秒超时：主循环若因 I2C 或程序异常彻底卡死，
 * MCU 会自动复位；复位后 PWM 和方向脚按初始化流程回到停转状态。 */
static void watchdog_init(void)
{
    /* WWDT0 已由 SysConfig 配置为约 1 秒、无关闭窗口的看门狗模式。
     * 此处只在各软件模块初始化完成后重新开始一次完整计时周期。 */
    DL_WWDT_restart(WWDT0_INST);
}

static void watchdog_feed(void)
{
    DL_WWDT_restart(WWDT0_INST);
}

/* 由 1 ms 定时器中断累加得到的时间戳，提供给其他模块做时间相关判断 */
static volatile uint32_t g_msTick = 0U;

/* 1 ms 周期定时器中断：只做时间戳累加，逻辑尽量短 */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_ZERO:
        g_msTick++;
        break;
    default:
        break;
    }
}

/* 提供给同文件外的毫秒时间查询接口（仅本工程内部使用） */
static uint32_t millis(void)
{
    return g_msTick;
}

#if (I2C_ADDRESS_SCAN_DIAG_ENABLE != 0)
static uint8_t i2c_probe_address(uint8_t address)
{
    uint8_t dummy = 0U;
    uint8_t addressAcked = 0U;
    uint32_t timeout = 20000U;

    while ((DL_I2C_getControllerStatus(I2C_BUS_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (timeout-- == 0U) return 0U;
    }

    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
    DL_I2C_clearInterruptStatus(I2C_BUS_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_fillControllerTXFIFO(I2C_BUS_INST, &dummy, 1U);
    DL_I2C_startControllerTransfer(I2C_BUS_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    timeout = 20000U;
    while (timeout-- != 0U) {
        uint32_t status = DL_I2C_getControllerStatus(I2C_BUS_INST);
        uint32_t raw = DL_I2C_getRawInterruptStatus(I2C_BUS_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK |
            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0U) {
            addressAcked = 1U;
        }
        if ((raw & (DL_I2C_INTERRUPT_CONTROLLER_NACK |
                    DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)) != 0U ||
            (raw & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            break;
        }
    }

    DL_I2C_resetControllerTransfer(I2C_BUS_INST);
    DL_I2C_flushControllerTXFIFO(I2C_BUS_INST);
    DL_I2C_flushControllerRXFIFO(I2C_BUS_INST);
    return addressAcked;
}

static void run_i2c_address_scan(void)
{
    uint8_t address;

    while (millis() < 6000U) {
        SERIAL_Task();
        watchdog_feed();
    }

    while (1) {
        SERIAL_SendString("I2C_SCAN,BEGIN\r\n");
        for (address = 0x08U; address <= 0x77U; address++) {
            if (i2c_probe_address(address) != 0U) {
                SERIAL_SendString("I2C_FOUND,");
                SERIAL_SendInt32((int32_t)address);
                SERIAL_SendString("\r\n");
            }
            SERIAL_Task();
            watchdog_feed();
            delay_cycles(CPUCLK_FREQ / 1000U);
        }
        SERIAL_SendString("I2C_SCAN,END\r\n");
        for (address = 0U; address < 100U; address++) {
            SERIAL_Task();
            watchdog_feed();
            delay_cycles(CPUCLK_FREQ / 1000U);
        }
    }
}
#endif

int main(void)
{
    /* 编码器和共享 I2C 任务独立分频，避免无效占用总线。 */
    uint32_t lastEncoderTick = 0U;
    uint32_t lastI2cTick = 0U;
    /* 当前灰度传感器的快照（循迹算法使用） */
    SENSOR_Data_t sensor;

    /* SysConfig 自动生成的底层初始化：时钟、GPIO、外设配置等 */
    SYSCFG_DL_init();

#if (I2C_PIN_SELFTEST_ENABLE != 0)
    i2c_pin_selftest();
#endif

    /* 各功能模块的初始化与安全默认状态：
     *   - SysConfig/MOTOR : STBY 上电置高一次，PWM/方向清零
     *   - ENCODER_Init    : 编码器 GPIO 中断使能、计数清零
     *   - SERIAL_Init     : 清空串口收发环形缓冲区、使能 UART 中断
     *   - SENSOR_Init     : 清空 I2C 状态，准备首次寻址
     *   - IMU_Init        : 复位 IMU 状态计数
     *   - LAB_Init        : 加载 Flash 参数并打印欢迎信息
     */
    MOTOR_Init();
    ENCODER_Init();
    SERIAL_Init();
    SENSOR_Init();
    IMU_Init();
    LAB_Init();
    watchdog_init();

    /* 启动 1 ms 周期定时器并开启全局中断，时间戳 g_msTick 开始累加 */
    NVIC_ClearPendingIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CTRL_TIMER_INST);
    __enable_irq();

#if (I2C_ADDRESS_SCAN_DIAG_ENABLE != 0)
    run_i2c_address_scan();
#endif

    while (1) {
        /* 高频：尽快把已到达的字符处理掉，避免环形缓冲区溢出 */
        SERIAL_Task();

        /* 编码器计数本身由中断完成；5 ms 任务只负责测速分频。 */
        if ((millis() - lastEncoderTick) >= ENCODER_TASK_PERIOD_MS) {
            lastEncoderTick = millis();
            ENCODER_Task5ms();
        }

        /* 灰度和 IMU 共用 I2C1，顺序访问并降到控制所需的 10 ms 周期。 */
        if ((millis() - lastI2cTick) >= I2C_TASK_PERIOD_MS) {
            lastI2cTick = millis();
            SENSOR_ReadData(&sensor);
            IMU_Task10ms(lastI2cTick);
        }

        /* 上位机控制任务：解析串口命令、按键、闭环控制、状态打印 */
        LAB_Task(millis());

        /* 只有完整执行完一轮主循环才喂狗；任一任务卡死都会触发安全复位。 */
        watchdog_feed();
    }
}
