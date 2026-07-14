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

int main(void)
{
    /* 编码器和共享 I2C 任务独立分频，避免无效占用总线。 */
    uint32_t lastEncoderTick = 0U;
    uint32_t lastI2cTick = 0U;
    /* 当前灰度传感器的快照（循迹算法使用） */
    SENSOR_Data_t sensor;

    /* SysConfig 自动生成的底层初始化：时钟、GPIO、外设配置等 */
    SYSCFG_DL_init();

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
            IMU_Task10ms();
        }

        /* 上位机控制任务：解析串口命令、按键、闭环控制、状态打印 */
        LAB_Task(millis());

        /* 只有完整执行完一轮主循环才喂狗；任一任务卡死都会触发安全复位。 */
        watchdog_feed();
    }
}
