/*
 * 文件：car.c
 * 用途：独立循迹小车的程序入口、系统初始化和周期任务调度。
 * 硬件：天猛星 MSPM0G3507 + 最新自制底板。
 * 时序：CTRL_TIMER 每1ms产生一次中断；灰度、IMU、编码器和比赛控制
 *       按5ms周期在主循环中执行。
 * 注意：关键任务顺序为 IMU -> 编码器 -> 比赛控制，保证转弯判断使用
 *       本周期最新的角度和里程数据。
 */

#include "ti_msp_dl_config.h"

#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "race_ctrl.h"
#include "ui.h"
#include "imu.h"

#include <stdint.h>

/* 主循环任务周期；编码器内部每10ms形成一帧新速度。 */

#define SENSOR_PERIOD_MS        5U
#define RACE_PERIOD_MS          5U

static volatile uint32_t g_msTick = 0U;

/* 1ms控制定时中断：维护系统毫秒时间并完成按键原始扫描。 */
void CTRL_TIMER_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CTRL_TIMER_INST)) {
    case DL_TIMER_IIDX_ZERO:
        g_msTick++;
        UI_Task1ms();
        break;
    default:
        break;
    }
}

static uint32_t millis(void)
{
    return g_msTick;
}

int main(void)
{
    uint32_t lastSensorTick = 0U;
    uint32_t lastRaceTick = 0U;
    SENSOR_Data_t sensor;

    /* 先初始化SysConfig生成的时钟、GPIO、I2C、PWM和定时器配置。 */
    SYSCFG_DL_init();

    MOTOR_Init();
    MOTOR_Stop();

    ENCODER_Init();
    SENSOR_Init();
    IMU_Init();

    RACE_Init();
    UI_Init();

    NVIC_ClearPendingIRQ(CTRL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CTRL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CTRL_TIMER_INST);
    __enable_irq();

    /* 所有耗时I2C和控制计算均放在主循环，避免阻塞1ms中断。 */
    while (1) {
        UI_Process();

        if ((millis() - lastSensorTick) >= SENSOR_PERIOD_MS) {
            lastSensorTick = millis();
            SENSOR_ReadData(&sensor);
        }

        if ((millis() - lastRaceTick) >= RACE_PERIOD_MS) {
            lastRaceTick = millis();
            IMU_Task5ms();
            ENCODER_Task5ms();
            RACE_Task5ms();
        }
    }
}
