#include "ti_msp_dl_config.h"

#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "race_ctrl.h"
#include "ui.h"
#include "imu.h"

#include <stdint.h>

/*
 * Main control loop.
 * - CTRL_TIMER and key scan: 1 ms
 * - Line sensor, IMU and race control: 5 ms
 * - Encoder speed sample: 10 ms inside ENCODER_Task5ms()
 */

#define SENSOR_PERIOD_MS        5U
#define RACE_PERIOD_MS          5U

static volatile uint32_t g_msTick = 0U;

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

    while (1) {
        UI_Process();

        if ((millis() - lastSensorTick) >= SENSOR_PERIOD_MS) {
            lastSensorTick = millis();
            SENSOR_ReadData(&sensor);
        }

        if ((millis() - lastRaceTick) >= RACE_PERIOD_MS) {
            lastRaceTick = millis();
            IMU_Task5ms();
            RACE_Task5ms();
            ENCODER_Task5ms();
        }
    }
}
