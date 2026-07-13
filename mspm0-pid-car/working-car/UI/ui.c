/*
 * 文件：ui.c
 * 用途：处理底板三个按键的消抖、短按、长按启动和运行中急停。
 * 硬件：SW1/PB23、SW2/PA8、SW3/PB6，均使用内部上拉并低电平有效。
 * 时序：UI_Task1ms() 必须在1ms定时中断中调用；UI_Process() 放在主循环，
 *       将中断中产生的动作安全地交给比赛控制模块执行。
 * 交互：空闲短按切换1～5圈，空闲长按500ms启动，运行中按键立即停车。
 */

#include "ui.h"
#include "race_ctrl.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/* 上拉输入：松开为1，按下为0。 */

#define KEY_DEBOUNCE_MS             25U
#define KEY_LONG_PRESS_MS           500U
#define KEY_POWER_ON_IGNORE_MS      150U

static uint8_t g_rawLast = 0U;
static uint8_t g_keyStable = 0U;
static uint8_t g_keyLastStable = 0U;

static uint16_t g_sameMs = 0U;
static uint16_t g_powerOnMs = 0U;
static uint16_t g_pressMs = 0U;

static uint8_t g_longDone = 0U;
static volatile uint8_t g_pendingAction = 0U;

#define UI_ACTION_NONE              0U
#define UI_ACTION_START             1U
#define UI_ACTION_STOP              2U
#define UI_ACTION_NEXT_LAP          3U

/* 读取三个上拉按键并合并为一个“任意键按下”状态，低电平表示按下。 */
static uint8_t key_pressed_raw(void)
{
    uint32_t sw1 = DL_GPIO_readPins(KEY_SW1_PORT, KEY_SW1_PIN);
    uint32_t sw2 = DL_GPIO_readPins(KEY_SW2_PORT, KEY_SW2_PIN);
    uint32_t sw3 = DL_GPIO_readPins(KEY_SW3_PORT, KEY_SW3_PIN);

    /* 三个按键都保持原工程的交互语义，任意一个低电平都表示按下。 */
    return (((sw1 & KEY_SW1_PIN) == 0U) ||
            ((sw2 & KEY_SW2_PIN) == 0U) ||
            ((sw3 & KEY_SW3_PIN) == 0U)) ? 1U : 0U;
}

/* 以当前按键电平作为初值，并清除上电、消抖和按键动作状态。 */
void UI_Init(void)
{
    g_rawLast = key_pressed_raw();
    g_keyStable = g_rawLast;
    g_keyLastStable = g_keyStable;

    g_sameMs = 0U;
    g_powerOnMs = 0U;
    g_pressMs = 0U;
    g_longDone = 0U;
    g_pendingAction = UI_ACTION_NONE;
}

/* 1 ms 中断任务：完成上电屏蔽、25 ms 消抖、长短按识别。
 * 不直接控制电机，而是写入 volatile 的待执行动作，避免在中断内运行赛道状态机。 */
void UI_Task1ms(void)
{
    uint8_t raw;

    if (g_powerOnMs < KEY_POWER_ON_IGNORE_MS) {
        g_powerOnMs++;
        return;
    }

    raw = key_pressed_raw();

    if (raw == g_rawLast) {
        if (g_sameMs < 60000U) {
            g_sameMs++;
        }
    } else {
        g_sameMs = 0U;
        g_rawLast = raw;
    }

    if (g_sameMs >= KEY_DEBOUNCE_MS) {
        g_keyStable = raw;
    }

    /*
     * 刚按下
     */
    if ((g_keyLastStable == 0U) && (g_keyStable != 0U)) {
        g_pressMs = 0U;
        g_longDone = 0U;
    }

    /*
     * 按住期间
     */
    if (g_keyStable != 0U) {
        if (g_pressMs < 60000U) {
            g_pressMs++;
        }

        /*
         * 运行中按下，立即停车。
         */
        if ((RACE_IsRunning() != 0U) &&
            (g_longDone == 0U) &&
            (g_pressMs >= KEY_DEBOUNCE_MS)) {
            g_pendingAction = UI_ACTION_STOP;
            g_longDone = 1U;
        }

        /*
         * 空闲长按启动。
         */
        if ((RACE_IsRunning() == 0U) &&
            (g_longDone == 0U) &&
            (g_pressMs >= KEY_LONG_PRESS_MS)) {
            g_pendingAction = UI_ACTION_START;
            g_longDone = 1U;
        }
    }

    /*
     * 松开瞬间
     */
    if ((g_keyLastStable != 0U) && (g_keyStable == 0U)) {
        /*
         * 没触发长按，就是短按。
         * 空闲短按切换圈数。
         */
        if (g_longDone == 0U) {
            if (RACE_IsRunning() == 0U) {
                g_pendingAction = UI_ACTION_NEXT_LAP;
            } else {
                g_pendingAction = UI_ACTION_STOP;
            }
        }

        g_pressMs = 0U;
        g_longDone = 0U;
    }

    g_keyLastStable = g_keyStable;
}

/* 主循环消费按键动作。读取后立即清零，且用关中断保证不会丢失/重复消费 ISR 写入的数据。 */
void UI_Process(void)
{
    uint8_t action;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    action = g_pendingAction;
    g_pendingAction = UI_ACTION_NONE;
    if (primask == 0U) {
        __enable_irq();
    }

    if (action == UI_ACTION_START) {
        RACE_Start();
    } else if (action == UI_ACTION_STOP) {
        RACE_Stop();
    } else if (action == UI_ACTION_NEXT_LAP) {
        RACE_NextTargetLap();
    }
}
