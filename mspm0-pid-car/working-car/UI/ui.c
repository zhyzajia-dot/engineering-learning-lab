#include "ui.h"
#include "race_ctrl.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * KEY 是 PB21，SysConfig 里是 PULL_UP。
 * 所以：
 *   松开 = 1
 *   按下 = 0
 */

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

static uint8_t key_pressed_raw(void)
{
    uint8_t high;

    high = (DL_GPIO_readPins(KEY_PORT, KEY_SET_PIN) != 0U) ? 1U : 0U;

    /*
     * 低电平表示按下。
     */
    return (high == 0U) ? 1U : 0U;
}

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
