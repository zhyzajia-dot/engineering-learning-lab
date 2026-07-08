#include "motor.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * motor.c - TB6612 电机驱动
 *
 * 逻辑左轮: MOTOR_A, PWM C0 (PA12), IN1 PA24, IN2 PA25
 * 逻辑右轮: MOTOR_B, PWM C1 (PA13), IN1 PB9,  IN2 PB8
 *
 * MOTOR_SetPWM() 的参数始终按 left, right 排列。正数表示小车前进，
 * 负数表示倒车。方向系数只用于适配电机安装方向。
 */

#define MOTOR_LEFT_DIR_SIGN             1
#define MOTOR_RIGHT_DIR_SIGN            1

#define MOTOR_PWM_INVERTED              0

#define MOTOR_OUTPUT_LIMIT              620

static int16_t g_leftPwmLogical = 0;
static int16_t g_rightPwmLogical = 0;

static int16_t clamp_i16(int16_t v, int16_t minV, int16_t maxV)
{
    if (v < minV) {
        return minV;
    }

    if (v > maxV) {
        return maxV;
    }

    return v;
}

static int16_t abs_i16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

static void set_pwm_compare(uint32_t ccIndex, int16_t pwmAbs)
{
    uint16_t duty;

    pwmAbs = clamp_i16(pwmAbs, 0, MOTOR_OUTPUT_LIMIT);

#if MOTOR_PWM_INVERTED
    duty = (uint16_t)(1000 - pwmAbs);
#else
    duty = (uint16_t)pwmAbs;
#endif

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, duty, ccIndex);
}

static void motor_a_set_dir(int16_t pwm)
{
    if (pwm > 0) {
        DL_GPIO_setPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    } else if (pwm < 0) {
        DL_GPIO_clearPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_setPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    }
}

static void motor_b_set_dir(int16_t pwm)
{
    if (pwm > 0) {
        DL_GPIO_setPins(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN);
    } else if (pwm < 0) {
        DL_GPIO_clearPins(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN);
        DL_GPIO_setPins(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN);
    }
}

void MOTOR_Init(void)
{
    g_leftPwmLogical = 0;
    g_rightPwmLogical = 0;

    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    set_pwm_compare(GPIO_PWM_0_C0_IDX, 0);
    set_pwm_compare(GPIO_PWM_0_C1_IDX, 0);

    motor_a_set_dir(0);
    motor_b_set_dir(0);

    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    /* SysConfig 只完成配置，此处正式启动 PWM 计数器。 */
    DL_TimerG_startCounter(PWM_0_INST);
}

void MOTOR_Stop(void)
{
    g_leftPwmLogical = 0;
    g_rightPwmLogical = 0;

    motor_a_set_dir(0);
    motor_b_set_dir(0);

    set_pwm_compare(GPIO_PWM_0_C0_IDX, 0);
    set_pwm_compare(GPIO_PWM_0_C1_IDX, 0);
}

void MOTOR_SetPWM(int16_t leftPwm, int16_t rightPwm)
{
    int16_t logicalLeft;
    int16_t logicalRight;
    int16_t physicalLeft;
    int16_t physicalRight;

    logicalLeft = clamp_i16(leftPwm, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
    logicalRight = clamp_i16(rightPwm, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);

    /* 保留逻辑值用于界面显示和诊断。 */
    g_leftPwmLogical = logicalLeft;
    g_rightPwmLogical = logicalRight;

    physicalLeft = (int16_t)(logicalLeft * MOTOR_LEFT_DIR_SIGN);
    physicalRight = (int16_t)(logicalRight * MOTOR_RIGHT_DIR_SIGN);

    motor_a_set_dir(physicalLeft);
    motor_b_set_dir(physicalRight);

    set_pwm_compare(GPIO_PWM_0_C0_IDX, abs_i16(physicalLeft));
    set_pwm_compare(GPIO_PWM_0_C1_IDX, abs_i16(physicalRight));
}

int16_t MOTOR_GetLeftPWM(void)
{
    return g_leftPwmLogical;
}

int16_t MOTOR_GetRightPWM(void)
{
    return g_rightPwmLogical;
}
