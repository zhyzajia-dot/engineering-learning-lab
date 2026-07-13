/*
 * 文件：motor.c
 * 用途：驱动TB6612，统一管理左右电机方向、PWM限幅和安全停止。
 * 接线：左轮=MOTOR_A/PA12，方向PA24/PA25；右轮=MOTOR_B/PA13，
 *       方向PB9/PB8；STBY=PB22。
 * 接口：MOTOR_SetPWM(left, right) 始终按逻辑左右轮排列，正数前进、
 *       负数后退、0停止，允许范围最终限制在 -620～620。
 * 注意：电机安装方向改变时只调整方向系数，不要交换上层控制的左右轮含义。
 */

#include "motor.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

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

/* 写入定时器比较值。输入必须是绝对 PWM；该处再次限幅，
 * 保证无论上层传入何值都不会超过 SysConfig 所配置的安全输出范围。 */
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

/* 以安全停机状态初始化 TB6612：先 STBY 拉低、PWM 归零、方向全低，
 * 再使能驱动并启动 PWM 定时器，避免上电瞬间产生不可预期的电机动作。 */
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

/* 安全停车：两路方向全低、占空比归零，但保持 STBY 使能，便于下一次平滑起步。 */
void MOTOR_Stop(void)
{
    g_leftPwmLogical = 0;
    g_rightPwmLogical = 0;

    motor_a_set_dir(0);
    motor_b_set_dir(0);

    set_pwm_compare(GPIO_PWM_0_C0_IDX, 0);
    set_pwm_compare(GPIO_PWM_0_C1_IDX, 0);
}

/* 电机唯一控制入口。输入是逻辑左右轮 PWM（正前进、负后退）；
 * 先记录逻辑值供 UI/调试读取，再通过方向修正系数变成实际引脚输出。
 * 若电机安装方向相反，只改 MOTOR_*_DIR_SIGN，不交换上层左右轮控制。 */
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
