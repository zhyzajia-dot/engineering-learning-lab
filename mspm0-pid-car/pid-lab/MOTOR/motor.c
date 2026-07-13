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

/* 每路电机的方向系数：1 表示正逻辑，-1 表示反装 */
#define MOTOR_LEFT_DIR_SIGN             1
#define MOTOR_RIGHT_DIR_SIGN            1

/* PWM 极性：1 表示占空比与输出反向（针对低电平有效的驱动板） */
#define MOTOR_PWM_INVERTED              0

/* PWM 输出限幅（对应 PWM 周期内允许的最大占空比） */
#define MOTOR_OUTPUT_LIMIT              620

/* 上层发来的“逻辑 PWM”值缓存，方便上位机回读 */
static int16_t g_leftPwmLogical = 0;
static int16_t g_rightPwmLogical = 0;

/* 把任意 int16 数值钳制在 [minV, maxV] 区间内 */
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

/* int16 取绝对值，避免调用标准库（也避免 -INT16_MIN 溢出） */
static int16_t abs_i16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

/* 把 PWM 绝对值写入定时器的某一通道（CC0/CC1） */
/* 将绝对 PWM 写入定时器比较寄存器；再次限幅以保证异常上层输入不会越界。 */
static void set_pwm_compare(uint32_t ccIndex, int16_t pwmAbs)
{
    uint16_t duty;

    pwmAbs = clamp_i16(pwmAbs, 0, MOTOR_OUTPUT_LIMIT);

#if MOTOR_PWM_INVERTED
    /* 反相输出：1000 - 占空比（适用于低有效驱动） */
    duty = (uint16_t)(1000 - pwmAbs);
#else
    /* 正相输出：直接使用占空比 */
    duty = (uint16_t)pwmAbs;
#endif

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, duty, ccIndex);
}

/* A 路（逻辑左轮）方向控制：IN1/IN2 决定正反转 */
static void motor_a_set_dir(int16_t pwm)
{
    if (pwm > 0) {
        /* 正转 */
        DL_GPIO_setPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    } else if (pwm < 0) {
        /* 反转 */
        DL_GPIO_clearPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_setPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    } else {
        /* 停转：两个输入都拉低，依赖 PWM=0 + 短接制动 */
        DL_GPIO_clearPins(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN);
        DL_GPIO_clearPins(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN);
    }
}

/* B 路（逻辑右轮）方向控制：IN1/IN2 决定正反转 */
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

/* 按安全顺序初始化 TB6612：先拉低 STBY、清 PWM/方向，再使能驱动和定时器，
 * 避免上电或复位时电机出现瞬时误动作。 */
void MOTOR_Init(void)
{
    /* 进入安全状态：PWM 与方向都清零 */
    g_leftPwmLogical = 0;
    g_rightPwmLogical = 0;

    /* 拉低 STBY 期间可以安全地配置寄存器 */
    DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    /* PWM 占空比清零 */
    set_pwm_compare(GPIO_PWM_0_C0_IDX, 0);
    set_pwm_compare(GPIO_PWM_0_C1_IDX, 0);

    /* 方向线清零 */
    motor_a_set_dir(0);
    motor_b_set_dir(0);

    /* 拉高 STBY，TB6612 退出待机 */
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    /* SysConfig 只完成配置，此处正式启动 PWM 计数器。 */
    DL_TimerG_startCounter(PWM_0_INST);
}

void MOTOR_Stop(void)
{
    /* 不动逻辑值以外的任何电机相关状态，仅输出 0 占空比 */
    g_leftPwmLogical = 0;
    g_rightPwmLogical = 0;

    motor_a_set_dir(0);
    motor_b_set_dir(0);

    set_pwm_compare(GPIO_PWM_0_C0_IDX, 0);
    set_pwm_compare(GPIO_PWM_0_C1_IDX, 0);
}

/* 电机控制唯一入口。参数使用逻辑左右轮（正前进、负后退），模块内部完成限幅、
 * 安装方向修正和方向/PWM 引脚输出；上层不应直接操作 TB6612 GPIO。 */
void MOTOR_SetPWM(int16_t leftPwm, int16_t rightPwm)
{
    int16_t logicalLeft;
    int16_t logicalRight;
    int16_t physicalLeft;
    int16_t physicalRight;

    /* 先钳制逻辑值，保证不超 MOTOR_OUTPUT_LIMIT */
    logicalLeft = clamp_i16(leftPwm, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
    logicalRight = clamp_i16(rightPwm, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);

    /* 保留逻辑值用于界面显示和诊断。 */
    g_leftPwmLogical = logicalLeft;
    g_rightPwmLogical = logicalRight;

    /* 通过方向系数把“逻辑正反转”翻译成“物理正反转” */
    physicalLeft = (int16_t)(logicalLeft * MOTOR_LEFT_DIR_SIGN);
    physicalRight = (int16_t)(logicalRight * MOTOR_RIGHT_DIR_SIGN);

    motor_a_set_dir(physicalLeft);
    motor_b_set_dir(physicalRight);

    /* PWM 通道只能写非负占空比，所以取绝对值 */
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
