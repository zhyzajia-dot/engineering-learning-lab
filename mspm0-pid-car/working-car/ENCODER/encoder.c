#include "encoder.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * encoder.c - MG310 霍尔编码器
 *
 * A 相上升沿中断负责累计脉冲。ENCODER_Task5ms() 每调用 4 次计算一次
 * 轮速，因此测速周期为 20 ms。
 *
 * 13 线编码器乘 20:1 减速比得到 260 count/rev。使用 48 mm 轮胎时，
 * 每个脉冲约为 0.580 mm。
 */

#ifndef ENCODER_MM_PER_COUNT_X1000
#define ENCODER_MM_PER_COUNT_X1000      580L
#endif

#ifndef ENCODER_LEFT_SIGN
#define ENCODER_LEFT_SIGN               1
#endif

#ifndef ENCODER_RIGHT_SIGN
#define ENCODER_RIGHT_SIGN              1
#endif

#define ENCODER_SAMPLE_PERIOD_MS        10U
#define ENCODER_TASKS_PER_SAMPLE        2U
#define ENCODER_SPEED_LIMIT_MMPS        2500

#define ENCODER_LEFT_PINS               ENCODER_ENC_LA_PIN
#define ENCODER_RIGHT_PINS              ENCODER_ENC_RA_PIN
#define ENCODER_LEFT_EDGE_CFG           DL_GPIO_PIN_14_EDGE_RISE
#define ENCODER_RIGHT_EDGE_CFG          DL_GPIO_PIN_12_EDGE_RISE

static volatile int32_t g_leftCount = 0;
static volatile int32_t g_rightCount = 0;

static volatile int32_t g_leftForwardCount = 0;
static volatile int32_t g_rightForwardCount = 0;

static volatile int16_t g_leftSpeedMmps = 0;
static volatile int16_t g_rightSpeedMmps = 0;
static volatile uint32_t g_speedSampleSequence = 0U;

static int32_t g_leftLastSampleCount = 0;
static int32_t g_rightLastSampleCount = 0;

static int16_t g_leftSpeedFilter = 0;
static int16_t g_rightSpeedFilter = 0;
static uint8_t g_speedTaskDiv = 0U;

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static int32_t count_to_mm(int32_t count)
{
    return (count * ENCODER_MM_PER_COUNT_X1000) / 1000L;
}

static void encoder_update_left(void)
{
    g_leftCount += ENCODER_LEFT_SIGN;
    g_leftForwardCount++;
}

static void encoder_update_right(void)
{
    g_rightCount += ENCODER_RIGHT_SIGN;
    g_rightForwardCount++;
}

static void encoder_update_speed_sample(void)
{
    int32_t leftNowCount;
    int32_t rightNowCount;
    int32_t dLeft;
    int32_t dRight;
    int32_t rawLeft;
    int32_t rawRight;
    int64_t calc;
    uint32_t primask;

    primask = irq_save();
    leftNowCount = g_leftForwardCount;
    rightNowCount = g_rightForwardCount;
    irq_restore(primask);

    dLeft = leftNowCount - g_leftLastSampleCount;
    dRight = rightNowCount - g_rightLastSampleCount;

    g_leftLastSampleCount = leftNowCount;
    g_rightLastSampleCount = rightNowCount;

    calc = (int64_t)dLeft *
           (int64_t)ENCODER_MM_PER_COUNT_X1000 *
           1000LL;
    rawLeft = (int32_t)(calc / ((int64_t)ENCODER_SAMPLE_PERIOD_MS * 1000LL));

    calc = (int64_t)dRight *
           (int64_t)ENCODER_MM_PER_COUNT_X1000 *
           1000LL;
    rawRight = (int32_t)(calc / ((int64_t)ENCODER_SAMPLE_PERIOD_MS * 1000LL));

    if (rawLeft > ENCODER_SPEED_LIMIT_MMPS) {
        rawLeft = ENCODER_SPEED_LIMIT_MMPS;
    }

    if (rawRight > ENCODER_SPEED_LIMIT_MMPS) {
        rawRight = ENCODER_SPEED_LIMIT_MMPS;
    }

    g_leftSpeedFilter =
        (int16_t)((((int32_t)g_leftSpeedFilter) + (rawLeft * 3)) / 4);
    g_rightSpeedFilter =
        (int16_t)((((int32_t)g_rightSpeedFilter) + (rawRight * 3)) / 4);

    primask = irq_save();
    g_leftSpeedMmps = g_leftSpeedFilter;
    g_rightSpeedMmps = g_rightSpeedFilter;
    g_speedSampleSequence++;
    irq_restore(primask);
}

void ENCODER_Init(void)
{
    DL_GPIO_setLowerPinsPolarity(GPIOA, ENCODER_LEFT_EDGE_CFG);
    DL_GPIO_setLowerPinsPolarity(GPIOB, ENCODER_RIGHT_EDGE_CFG);

    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_LEFT_PINS);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_RIGHT_PINS);

    DL_GPIO_enableInterrupt(GPIOA, ENCODER_LEFT_PINS);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER_RIGHT_PINS);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
#ifdef GPIOB_INT_IRQn
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
#endif

    ENCODER_Reset();
}

void ENCODER_Reset(void)
{
    uint32_t primask;

    primask = irq_save();

    g_leftCount = 0;
    g_rightCount = 0;

    g_leftForwardCount = 0;
    g_rightForwardCount = 0;

    g_leftSpeedMmps = 0;
    g_rightSpeedMmps = 0;
    g_speedSampleSequence = 0U;

    g_leftSpeedFilter = 0;
    g_rightSpeedFilter = 0;
    g_speedTaskDiv = 0U;

    g_leftLastSampleCount = 0;
    g_rightLastSampleCount = 0;

    irq_restore(primask);
}

void ENCODER_ResetSpeedFeedback(void)
{
    uint32_t primask;

    primask = irq_save();

    g_leftSpeedMmps = 0;
    g_rightSpeedMmps = 0;
    g_speedSampleSequence = 0U;

    g_leftSpeedFilter = 0;
    g_rightSpeedFilter = 0;
    g_speedTaskDiv = 0U;

    g_leftLastSampleCount = g_leftForwardCount;
    g_rightLastSampleCount = g_rightForwardCount;

    irq_restore(primask);
}

void ENCODER_Task5ms(void)
{
    g_speedTaskDiv++;
    if (g_speedTaskDiv >= ENCODER_TASKS_PER_SAMPLE) {
        g_speedTaskDiv = 0U;
        encoder_update_speed_sample();
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t statusA = DL_GPIO_getEnabledInterruptStatus(GPIOA, ENCODER_LEFT_PINS);
    uint32_t statusB = DL_GPIO_getEnabledInterruptStatus(GPIOB, ENCODER_RIGHT_PINS);

    if (statusA != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOA, statusA);
        encoder_update_left();
    }

    if (statusB != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOB, statusB);
        encoder_update_right();
    }
}

int16_t ENCODER_GetLeftSpeed(void)
{
    int16_t v;
    uint32_t primask;

    primask = irq_save();
    v = g_leftSpeedMmps;
    irq_restore(primask);

    return v;
}

int16_t ENCODER_GetRightSpeed(void)
{
    int16_t v;
    uint32_t primask;

    primask = irq_save();
    v = g_rightSpeedMmps;
    irq_restore(primask);

    return v;
}

uint32_t ENCODER_GetSpeedSampleSequence(void)
{
    uint32_t v;
    uint32_t primask;

    primask = irq_save();
    v = g_speedSampleSequence;
    irq_restore(primask);

    return v;
}

int32_t ENCODER_GetLeftCount(void)
{
    int32_t v;
    uint32_t primask;

    primask = irq_save();
    v = g_leftCount;
    irq_restore(primask);

    return v;
}

int32_t ENCODER_GetRightCount(void)
{
    int32_t v;
    uint32_t primask;

    primask = irq_save();
    v = g_rightCount;
    irq_restore(primask);

    return v;
}

int32_t ENCODER_GetLeftDistanceMm(void)
{
    int32_t v;
    uint32_t primask;

    primask = irq_save();
    v = count_to_mm(g_leftForwardCount);
    irq_restore(primask);

    return v;
}

int32_t ENCODER_GetRightDistanceMm(void)
{
    int32_t v;
    uint32_t primask;

    primask = irq_save();
    v = count_to_mm(g_rightForwardCount);
    irq_restore(primask);

    return v;
}
