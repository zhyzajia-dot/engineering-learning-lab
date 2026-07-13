/*
 * 文件：encoder.c
 * 用途：累计左右轮编码器脉冲，换算轮程和10ms周期轮速。
 * 接线：左A/B=PA14/PA15，右A/B=PB12/PB13。
 * 当前实现：只对左右A相上升沿计数，B相暂未参与方向判定；逻辑方向由
 *           ENCODER_LEFT_SIGN/ENCODER_RIGHT_SIGN适配。因此倒转时本模块
 *           仍累计绝对转动脉冲，正适合原地转弯里程判定。
 * 标定：13线编码器、20:1减速比、48mm轮胎，每个脉冲约0.580mm。
 */

#include "encoder.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

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

/* 按标定常量把脉冲数换算为毫米。使用放大 1000 倍的整数常量，
 * 避免在 MCU 控制路径中引入浮点运算。 */
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

/* 每 10 ms 取一次“前进脉冲”快照，计算 mm/s 并进行 1:3 的低通滤波。
 * 中断会同时更新计数器，因此读取和发布速度均以短暂关中断保护，
 * 防止得到半更新的 32 位数据。这里的速度仅表示脉冲增加量，默认非负。 */
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

/* 配置左右 A 相上升沿 GPIO 中断并清空全部计数/测速历史。
 * B 相已在硬件上引出，但当前算法不用它判方向。 */
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

/* 清除里程、速度和滤波历史，用于开始一趟新比赛。 */
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

/* 仅重置速度反馈，不清除累计里程；适合控制器切换后重新建立测速基准。 */
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

/* 由主循环每 5 ms 调用；累计两次后执行一次 10 ms 测速采样。 */
void ENCODER_Task5ms(void)
{
    g_speedTaskDiv++;
    if (g_speedTaskDiv >= ENCODER_TASKS_PER_SAMPLE) {
        g_speedTaskDiv = 0U;
        encoder_update_speed_sample();
    }
}

/* GPIO 组中断服务：识别左右 A 相的待处理边沿、清中断标志并累加脉冲。
 * 处理尽量短小，耗时的速度换算留给主循环任务。 */
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
