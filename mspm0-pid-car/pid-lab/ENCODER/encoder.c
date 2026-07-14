#include "encoder.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/*
 * encoder.c - MG310 霍尔编码器
 *
 * A 相上升沿中断负责累计脉冲。ENCODER_Task5ms() 每调用 2 次计算一次
 * 轮速，因此测速周期为 10 ms。
 *
 * 13 线编码器乘 20:1 减速比得到 260 count/rev。使用 48 mm 轮胎时，
 * 每个脉冲约为 0.580 mm。
 */

#ifndef ENCODER_MM_PER_COUNT_X1000
/* 每个编码器脉冲对应的行驶距离（mm * 1000），可用宏在编译期覆盖 */
#define ENCODER_MM_PER_COUNT_X1000      580L
#endif

#ifndef ENCODER_LEFT_SIGN
/* 左轮方向系数：1 表示正装，-1 表示反装 */
#define ENCODER_LEFT_SIGN               1
#endif

#ifndef ENCODER_RIGHT_SIGN
/* 右轮方向系数：1 表示正装，-1 表示反装 */
#define ENCODER_RIGHT_SIGN              1
#endif

/* 速度采样的实际周期（每 N 次 5ms 任务才计算一次） */
#define ENCODER_SAMPLE_PERIOD_MS        10U
#define ENCODER_TASKS_PER_SAMPLE        2U

/* 速度上限：超过该值视为异常并钳制（避免数值爆掉） */
#define ENCODER_SPEED_LIMIT_MMPS        2500

/* 左右轮编码器使用的 GPIO 引脚 / 触发边沿配置 */
#define ENCODER_LEFT_PINS               ENCODER_ENC_LA_PIN
#define ENCODER_RIGHT_PINS              ENCODER_ENC_RA_PIN
#define ENCODER_LEFT_EDGE_CFG           DL_GPIO_PIN_14_EDGE_RISE
#define ENCODER_RIGHT_EDGE_CFG          DL_GPIO_PIN_12_EDGE_RISE

/* ---------- 中断里更新的原始计数（带方向） ---------- */
static volatile int32_t g_leftCount = 0;
static volatile int32_t g_rightCount = 0;

/* 中断里只累加“正转方向”的计数，用于测速与里程（不受反转影响） */
static volatile int32_t g_leftForwardCount = 0;
static volatile int32_t g_rightForwardCount = 0;

/* ---------- 任务上下文里的速度/序列号 ---------- */
static volatile int16_t g_leftSpeedMmps = 0;
static volatile int16_t g_rightSpeedMmps = 0;
static volatile uint32_t g_speedSampleSequence = 0U;

/* 上一次测速采样时的“正转计数”快照 */
static int32_t g_leftLastSampleCount = 0;
static int32_t g_rightLastSampleCount = 0;

/* 简单 IIR 滤波后的速度，用于平滑显示与控制 */
static int16_t g_leftSpeedFilter = 0;
static int16_t g_rightSpeedFilter = 0;

/* 5ms 任务调用计数器，配合 TASKS_PER_SAMPLE 形成分频 */
static uint8_t g_speedTaskDiv = 0U;

/* 关闭全局中断并返回中断前的 PRIMASK，便于临界区保护 */
static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/* 恢复 PRIMASK；仅当中断原先是开着的才重新开启 */
static void irq_restore(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 把脉冲计数换算成行驶距离（mm），使用 x1000 缩放系数做定点运算 */
static int32_t count_to_mm(int32_t count)
{
    return (count * ENCODER_MM_PER_COUNT_X1000) / 1000L;
}

/* 左轮编码器中断回调：增加带方向的总计数和正转专用计数 */
static void encoder_update_left(void)
{
    g_leftCount += ENCODER_LEFT_SIGN;
    g_leftForwardCount++;
}

/* 右轮编码器中断回调：增加带方向的总计数和正转专用计数 */
static void encoder_update_right(void)
{
    g_rightCount += ENCODER_RIGHT_SIGN;
    g_rightForwardCount++;
}

/* 真正的测速计算：定时读取一次正转计数差，换算成 mm/s 并滤波 */
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

    /* 读出最新的正转累计计数后立即开中断 */
    primask = irq_save();
    leftNowCount = g_leftForwardCount;
    rightNowCount = g_rightForwardCount;
    irq_restore(primask);

    dLeft = leftNowCount - g_leftLastSampleCount;
    dRight = rightNowCount - g_rightLastSampleCount;

    g_leftLastSampleCount = leftNowCount;
    g_rightLastSampleCount = rightNowCount;

    /* raw = dCount * (mm/count * 1000) / 1000 / period_ms
     * = dCount * mm_per_count / period_ms
     * 用 64 位中间量避免中间溢出 */
    calc = (int64_t)dLeft *
           (int64_t)ENCODER_MM_PER_COUNT_X1000 *
           1000LL;
    rawLeft = (int32_t)(calc / ((int64_t)ENCODER_SAMPLE_PERIOD_MS * 1000LL));

    calc = (int64_t)dRight *
           (int64_t)ENCODER_MM_PER_COUNT_X1000 *
           1000LL;
    rawRight = (int32_t)(calc / ((int64_t)ENCODER_SAMPLE_PERIOD_MS * 1000LL));

    /* 速度上限钳制，防止异常脉冲导致控制量跳变 */
    if (rawLeft > ENCODER_SPEED_LIMIT_MMPS) {
        rawLeft = ENCODER_SPEED_LIMIT_MMPS;
    }

    if (rawRight > ENCODER_SPEED_LIMIT_MMPS) {
        rawRight = ENCODER_SPEED_LIMIT_MMPS;
    }

    /* 简单 IIR 低通：y = (y + 3*x) / 4  */
    g_leftSpeedFilter =
        (int16_t)((((int32_t)g_leftSpeedFilter) + (rawLeft * 3)) / 4);
    g_rightSpeedFilter =
        (int16_t)((((int32_t)g_rightSpeedFilter) + (rawRight * 3)) / 4);

    /* 把滤波后的速度发到“对外可见”的 volatile 变量，
     * 同时让序列号 +1，让闭环任务知道本次有新数据 */
    primask = irq_save();
    g_leftSpeedMmps = g_leftSpeedFilter;
    g_rightSpeedMmps = g_rightSpeedFilter;
    g_speedSampleSequence++;
    irq_restore(primask);
}

void ENCODER_Init(void)
{
    /* 配置左右编码器 A 相为上升沿触发 */
    DL_GPIO_setLowerPinsPolarity(GPIOA, ENCODER_LEFT_EDGE_CFG);
    DL_GPIO_setLowerPinsPolarity(GPIOB, ENCODER_RIGHT_EDGE_CFG);

    /* 先清掉可能存在的旧中断标志 */
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_LEFT_PINS);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_RIGHT_PINS);

    /* 使能 GPIO 中断并打开 NVIC */
    DL_GPIO_enableInterrupt(GPIOA, ENCODER_LEFT_PINS);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER_RIGHT_PINS);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
#ifdef GPIOB_INT_IRQn
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
#endif

    /* 计数 / 速度清零 */
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

    /* 只清零速度相关变量，保留累计脉冲，便于闭环模式切换时不丢里程 */
    g_leftSpeedMmps = 0;
    g_rightSpeedMmps = 0;
    g_speedSampleSequence = 0U;

    g_leftSpeedFilter = 0;
    g_rightSpeedFilter = 0;
    g_speedTaskDiv = 0U;

    /* 把“上次采样点”也设到当前位置，避免下一次算出大速度 */
    g_leftLastSampleCount = g_leftForwardCount;
    g_rightLastSampleCount = g_rightForwardCount;

    irq_restore(primask);
}

void ENCODER_Task5ms(void)
{
    /* 5 ms 任务 + 2:1 分频 -> 实际 10 ms 一次测速 */
    g_speedTaskDiv++;
    if (g_speedTaskDiv >= ENCODER_TASKS_PER_SAMPLE) {
        g_speedTaskDiv = 0U;
        encoder_update_speed_sample();
    }
}

void GROUP1_IRQHandler(void)
{
    /* 一次 GROUP1 中断里同时处理 GPIOA / GPIOB 的编码器事件 */
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
    /* 里程只算“正转”，避免倒车脉冲把里程减回去 */
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

int32_t ENCODER_GetMmPerCountX1000(void)
{
    return ENCODER_MM_PER_COUNT_X1000;
}

uint16_t ENCODER_GetSamplePeriodMs(void)
{
    return ENCODER_SAMPLE_PERIOD_MS;
}
