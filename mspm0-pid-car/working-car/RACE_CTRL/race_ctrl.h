/*
 * 文件：race_ctrl.h
 * 用途：声明独立比赛控制状态、实时诊断数据和控制接口。
 * 调用：完成电机、编码器、灰度和IMU初始化后调用RACE_Init()；运行期间
 *       每5ms调用RACE_Task5ms()。
 * 单位：速度mm/s，距离mm，PWM为带符号逻辑值，lineError为灰度加权误差；
 *       转弯状态下lineError临时用于保存相对yaw（0.1°）。
 */

#ifndef RACE_CTRL_H
#define RACE_CTRL_H

#include <stdint.h>

typedef enum {
    RACE_STATE_IDLE = 0,  /* 空闲/人工停止，可重新启动 */
    RACE_STATE_RUN,       /* 正在循迹或转弯 */
    RACE_STATE_DONE       /* 已完成目标圈数 */
} RACE_State_t;

/* 运行状态快照；通过RACE_GetStatus()原子复制，供显示和诊断使用。 */
typedef struct {
    uint8_t targetLaps;       /* 目标圈数，范围1～5 */
    uint8_t doneLaps;         /* 已完成圈数 */
    uint8_t cornerCount;      /* 当前圈已完成左直角数，过渡时可到4 */
    RACE_State_t state;       /* 当前比赛状态 */

    uint8_t sensorMask;       /* 当前有效灰度位图 */
    int16_t lineError;        /* 循迹误差；转弯时为相对yaw×10 */

    int16_t leftTargetMmps;   /* 左轮目标速度，mm/s */
    int16_t rightTargetMmps;  /* 右轮目标速度，mm/s */
    int16_t leftSpeedMmps;    /* 左轮实测速度，mm/s */
    int16_t rightSpeedMmps;   /* 右轮实测速度，mm/s */
    int32_t leftEncoderCount; /* 左轮累计脉冲 */
    int32_t rightEncoderCount;/* 右轮累计脉冲 */

    int16_t leftPwm;          /* 最近一次左轮逻辑PWM */
    int16_t rightPwm;         /* 最近一次右轮逻辑PWM */
    int32_t distanceMm;       /* 左右轮平均累计距离，mm */

    int16_t avgLeftSpeedMmps;     /* 有效循迹样本的左轮平均速度 */
    int16_t avgRightSpeedMmps;    /* 有效循迹样本的右轮平均速度 */
    int16_t maxSpeedDiffMmps;     /* 观察到的最大左右速度差 */
    int16_t maxLineTurnPwm;       /* 最大循迹转向修正绝对值 */
    uint16_t lineTurnReverseCount;/* 转向修正符号反转次数 */
    uint16_t lineLostEventCount;  /* 丢线事件次数 */
    uint8_t centerStablePercent;  /* 中心稳定样本比例，0～100 */
    uint16_t sensorMaskChangeCount;/* 灰度位图变化次数 */
} RACE_Status_t;

/* 初始化比赛状态并保证电机停止，默认目标1圈。 */
void RACE_Init(void);
/* 清零本次比赛数据并开始运行。 */
void RACE_Start(void);
/* 立即停止并回到IDLE。 */
void RACE_Stop(void);
/* 5ms控制任务：循迹、速度控制、拐角、转弯和圈数状态机。 */
void RACE_Task5ms(void);
/* 仅在RACE_STATE_RUN时返回1。 */
uint8_t RACE_IsRunning(void);
/* 在临界区内复制完整状态；status为NULL时不操作。 */
void RACE_GetStatus(RACE_Status_t *status);
/* 设置目标圈数，输入自动限制在1～5。 */
void RACE_SetTargetLaps(uint8_t laps);
/* 目标圈数按1→2→3→4→5→1循环。 */
void RACE_NextTargetLap(void);

#endif
