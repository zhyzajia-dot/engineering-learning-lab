# 睡眠分析算法 — 嵌入式 C 语言数据流说明

## 目录

1. [概述](#1-概述)
2. [数据结构](#2-数据结构)
3. [数据输入接口](#3-数据输入接口)
4. [归一化层](#4-归一化层)
5. [评分函数层](#5-评分函数层)
6. [状态机层](#6-状态机层)
7. [统计与评分层](#7-统计与评分层)
8. [主入口 update() 函数](#8-主入口-update-函数)
9. [参数总表](#9-参数总表)
10. [调用流程](#10-调用流程)

---

## 1. 概述

**运行平台**：ESP32-S3（Arduino 框架）

**核心文件**：
- `include/sleep_analyzer.h` — 数据结构、枚举、类声明
- `src/sleep_analyzer.cpp` — 算法实现

**调用频率**：每秒 1 次（由 `tasks_manager.cpp` 定时触发）

**核心设计原则**：
- **非对称状态机**：入睡困难、觉醒容易（人体睡眠自然特性）
- **双时间尺度体动 EMA**：慢速 EMA（~5分钟）用于入睡/深睡判断，快速 EMA（~3秒）用于觉醒检测
- **相对稳定性判定**：不依赖绝对阈值，依据信号相对变化评估睡眠深度

---

## 2. 数据结构

### 2.1 睡眠状态枚举

```c
typedef enum {
    SLEEP_NO_PERSON = 0,  // 无人
    SLEEP_IN_BED,         // 在床
    SLEEP_AWAKE,          // 清醒
    SLEEP_LIGHT_SLEEP,    // 浅睡
    SLEEP_DEEP_SLEEP,     // 深睡
    SLEEP_REM_SLEEP,      // 快速眼动
    SLEEP_OUT_OF_BED,     // 离床
    SLEEP_GETTING_UP,     // 起床
    SLEEP_SESSION_END     // 会话结束
} SleepState;

static const char* SLEEP_STATE_NAMES[] = {
    "无人", "在床", "清醒", "浅睡", "深睡", "REM", "离床", "起床", "会话结束"
};
```

### 2.2 输入数据结构

```c
/* 人员存在检测 */
typedef struct {
    bool isPresent;       // 是否有人在
    float distance;       // 人体距离（cm）
    float confidence;     // 置信度 0~1
    float motionEnergy;   // 运动能量 0~1
} PresenceData;

/* 心率数据（由 data_processor.cpp 平滑后传入） */
typedef struct {
    float bpmSmoothed;    // EMA 平滑后的心率（bpm）
    bool isValid;         // 数据是否有效
} HeartRateData;

/* 呼吸率数据 */
typedef struct {
    float rateSmoothed;   // EMA 平滑后的呼吸率（bpm）
    float variability;    // 呼吸变异性
    float regularity;     // 呼吸规律性 0~1
    bool isValid;
} RespirationData;

/* 心率变异性 */
typedef struct {
    float rmssd;          // RMSSD 值（ms）
    bool isValid;
} HRVEstimate;

/* 体动数据 */
typedef struct {
    float movement;       // 体动幅度（0~100）
    bool isValid;
} BodyMovementData;
```

### 2.3 输出数据结构

```c
/* 睡眠周期追踪 */
typedef struct {
    int cycleCount;              // 完成的睡眠周期数
    unsigned long cycleStartTime; // 当前周期开始时间（millis）
    bool inDeepPhase;             // 当前周期是否已完成深睡阶段
    bool inRemPhase;              // 当前周期是否已完成REM阶段
    unsigned long lastDeepEndTime;
    unsigned long lastRemEndTime;
} SleepCycle;

/* 睡眠统计 */
typedef struct {
    unsigned long totalSleepTime;   // 总睡眠时长（ms）
    unsigned long deepSleepTime;    // 深睡时长（ms）
    unsigned long lightSleepTime;   // 浅睡时长（ms）
    unsigned long remSleepTime;     // 快速眼动时长（ms）
    unsigned long awakeTime;        // 清醒时长（ms）
    unsigned long outOfBedTime;     // 离床时长（ms）
    unsigned long sleepLatency;     // 入睡耗时（s）
    int wakeCount;                  // 醒来次数
    int sleepCycles;                // 睡眠周期数
    unsigned long sessionStartTime; // 会话开始时间
    unsigned long sleepStartTime;   // 入睡时间
} SleepStatistics;

/* 睡眠评分 */
typedef struct {
    float durationScore;      // 时长评分 /18
    float deepScore;          // 深睡评分 /14
    float continuityScore;    // 连续性评分 /11
    float physiologyScore;    // 生理质量评分 /7
    float latencyScore;       // 入睡速度评分 /8
    float efficiencyScore;    // 睡眠效率评分 /14
    float cycleScore;         // 周期评分 /28
    float totalScore;         // 总分 /100
} SleepScore;
```

---

## 3. 数据输入接口

### 3.1 传感器数据来源

ESP32 端数据来自雷达串口帧（`radar_manager.cpp` 解析），经 `data_processor.cpp` 平滑后，由 `tasks_manager.cpp` 每秒调用一次 `SleepAnalyzer::update()`：

```c
void SleepAnalyzer::update(
    const HeartRateData& hrData,        // 心率（已平滑）
    const RespirationData& rrData,       // 呼吸率（已平滑）
    const HRVEstimate& hrvData,          // HRV
    const BodyMovementData& movementData, // 体动
    bool bedStatus                        // 床位状态（0=离床, 1=在床）
);
```

### 3.2 存在检测

`evaluatePresence()` 根据传感器原始数据判定是否有人：

```c
PresenceData SleepAnalyzer::evaluatePresence() {
    PresenceData p;
    memset(&p, 0, sizeof(PresenceData));

    // 心跳或呼吸有效 → 有人，置信度 0.9
    if (sensorData.heart_valid || sensorData.breath_valid) {
        p.isPresent = true;
        p.confidence = 0.9f;
    }

    // presence 标志为 1 → 有人，置信度 0.7
    if (sensorData.presence == 1) {
        p.isPresent = true;
        p.confidence = max(p.confidence, 0.7f);
    }

    // 距离在 20~100cm → 有人
    if (sensorData.distance > 0) {
        p.distance = sensorData.distance;
        if (p.distance > 20 && p.distance < 100) {
            p.isPresent = true;
        }
    }

    // 运动能量 = body_movement / 100
    p.motionEnergy = sensorData.body_movement / 100.0f;
    return p;
}
```

---

## 4. 归一化层

将所有生理参数映射到 `[0, 1]` 或 `[-1, 1]` 区间。

### 4.1 心率归一化

```c
float SleepAnalyzer::normalizeHR(float hr) {
    if (!baselineCalibrated) return 0.0f;
    float norm = (hr - baselineHR) / 20.0f;     // 公式 (A)
    return constrain_value(norm, -1.0f, 1.0f);
}
```

**公式 (A)**：`HR_norm = constrain((HR - baselineHR) / 20.0, -1.0, 1.0)`

> HR 等于基线时 → 0；每偏离基线 20 bpm → ±1

### 4.2 呼吸率归一化

```c
float SleepAnalyzer::normalizeRR(float rr) {
    if (!baselineCalibrated) return 0.0f;
    float norm = (rr - baselineRR) / 4.0f;      // 公式 (B)
    return constrain_value(norm, -1.0f, 1.0f);
}
```

**公式 (B)**：`RR_norm = constrain((RR - baselineRR) / 4.0, -1.0, 1.0)`

### 4.3 心率变异性归一化

```c
float SleepAnalyzer::normalizeHRV(float hrv) {
    float norm = hrv / 50.0f;                   // 公式 (C)
    return constrain_value(norm, 0.0f, 1.0f);
}
```

**公式 (C)**：`HRV_norm = constrain(HRV / 50.0, 0.0, 1.0)`

### 4.4 体动归一化（非线性开平方映射）

```c
float SleepAnalyzer::normalizeMovement(float movement) {
    return constrain_value(
        powf(movement / 100.0f, 0.5f),          // 公式 (D)
        0.0f, 1.0f
    );
}
```

**公式 (D)**：`Movement_norm = constrain((Movement / 100.0)^0.5, 0.0, 1.0)`

**效果对照表**：

| Movement 原始值 | 实际含义 | 线性 (mov/100) | 开平方 √(mov/100) | 效果 |
|:---:|:---|:---:|:---:|:---|
| 0~3 | 非常静止 | 0~0.03 | 0~0.17 | 小动作被放大 |
| 3~10 | 微小动作 | 0.03~0.10 | 0.17~0.32 | 微动可感知 |
| 10~20 | 正常翻身 | 0.10~0.20 | 0.32~0.45 | 翻身明显贡献 |
| 20~40 | 明显活动 | 0.20~0.40 | 0.45~0.63 | 接近觉醒 |
| 40~60 | 接近觉醒 | 0.40~0.60 | 0.63~0.77 | 大动作 |
| 60~80 | 清醒大动作 | 0.60~0.80 | 0.77~0.89 | 保留大动作优势 |

### 4.5 双时间尺度体动 EMA

```c
// 在 update() 中每帧调用：
float rawMove = movementData.isValid ? movementData.movement : 0;

// 慢速 EMA（α=0.01，~5分钟）：用于入睡判断 / 深睡判断
moveSleepEMA = emaSmooth(rawMove, moveSleepEMA, EMA_SLEEP_ALPHA);   // 式 (E1)

// 快速 EMA（α=0.3，~3秒）：用于觉醒检测
moveAwakeEMA = emaSmooth(rawMove, moveAwakeEMA, EMA_AWAKE_ALPHA);    // 式 (E2)
```

**EMA 平滑公式**：

```c
float SleepAnalyzer::emaSmooth(float input, float last, float alpha) {
    return alpha * input + (1.0f - alpha) * last;   // 公式 (F)
}
```

**公式 (F)**：`EMA = α × Input + (1 − α) × Prev`

### 4.6 心率/呼吸率基线校准

在 AWAKE 或 IN_BED 状态下，每帧调用 `calibrateBaseline()`：

**阶段一：初始采集（30 个样本）**：

```
baselineHRSum += hr
baselineRRSum += rr
sampleCount++

当 sampleCount ≥ 30:
    baselineHR = baselineHRSum / 30
    baselineRR = baselineRRSum / 30
    baselineCalibrated = true
```

**阶段二：滚动更新**：

```c
// 稳定性检查：每 5 个样本计算一次 HR/RR 平均波动
if (avgHrDiff > 5.0f || avgRrDiff > 2.0f) {
    // 波动过大，跳过此轮更新
    return;
}
// 慢速 EMA 更新基线
baselineHR = (1 - 0.01) * baselineHR + 0.01 * hr   // β=0.01
baselineRR = (1 - 0.01) * baselineRR + 0.01 * rr
```

> 检测期间体动归一化值 > 0.2 时跳过本次校准。

---

## 5. 评分函数层

### 5.1 基础工具函数

**Sigmoid 激活**：

```c
float SleepAnalyzer::sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));    // 公式 (G)
}
```

**公式 (G)**：`Sigmoid(x) = 1 / (1 + e^(-x))`

**Best Score 判定**：

```c
bool SleepAnalyzer::isBestScore(float score, float s2, float s3, float s4, float margin) {
    return (score > s2 + margin && score > s3 + margin && score > s4 + margin);
}
```

> 某评分需比其余三个均高出 `margin`（0.10）才算"胜出"。

**EMA 平滑（所有评分共用 α=0.2）**：

```c
currentDeepScore  = emaSmooth(rawDeep,  currentDeepScore,  0.2f);
currentLightScore = emaSmooth(rawLight, currentLightScore, 0.2f);
currentAwakeScore = emaSmooth(rawAwake, currentAwakeScore, 0.2f);
currentRemScore   = emaSmooth(rawRem,   currentRemScore,   0.2f);
```

---

### 5.2 入睡倾向评分（Sleepiness Score）

**用途**：IN_BED → LIGHT_SLEEP、AWAKE → LIGHT_SLEEP

**运算公式**：

```c
float SleepAnalyzer::calculateSleepinessScore(
    const HeartRateData& hrData,
    const RespirationData& rrData,
    const HRVEstimate& hrvData,
    const BodyMovementData& movementData) {

    float hrSleepFactor = 0.5f, hrvNorm = 0, rrStable = 0, moveNorm = 0;

    if (hrData.isValid) {
        float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrSleepFactor = (1.0f - hrNorm) * 0.5f;               // (H1)
    }
    if (hrvData.isValid) {
        hrvNorm = normalizeHRV(hrvData.rmssd);                // (H2)
    }
    if (rrData.isValid) {
        float rrDev = fabs(rrData.rateSmoothed - baselineRR) / 5.0f;
        rrStable = 1.0f - constrain_value(rrDev, 0.0f, 1.0f); // (H3)
    }
    if (movementData.isValid) {
        // 慢速 EMA：反映 ~5 分钟长期平均，仅长期静止才高困倦
        moveNorm = normalizeMovement(moveSleepEMA);            // (H4)
    }

    float x = 3.0f * (hrSleepFactor - 0.5f)
            + 2.5f * (hrvNorm - 0.5f)
            + 2.0f * (rrStable - 0.5f)
            + 3.0f * (0.5f - moveNorm);                       // (H5) 慢速体动

    return sigmoid(x);                                        // (H6)
}
```

**公式 (H)**：

```
hr_factor  = (1 − HR_norm) × 0.5                                              ...(H1)
hrv_n      = HRV / 50                                                          ...(H2)
rr_stable  = 1 − min(|RR − baselineRR| / 5.0, 1.0)                            ...(H3)
move_n     = √(moveSleepEMA / 100)                                            ...(H4)

x = 3.0 × (hr_factor − 0.5)
  + 2.5 × (hrv_n − 0.5)
  + 2.0 × (rr_stable − 0.5)
  + 3.0 × (0.5 − move_n)                                                       ...(H5)

Sleepiness = Sigmoid(x)                                                        ...(H6)
```

> HR 权重 3.0 = 体动权重 3.0 > HRV 2.5 > RR稳定性 2.0

---

### 5.3 深睡评分（Deep Sleep Score）

**用途**：LIGHT_SLEEP → DEEP_SLEEP

**运算公式**：

```c
float SleepAnalyzer::calculateDeepSleepScore(
    const HeartRateData& hrData,
    const RespirationData& rrData,
    const HRVEstimate& hrvData,
    const BodyMovementData& movementData) {

    // 硬体动上限：大动作直接否决
    if (movementData.isValid && movementData.movement > DEEP_SLEEP_HARD_MOVEMENT_LIMIT) {
        return 0.0f;                                         // (I0)
    }

    float hrSleepFactor = 0.5f, hrvNorm = 0.5f, moveNorm = 0;

    if (hrData.isValid) {
        float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrSleepFactor = (1.0f - hrNorm) * 0.5f;              // (I1)
    }
    if (hrvData.isValid) {
        hrvNorm = normalizeHRV(hrvData.rmssd);               // (I2)
    }
    if (movementData.isValid) {
        // 慢速 EMA：深睡要求长期低体动，短暂活动不误伤
        moveNorm = normalizeMovement(moveSleepEMA);           // (I3)
    }

    float x = 5.0f * (hrSleepFactor - 0.5f)                  // HR 稳定性为主
            + 1.5f * (0.5f - hrvNorm)                        // HRV 权重降
            + 3.0f * (0.5f - moveNorm);                      // (I4) 慢速体动

    return sigmoid(x);                                       // (I5)
}
```

**公式 (I)**：

```
if Movement > 25: DeepScore = 0                                                ...(I0)

hr_factor  = (1 − HR_norm) × 0.5                                              ...(I1)
hrv_n      = HRV / 50                                                          ...(I2)
move_n     = √(moveSleepEMA / 100)                                            ...(I3)

x = 5.0 × (hr_factor − 0.5)      // HR 稳定性权重最高
  + 1.5 × (0.5 − hrv_n)          // HRV 权重降低
  + 3.0 × (0.5 − move_n)         // 长期体动低水平                        ...(I4)

DeepScore = Sigmoid(x)                                                         ...(I5)
```

> **相对稳定性判定**：不要求极低绝对体动/HR，而是看 HR 低于近期平均 + 体动稳定低水平。

---

### 5.4 浅睡评分（Light Sleep Score）

**用途**：作为"中间稳定态"基准，各项指标追求中值。

**运算公式**：

```c
float SleepAnalyzer::calculateLightSleepScore(...) {
    float hrMid = 0, hrvMid = 0, moveMid = 0, rrStable = 0;

    if (hrData.isValid) {
        float hrNorm = normalizeHR(hrData.bpmSmoothed);
        float hrSleepFactor = (1.0f - hrNorm) * 0.5f;
        hrMid = 1.0f - fabs(hrSleepFactor - 0.5f) * 2.0f;     // (J1)
    }
    if (hrvData.isValid) {
        float hrvNorm = normalizeHRV(hrvData.rmssd);
        hrvMid = 1.0f - fabs(hrvNorm - 0.5f) * 2.0f;          // (J2)
    }
    if (movementData.isValid) {
        float moveNorm = normalizeMovement(movementData.movement);
        if (moveNorm >= 0.1f && moveNorm <= 0.4f) {
            moveMid = 1.0f - fabs(moveNorm - 0.25f) * 4.0f;   // (J3)
        }
    }
    if (rrData.isValid) {
        float rrDev = fabs(rrData.rateSmoothed - baselineRR) / 5.0f;
        rrStable = 1.0f - constrain_value(rrDev, 0.0f, 1.0f);  // (J4)
    }

    float light = 0.3f * hrMid + 0.3f * hrvMid
                + 0.2f * moveMid + 0.2f * rrStable;            // (J5)

    return constrain_value(light, 0.0f, 1.0f);                  // (J6)
}
```

**公式 (J)**：

```
hr_mid    = 1 − |hr_factor − 0.5| × 2                    // HR 接近中线得分高   ...(J1)
hrv_mid   = 1 − |HRV_norm − 0.5| × 2                     // HRV 接近中线得分高  ...(J2)
move_mid  = 1 − |move_n − 0.25| × 4 (if 0.1≤move_n≤0.4) // 中等体动得分高      ...(J3)
rr_stable = 1 − min(|RR − baseline| / 5, 1)              // RR 稳定得分高       ...(J4)

Light = 0.3 × hr_mid + 0.3 × hrv_mid + 0.2 × move_mid + 0.2 × rr_stable         ...(J5)
```

---

### 5.5 觉醒评分（Awake Score）

**用途**：任意睡眠状态 → AWAKE。对瞬时体动突峰极度敏感。

**运算公式**：

```c
float SleepAnalyzer::calculateAwakeScore(...) {
    float moveNorm = 0, hrAwakeFactor = 0.5f, rrVar = 0;

    if (movementData.isValid) {
        // 快速 EMA：~3 秒响应瞬时体动突峰
        moveNorm = normalizeMovement(moveAwakeEMA);            // (K1)
    }
    if (hrData.isValid) {
        float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrAwakeFactor = (hrNorm + 1.0f) * 0.5f;               // (K2)
    }
    if (rrData.isValid) {
        float rrDev = fabs(rrData.rateSmoothed - baselineRR) / 5.0f;
        rrVar = constrain_value(rrDev, 0.0f, 1.0f);            // (K3)
    }

    float x = 5.0f * (moveNorm - 0.20f)      // 快速体动为主
            + 1.5f * (hrAwakeFactor - 0.5f)
            + 1.0f * (rrVar - 0.3f);                          // (K4)

    return sigmoid(x);                                         // (K5)
}
```

**公式 (K)**：

```
move_n      = √(moveAwakeEMA / 100)                                             ...(K1)
hr_awake    = (HR_norm + 1.0) × 0.5                                             ...(K2)
rr_var      = min(|RR − baseline| / 5.0, 1.0)                                  ...(K3)

x = 5.0 × (move_n − 0.20)    // 快速体动为主（降低偏置，更激进）
  + 1.5 × (hr_awake − 0.5)
  + 1.0 × (rr_var − 0.3)                                                         ...(K4)

AwakeScore = Sigmoid(x)                                                           ...(K5)
```

---

### 5.6 快速眼动睡眠评分（REM Score）

**用途**：LIGHT_SLEEP / DEEP_SLEEP → REM_SLEEP

**REM 特征**：HR 较高、HRV 低、RR 不稳定但体动极低。

**运算公式**：

```c
float SleepAnalyzer::calculateRemScore(...) {
    float hrIrregular = 0, hrvHigh = 0, moveLow = 0, rrIrregular = 0;

    if (hrData.isValid) {
        float hrNorm = normalizeHR(hrData.bpmSmoothed);
        float hrSleepFactor = (1.0f - hrNorm) * 0.5f;
        hrIrregular = fabs(hrSleepFactor - 0.5f) * 2.0f;     // (L1)
    }
    if (hrvData.isValid) {
        hrvHigh = normalizeHRV(hrvData.rmssd);                // (L2)
    }
    if (movementData.isValid) {
        moveLow = 1.0f - normalizeMovement(movementData.movement); // (L3)
    }
    if (rrData.isValid) {
        float rrDev = fabs(rrData.rateSmoothed - baselineRR) / 3.0f;
        rrIrregular = constrain_value(rrDev, 0.0f, 1.0f);     // (L4)
    }

    float rem = 0.25f * hrIrregular + 0.25f * hrvHigh
              + 0.25f * moveLow + 0.25f * rrIrregular;        // (L5)

    return constrain_value(rem, 0.0f, 1.0f);                   // (L6)
}
```

**公式 (L)**：

```
hr_irregular = |(1−HR_norm)×0.5 − 0.5| × 2   // HR 偏离中线 → 高 REM 分    ...(L1)
hrv_high     = HRV / 50                                                     ...(L2)
move_low     = 1 − √(movement / 100)         // 体动低 → 高 REM 分           ...(L3)
rr_irregular = min(|RR − baseline| / 3.0, 1) // RR 偏离基线 → 高 REM 分     ...(L4)

REM = 0.25 × hr_irregular + 0.25 × hrv_high
    + 0.25 × move_low     + 0.25 × rr_irregular                              ...(L5)
```

---

### 5.7 HR 持续升高觉醒检测

独立于评分函数的觉醒补充检测，检测 HR 相对于睡眠期基线的持续升高。

```c
bool SleepAnalyzer::checkHRAwakening(float hr, bool hrValid) {
    static const int   HR_AWAKE_LOOKBACK   = 60;    // 回溯窗口（秒）
    static const float HR_ELEVATED_RATIO   = 1.10f; // HR 超过基线 10%
    static const int   HR_ELEVATED_MIN_SEC = 5;     // 升高至少持续 5 秒

    if (!hrValid || hr <= 0) return false;

    // 仅在睡眠状态中追踪基线
    bool inSleepState = (currentState == SLEEP_LIGHT_SLEEP ||
                         currentState == SLEEP_DEEP_SLEEP ||
                         currentState == SLEEP_REM_SLEEP);

    if (inSleepState) {
        if (sleepHRBaselineCount == 0) {
            sleepHRBaseline = hr;                                     // (M1)
        } else {
            sleepHRBaseline = sleepHRBaseline * 0.95f + hr * 0.05f;   // (M2)
        }
        sleepHRBaselineCount++;
        if (sleepHRBaselineCount > HR_AWAKE_LOOKBACK) {
            sleepHRBaselineCount = HR_AWAKE_LOOKBACK;
        }
    }

    if (sleepHRBaseline > 0 && hr > sleepHRBaseline * HR_ELEVATED_RATIO) {
        hrAwakeTimer++;                                                // (M3)
    } else {
        hrAwakeTimer = 0;
    }

    if (hrAwakeTimer >= HR_ELEVATED_MIN_SEC) {
        hrAwakeTimer = 0;
        return true;                                                   // (M4)
    }
    return false;
}
```

**公式 (M)**：

```
睡眠状态中：sleepHRBaseline = 0.95 × prev + 0.05 × hr    // EMA 追踪         ...(M2)
if HR > sleepHRBaseline × 1.10:  hrAwakeTimer++           // 升高 10% 以上    ...(M3)
if hrAwakeTimer ≥ 5 秒:         返回 true                  // 持续 5 秒即触发  ...(M4)
```

---

## 6. 状态机层

### 6.1 状态切换总览

```
NO_PERSON(0) → IN_BED(1) → AWAKE(2) → LIGHT_SLEEP(3) → DEEP_SLEEP(4)
                                                          ↕
                                                    REM_SLEEP(5)
                                                          ↓
                                              OUT_OF_BED(6) → GETTING_UP(7)
                                                          ↓
                                                   SESSION_END(8)
```

### 6.2 状态切换确认机制

```c
bool SleepAnalyzer::tryTransitionTo(SleepState target, unsigned long confirmMs) {
    if (pendingState != target) {
        pendingState = target;
        pendingStateTime = millis();           // 记录首次出现时间
        return false;
    }
    if (millis() - pendingStateTime >= confirmMs) {
        currentState = target;                 // 确认时间达到，切换
        stateEnterTime = millis();
        pendingState = target;
        return true;
    }
    return false;                              // 等待确认时间
}
```

> 所有评分驱动的状态切换均需经过 `tryTransitionTo`，确保状态稳定性。

### 6.3 最小驻留时间

```c
static const int MIN_STATE_DWELL_MS = 10000;  // 10 秒

// 在 updateState() 入口：
if (currentState != SLEEP_NO_PERSON && currentState != SLEEP_SESSION_END &&
    (now - stateEnterTime) < MIN_STATE_DWELL_MS) {
    return;  // 不处理，防止快速抖动
}
```

### 6.4 bedStatus 优先机制

所有有人员状态（IN_BED / AWAKE / LIGHT_SLEEP / DEEP_SLEEP / REM_SLEEP）在每个 tick 的第一步检查：

```c
if (!bedStatus) {              // bedStatus == 0（离床）
    currentState = SLEEP_OUT_OF_BED;   // 无条件切换到离床
    stateEnterTime = now;
    pendingState = currentState;
    noPersonTimer = 0;
    break;                     // 不再执行后续评分逻辑
}
```

### 6.5 各状态切换规则

#### NO_PERSON → IN_BED / OUT_OF_BED

```
if 有人 (presence.isPresent):
    if distance > 80 cm:
        → OUT_OF_BED
    else:
        → IN_BED
```

#### IN_BED → *

```
1. bedStatus==0 → OUT_OF_BED（无条件）

2. 无人 > 600s && 无 HR → SESSION_END
3. 无人 > 30s → OUT_OF_BED

4. 入睡：Sleepiness > 0.45 && moveSleepEMA < 8 持续 300s
   → LIGHT_SLEEP（记录 sleepStartTime, sleepLatency）

5. 体动觉醒：moveAwakeEMA > 25
   → AWAKE
```

#### AWAKE → *

```
1. bedStatus==0 → OUT_OF_BED

2. 无人 > 600s → SESSION_END
3. 无人 > 30s → OUT_OF_BED

4. 起床：曾入睡 && moveAwakeEMA ≥ 30 持续 300s → GETTING_UP

5. 重新入睡：Sleepiness > 0.45 && moveSleepEMA < 8 持续 300s
   → LIGHT_SLEEP
```

#### LIGHT_SLEEP → *

```
1. bedStatus==0 → OUT_OF_BED
2. 无人 > 30s → OUT_OF_BED

3. 快速觉醒：movement > 45 → AWAKE（即时）
4. HR 持续升高（checkHRAwakening）→ AWAKE

5. AwakeScore 最高（margin 0.1）确认 30s → AWAKE

6. DeepScore 最高 && > 0.40 && 浅睡稳定 ≥ 20min
   && 深睡评分稳定 ≥ 60s && 确认 180s → DEEP_SLEEP

7. REMScore 最高 && > 0.45（确认 60s）→ REM_SLEEP
```

#### DEEP_SLEEP → *

```
1. bedStatus==0 → OUT_OF_BED
2. 无人 > 30s → OUT_OF_BED

3. movement > 45 → AWAKE（即时）
4. HR 持续升高 → AWAKE

5. movement > 25 确认 30s → LIGHT_SLEEP
6. movement > 25 确认 15s → AWAKE

7. REMScore > LightScore && REMScore > 0.30 && DeepScore < 0.30
   确认 60s → REM_SLEEP
```

#### REM_SLEEP → *

```
1. bedStatus==0 → OUT_OF_BED
2. 无人 > 30s → OUT_OF_BED

3. movement > 45 → AWAKE（即时）
4. HR 持续升高 → AWAKE

5. AwakeScore 最高（确认 30s）→ AWAKE
6. LightScore 最高 && REMScore < 0.30 → LIGHT_SLEEP
7. DeepScore 最高 && > 0.40 && 稳定 ≥ 60s（确认 180s）→ DEEP_SLEEP
```

#### OUT_OF_BED / GETTING_UP / SESSION_END

```
OUT_OF_BED:
  有人 → IN_BED
  无人 > 600s → SESSION_END

GETTING_UP:
  无人 > 600s → SESSION_END
  movement < 30 持续 60s → AWAKE（重新躺下）

SESSION_END:
  有人 → IN_BED（新会话）
```

---

## 7. 统计与评分层

### 7.1 统计累计

```c
void SleepAnalyzer::updateStatistics(unsigned long dt) {
    switch (currentState) {
        case SLEEP_LIGHT_SLEEP:
            stats.lightSleepTime += dt;     // 累加毫秒
            stats.totalSleepTime += dt;
            break;
        case SLEEP_DEEP_SLEEP:
            stats.deepSleepTime += dt;
            stats.totalSleepTime += dt;
            break;
        case SLEEP_REM_SLEEP:
            stats.remSleepTime += dt;
            stats.totalSleepTime += dt;
            break;
        case SLEEP_AWAKE:
            stats.awakeTime += dt;
            break;
        case SLEEP_OUT_OF_BED:
            stats.outOfBedTime += dt;
            break;
        default: break;
    }
}
```

### 7.2 睡眠周期计数

```c
void SleepAnalyzer::updateSleepCycle() {
    // 首次进入深睡 → 记录周期开始时间
    if (currentState == SLEEP_DEEP_SLEEP && !cycle.inDeepPhase) {
        cycle.inDeepPhase = true;
        if (cycle.cycleStartTime == 0) {
            cycle.cycleStartTime = millis();
        }
    }

    // 标记 REM 阶段完成
    if (currentState == SLEEP_REM_SLEEP && !cycle.inRemPhase) {
        cycle.inRemPhase = true;
    }

    // 从深睡进入浅睡 → 标记深睡阶段结束
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inDeepPhase) {
        cycle.inDeepPhase = false;
    }

    // 从 REM 进入浅睡 → 标记 REM 阶段结束
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inRemPhase) {
        cycle.inRemPhase = false;
    }

    // 清醒/离床 + 已完成深睡或已完成 REM → 周期计数+1
    if ((currentState == SLEEP_AWAKE || currentState == SLEEP_OUT_OF_BED) &&
        (cycle.inDeepPhase || cycle.inRemPhase) &&
        cycle.cycleStartTime > 0) {
        cycle.cycleCount++;
        cycle.inDeepPhase = false;
        cycle.inRemPhase = false;
        cycle.cycleStartTime = millis();
        stats.sleepCycles = cycle.cycleCount;
    }
}
```

### 7.3 睡眠评分模型（满分 100）

| 维度 | 满分 | C 代码逻辑 |
|:---|:---:|:---|
| **时长评分** | 18 | `totalHours ∈ [7,9] → 18; [6,7) or (9,10] → 13; else → 5` |
| **深睡评分** | 14 | `deepRatio > 0.20 → 14; > 0.15 → 11; > 0.10 → 7; else → 3` |
| **连续性评分** | 11 | `wakeCount ≤ 1 → 11; ≤ 3 → 7; ≤ 5 → 4; else → 2` |
| **生理质量** | 7 | 固定值 `7.0f`（预留校准接口） |
| **入睡速度** | 8 | `latencyMin < 20 → 8; < 30 → 6; < 45 → 3; else → 1` |
| **睡眠效率** | 14 | `eff > 0.90 → 14; > 0.80 → 10; > 0.70 → 6; else → 3` |
| **周期评分** | 28 | 周期数 + REM 占比加成（见下表） |

**周期评分明细**：

```
周期数部分（0~20）：
  cycles ≥ 4 → 20; ≥ 3 → 15; ≥ 2 → 10; ≥ 1 → 6; else → 2

REM 占比加成（0~8）：
  20% ≤ remRatio ≤ 25% → +8
  remRatio ≥ 15% → +5
  remRatio > 0%  → +2

cycleScore = clamp(周期数部分 + REM加成, 0, 28)
```

**总分计算**：

```c
float rawTotal = durationScore + deepScore + continuityScore
               + physiologyScore + latencyScore + efficiencyScore + cycleScore;

totalScore = constrain_value(rawTotal / 100.0f * 100.0f, 0.0f, 100.0f);
```

### 7.4 串口输出格式

```c
// 状态打印（每秒）
void SleepAnalyzer::printState() {
    unsigned long stateDuration = (millis() - stateEnterTime) / 1000;
    Serial.printf("🛏️ 睡眠状态: %s | 持续: %02lu:%02lu:%02lu | 困倦度: %.2f | 周期: %d\n",
        SLEEP_STATE_NAMES[currentState],
        stateDuration / 3600, (stateDuration % 3600) / 60, stateDuration % 60,
        currentSleepiness, cycle.cycleCount);
}

// 统计打印（会话结束时）
void SleepAnalyzer::printStatistics() {
    Serial.printf("  总睡眠: %02lu:%02lu:%02lu\n", ...);
    Serial.printf("  深睡:   %02lu:%02lu:%02lu\n", ...);
    Serial.printf("  浅睡:   %02lu:%02lu:%02lu\n", ...);
    Serial.printf("  REM:    %02lu:%02lu:%02lu\n", ...);
    Serial.printf("  清醒:   %02lu:%02lu:%02lu\n", ...);
    Serial.printf("  醒来次数: %d\n", stats.wakeCount);
    Serial.printf("  入睡耗时: %lus\n", stats.sleepLatency);
    Serial.printf("  睡眠周期: %d\n", stats.sleepCycles);
    Serial.printf("  睡眠效率: %.0f%%\n", sleepEfficiency * 100);
}

// 评分打印
void SleepAnalyzer::calculateSleepScore() {
    // ...计算...
    Serial.printf("  时长评分: %.0f/18\n", score.durationScore);
    Serial.printf("  深睡评分: %.0f/14\n", score.deepScore);
    Serial.printf("  连续性评分: %.0f/11\n", score.continuityScore);
    Serial.printf("  生理质量评分: %.0f/7\n", score.physiologyScore);
    Serial.printf("  入睡速度评分: %.0f/8\n", score.latencyScore);
    Serial.printf("  睡眠效率评分: %.0f/14\n", score.efficiencyScore);
    Serial.printf("  周期评分: %.0f/28\n", score.cycleScore);
    Serial.printf("  总分: %.0f/100\n", score.totalScore);
}
```

---

## 8. 主入口 update() 函数

```c
void SleepAnalyzer::update(
    const HeartRateData& hrData,
    const RespirationData& rrData,
    const HRVEstimate& hrvData,
    const BodyMovementData& movementData,
    bool bedStatus) {

    // 步骤 1：基线校准（仅在 AWAKE/IN_BED 状态执行）
    calibrateBaseline(hrData, rrData, movementData);

    // 步骤 2：双时间尺度体动 EMA 更新
    float rawMove = movementData.isValid ? movementData.movement : 0;
    moveSleepEMA = emaSmooth(rawMove, moveSleepEMA, EMA_SLEEP_ALPHA);   // α=0.01
    moveAwakeEMA = emaSmooth(rawMove, moveAwakeEMA, EMA_AWAKE_ALPHA);   // α=0.3

    // 步骤 3：存在检测
    PresenceData presence = evaluatePresence();

    // 步骤 4：执行状态机
    updateState(presence, hrData, rrData, hrvData, movementData, bedStatus);

    // 步骤 5：睡眠周期计数
    updateSleepCycle();

    // 步骤 6：统计更新（+1000ms）
    updateStatistics(1000);
}
```

**调用时序**：

```
tasks_manager.cpp（每秒定时器）
    │
    ├─ 读取雷达数据帧（radar_manager.cpp）
    ├─ 生理信号平滑（data_processor.cpp）
    ├─ HRV 计算
    │
    └─ SleepAnalyzer::update(hr, rr, hrv, movement, bedStatus)
        ├─ calibrateBaseline()         ← 仅 AWAKE/IN_BED
        ├─ emaSmooth × 2                ← 双时间尺度体动 EMA
        ├─ evaluatePresence()           ← 存在检测
        ├─ updateState()                ← 状态机核心
        │   ├─ calculateSleepinessScore()
        │   ├─ calculateDeepSleepScore()
        │   ├─ calculateLightSleepScore()
        │   ├─ calculateAwakeScore()
        │   ├─ calculateRemScore()
        │   ├─ checkHRAwakening()
        │   └─ tryTransitionTo()
        ├─ updateSleepCycle()
        └─ updateStatistics(1000)
```

---

## 9. 参数总表

### 9.1 EMA 平滑参数

| 常量名 | 值 | 用途 |
|:---|:---:|:---|
| `EMA_ALPHA` | 0.2 | 评分函数 EMA 平滑（~5秒） |
| `EMA_SLEEP_ALPHA` | 0.01 | 慢速体动 EMA（~5分钟），入睡/深睡判断 |
| `EMA_AWAKE_ALPHA` | 0.3 | 快速体动 EMA（~3秒），觉醒检测 |
| `BASELINE_BETA` | 0.01 | HR/RR 基线滚动更新速率 |

### 9.2 评分阈值

| 常量名 | 值 | 用途 |
|:---|:---:|:---|
| `SLEEPINESS_THRESHOLD` | 0.45f | 入睡所需最低困倦度 |
| `HYSTERESIS_ENTER_DEEP` | 0.40f | 进入深睡所需最低评分 |
| `HYSTERESIS_EXIT_DEEP` | 0.30f | 退出深睡评分下限 |
| `HYSTERESIS_ENTER_REM` | 0.45f | 进入快速眼动所需最低评分 |
| `HYSTERESIS_EXIT_REM` | 0.30f | 退出快速眼动评分下限 |
| `CONFIDENCE_MARGIN` | 0.10f | Best Score 胜出所需最小分差 |

### 9.3 体动阈值

| 常量名 | 值 | 用途 |
|:---|:---:|:---|
| `SLEEP_EMA_MOVEMENT_MAX` | 8 | 入睡条件：慢速体动 EMA 上限 |
| `AWAKE_EMA_MOVEMENT_MIN` | 25 | 觉醒条件：快速体动 EMA 下限 |
| `FAST_AWAKE_MOVEMENT_THRESHOLD` | 45 | 快速觉醒：体动超过此值立即切换（绕过评分） |
| `DEEP_SLEEP_HARD_MOVEMENT_LIMIT` | 25 | 深睡/REM 体动硬上限（超过直接否决） |
| `MOVEMENT_HIGH_THRESHOLD` | 25 | 持续高体动触发觉醒 |
| `DEEP_SLEEP_MOVEMENT_THRESHOLD` | 10 | 深睡评分体动偏好上限 |
| `SLEEPINESS_MOVEMENT_THRESHOLD` | 10 | 困倦度评分体动参考上限 |
| `GETTING_UP_MOVEMENT_THRESHOLD` | 30 | 起床检测体动阈值 |

### 9.4 确认时间（秒）

| 常量名 | 值 | 用途 |
|:---|:---:|:---|
| `SLEEPINESS_MIN_SECONDS` | 300 | 入睡需持续满足条件的秒数 |
| `DEEP_SLEEP_CONFIRM_SECONDS` | 180 | 进入深睡确认时间 |
| `LIGHT_SLEEP_CONFIRM_SECONDS` | 30 | 进入浅睡确认时间 |
| `AWAKE_CONFIRM_SECONDS` | 15 | 快速觉醒确认时间 |
| `AWAKE_SLOW_CONFIRM_SECONDS` | 30 | 慢速觉醒（评分驱动）确认时间 |
| `REM_CONFIRM_SECONDS` | 60 | 进入快速眼动确认时间 |
| `DEEP_STABLE_MIN_SECONDS` | 60 | 深睡评分稳定确认时间 |
| `LIGHT_SLEEP_STABILITY_MIN_SECONDS` | 1200 | 进入深睡前需浅睡稳定最少时间 |
| `OUT_OF_BED_SECONDS` | 30 | 无人判定为离床的时间 |
| `NO_PERSON_END_SECONDS` | 600 | 无人判定为会话结束的时间 |
| `MIN_STATE_DWELL_MS` | 10000 | 状态最小驻留时间（10秒，防抖动） |
| `GETTING_UP_MIN_SECONDS` | 300 | 起床检测持续时间 |

### 9.5 HR 觉醒检测参数

| 常量名 | 值 | 用途 |
|:---|:---:|:---|
| `HR_AWAKE_LOOKBACK` | 60 | 睡眠 HR 基线回溯窗口（秒） |
| `HR_ELEVATED_RATIO` | 1.10f | HR 超过基线 10% 视为升高 |
| `HR_ELEVATED_MIN_SEC` | 5 | 升高至少持续 5 秒才触发觉醒 |

---

## 10. 调用流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    雷达串口原始数据帧                             │
│  心率 / 呼吸率 / 体动 / 人员检测 / 床位状态 / 距离 / 波形 ...   │
└──────────────────────────┬──────────────────────────────────────┘
                           │ radar_manager.cpp 解析
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    data_processor.cpp                             │
│                                                                   │
│  HR_smoothed = 0.3 × HR_raw + 0.7 × HR_smoothed_prev            │
│  RR_smoothed = 0.3 × RR_raw + 0.7 × RR_smoothed_prev            │
│  HRV_rmssd   = sqrta 计算                                        │
└──────────────────────────┬───────────────────────────────────────┘
                           │ HeartRateData, RespirationData,
                           │ HRVEstimate, BodyMovementData, bedStatus
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    tasks_manager.cpp                              │
│                                                                   │
│  每秒定时器触发 → analyzer.update(hr, rr, hrv, move, bed)       │
└──────────────────────────┬───────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    sleep_analyzer.cpp                             │
│                                                                   │
│  update()                                                        │
│  ├─ calibrateBaseline()         HR/RR 基线滚动校准                │
│  ├─ moveSleepEMA  ← α=0.01     慢速体动（~5min）                 │
│  ├─ moveAwakeEMA  ← α=0.3      快速体动（~3sec）                 │
│  ├─ evaluatePresence()         存在检测                           │
│  ├─ updateState()      ─────────────────────────────┐            │
│  │   ├─ calculateSleepinessScore()  入睡倾向评分     │            │
│  │   ├─ calculateDeepSleepScore()   深睡评分         │            │
│  │   ├─ calculateLightSleepScore()  浅睡评分         │ 评分层     │
│  │   ├─ calculateAwakeScore()       觉醒评分         │            │
│  │   ├─ calculateRemScore()         快速眼动评分     │            │
│  │   ├─ checkHRAwakening()          HR 觉醒检测      │            │
│  │   └─ tryTransitionTo()          确认切换    ─────┘            │
│  ├─ updateSleepCycle()           睡眠周期计数                     │
│  └─ updateStatistics(1000)       时长累计 +1000ms                │
│                                                                   │
│  ┌─────────────────────────────────────────────────────┐         │
│  │ 快速路径（绕过评分，即时切换）：                      │         │
│  │   movement > 45   → AWAKE                            │         │
│  │   bedStatus == 0  → OUT_OF_BED                       │         │
│  │   HR 持续升高     → AWAKE                            │         │
│  └─────────────────────────────────────────────────────┘         │
└──────────────────────────┬───────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│                    输出                                           │
│                                                                   │
│  Serial.printf() → 状态切换日志 / 统计 / 评分                    │
│  MQTT / BLE     → 实时状态上报                                    │
│  SleepStatistics → 供外部查询                                     │
│  SleepScore      → 供外部查询                                     │
└──────────────────────────────────────────────────────────────────┘
```
