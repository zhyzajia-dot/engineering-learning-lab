/**
 * @file sleep_analyzer.h
 * @brief 本地睡眠状态分析器的类型、状态和算法接口。
 *
 * 综合在床状态、体动、心率和呼吸数据判断清醒及各睡眠阶段。
 */
#ifndef SLEEP_ANALYZER_H
#define SLEEP_ANALYZER_H

#include <Arduino.h>
#include "config.h"
#include "data_processor.h"
#include "emotion_analyzer_simple.h"
#include "radar_manager.h"

enum SleepState {
    SLEEP_NO_PERSON = 0,
    SLEEP_IN_BED,
    SLEEP_AWAKE,
    SLEEP_LIGHT_SLEEP,
    SLEEP_DEEP_SLEEP,
    SLEEP_REM_SLEEP,
    SLEEP_OUT_OF_BED,
    SLEEP_GETTING_UP,
    SLEEP_SESSION_END // 睡眠会话结束（无人且睡眠结束，等待下一次入睡）
};

static const char* SLEEP_STATE_NAMES[] = {
    "无人",
    "在床",
    "清醒",
    "浅睡",
    "深睡",
    "REM",
    "离床",
    "起床",
    "会话结束"
};

struct PresenceData {
    bool isPresent;
    float distance;
    float confidence;
    float motionEnergy;
};

struct SleepCycle {
    int cycleCount;
    unsigned long cycleStartTime;
    bool inDeepPhase;
    bool inRemPhase;
    unsigned long lastDeepEndTime;
    unsigned long lastRemEndTime;
};

struct SleepStatistics {
    unsigned long totalSleepTime;
    unsigned long deepSleepTime;
    unsigned long lightSleepTime;
    unsigned long remSleepTime;
    unsigned long awakeTime;
    unsigned long outOfBedTime;
    unsigned long sleepLatency;
    int wakeCount;
    int sleepCycles;
    unsigned long sessionStartTime;
    unsigned long sleepStartTime;
    unsigned long lastWakeTime;
};

struct SleepScore {
    float durationScore;
    float deepScore;
    float continuityScore;
    float physiologyScore;
    float latencyScore;
    float efficiencyScore;
    float cycleScore;
    float totalScore;
};

struct TrainingData {
    float hr;
    float rr;
    float hrv;
    float movement;
    int label;
};

class SleepAnalyzer {
private:
    static const float EMA_ALPHA;
    static const float CONFIDENCE_MARGIN;
    static const float BASELINE_BETA;
    static const int MIN_STATE_DWELL_MS = 10000;
    static const float HYSTERESIS_ENTER_DEEP;
    static const float HYSTERESIS_EXIT_DEEP;
    static const float HYSTERESIS_ENTER_REM;
    static const float HYSTERESIS_EXIT_REM;

    SleepState currentState;
    SleepState pendingState;
    SleepStatistics stats;
    SleepScore score;
    SleepCycle cycle;

    unsigned long stateEnterTime;
    unsigned long pendingStateTime;
    unsigned long noPersonTimer;
    unsigned long outOfBedTimer;
    unsigned long sleepinessDuration;
    unsigned long awakeDuration;
    unsigned long deepSleepDuration;
    unsigned long lightSleepDuration;
    unsigned long remSleepDuration;
    unsigned long movementHighDuration;
    unsigned long gettingUpDuration;
    unsigned long deepStableDuration;

    float baselineHR;
    float baselineRR;
    bool baselineCalibrated;
    int baselineSampleCount;
    float baselineHRSum;
    float baselineRRSum;
    float lastBaselineHR;
    float lastBaselineRR;
    float hrStabilitySum;
    float rrStabilitySum;
    int stabilitySampleCount;

    float lastRRValue;

    // 深睡期生理质量累计（用于睡眠评分）
    float deepHrvSum;         // 深睡期 RMSSD 累计
    float deepRrRegSum;       // 深睡期呼吸规律性累计
    int   deepPhysioSamples;  // 深睡期样本数

    // HR 觉醒检测（从 Python 移植：追踪睡眠期 HR 基线，检测持续升高）
    float sleepHRBaseline;
    int sleepHRBaselineCount;
    int hrAwakeTimer;

    static const int DEEP_SLEEP_CONFIRM_SECONDS = 180;
    static const int LIGHT_SLEEP_CONFIRM_SECONDS = 30;
    static const int AWAKE_CONFIRM_SECONDS = 15;
    static const int AWAKE_SLOW_CONFIRM_SECONDS = 30;
    static const int NO_PERSON_END_SECONDS = 600;
    static const int OUT_OF_BED_SECONDS = 30;
    static const int SLEEPINESS_MIN_SECONDS = 300;
    static const int SLEEPINESS_MAX_SECONDS = 600;
    static const int MOVEMENT_HIGH_THRESHOLD = 25;
    static const int DEEP_SLEEP_MOVEMENT_THRESHOLD = 10;
    static const int DEEP_SLEEP_HARD_MOVEMENT_LIMIT = 25;
    static const int FAST_AWAKE_MOVEMENT_THRESHOLD = 45;
    static const int SLEEPINESS_MOVEMENT_THRESHOLD = 10;
    static const int DEEP_STABLE_MIN_SECONDS = 60;   // 深睡稳定确认时间
    static const int REM_CONFIRM_SECONDS = 60;
    static const int GETTING_UP_MIN_SECONDS = 300;
    static const int GETTING_UP_MOVEMENT_THRESHOLD = 30;
    static const int MIN_AWAKE_COUNT_SEC = 120;
    static const int LIGHT_SLEEP_STABILITY_MIN_SECONDS = 1200;  // 进入深睡前需浅睡稳定
    // 双时间尺度体动 EMA（非对称状态机核心）
    static const float EMA_SLEEP_ALPHA;           // 慢速 ~5分钟，用于入睡判断
    static const float EMA_AWAKE_ALPHA;           // 快速 ~3秒，用于觉醒检测
    static const int SLEEP_EMA_MOVEMENT_MAX = 8;  // 入睡条件：慢速体动 EMA 低于此值
    static const int AWAKE_EMA_MOVEMENT_MIN = 25; // 觉醒条件：快速体动 EMA 高于此值
    static const float SLEEPINESS_THRESHOLD;
    static const float BASELINE_MOVEMENT_THRESHOLD;
    static const float BASELINE_HR_STABILITY_THRESHOLD;
    static const float BASELINE_RR_STABILITY_THRESHOLD;

    float currentSleepiness;
    float currentDeepScore;
    float currentLightScore;
    float currentAwakeScore;
    float currentRemScore;

    // 双时间尺度体动 EMA（非对称状态机核心）
    float moveSleepEMA;  // 慢速 ~5分钟，用于入睡判断
    float moveAwakeEMA;  // 快速 ~3秒，用于觉醒检测

    bool wasAsleep;

    PresenceData evaluatePresence();
    float sigmoid(float x);
    float emaSmooth(float input, float last, float alpha);
    float updateBaseline(float current, float input, float beta);
    float calculateSleepinessScore(const HeartRateData& hrData,
                                   const RespirationData& rrData,
                                   const HRVEstimate& hrvData,
                                   const BodyMovementData& movementData);
    float calculateDeepSleepScore(const HeartRateData& hrData,
                                  const RespirationData& rrData,
                                  const HRVEstimate& hrvData,
                                  const BodyMovementData& movementData);
    float calculateLightSleepScore(const HeartRateData& hrData,
                                   const RespirationData& rrData,
                                   const HRVEstimate& hrvData,
                                   const BodyMovementData& movementData);
    float calculateAwakeScore(const HeartRateData& hrData,
                              const RespirationData& rrData,
                              const HRVEstimate& hrvData,
                              const BodyMovementData& movementData);
    float calculateRemScore(const HeartRateData& hrData,
                            const RespirationData& rrData,
                            const HRVEstimate& hrvData,
                            const BodyMovementData& movementData);

    void updateState(PresenceData& presence,
                     const HeartRateData& hrData,
                     const RespirationData& rrData,
                     const HRVEstimate& hrvData,
                     const BodyMovementData& movementData,
                     bool bedStatus);
    void updateStatistics(unsigned long dt);
    void updateSleepCycle();
    void calculateSleepScore();
    void calibrateBaseline(const HeartRateData& hrData,
                           const RespirationData& rrData,
                           const BodyMovementData& movementData);

    float normalizeHR(float hr);
    float normalizeRR(float rr);
    float normalizeHRV(float hrv);
    float normalizeMovement(float movement);

    bool tryTransitionTo(SleepState target, unsigned long confirmMs);
    bool isBestScore(float score, float s2, float s3, float s4, float margin);
    bool checkHRAwakening(float hr, bool hrValid);  // HR 持续升高 → 觉醒检测

public:
    SleepAnalyzer();
    ~SleepAnalyzer();

    void update(const HeartRateData& hrData,
                const RespirationData& rrData,
                const HRVEstimate& hrvData,
                const BodyMovementData& movementData,
                bool bedStatus);

    SleepState getCurrentState() const { return currentState; }
    SleepStatistics getStatistics() const { return stats; }
    SleepScore getScore() const { return score; }
    SleepCycle getCycle() const { return cycle; }
    float getSleepiness() const { return currentSleepiness; }

    void reset();
    void printState();
    void printStatistics();
};

#endif
