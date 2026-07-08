/**
 * @file emotion_analyzer_simple.cpp
 * @brief 简化版情绪分析器实现
 */

#include "emotion_analyzer_simple.h"
#include <math.h>

// ==================== 情绪分析器实现 ====================

SimpleEmotionAnalyzer::SimpleEmotionAnalyzer(int histSize) 
    : smoothingFactor(0.15f), historySize(histSize), historyIndex(0), historyCount(0) {
    
    memset(emotionProbs, 0, sizeof(emotionProbs));
    memset(prevProbs, 0, sizeof(prevProbs));
    memset(hrWindow, 0, sizeof(hrWindow));
    memset(rrWindow, 0, sizeof(rrWindow));
    memset(hrvWindow, 0, sizeof(hrvWindow));
    windowIndex = 0;
    windowCount = 0;
    emotionHistory = new EmotionType[historySize];
    memset(emotionHistory, 0, historySize * sizeof(EmotionType));
    
    memset(&baseline, 0, sizeof(baseline));
    baseline.hrResting = 72;
    baseline.rrResting = 16;
    baseline.isColdStarting = true;
    baseline.coldStartHrSum = 0;
    baseline.coldStartRrSum = 0;
    baseline.coldStartCount = 0;
    
    memset(&lastResult, 0, sizeof(lastResult));
}

SimpleEmotionAnalyzer::~SimpleEmotionAnalyzer() {
    delete[] emotionHistory;
}

EmotionResult SimpleEmotionAnalyzer::analyze(const HeartRateData& hrData, 
                                             const RespirationData& rrData,
                                             const HRVEstimate& hrvData,
                                             const BodyMovementData& movementData) {
    EmotionResult result;
    memset(&result, 0, sizeof(result));
    result.timestamp = millis();
    
    // 检查数据有效性
    if (!hrData.isValid && !rrData.isValid) {
        result.isValid = false;
        return result;
    }
    
    // 时间窗口平滑（减少抖动）
    float smoothedHR = getSmoothedHR(hrData);
    float smoothedRR = getSmoothedRR(rrData);
    float smoothedHRV = getSmoothedHRV(hrvData);
    
    // 统一推进窗口索引（确保 HR/RR/HRV 同步）
    windowIndex = (windowIndex + 1) % WINDOW_SIZE;
    if (windowCount < WINDOW_SIZE - 1) windowCount++;
    
    HeartRateData smoothHrData = hrData;
    if (hrData.isValid) {
        smoothHrData.bpmSmoothed = smoothedHR;
    }
    
    RespirationData smoothRrData = rrData;
    if (rrData.isValid) {
        smoothRrData.rateSmoothed = smoothedRR;
    }
    
    HRVEstimate smoothHrvData = hrvData;
    if (hrvData.isValid) {
        smoothHrvData.rmssd = smoothedHRV;
    }
    
    // 计算各情绪得分
    emotionProbs[EMOTION_CALM] = calculateCalmScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_HAPPY] = calculateHappyScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_EXCITED] = calculateExcitedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_ANXIOUS] = calculateAnxiousScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_ANGRY] = calculateAngryScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_SAD] = calculateSadScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_STRESSED] = calculateStressedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_RELAXED] = calculateRelaxedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_UNKNOWN] = 0.02f;
    
    // 归一化
    normalizeProbabilities();
    
    // Top1 放大（强行制造赢家）
    int maxIdx = 0;
    for (int i = 1; i < 9; i++) {
        if (emotionProbs[i] > emotionProbs[maxIdx]) {
            maxIdx = i;
        }
    }
    emotionProbs[maxIdx] *= 1.3f;
    
    // 再次归一化
    normalizeProbabilities();
    
    smoothProbabilities();
    
    // 找出主要情绪（同时找第二大概率）
    int secondIdx = 0;
    float maxProb = emotionProbs[0], secondProb = 0;
    maxIdx = 0;  // 重置 maxIdx
    for (int i = 1; i < 9; i++) {
        if (emotionProbs[i] > maxProb) {
            secondProb = maxProb;
            secondIdx = maxIdx;
            maxProb = emotionProbs[i];
            maxIdx = i;
        } else if (emotionProbs[i] > secondProb) {
            secondProb = emotionProbs[i];
            secondIdx = i;
        }
    }
    
    result.primaryEmotion = (EmotionType)maxIdx;
    result.secondaryEmotion = (EmotionType)secondIdx;
    result.confidence = maxProb;

    // 智能 UNKNOWN 判断：低置信度 且 多情绪接近（改为 AND）
    if (maxProb < 0.20f && (maxProb - secondProb) < 0.03f) {
        result.primaryEmotion = EMOTION_UNKNOWN;
        result.confidence = maxProb;
    }

    if (result.primaryEmotion == EMOTION_ANXIOUS || 
        result.primaryEmotion == EMOTION_ANGRY ||
        result.primaryEmotion == EMOTION_STRESSED) {
        
        float combinedProb = emotionProbs[EMOTION_ANXIOUS] + 
                            emotionProbs[EMOTION_ANGRY] + 
                            emotionProbs[EMOTION_STRESSED];
        result.primaryEmotion = EMOTION_STRESSED;
        result.confidence = max(combinedProb, 
                              max(emotionProbs[EMOTION_ANXIOUS],
                                  max(emotionProbs[EMOTION_ANGRY], 
                                      emotionProbs[EMOTION_STRESSED])));
    }

    float hrFactor = fabs(hrData.bpmSmoothed - baseline.hrResting) / 40.0f;
    float hrvFactor = hrvData.isValid ? (1.0f - sigmoid(hrvData.rmssd, 0.02f, 40)) : 0.5f;
    float rrFactor = rrData.isValid ? fabs(rrData.rateSmoothed - baseline.rrResting) / 10.0f : 0.3f;
    
    result.intensity = 0.4f 
                     + 0.3f * constrain_value(hrFactor, 0.0f, 1.0f)
                     + 0.2f * constrain_value(hrvFactor, 0.0f, 1.0f)
                     + 0.1f * constrain_value(rrFactor, 0.0f, 1.0f);
    result.intensity = constrain_value(result.intensity, 0.3f, 1.0f);
    
    // 计算情绪维度
    calculateDimensions(hrData, rrData);
    result.valence = lastResult.valence;
    result.arousal = lastResult.arousal;
    
    // 强制分类：UNKNOWN 必须落地（根据 arousal/valence 强制选择）
    if (result.primaryEmotion == EMOTION_UNKNOWN) {
        if (result.arousal > 0.6f) {
            result.primaryEmotion = EMOTION_EXCITED;
        } else if (result.valence < -0.2f) {
            result.primaryEmotion = EMOTION_STRESSED;
        } else {
            result.primaryEmotion = EMOTION_CALM;
        }
    }
    
    // 计算压力水平
    calculateStressLevels(hrData, rrData, hrvData);
    result.stressLevel = lastResult.stressLevel;
    result.anxietyLevel = lastResult.anxietyLevel;
    result.relaxationLevel = lastResult.relaxationLevel;
    
    // 自主神经活动
    if (hrvData.isValid) {
        result.sympatheticActivity = 1.0f - hrvData.autonomicBalance;
        result.parasympatheticActivity = hrvData.autonomicBalance;
    } else {
        result.sympatheticActivity = 0.5f;
        result.parasympatheticActivity = 0.5f;
    }
    
    result.isValid = true;
    
    // 记录历史
    emotionHistory[historyIndex] = result.primaryEmotion;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) historyCount++;
    
    lastResult = result;
    return result;
}

void SimpleEmotionAnalyzer::setBaseline(const UserBaseline& bl) {
    baseline = bl;
}

void SimpleEmotionAnalyzer::calibrateBaseline(const HeartRateData& hrData, 
                                              const RespirationData& rrData,
                                              const BodyMovementData& movementData) {
    bool isRestingState = true;
    
    if (movementData.isValid && movementData.movementSmoothed > 30) {
        isRestingState = false;
    }
    
    if (hrData.isValid && hrData.bpmStd > 8.0f) {
        isRestingState = false;
    }

    if (!isRestingState) return;

    // 冷启动阶段：收集前30秒数据用于初始化基线
    if (baseline.isColdStarting && baseline.coldStartCount < UserBaseline::COLD_START_SAMPLES) {
        if (hrData.isValid) {
            baseline.coldStartHrSum += hrData.bpmSmoothed;
        }
        if (rrData.isValid) {
            baseline.coldStartRrSum += rrData.rateSmoothed;
        }
        baseline.coldStartCount++;
        
        // 冷启动完成：使用平均值初始化基线
        if (baseline.coldStartCount >= UserBaseline::COLD_START_SAMPLES) {
            if (hrData.isValid && baseline.coldStartHrSum > 0) {
                baseline.hrResting = baseline.coldStartHrSum / baseline.coldStartCount;
            }
            if (rrData.isValid && baseline.coldStartRrSum > 0) {
                baseline.rrResting = baseline.coldStartRrSum / baseline.coldStartCount;
            }
            baseline.isColdStarting = false;
            baseline.isCalibrated = true;
            baseline.calibrationTime = millis();
        }
        return;  // 冷启动期间不进行正常校准
    }

    // 正常校准模式（指数移动平均）
    if (hrData.isValid) {
        baseline.hrResting = baseline.hrResting * 0.7f + hrData.bpmSmoothed * 0.3f;
        if (hrData.bpmMin > 0 && hrData.bpmMin < baseline.hrMin) {
            baseline.hrMin = hrData.bpmMin;
        }
        if (hrData.bpmMax > baseline.hrMax) {
            baseline.hrMax = hrData.bpmMax;
        }
    }
    
    if (rrData.isValid) {
        baseline.rrResting = baseline.rrResting * 0.7f + rrData.rateSmoothed * 0.3f;
    }
    
    baseline.isCalibrated = true;
    baseline.calibrationTime = millis();
}

EmotionType SimpleEmotionAnalyzer::getRecentDominantEmotion(int seconds) {
    int counts[9] = {0};
    int entries = min(historyCount, (int)(seconds / 2));  // 假设每2秒一个记录
    
    for (int i = 0; i < entries; i++) {
        int idx = (historyIndex - 1 - i + historySize) % historySize;
        counts[emotionHistory[idx]]++;
    }
    
    int maxCount = 0;
    EmotionType dominant = EMOTION_CALM;
    for (int i = 0; i < 9; i++) {
        if (counts[i] > maxCount) {
            maxCount = counts[i];
            dominant = (EmotionType)i;
        }
    }
    
    return dominant;
}

void SimpleEmotionAnalyzer::reset() {
    memset(emotionProbs, 0, sizeof(emotionProbs));
    memset(prevProbs, 0, sizeof(prevProbs));
    memset(hrWindow, 0, sizeof(hrWindow));
    memset(rrWindow, 0, sizeof(rrWindow));
    memset(hrvWindow, 0, sizeof(hrvWindow));
    windowIndex = 0;
    windowCount = 0;
    memset(emotionHistory, 0, historySize * sizeof(EmotionType));
    historyIndex = 0;
    historyCount = 0;
    memset(&lastResult, 0, sizeof(lastResult));
}

// ==================== 情绪规则匹配 ====================

float SimpleEmotionAnalyzer::calculateCalmScore(const HeartRateData& hr, 
                                                 const RespirationData& rr, 
                                                 const HRVEstimate& hrv,
                                                 const BodyMovementData& movement) {
    float score = 0;
    
    // 心率接近基线，变化小
    if (hr.isValid) {
        float hrNorm = fabs(normalizeHR(hr.bpmSmoothed, baseline.hrResting));
        float hrScore = gaussian(hrNorm, 0, 0.3f);
        float stabilityScore = 1.0f - constrain_value(hr.bpmStd / 12.0f, 0.0f, 1.0f);
        score += 0.45f * hrScore + 0.22f * stabilityScore;
    }
    
    // 呼吸规律，正常频率
    if (rr.isValid) {
        float rrNorm = fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting));
        float rateScore = gaussian(rrNorm, 0, 0.5f);
        float regularityScore = rr.regularity;
        score += 0.27f * rateScore + 0.16f * regularityScore;
    }
    
    // HRV正常或较高
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = sigmoid(hrvNorm, 2.0f, 0.7f);
        score += 0.10f * hrvScore;
    }
    
    // 体动低或适中（平静状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = gaussian(moveNorm, 0.15f, 0.2f);
        score += 0.08f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateHappyScore(const HeartRateData& hr, 
                                                  const RespirationData& rr, 
                                                  const HRVEstimate& hrv,
                                                  const BodyMovementData& movement) {
    float score = 0;
    
    // 心率略高于基线
    if (hr.isValid) {
        float hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = gaussian(hrNorm, 0.3f, 0.25f);
        score += 0.45f * hrScore;
        
        // 有适度变异性
        if (hr.bpmStd > 1.5f && hr.bpmStd < 10) {
            score += 0.10f;
        }
    }
    
    // 呼吸正常
    if (rr.isValid) {
        float rrNorm = fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting));
        float rateScore = gaussian(rrNorm, 0, 0.5f);
        score += 0.34f * rateScore;
    }
    
    // HRV较高
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = sigmoid(hrvNorm, 2.0f, 0.9f);
        score += 0.10f * hrvScore;
    }
    
    // 体动适中到较高（开心的活跃状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = gaussian(moveNorm, 0.45f, 0.25f);
        score += 0.08f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateExcitedScore(const HeartRateData& hr, 
                                                    const RespirationData& rr, 
                                                    const HRVEstimate& hrv,
                                                    const BodyMovementData& movement) {
    float score = 0;
    
    // 心率显著升高
    if (hr.isValid) {
        float hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.6f);
        score += 0.45f * hrScore;
        
        // 趋势上升
        if (hr.trend > 1.5f) {
            score += 0.10f;
        }
    }
    
    // 呼吸加快但规律
    if (rr.isValid) {
        float rrNorm = normalizeRR(rr.rateSmoothed, baseline.rrResting);
        float rateScore = sigmoid(rrNorm, 2.0f, 0.5f);
        score += 0.34f * rateScore;
    }
    
    // HRV中等
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = gaussian(hrvNorm, 0.7f, 0.4f);
        score += 0.10f * hrvScore;
    }
    
    // 体动较高（兴奋的活跃状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = sigmoid(moveNorm, 2.0f, 0.6f);
        score += 0.10f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateAnxiousScore(const HeartRateData& hr, 
                                                    const RespirationData& rr, 
                                                    const HRVEstimate& hrv,
                                                    const BodyMovementData& movement) {
    float score = 0;
    
    // 心率升高，变异小
    if (hr.isValid) {
        float hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.4f);
        float stdScore = 1.0f - sigmoid(hr.bpmStd / 6.0f, 2.0f, 0.5f);
        score += 0.36f * hrScore + 0.16f * stdScore;
    }
    
    // HRV低
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = 1.0f - sigmoid(hrvNorm, 2.0f, 0.6f);
        score += 0.25f * hrvScore;
        
        // 压力指数高
        float stressScore = sigmoid(hrv.stressIndex / 30.0f, 2.0f, 0.5f);
        score += 0.10f * stressScore;
    }
    
    // 呼吸不规律
    if (rr.isValid) {
        float irregularityScore = 1.0f - rr.regularity;
        score += 0.16f * irregularityScore;
    }
    
    // 体动适中到较高（焦虑的躁动状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = gaussian(moveNorm, 0.55f, 0.3f);
        score += 0.10f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateAngryScore(const HeartRateData& hr, 
                                                  const RespirationData& rr, 
                                                  const HRVEstimate& hrv,
                                                  const BodyMovementData& movement) {
    float score = 0;
    
    // 心率快速升高
    if (hr.isValid) {
        float hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.75f);
        float trendScore = sigmoid(hr.trend / 4.0f, 2.0f, 0.5f);
        score += 0.36f * hrScore + 0.20f * trendScore;
    }
    
    // HRV低
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = 1.0f - sigmoid(hrvNorm, 2.0f, 0.5f);
        score += 0.25f * hrvScore;
    }
    
    // 呼吸浅或不规律
    if (rr.isValid) {
        float irregularityScore = 1.0f - rr.regularity;
        score += 0.20f * irregularityScore;
    }
    
    // 体动高（愤怒的激动状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = sigmoid(moveNorm, 2.0f, 0.7f);
        score += 0.10f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateSadScore(const HeartRateData& hr, 
                                                const RespirationData& rr, 
                                                const HRVEstimate& hrv,
                                                const BodyMovementData& movement) {
    float score = 0;
    
    // 心率偏低
    if (hr.isValid) {
        float hrNorm = -normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.2f);
        score += 0.40f * hrScore;
    }
    
    // 呼吸浅慢
    if (rr.isValid) {
        float rrNorm = -normalizeRR(rr.rateSmoothed, baseline.rrResting);
        float rateScore = sigmoid(rrNorm, 2.0f, 0.3f);
        float varScore = 1.0f - constrain_value(rr.variability / 4.5f, 0.0f, 1.0f);
        score += 0.28f * rateScore + 0.20f * varScore;
    }
    
    // HRV可能降低
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = 1.0f - sigmoid(hrvNorm, 2.0f, 0.9f);
        score += 0.10f * hrvScore;
    }
    
    // 体动低（悲伤的无精打采状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = 1.0f - sigmoid(moveNorm, 2.0f, 0.2f);
        score += 0.10f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateStressedScore(const HeartRateData& hr, 
                                                     const RespirationData& rr, 
                                                     const HRVEstimate& hrv,
                                                     const BodyMovementData& movement) {
    float score = 0;
    
    // 心率持续偏高
    if (hr.isValid) {
        float hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.4f);
        score += 0.36f * hrScore;
    }
    
    // HRV显著降低
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = 1.0f - sigmoid(hrvNorm, 2.0f, 0.6f);
        float stressScore = sigmoid(hrv.stressIndex / 35.0f, 2.0f, 0.5f);
        score += 0.34f * hrvScore + 0.10f * stressScore;
    }
    
    // 呼吸浅快
    if (rr.isValid) {
        float rrNorm = normalizeRR(rr.rateSmoothed, baseline.rrResting);
        float rateScore = sigmoid(rrNorm, 2.0f, 0.5f);
        score += 0.20f * rateScore;
    }
    
    // 体动适中（压力的紧张状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = gaussian(moveNorm, 0.4f, 0.25f);
        score += 0.08f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateRelaxedScore(const HeartRateData& hr, 
                                                    const RespirationData& rr, 
                                                    const HRVEstimate& hrv,
                                                    const BodyMovementData& movement) {
    float score = 0;
    
    // 心率低且稳定
    if (hr.isValid) {
        float hrNorm = -normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        float hrScore = sigmoid(hrNorm, 2.0f, 0.25f);
        float stabilityScore = 1.0f - constrain_value(hr.bpmStd / 8.0f, 0.0f, 1.0f);
        score += 0.31f * hrScore + 0.16f * stabilityScore;
    }
    
    // HRV高
    if (hrv.isValid) {
        float hrvNorm = normalizeHRV(hrv.rmssd);
        float hrvScore = sigmoid(hrvNorm, 2.0f, 1.0f);
        score += 0.28f * hrvScore;
    }
    
    // 呼吸慢且规律
    if (rr.isValid) {
        float rrNorm = -normalizeRR(rr.rateSmoothed, baseline.rrResting);
        float rateScore = sigmoid(rrNorm, 2.0f, 0.3f);
        float regularityScore = rr.regularity;
        score += 0.16f * rateScore + 0.10f * regularityScore;
    }
    
    // 体动低（放松的静止状态）
    if (movement.isValid) {
        float moveNorm = normalizeMovement(movement.movementSmoothed);
        float movementScore = 1.0f - sigmoid(moveNorm, 2.0f, 0.15f);
        score += 0.10f * movementScore;
    }
    
    return constrain_value(score, 0.0f, 1.0f);
}

// ==================== 辅助函数 ====================

float SimpleEmotionAnalyzer::sigmoid(float x, float k, float x0) {
    return 1.0f / (1.0f + exp(-k * (x - x0)));
}

float SimpleEmotionAnalyzer::gaussian(float x, float mean, float std) {
    float diff = x - mean;
    return exp(-(diff * diff) / (2 * std * std));
}

void SimpleEmotionAnalyzer::normalizeProbabilities() {
    float sum = 0;
    for (int i = 0; i < 9; i++) {
        sum += emotionProbs[i];
    }
    if (sum > 0) {
        for (int i = 0; i < 9; i++) {
            emotionProbs[i] /= sum;
        }
    }
}

void SimpleEmotionAnalyzer::smoothProbabilities() {
    for (int i = 0; i < 9; i++) {
        // 自适应平滑：变化快→快响应，变化慢→适度抑制
        float diff = fabs(emotionProbs[i] - prevProbs[i]);
        float adaptiveAlpha = (diff > 0.2f) ? 0.6f : 0.25f;
        
        emotionProbs[i] = adaptiveAlpha * emotionProbs[i] + 
                         (1.0f - adaptiveAlpha) * prevProbs[i];
        prevProbs[i] = emotionProbs[i];
    }
}

void SimpleEmotionAnalyzer::calculateDimensions(const HeartRateData& hr, 
                                                 const RespirationData& rr) {
    // 效价：正面到负面
    float positive = emotionProbs[EMOTION_HAPPY] + emotionProbs[EMOTION_EXCITED] + 
                    emotionProbs[EMOTION_RELAXED] + emotionProbs[EMOTION_CALM];
    float negative = emotionProbs[EMOTION_ANXIOUS] + emotionProbs[EMOTION_ANGRY] + 
                    emotionProbs[EMOTION_SAD] + emotionProbs[EMOTION_STRESSED];
    
    lastResult.valence = (positive - negative);
    
    // 唤醒度
    float highArousal = emotionProbs[EMOTION_EXCITED] + emotionProbs[EMOTION_ANXIOUS] + 
                       emotionProbs[EMOTION_ANGRY];
    float lowArousal = emotionProbs[EMOTION_CALM] + emotionProbs[EMOTION_RELAXED] + 
                      emotionProbs[EMOTION_SAD];
    float total = highArousal + lowArousal;
    
    lastResult.arousal = total > 0 ? highArousal / total : 0.5f;
}

void SimpleEmotionAnalyzer::calculateStressLevels(const HeartRateData& hr, 
                                                   const RespirationData& rr, 
                                                   const HRVEstimate& hrv) {
    // 压力水平（标准化归一化模型）
    float hrNorm = 0, hrvNorm = 0, rrNorm = 0;
    
    if (hr.isValid) {
        hrNorm = normalizeHR(hr.bpmSmoothed, baseline.hrResting);
        hrNorm = constrain_value(hrNorm, -1.0f, 1.0f);
    }
    
    if (hrv.isValid) {
        hrvNorm = 1.0f - normalizeHRV(hrv.rmssd);
        hrvNorm = constrain_value(hrvNorm, 0.0f, 1.0f);
    }
    
    if (rr.isValid) {
        rrNorm = normalizeRR(rr.rateSmoothed, baseline.rrResting);
        rrNorm = constrain_value(rrNorm, -1.0f, 1.0f);
    }
    
    // 标准化权重：HR(35%) + HRV(40%) + RR(25%)
    lastResult.stressLevel = constrain_value(
        (0.35f * sigmoid(hrNorm, 2.0f, 0.3f) + 
         0.40f * hrvNorm + 
         0.25f * sigmoid(rrNorm, 2.0f, 0.3f)) * 100.0f,
        0.0f, 100.0f
    );
    
    // 焦虑水平
    lastResult.anxietyLevel = emotionProbs[EMOTION_ANXIOUS] * 70 + 
                              emotionProbs[EMOTION_STRESSED] * 50 +
                              (1.0f - rr.regularity) * 20;
    lastResult.anxietyLevel = constrain_value(lastResult.anxietyLevel, 0.0f, 100.0f);
    
    // 放松水平
    lastResult.relaxationLevel = emotionProbs[EMOTION_RELAXED] * 70 +
                                 emotionProbs[EMOTION_CALM] * 50;
    if (hrv.isValid) {
        lastResult.relaxationLevel += hrv.rmssd / 150.0f * 20;
    }
    lastResult.relaxationLevel = constrain_value(lastResult.relaxationLevel, 0.0f, 100.0f);
}

// ==================== 时间窗口平滑函数 ====================

float SimpleEmotionAnalyzer::getSmoothedHR(const HeartRateData& hr) {
    if (!hr.isValid) return baseline.hrResting;
    
    hrWindow[windowIndex] = hr.bpmSmoothed;
    
    float sum = 0;
    int count = min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; i++) {
        int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += hrWindow[idx];
    }
    
    return count > 0 ? sum / count : hr.bpmSmoothed;
}

float SimpleEmotionAnalyzer::getSmoothedRR(const RespirationData& rr) {
    if (!rr.isValid) return baseline.rrResting;
    
    rrWindow[windowIndex] = rr.rateSmoothed;
    
    float sum = 0;
    int count = min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; i++) {
        int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += rrWindow[idx];
    }
    
    return count > 0 ? sum / count : rr.rateSmoothed;
}

float SimpleEmotionAnalyzer::getSmoothedHRV(const HRVEstimate& hrv) {
    if (!hrv.isValid) return 40.0f;
    
    hrvWindow[windowIndex] = hrv.rmssd;
    
    float sum = 0;
    int count = min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; i++) {
        int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += hrvWindow[idx];
    }
    
    return count > 0 ? sum / count : hrv.rmssd;
}

// ==================== 归一化辅助函数 ====================

float SimpleEmotionAnalyzer::normalizeHR(float hr, float baselineHR) {
    return (hr - baselineHR) / 20.0f;
}

float SimpleEmotionAnalyzer::normalizeRR(float rr, float baselineRR) {
    return (rr - baselineRR) / 4.0f;
}

float SimpleEmotionAnalyzer::normalizeHRV(float hrv) {
    return hrv / 50.0f;
}

float SimpleEmotionAnalyzer::normalizeMovement(float movement) {
    return movement / 100.0f;
}

// ==================== 输出格式化工具实现 ====================

String EmotionOutput::toBrief(const EmotionResult& result) {
    if (!result.isValid) return "检测中...";
    return String(EMOTION_NAMES[result.primaryEmotion]) + " " + 
           String((int)(result.confidence * 100)) + "%";
}

String EmotionOutput::toDetailed(const EmotionResult& result) {
    String output = "";
    
    if (!result.isValid) {
        return "数据无效，无法分析情绪";
    }
    
    output += "情绪: " + String(EMOTION_NAMES[result.primaryEmotion]);
    output += " (" + String((int)(result.confidence * 100)) + "%)";
    output += "\n强度: " + String((int)(result.intensity * 100)) + "%";
    output += "\n\n维度分析:";
    output += "\n  效价: " + String(result.valence, 2);
    output += "\n  唤醒: " + String(result.arousal, 2);
    output += "\n\n指标:";
    output += "\n  压力: " + String((int)result.stressLevel);
    output += "\n  焦虑: " + String((int)result.anxietyLevel);
    output += "\n  放松: " + String((int)result.relaxationLevel);
    output += "\n\n描述: " + String(EMOTION_DESCRIPTIONS[result.primaryEmotion]);
    output += "\n建议: " + String(EMOTION_SUGGESTIONS[result.primaryEmotion]);
    
    return output;
}

String EmotionOutput::toJson(const EmotionResult& result) {
    String json = "{";
    json += "\"emotion\":\"" + String(EMOTION_NAMES[result.primaryEmotion]) + "\",";
    json += "\"confidence\":" + String(result.confidence, 3) + ",";
    json += "\"intensity\":" + String(result.intensity, 3) + ",";
    json += "\"valence\":" + String(result.valence, 3) + ",";
    json += "\"arousal\":" + String(result.arousal, 3) + ",";
    json += "\"stressLevel\":" + String(result.stressLevel, 1) + ",";
    json += "\"anxietyLevel\":" + String(result.anxietyLevel, 1) + ",";
    json += "\"relaxationLevel\":" + String(result.relaxationLevel, 1) + ",";
    json += "\"sympatheticActivity\":" + String(result.sympatheticActivity, 3) + ",";
    json += "\"parasympatheticActivity\":" + String(result.parasympatheticActivity, 3) + ",";
    json += "\"isValid\":" + String(result.isValid ? "true" : "false") + ",";
    json += "\"timestamp\":" + String(result.timestamp);
    json += "}";
    return json;
}

String EmotionOutput::toCsv(const EmotionResult& result, unsigned long timestamp) {
    String csv = "";
    csv += String(timestamp) + ",";
    csv += String(result.primaryEmotion) + ",";
    csv += String(result.confidence, 3) + ",";
    csv += String(result.intensity, 3) + ",";
    csv += String(result.valence, 3) + ",";
    csv += String(result.arousal, 3) + ",";
    csv += String(result.stressLevel, 1) + ",";
    csv += String(result.anxietyLevel, 1) + ",";
    csv += String(result.relaxationLevel, 1);
    return csv;
}
