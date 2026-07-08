/**
 * @file data_processor.cpp
 * @brief 心率、呼吸和 HRV 数据处理算法实现。
 *
 * 对雷达原始生理数据进行滤波、统计、稳定性和可信度计算。
 */

#include "data_processor.h"
#include <math.h>

// ==================== 心率数据处理器实现 ====================

HeartRateProcessor::HeartRateProcessor(int histSize) 
    : historySize(histSize), historyIndex(0), historyCount(0),
      lastSmoothed(72), alpha(EMA_ALPHA_HR), bpmSum(0), bpmSumSq(0),
      lastValidBpm(72), lastValidTime(0) {
    bpmHistory = new float[historySize];
    memset(bpmHistory, 0, historySize * sizeof(float));
}

HeartRateProcessor::~HeartRateProcessor() {
    delete[] bpmHistory;
}

void HeartRateProcessor::addData(float bpm, float confidence) {
    // 数据验证
    if (bpm < HR_MIN_NORMAL || bpm > HR_MAX_NORMAL) {
        return;
    }
    
    // 突变检测
    if (historyCount > 0) {
        float lastBpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
        if (fabs(bpm - lastBpm) > HR_SUDDEN_CHANGE) {
            // 可能是异常值，进行平滑处理
            bpm = lastSmoothed + (bpm - lastSmoothed) * 0.3f;
        }
    }
    
    // 存储数据
    bpmHistory[historyIndex] = bpm;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) historyCount++;
    
    // 更新统计
    bpmSum += bpm;
    bpmSumSq += bpm * bpm;
    
    // 平滑处理
    lastSmoothed = alpha * bpm + (1.0f - alpha) * lastSmoothed;
    
    lastValidBpm = bpm;
    lastValidTime = millis();
}

HeartRateData HeartRateProcessor::getData() {
    HeartRateData data;
    memset(&data, 0, sizeof(data));
    
    if (historyCount == 0) {
        data.isValid = false;
        return data;
    }
    
    // 当前值
    data.bpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
    data.bpmSmoothed = lastSmoothed;
    
    // 计算均值
    data.bpmMean = bpmSum / historyCount;
    
    // 计算标准差
    float variance = (bpmSumSq / historyCount) - (data.bpmMean * data.bpmMean);
    data.bpmStd = variance > 0 ? sqrt(variance) : 0;
    
    // 计算最值
    data.bpmMin = 300;
    data.bpmMax = 0;
    for (int i = 0; i < historyCount; i++) {
        if (bpmHistory[i] < data.bpmMin) data.bpmMin = bpmHistory[i];
        if (bpmHistory[i] > data.bpmMax) data.bpmMax = bpmHistory[i];
    }
    
    // 计算趋势
    data.trend = calculateTrend();
    
    // 计算变异性
    data.variability = calculateVariability();
    
    // 数据质量评估
    if (millis() - lastValidTime < 3000) {
        data.quality = 0.8f + 0.2f * (1.0f - data.bpmStd / 30.0f);
        data.quality = constrain_value(data.quality, 0.0f, 1.0f);
    } else {
        data.quality = 0.3f;
    }
    
    data.isValid = true;
    data.timestamp = millis();
    
    return data;
}

HRVEstimate HeartRateProcessor::estimateHRV() {
    HRVEstimate hrv;
    memset(&hrv, 0, sizeof(hrv));
    
    if (historyCount < 10) {
        hrv.isValid = false;
        return hrv;
    }
    
    // 从心率变异性估算HRV
    // 这是一个近似方法，真实的HRV需要RR间期数据
    
    // 估算RMSSD：基于心率标准差
    HeartRateData hrData = getData();
    float hrVariability = hrData.bpmStd;
    
    // 经验公式：RMSSD ≈ 心率标准差 * 某个系数
    // 这个系数需要根据实际情况调整
    hrv.rmssd = hrVariability * 8.0f;  // 经验系数
    
    // 估算SDNN
    hrv.sdnn = hrVariability * 10.0f;
    
    // 压力指数
    if (hrv.rmssd > 0) {
        hrv.stressIndex = 1000.0f / hrv.rmssd;
    } else {
        hrv.stressIndex = 50;
    }
    
    // 自主神经平衡（使用 sigmoid 归一化，正常范围 0.3~0.7）
    // rmssd 正常值：20-50ms，映射到 autonomicBalance 0.3-0.7
    if (hrv.rmssd > 0) {
        float normalized = (hrv.rmssd - 20.0f) / 30.0f;  // 归一化到 0~1
        hrv.autonomicBalance = 0.3f + 0.4f * constrain_value(normalized, 0.0f, 1.0f);
    } else {
        hrv.autonomicBalance = 0.5f;  // 默认中性值
    }
    
    hrv.isValid = true;
    return hrv;
}

void HeartRateProcessor::reset() {
    historyIndex = 0;
    historyCount = 0;
    lastSmoothed = 72;
    bpmSum = 0;
    bpmSumSq = 0;
    memset(bpmHistory, 0, historySize * sizeof(float));
}

float HeartRateProcessor::calculateVariability() {
    if (historyCount < 3) return 0;
    
    // 计算相邻值差异的均方根
    float sumSqDiff = 0;
    int count = 0;
    
    for (int i = 1; i < historyCount; i++) {
        int idx1 = (historyIndex - i - 1 + historySize) % historySize;
        int idx2 = (historyIndex - i + historySize) % historySize;
        float diff = bpmHistory[idx1] - bpmHistory[idx2];
        sumSqDiff += diff * diff;
        count++;
    }
    
    return count > 0 ? sqrt(sumSqDiff / count) : 0;
}

float HeartRateProcessor::calculateTrend() {
    if (historyCount < 10) return 0;
    
    // 计算前半和后半的均值差
    int half = historyCount / 2;
    float firstHalf = 0, secondHalf = 0;
    
    for (int i = 0; i < half; i++) {
        int idx = (historyIndex - historyCount + i + historySize) % historySize;
        firstHalf += bpmHistory[idx];
    }
    firstHalf /= half;
    
    for (int i = historyCount - half; i < historyCount; i++) {
        int idx = (historyIndex - historyCount + i + historySize) % historySize;
        secondHalf += bpmHistory[idx];
    }
    secondHalf /= half;
    
    return secondHalf - firstHalf;
}

// ==================== 呼吸数据处理器实现 ====================

RespirationProcessor::RespirationProcessor(int histSize) 
    : historySize(histSize), historyIndex(0), historyCount(0),
      lastSmoothed(16), alpha(EMA_ALPHA_RR), lastValidRate(16), lastValidTime(0) {
    rateHistory = new float[historySize];
    memset(rateHistory, 0, historySize * sizeof(float));
}

RespirationProcessor::~RespirationProcessor() {
    delete[] rateHistory;
}

void RespirationProcessor::addData(float rate, float confidence) {
    // 数据验证
    if (rate < RR_MIN_NORMAL || rate > RR_MAX_NORMAL) {
        return;
    }
    
    // 突变检测
    if (historyCount > 0) {
        float lastRate = rateHistory[(historyIndex - 1 + historySize) % historySize];
        if (fabs(rate - lastRate) > RR_SUDDEN_CHANGE) {
            rate = lastSmoothed + (rate - lastSmoothed) * 0.3f;
        }
    }
    
    // 存储数据
    rateHistory[historyIndex] = rate;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) historyCount++;
    
    // 平滑处理
    lastSmoothed = alpha * rate + (1.0f - alpha) * lastSmoothed;
    
    lastValidRate = rate;
    lastValidTime = millis();
}

RespirationData RespirationProcessor::getData() {
    RespirationData data;
    memset(&data, 0, sizeof(data));
    
    if (historyCount == 0) {
        data.isValid = false;
        return data;
    }
    
    // 当前值
    data.rate = rateHistory[(historyIndex - 1 + historySize) % historySize];
    data.rateSmoothed = lastSmoothed;
    
    // 计算均值
    float sum = 0;
    for (int i = 0; i < historyCount; i++) {
        sum += rateHistory[i];
    }
    data.rateMean = sum / historyCount;
    
    // 计算标准差
    float sumSq = 0;
    for (int i = 0; i < historyCount; i++) {
        float diff = rateHistory[i] - data.rateMean;
        sumSq += diff * diff;
    }
    data.rateStd = sqrt(sumSq / historyCount);
    
    // 计算规律性
    data.regularity = calculateRegularity();
    
    // 计算变异性
    data.variability = calculateVariability();
    
    // 数据质量评估
    if (millis() - lastValidTime < 5000) {
        data.quality = 0.7f + 0.3f * data.regularity;
        data.quality = constrain_value(data.quality, 0.0f, 1.0f);
    } else {
        data.quality = 0.3f;
    }
    
    data.isValid = true;
    data.timestamp = millis();
    
    return data;
}

void RespirationProcessor::reset() {
    historyIndex = 0;
    historyCount = 0;
    lastSmoothed = 16;
    memset(rateHistory, 0, historySize * sizeof(float));
}

float RespirationProcessor::calculateRegularity() {
    if (historyCount < 5) return 0.8f;
    
    // 计算变异系数
    float sum = 0;
    for (int i = 0; i < historyCount; i++) {
        sum += rateHistory[i];
    }
    float mean = sum / historyCount;
    
    float sumSq = 0;
    for (int i = 0; i < historyCount; i++) {
        float diff = rateHistory[i] - mean;
        sumSq += diff * diff;
    }
    float std = sqrt(sumSq / historyCount);
    
    float cv = (mean > 0) ? std / mean : 0;
    
    // 转换为规律性（CV越小，规律性越高）
    return constrain_value(1.0f - cv * 3.0f, 0.0f, 1.0f);
}

float RespirationProcessor::calculateVariability() {
    if (historyCount < 3) return 0;
    
    float sumDiff = 0;
    for (int i = 1; i < historyCount; i++) {
        int idx1 = (historyIndex - i - 1 + historySize) % historySize;
        int idx2 = (historyIndex - i + historySize) % historySize;
        sumDiff += fabs(rateHistory[idx1] - rateHistory[idx2]);
    }
    
    return sumDiff / (historyCount - 1);
}

// ==================== 综合数据处理器实现 ====================

PhysioDataProcessor::PhysioDataProcessor() {
    hrProcessor = new HeartRateProcessor(100);
    rrProcessor = new RespirationProcessor(50);
}

PhysioDataProcessor::~PhysioDataProcessor() {
    delete hrProcessor;
    delete rrProcessor;
}

void PhysioDataProcessor::update(float hr, float rr, float hrConf, float rrConf) {
    hrProcessor->addData(hr, hrConf);
    rrProcessor->addData(rr, rrConf);
}

HeartRateData PhysioDataProcessor::getHeartRateData() {
    return hrProcessor->getData();
}

RespirationData PhysioDataProcessor::getRespirationData() {
    return rrProcessor->getData();
}

HRVEstimate PhysioDataProcessor::getHRVEstimate() {
    return hrProcessor->estimateHRV();
}

void PhysioDataProcessor::reset() {
    hrProcessor->reset();
    rrProcessor->reset();
}
