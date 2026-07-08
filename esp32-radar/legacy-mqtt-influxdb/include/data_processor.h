/**
 * @file data_processor.h
 * @brief 数据处理模块头文件
 * @description 处理雷达传感器提供的心率和呼吸数据
 */

#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include <Arduino.h>
#include "config.h"

// ==================== 心率数据结构 ====================

/**
 * @brief 心率处理后的数据
 */
struct HeartRateData {
    float bpm;                  // 当前心率 BPM
    float bpmSmoothed;          // 平滑后心率
    float bpmMean;              // 平均心率
    float bpmStd;               // 心率标准差
    float bpmMin;               // 最小心率
    float bpmMax;               // 最大心率
    float trend;                // 趋势（上升/下降）
    float variability;          // 心率变异性
    float quality;              // 数据质量 0-1
    bool isValid;               // 数据是否有效
    unsigned long timestamp;    // 时间戳
};

/**
 * @brief 呼吸处理后的数据
 */
struct RespirationData {
    float rate;                 // 呼吸频率 次/分钟
    float rateSmoothed;         // 平滑后呼吸频率
    float rateMean;             // 平均呼吸频率
    float rateStd;              // 呼吸频率标准差
    float regularity;           // 呼吸规律性 0-1
    float variability;          // 呼吸变异性
    float quality;              // 数据质量 0-1
    bool isValid;               // 数据是否有效
    unsigned long timestamp;    // 时间戳
};

/**
 * @brief HRV估算数据
 */
struct HRVEstimate {
    float rmssd;                // 估算的RMSSD
    float sdnn;                 // 估算的SDNN
    float stressIndex;          // 压力指数
    float autonomicBalance;     // 自主神经平衡
    bool isValid;               // 是否有效
};

// ==================== 心率数据处理器 ====================

/**
 * @brief 心率数据处理器
 */
class HeartRateProcessor {
private:
    // 历史数据缓冲
    float* bpmHistory;
    int historySize;
    int historyIndex;
    int historyCount;
    
    // 平滑滤波
    float lastSmoothed;
    float alpha;
    
    // 统计数据
    float bpmSum;
    float bpmSumSq;
    
    // 上一次有效值
    float lastValidBpm;
    unsigned long lastValidTime;
    
public:
    HeartRateProcessor(int histSize = 100);
    ~HeartRateProcessor();
    
    // 添加新的心率数据
    void addData(float bpm, float confidence = 80);
    
    // 获取处理后的数据
    HeartRateData getData();
    
    // 计算HRV估算
    HRVEstimate estimateHRV();
    
    // 重置
    void reset();
    
private:
    void updateStatistics();
    float calculateVariability();
    float calculateTrend();
};

// ==================== 呼吸数据处理器 ====================

/**
 * @brief 呼吸数据处理器
 */
class RespirationProcessor {
private:
    // 历史数据缓冲
    float* rateHistory;
    int historySize;
    int historyIndex;
    int historyCount;
    
    // 平滑滤波
    float lastSmoothed;
    float alpha;
    
    // 上一次有效值
    float lastValidRate;
    unsigned long lastValidTime;
    
public:
    RespirationProcessor(int histSize = 50);
    ~RespirationProcessor();
    
    // 添加新的呼吸数据
    void addData(float rate, float confidence = 80);
    
    // 获取处理后的数据
    RespirationData getData();
    
    // 重置
    void reset();
    
private:
    float calculateRegularity();
    float calculateVariability();
};

// ==================== 综合数据处理 ====================

/**
 * @brief 生理数据综合处理器
 */
class PhysioDataProcessor {
private:
    HeartRateProcessor* hrProcessor;
    RespirationProcessor* rrProcessor;
    
public:
    PhysioDataProcessor();
    ~PhysioDataProcessor();
    
    // 更新数据
    void update(float hr, float rr, float hrConf = 80, float rrConf = 80);
    
    // 获取心率数据
    HeartRateData getHeartRateData();
    
    // 获取呼吸数据
    RespirationData getRespirationData();
    
    // 获取HRV估算
    HRVEstimate getHRVEstimate();
    
    // 重置
    void reset();
};

#endif // DATA_PROCESSOR_H
