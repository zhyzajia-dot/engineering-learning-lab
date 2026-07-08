/**
 * @file emotion_analyzer_simple.h
 * @brief 简化版情绪分析器的类型和对外接口。
 *
 * 根据生命体征、体动和个人基线估算当前情绪状态。
 */

#ifndef EMOTION_ANALYZER_SIMPLE_H
#define EMOTION_ANALYZER_SIMPLE_H

#include <Arduino.h>
#include "config.h"
#include "data_processor.h"

// ==================== 情绪分析结果 ====================

/**
 * @brief 情绪分析结果
 */
struct EmotionResult {
    // 主要情绪
    EmotionType primaryEmotion;
    EmotionType secondaryEmotion;  // 次要情绪
    float confidence;           // 置信度 0-1
    float intensity;            // 强度 0-1
    
    // 情绪维度
    float valence;              // 效价 -1到+1
    float arousal;              // 唤醒度 0-1
    
    // 压力评估
    float stressLevel;          // 压力水平 0-100
    float anxietyLevel;         // 焦虑水平 0-100
    float relaxationLevel;      // 放松水平 0-100
    
    // 自主神经评估
    float sympatheticActivity;  // 交感神经活动
    float parasympatheticActivity; // 副交感神经活动
    
    // 数据有效性
    bool isValid;
    unsigned long timestamp;
};

// ==================== 用户基线 ====================

/**
 * @brief 用户生理基线
 */
struct UserBaseline {
    float hrResting;            // 静息心率
    float hrMin;                // 最小心率
    float hrMax;                // 最大心率
    float rrResting;            // 静息呼吸频率
    bool isCalibrated;
    unsigned long calibrationTime;
    
    // 冷启动缓冲
    static const int COLD_START_SAMPLES = 15;  // 约30秒数据（2秒/样本）
    float coldStartHrSum;
    float coldStartRrSum;
    int coldStartCount;
    bool isColdStarting;
};

/**
 * @brief 体动数据
 */
struct BodyMovementData {
    uint8_t movement;           // 体动参数 0-100
    float movementSmoothed;     // 平滑后体动
    float movementMean;         // 平均体动
    float movementStd;          // 体动标准差
    float activityLevel;        // 活动水平 0-1
    bool isValid;               // 数据是否有效
    unsigned long timestamp;    // 时间戳
};

// ==================== 情绪分析器 ====================

/**
 * @brief 简化版情绪分析器
 */
class SimpleEmotionAnalyzer {
private:
    // 用户基线
    UserBaseline baseline;
    
    // 上一次结果
    EmotionResult lastResult;
    
    // 情绪概率缓冲（用于平滑）
    float emotionProbs[9];
    float smoothingFactor;
    float prevProbs[9];
    
    // 历史情绪记录
    EmotionType* emotionHistory;
    int historySize;
    int historyIndex;
    int historyCount;
    
    // 时间窗口缓冲（用于输入数据平滑）
    static const int WINDOW_SIZE = 15;
    float hrWindow[WINDOW_SIZE];
    float rrWindow[WINDOW_SIZE];
    float hrvWindow[WINDOW_SIZE];
    int windowIndex;
    int windowCount;
    
public:
    SimpleEmotionAnalyzer(int histSize = 30);
    ~SimpleEmotionAnalyzer();
    
    // 分析情绪
    EmotionResult analyze(const HeartRateData& hrData, 
                         const RespirationData& rrData,
                         const HRVEstimate& hrvData,
                         const BodyMovementData& movementData);
    
    // 设置基线
    void setBaseline(const UserBaseline& bl);
    UserBaseline getBaseline() const { return baseline; }
    
    // 自动校准基线
    void calibrateBaseline(const HeartRateData& hrData, 
                          const RespirationData& rrData,
                          const BodyMovementData& movementData);
    
    // 设置平滑因子
    void setSmoothing(float factor) { smoothingFactor = constrain_value(factor, 0.0f, 1.0f); }
    
    // 获取最近的主要情绪
    EmotionType getRecentDominantEmotion(int seconds = 60);
    
    // 重置
    void reset();
    
private:
    // 情绪规则匹配
    float calculateCalmScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateHappyScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateExcitedScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateAnxiousScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateAngryScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateSadScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateStressedScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    float calculateRelaxedScore(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv, const BodyMovementData& movement);
    
    // 辅助函数
    float sigmoid(float x, float k = 1.0f, float x0 = 0.0f);
    float gaussian(float x, float mean, float std);
    void normalizeProbabilities();
    void smoothProbabilities();
    void calculateDimensions(const HeartRateData& hr, const RespirationData& rr);
    void calculateStressLevels(const HeartRateData& hr, const RespirationData& rr, const HRVEstimate& hrv);
    
    // 时间窗口平滑
    float getSmoothedHR(const HeartRateData& hr);
    float getSmoothedRR(const RespirationData& rr);
    float getSmoothedHRV(const HRVEstimate& hrv);
    
    // 归一化辅助函数（统一尺度）
    float normalizeHR(float hr, float baseline);
    float normalizeRR(float rr, float baseline);
    float normalizeHRV(float hrv);
    float normalizeMovement(float movement);
};

// ==================== 输出格式化工具 ====================

/**
 * @brief 情绪结果格式化工具
 */
class EmotionOutput {
public:
    // 简洁输出
    static String toBrief(const EmotionResult& result);
    
    // 详细输出
    static String toDetailed(const EmotionResult& result);
    
    // JSON输出
    static String toJson(const EmotionResult& result);
    
    // CSV输出
    static String toCsv(const EmotionResult& result, unsigned long timestamp);
};

#endif // EMOTION_ANALYZER_SIMPLE_H
