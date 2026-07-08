/**
 * @file config.h
 * @brief ESP32情绪分析系统配置文件
 * @description 适配雷达传感器串口数据输入
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== 串口配置 ====================
// 雷达传感器串口 (UART2)
#define RADAR_UART_NUM         UART_NUM_2
#define RADAR_RX_PIN           11          // 接收引脚
#define RADAR_TX_PIN           10          // 发送引脚
#define RADAR_BAUD_RATE        115200      // 波特率

// 调试串口 (USB)
#define DEBUG_UART_NUM         UART_NUM_0
#define DEBUG_BAUD_RATE        115200

// ==================== 数据协议配置 ====================
// 支持的协议格式
#define PROTOCOL_JSON          0           // JSON格式
#define PROTOCOL_BINARY        1           // 二进制格式
#define PROTOCOL_TEXT          2           // 文本格式

// 当前使用的协议
#define CURRENT_PROTOCOL       PROTOCOL_JSON

// JSON数据字段名 (根据您的雷达协议修改)
#define JSON_FIELD_HEART_RATE  "heart_rate"
#define JSON_FIELD_RESPIRATION "respiration_rate"
#define JSON_FIELD_HR_QUALITY  "hr_quality"
#define JSON_FIELD_RR_QUALITY  "rr_quality"

// 文本格式示例: "HR:72,RR:16\n"
#define TEXT_HR_PREFIX         "HR:"
#define TEXT_RR_PREFIX         "RR:"

// 二进制协议帧头帧尾
#define BINARY_FRAME_HEADER    0xAA
#define BINARY_FRAME_TAIL      0x55

// ==================== 采样配置 ====================
#define SAMPLE_RATE_HZ         50          // 数据更新率 50Hz
#define SAMPLE_INTERVAL_MS     20          // 采样间隔 20ms

// 数据缓冲大小
#define RX_BUFFER_SIZE         512         // 串口接收缓冲
#define DATA_BUFFER_SIZE       300         // 数据存储缓冲

// ==================== 心率参数 ====================
// 正常心率范围
#define HR_MIN_NORMAL          40          // 最低正常心率 BPM
#define HR_MAX_NORMAL          200         // 最高正常心率 BPM
#define HR_RESTING_MIN         50          // 静息心率下限
#define HR_RESTING_MAX         90          // 静息心率上限

// 异常检测阈值
#define HR_SUDDEN_CHANGE       30          // 突然变化阈值 BPM
#define HR_STABLE_WINDOW       10          // 稳定性检测窗口（秒）

// ==================== 呼吸参数 ====================
// 正常呼吸频率范围 (次/分钟)
#define RR_MIN_NORMAL          8           // 最低正常呼吸频率
#define RR_MAX_NORMAL          30          // 最高正常呼吸频率
#define RR_RESTING_MIN         12          // 静息呼吸频率下限
#define RR_RESTING_MAX         20          // 静息呼吸频率上限

// 异常检测阈值
#define RR_SUDDEN_CHANGE       10          // 突然变化阈值
#define RR_STABLE_WINDOW       15          // 稳定性检测窗口（秒）

// ==================== HRV参数 ====================
// HRV分析窗口
#define HRV_WINDOW_BEATS       50          // HRV分析所需心跳数
#define HRV_MIN_BEATS          10          // 最小心跳数要求
#define HRV_ESTIMATION_WINDOW  60          // HRV估算窗口（秒）

// HRV指标范围 (RMSSD, ms)
#define HRV_VERY_LOW           20          // 极低HRV
#define HRV_LOW                50          // 低HRV
#define HRV_NORMAL             100         // 正常HRV
#define HRV_HIGH               150         // 高HRV

// ==================== 情绪分类阈值 ====================
// 基于心率的情绪阈值
#define EMOTION_HR_LOW         60          // 低心率阈值
#define EMOTION_HR_NORMAL      80          // 正常心率阈值
#define EMOTION_HR_ELEVATED    100         // 升高心率阈值
#define EMOTION_HR_HIGH        120         // 高心率阈值

// 基于呼吸的情绪阈值
#define EMOTION_RR_SLOW        10          // 缓慢呼吸
#define EMOTION_RR_NORMAL      16          // 正常呼吸
#define EMOTION_RR_FAST        22          // 快速呼吸

// HRV情绪阈值
#define EMOTION_HRV_LOW        30          // 低HRV (压力)
#define EMOTION_HRV_NORMAL     60          // 正常HRV
#define EMOTION_HRV_HIGH       100         // 高HRV (放松)

// ==================== 数据平滑参数 ====================
// 指数移动平均系数
#define EMA_ALPHA_HR           0.15f       // 心率平滑系数
#define EMA_ALPHA_RR           0.10f       // 呼吸平滑系数

// 异常值过滤
#define OUTLIER_FILTER_ENABLED true        // 启用异常值过滤
#define OUTLIER_THRESHOLD      3.0f        // 异常值标准差倍数

// ==================== 情绪定义 ====================
enum EmotionType {
    EMOTION_CALM = 0,       // 平静
    EMOTION_HAPPY,          // 快乐
    EMOTION_EXCITED,        // 兴奋
    EMOTION_ANXIOUS,        // 焦虑
    EMOTION_ANGRY,          // 愤怒
    EMOTION_SAD,            // 悲伤
    EMOTION_STRESSED,       // 压力
    EMOTION_RELAXED,        // 放松
    EMOTION_UNKNOWN         // 未知
};

// 情绪名称字符串
static const char* EMOTION_NAMES[] = {
    "平静",
    "快乐",
    "兴奋",
    "焦虑",
    "愤怒",
    "悲伤",
    "压力",
    "放松",
    "未知"
};

// 情绪描述
static const char* EMOTION_DESCRIPTIONS[] = {
    "心情平静，情绪稳定",
    "心情愉悦，积极正向",
    "情绪高涨，充满活力",
    "感到焦虑或不安",
    "情绪激动，可能有愤怒",
    "情绪低落，可能感到悲伤",
    "压力较大，需要注意休息",
    "身心放松，状态良好",
    "情绪状态未知"
};

// 情绪建议
static const char* EMOTION_SUGGESTIONS[] = {
    "状态良好，继续保持",
    "心情不错，享受当下",
    "精力充沛，注意适当调节",
    "建议深呼吸放松，或进行冥想",
    "建议冷静下来，可以尝试深呼吸",
    "可以尝试与朋友交流，或做些喜欢的事",
    "建议休息放松，避免过度工作",
    "状态很好，继续保持",
    ""
};

// ==================== 调试配置 ====================
#define DEBUG_SERIAL           true        // 启用串口调试
#define DEBUG_VERBOSE          false       // 详细调试信息
#define DEBUG_PROTOCOL         true        // 显示协议解析信息

// ==================== 工具函数宏 ====================
#define constrain_value(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define map_value(x, in_min, in_max, out_min, out_max) \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

// 调试宏
#if DEBUG_SERIAL
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(fmt, ...)
#endif

#if DEBUG_VERBOSE
    #define VERBOSE_PRINT(x) Serial.print(x)
    #define VERBOSE_PRINTLN(x) Serial.println(x)
#else
    #define VERBOSE_PRINT(x)
    #define VERBOSE_PRINTLN(x)
#endif

#endif // CONFIG_H
