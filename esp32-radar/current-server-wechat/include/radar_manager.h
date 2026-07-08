/**
 * @file radar_manager.h
 * @brief R60ABD1 睡眠雷达、雷达串口和雷达 BLE 服务的公共接口。
 *
 * 声明雷达数据结构、解析状态、BLE 命令处理及数据发送函数。
 */
#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include "ble_tlv_protocol.h"
#include "wifi_manager.h"

// Radar Data Service
#define RADAR_DATA_SERVICE_UUID      "a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1a"
#define RADAR_STREAM_CHAR_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a1"
#define RADAR_STATUS_CHAR_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a2"

// Device Config Service
#define DEVICE_CONFIG_SERVICE_UUID   "a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1b"
#define DEVICE_COMMAND_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26b1"
#define DEVICE_RESULT_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26b2"
#define DEVICE_INFO_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26b3"
#define UART_RX_BUFFER_SIZE 4096 // UART接收缓冲区大小
#define QUEUE_SIZE 200 // 队列大小（增加到200以防止溢出）
#define TASK_STACK_SIZE 8192 // 任务堆栈大小

constexpr unsigned long SLEEP_ANALYSIS_SNAPSHOT_TTL_MS = 15000;

#define FRAME_HEADER1 0x53  // 帧头字节1
#define FRAME_HEADER2 0x59  // 帧头字节2
#define FRAME_TAIL1   0x54  // 帧尾字节1
#define FRAME_TAIL2   0x43  // 帧尾字节2

// 控制字定义
#define CTRL_PRESENCE    0x80  // 人体存在检测
#define CTRL_BREATH      0x81  // 呼吸检测
#define CTRL_SLEEP       0x84  // 睡眠监测
#define CTRL_HEARTRATE   0x85  // 心率监测

// 命令字定义
#define CMD_REPORT       0x80  // 主动上报
#define CMD_QUERY        0x81  // 查询命令
#define CMD_SET          0x82  // 设置命令

// 定义R60ABD1数据结构
typedef struct {
    uint8_t present;      // 有人/无人状态 (DP1)
    uint16_t distance;    // 人体距离 (DP3) 单位cm
    uint8_t heartRate;    // 心率 (DP6) 单位BPM
    uint8_t breathRate;   // 呼吸率 (DP8) 单位次/分钟
    uint8_t heartWave;    // 心率波形 (DP7) 数值+128
    uint8_t breathWave;   // 呼吸波形 (DP10) 数值+128
    uint8_t sleepState;   // 睡眠状态 (DP12)
    uint32_t sleepTime;   // 睡眠时长 (DP13) 单位秒
    uint8_t sleepScore;   // 睡眠质量评分 (DP14)
    uint8_t bedEntry;     // 入床/离床状态 (DP11)
    uint8_t abnormal;     // 异常状态 (DP18)
} R60ABD1Data; // R60ABD1雷达数据结构体

typedef struct { // 传感器数据结构体
    float breath_rate; // 呼吸率
    float heart_rate; // 心率
    uint8_t breath_valid; // 呼吸率有效标志
    uint8_t heart_valid; // 心率有效标志
    uint8_t presence; // 存在状态
    uint8_t motion; // 运动状态
    int heartbeat_waveform; // 心跳波形
    int breathing_waveform; // 呼吸波形
    uint16_t distance; // 距离
    uint8_t body_movement; // 身体运动（1~100）
    uint8_t breath_status; // 呼吸状态
    uint8_t sleep_state; // 睡眠状态
    uint32_t sleep_time; // 睡眠时长
    uint8_t sleep_score; // 睡眠评分
    uint8_t sleep_grade; // 睡眠等级
    uint8_t bed_entry; // 入床状态
    uint8_t abnormal_state; // 异常状态
    uint8_t avg_heart_rate; // 平均心率
    uint8_t avg_breath_rate; // 平均呼吸率
    uint8_t turn_count; // 翻身次数
    uint8_t large_move_ratio; // 大幅运动比例
    uint8_t small_move_ratio; // 小幅运动比例
    int16_t pos_x; // X坐标
    int16_t pos_y; // Y坐标
    int16_t pos_z; // Z坐标
    int8_t breath_waveform[5]; // 呼吸波形数组
    int8_t heart_waveform[5]; // 心跳波形数组
    uint16_t deep_sleep_time; // 深度睡眠时长
    uint16_t light_sleep_time; // 浅度睡眠时长
    uint16_t awake_time; // 清醒时长
    uint16_t sleep_total_time; // 总睡眠时长
    uint8_t deep_sleep_ratio; // 深度睡眠比例
    uint8_t light_sleep_ratio; // 浅度睡眠比例
    uint8_t awake_ratio; // 清醒比例
    uint8_t turnover_count; // 翻身计数
    uint8_t struggle_alert; // 挣扎警报
    uint8_t no_one_alert; // 无人警报
    uint8_t bed_status; // 床状态
    uint8_t bed_Out_Time; // 离床时间
    uint8_t apnea_count; // 呼吸暂停次数
} SensorData; // 传感器数据结构体

// SleepAnalyzer 算法输出快照（用于跨任务传递）
typedef struct {
    // 当前睡眠状态（算法定义）
    int algorithm_state;      // 0-无人, 1-在床, 2-清醒, 3-浅睡, 4-深睡, 5-REM, 6-离床, 7-起床, 8-会话结束
    float current_sleepiness; // 当前困倦度

    // 睡眠统计
    unsigned long total_sleep_time;     // 总睡眠时间
    unsigned long deep_sleep_time;      // 深睡时间
    unsigned long light_sleep_time;     // 浅睡时间
    unsigned long rem_sleep_time;       // REM时间
    unsigned long awake_time;           // 清醒时间
    unsigned long out_of_bed_time;      // 离床时间
    unsigned long sleep_latency;        // 入睡潜伏期
    int wake_count;                     // 醒来次数
    int sleep_cycles;                   // 睡眠周期数
    unsigned long session_start_time;   // 会话开始时间
    unsigned long sleep_start_time;     // 入睡时间
    unsigned long last_wake_time;       // 最后醒来时间

    // 睡眠评分
    float duration_score;      // 时长评分
    float deep_score;          // 深睡评分
    float continuity_score;    // 连续性评分
    float physiology_score;    // 生理指标评分
    float latency_score;       // 入睡潜伏期评分
    float efficiency_score;    // 睡眠效率评分
    float cycle_score;         // 周期评分
    float total_score;         // 总分

    // 睡眠周期
    int cycle_count;              // 当前周期数
    unsigned long cycle_start_time; // 当前周期开始时间
    bool in_deep_phase;          // 是否在深睡阶段
    bool in_rem_phase;           // 是否在REM阶段

    unsigned long updated_at; // 更新时间
    bool valid; // 数据有效性标志
} SleepAnalysisSnapshot;

typedef struct { // 相位数据结构体
    int heartbeat_waveform; // 心跳波形
    int breathing_waveform; // 呼吸波形
} PhaseData; // 相位数据结构体

typedef struct { // 生命体征数据结构体
    float heart_rate; // 心率
    float breath_rate; // 呼吸率
    uint8_t presence; // 存在状态
    uint8_t motion; // 运动状态
    uint16_t distance; // 距离
    uint8_t sleep_state; // 睡眠状态
    uint8_t sleep_score; // 睡眠评分
    uint8_t body_movement; // 身体运动
    uint8_t breath_status; // 呼吸状态
    uint32_t sleep_time; // 睡眠时长
    uint8_t bed_entry; // 入床状态
    uint8_t abnormal_state; // 异常状态
    uint8_t avg_heart_rate; // 平均心率
    uint8_t avg_breath_rate; // 平均呼吸率
    uint8_t turn_count; // 翻身次数
    uint8_t large_move_ratio; // 大幅运动比例
    uint8_t small_move_ratio; // 小幅运动比例
    int16_t pos_x; // X坐标
    int16_t pos_y; // Y坐标
    int16_t pos_z; // Z坐标
    uint16_t deep_sleep_time; // 深度睡眠时长
    uint16_t light_sleep_time; // 浅度睡眠时长
    uint16_t awake_time; // 清醒时长
    uint16_t sleep_total_time; // 总睡眠时长
    uint8_t deep_sleep_ratio; // 深度睡眠比例
    uint8_t light_sleep_ratio; // 浅度睡眠比例
    uint8_t awake_ratio; // 清醒比例
    uint8_t turnover_count; // 翻身计数
    uint8_t struggle_alert; // 挣扎警报
    uint8_t no_one_alert; // 无人警报
    uint8_t bed_status; // 床状态
    uint8_t apnea_count; // 呼吸暂停次数
    int heartbeat_waveform; // 心跳波形
    int breathing_waveform; // 呼吸波形
} VitalData; // 生命体征数据结构体

typedef struct { // 上次发送数据结构体
    float heart_rate; // 心率
    float breath_rate; // 呼吸率
    uint8_t presence; // 存在状态
    uint8_t motion; // 运动状态
    uint8_t sleep_state; // 睡眠状态
} LastSentData; // 上次发送数据结构体

class BLEFlowController { // BLE流控制器类
private:
    size_t maxBytesPerSecond; // 最大每秒发送字节数
    size_t bytesSent; // 已发送字节数
    unsigned long lastResetTime; // 上次重置时间
    unsigned long lastSendTime; // 上次发送时间
    
public:
    BLEFlowController(size_t maxBps);
    bool canSend(size_t dataSize);
    bool check();
    void recordSend(size_t dataSize);
    void reset();
};

extern SensorData sensorData; // 传感器数据
extern SleepAnalysisSnapshot sleepAnalysisSnapshot; // 睡眠分析快照（全局缓存）
void updateSleepAnalysisSnapshot(const SleepAnalysisSnapshot& snapshot);
bool getFreshSleepAnalysisSnapshot(SleepAnalysisSnapshot& snapshot);
bool isSleepAnalysisSnapshotFresh();
void invalidateSleepAnalysisSnapshotIfStale();
extern HardwareSerial mySerial1; // 硬件串口1
extern QueueHandle_t phaseDataQueue; // 相位数据队列
extern QueueHandle_t vitalDataQueue; // 生命体征数据队列
extern QueueHandle_t uartQueue; // UART数据队列

/**
 * @brief BLE命令消息结构
 * 
 * 用于在 onWrite 回调和命令处理任务之间传递命令数据。
 * 
 * 限制：
 * - raw 缓冲区固定 256 字节
 * - 如果客户端发送超过 256 字节的命令，固件会拒绝并返回 ERR_PROTO_FRAME_TOO_LARGE 错误
 * - 不会静默截断，避免后续 CRC 错误或解析失败
 */
typedef struct {
    uint8_t raw[256];// 原始数据缓冲区（最大 256 字节）
    size_t len;      // 实际数据长度
} BleCommandMessage;

extern TaskHandle_t bleSendTaskHandle; // BLE发送任务句柄
extern TaskHandle_t vitalSendTaskHandle; // 生命体征发送任务句柄
extern TaskHandle_t uartProcessTaskHandle; // UART处理任务句柄
extern BLEServer* pServer; // BLE服务器指针

// Radar Data Service
extern BLEService* radarDataService;// 雷达数据服务
extern BLECharacteristic* radarStreamCharacteristic;// 雷达数据流特征
extern BLECharacteristic* radarStatusCharacteristic;// 雷达状态特征

// Device Config Service
extern BLEService* deviceConfigService;// 设备配置服务
extern BLECharacteristic* deviceCommandCharacteristic;// 设备命令特征
extern BLECharacteristic* deviceResultCharacteristic; // 设备结果特征
extern BLECharacteristic* deviceInfoCharacteristic; // 设备信息特征


extern bool deviceConnected; // 设备连接状态
extern bool oldDeviceConnected; // 旧设备连接状态
extern BleProto::FrameParser bleFrameParser; // BLE帧解析器
extern uint8_t bleSequenceCounter; // BLE序列号计数器
extern QueueHandle_t bleCommandQueue; // BLE命令队列

/*
 * BLE 分包模式开关
 *
 * BLE_FIXED_20_BYTE_MODE = 1 (当前)：
 *   固定 20 字节分包，兼容所有 Android/iOS 小程序，不依赖 MTU 协商。
 *   适合配网、状态上报等场景，吞吐低但稳定。
 *
 * BLE_FIXED_20_BYTE_MODE = 0 (未来优化)：
 *   启用 MTU 协商，协商成功后使用 TARGET_ATT_MTU - ATT_HEADER_SIZE 作为分包大小。
 *   需要小程序端同步支持 setBLEMTU，且经过 Android/iOS 双端验证后再切换。
 *
 * 切换方式：将下面的 1 改为 0 即可，所有相关代码已通过条件编译自动切换。
 */
#define BLE_FIXED_20_BYTE_MODE 1

// BLE MTU 协商相关常量
static constexpr uint16_t DEFAULT_ATT_MTU = 23;   // BLE 默认 ATT MTU
static constexpr uint16_t TARGET_ATT_MTU  = 247;  // 目标 ATT MTU（ESP32 BLE 库上限）
static constexpr size_t   ATT_HEADER_SIZE = 3;    // ATT 头部（opcode 1B + handle 2B）
static constexpr size_t   FALLBACK_PAYLOAD = 20;  // 固定兼容模式分包大小

extern size_t g_blePayloadSize;  // 全局BLE有效载荷大小（当前固定为 FALLBACK_PAYLOAD）
extern bool continuousSendEnabled; // 持续发送使能标志
extern unsigned long continuousSendInterval; // 持续发送间隔
extern bool radarSleepQueryEnabled;   // 雷达睡眠/综合状态查询开关（0x8D/0x90）
extern unsigned long lastSleepDataTime; // 上次发送睡眠数据时间
extern BLEFlowController bleFlow; // BLE流控制器
extern unsigned long lastSensorUpdate; // 上次传感器更新时间
extern LastSentData lastSentData; // 上次发送的数据
extern unsigned long lastCheckTime; // 上次检测时间

extern const unsigned long SENSOR_TIMEOUT; // 传感器超时时间

extern Preferences preferences; // Flash存储对象
extern WiFiManager wifiManager; // WiFi管理器对象

// BLE请求上下文，用于跟踪异步命令的seq
struct BleRequestContext {
    uint8_t seq = 0;
    bool active = false;
};
extern BleRequestContext wifiConfigRequestCtx;// WiFi配置请求上下文
extern BleRequestContext wifiScanRequestCtx;  // WiFi扫描请求上下文
extern BleRequestContext savedNetworksRequestCtx; // 已保存网络请求上下文

void initRadarManager();// 初始化雷达管理器
void initR60ABD1();// 初始化R60ABD1雷达
bool parseR60ABD1Frame(uint8_t *frame, uint16_t frameLen);// 解析R60ABD1雷达数据帧
int16_t parseSignedCoordinate(uint16_t raw_value);// 解析有符号坐标值
void updateSensorData(const R60ABD1Data& radarData); // 更新传感器数据
void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value);// 发送雷达命令
void IRAM_ATTR serialRxCallback();// UART接收回调函数

void bleSendTask(void *parameter);// BLE发送任务函数
void vitalSendTask(void *parameter);// 生命体征发送任务函数
void radarDataTask(void *parameter);// 雷达数据处理任务函数
void uartProcessTask(void *parameter);// UART处理任务函数


// ---- BLE 数据发送接口 ----
void sendFrameToBLE(const BleProto::Frame& frame, BLECharacteristic* pChar);
void sendRawEchoResponse(const String& rawData);

// ---- BLE 统一错误响应函数 ----
/**
 * @brief 发送即时命令错误响应
 * 用于一问一答的即时命令失败场景，不包含 TLV_STATE/TLV_STEP
 * @param respCmd 响应命令码（如 CMD_QUERY_STATUS）
 * @param seq 请求的序列号
 * @param resultCode 错误码（如 ERR_PROTO_PARAM_INVALID）
 * @param errorMessage 错误详细说明（可选）
 */
void sendCommandErrorResponse(uint8_t respCmd, uint8_t seq, uint8_t resultCode);

/**
 * @brief 发送设备状态推送（b3 通道）
 * 用于设备状态变化时主动推送，与命令响应解耦
 * @param status 设备状态码（见 BleProto::DeviceStatus）
 * @param message 可选的状态描述
 */
void sendDeviceStatusToBLE(uint8_t status);

/**
 * @brief 推送设备状态变化（b3 通道，带去重）
 * 只有状态真正变化时才推送，避免重复推送
 * @param status 设备状态码（见 BleProto::DeviceStatus）
 */
void pushDeviceStatusIfChanged(uint8_t status);

// ---- BLE 命令处理函数 (被 processBLEConfig 分派) ----
bool processEchoRequest(const BleProto::Frame& frame);         // CMD_PING (0x01)
bool processQueryStatus(const BleProto::Frame& frame);         // CMD_QUERY_STATUS (0x10)
bool processQueryRadarData(const BleProto::Frame& frame);      // CMD_QUERY_RADAR (0x12)
bool processStartContinuousSend(const BleProto::Frame& frame); // CMD_START_CONTINUOUS (0x14)
bool processStopContinuousSend(const BleProto::Frame& frame);  // CMD_STOP_CONTINUOUS (0x16)
bool processWiFiConfigCommand(const BleProto::Frame& frame);   // CMD_WIFI_CONFIG (0x22)
bool processScanWiFi(const BleProto::Frame& frame);            // CMD_WIFI_SCAN (0x20)
bool processGetSavedNetworks(const BleProto::Frame& frame);    // CMD_GET_SAVED_WIFI (0x24)
bool processDeleteSavedNetwork(const BleProto::Frame& frame);  // CMD_DELETE_SAVED_WIFI (0x26)
bool processRadarSleepQuery(const BleProto::Frame& frame);  // CMD_RADAR_SLEEP_QUERY (0x17)
void processBLEConfig();                                        // 主分派入口

// ---- WiFi 异步结果推送 (纯TLV) ----
void sendWiFiConfigResultToBLE(uint8_t resultCode,
                              const String& ssid = "", const String& ipAddress = "");
void sendWiFiScanResultToBLE(uint8_t resultCode,
                            const std::vector<WiFiScanResult>& networks = {});
void sendSavedNetworksResultToBLE(bool success, const std::vector<WiFiScanResult>& networks = {});

bool sendDailyDataToInfluxDB(String dailyDataLine);// 发送每日数据到InfluxDB，参数是符合InfluxDB Line Protocol格式的字符串
bool sendSleepDataToInfluxDB(bool allowSessionEnd = false);// 发送睡眠数据到InfluxDB，函数内部会构建符合InfluxDB Line Protocol格式的数据字符串并发送

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override;
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override;
    void onDisconnect(BLEServer* pServer) override;
    void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic);
};

#endif
