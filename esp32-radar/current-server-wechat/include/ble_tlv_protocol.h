/**
 * @file ble_tlv_protocol.h
 * @brief BLE 二进制 TLV 协议的公共定义和编解码接口。
 *
 * 小程序与设备之间的命令、响应和字段格式统一在此声明。
 */
#ifndef BLE_TLV_PROTOCOL_H
#define BLE_TLV_PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>//用于存储字节数据的动态数组

namespace BleProto {

static const uint8_t SOF1 = 0xAA;//帧头
static const uint8_t SOF2 = 0x55;//帧头
static const uint8_t VERSION = 0x01;//协议版本

// ==================== 命令码定义 ====================
// 范围分配：系统 0x01-0x0F | 雷达/传感器 0x10-0x1F | WiFi 0x20-0x2F | 设备 0x30-0x3F | 通用 0x7E-0x7F
//
// 设计规则：
// - 一问一答命令：REQ 和 RESP 共用同一个命令码，方向由 GATT 通道区分
//   客户端写 b1（CMD_XXX）→ 设备 notify b2（CMD_XXX，seq 原样返回）
// - 主动推送命令：保留独立命令码，不与请求响应复用
//   原因：主动推送不由请求触发，seq 不是请求 seq，客户端不能按 seq 匹配
enum Command : uint8_t {
    // --- 系统命令（一问一答）---
    CMD_PING = 0x01,                  // Ping 请求/响应（合并）

    // --- 雷达/传感器命令（一问一答）---
    CMD_QUERY_STATUS = 0x10,          // 查询设备状态 请求/响应（合并）
    CMD_QUERY_RADAR = 0x12,           // 查询雷达数据 请求/响应（合并）
    CMD_START_CONTINUOUS = 0x14,      // 启动连续推送 请求/响应（合并）
    CMD_STOP_CONTINUOUS = 0x16,       // 停止连续推送 请求/响应（合并）
    CMD_RADAR_SLEEP_QUERY = 0x17,     // 雷达睡眠/综合状态查询开关 请求/响应（合并）
    
    // --- 主动推送命令（独立命令码，不与请求响应复用）---
    CMD_CONTINUOUS_PUSH = 0x18,       // 连续数据主动推送（a1 通道）
    CMD_DEVICE_INFO_PUSH = 0x19,      // 设备信息主动推送（b3 通道）
    CMD_RADAR_STATUS_PUSH = 0x1A,     // 雷达状态主动推送（a2 通道）

    // --- WiFi 命令（一问一答）---
    CMD_WIFI_SCAN = 0x20,             // WiFi 扫描 请求/响应（合并）
    CMD_WIFI_CONFIG = 0x22,           // WiFi 配置 请求/响应（合并）
    CMD_GET_SAVED_WIFI = 0x24,        // 获取已保存 WiFi 请求/响应（合并）
    CMD_DELETE_SAVED_WIFI = 0x26,     // 删除指定已保存 WiFi 请求/响应（合并）

    // --- 通用命令 ---
    CMD_ERROR_RESP = 0x7E             // 协议层错误响应（无对应请求）
};

// ==================== TLV 类型码定义 ====================
enum TlvType : uint8_t {
    // --- 设备信息 (0x01-0x0F) ---
    TLV_RESULT_CODE = 0x02,//结果码
    TLV_TIMESTAMP = 0x04,//时间戳
    TLV_PROTOCOL_VERSION = 0x05,//协议版本
    TLV_DEVICE_SN = 0x06,//设备序列号 uint64（仅当存在时发送，不用 MAC 替代）
    TLV_FIRMWARE_VERSION = 0x07,//固件版本 string
    TLV_DEVICE_TYPE = 0x08,//设备类型 string
    TLV_MAC_ADDRESS = 0x09,//MAC地址 string

    // --- 雷达/传感器数据 (0x10-0x1F) ---
    TLV_HEART_RATE_X10 = 0x10,//心率（x10）
    TLV_BREATH_RATE_X10 = 0x11,//呼吸率（x10）
    TLV_PRESENCE = 0x12,//存在
    TLV_MOTION = 0x13,//运动
    TLV_SLEEP_STATE = 0x14,//睡眠状态
    TLV_DISTANCE_CM = 0x15,//距离（cm）
    TLV_POS_X_MM = 0x16,//X坐标（mm）
    TLV_POS_Y_MM = 0x17,//Y坐标（mm）
    TLV_POS_Z_MM = 0x18,//Z坐标（mm）
    TLV_BODY_MOVEMENT = 0x19, // 身体移动状态


    // --- WiFi 相关 (0x20-0x2F) ---
    TLV_SSID = 0x20,//SSID
    TLV_PASSWORD = 0x21,//密码
    TLV_WIFI_COUNT = 0x22,//WiFi数量
    TLV_WIFI_ITEM = 0x23,//WiFi项
    TLV_RSSI = 0x24,//RSSI
    TLV_SECURITY = 0x25,//安全类型（uint8，见WifiSecurityType枚举）

    // --- 控制参数 (0x30-0x3F) ---
    TLV_INTERVAL_MS = 0x31,           // 间隔时间（毫秒）
    TLV_RADAR_SLEEP_ENABLED = 0x32,   // 雷达睡眠查询开关 uint8（0=关闭，1=开启）
    TLV_DEVICE_STATUS = 0x33,         // 设备状态 uint8（用于 b3 推送）
    TLV_WIFI_STATUS = 0x34,          // WiFi 状态 uint8（用于查询响应）
    TLV_MQTT_STATUS = 0x35,          // MQTT 状态 uint8（用于查询响应）
    TLV_RADAR_SLEEP_STATUS = 0x36,   // 雷达睡眠查询状态 uint8（用于查询响应）
    


    // --- 通用消息 (0x40-0x4F) ---
    TLV_IP_ADDRESS = 0x41,//IP地址
    TLV_WIFI_CONFIGURED = 0x42,//WiFi配置
    TLV_WIFI_CONNECTED = 0x43,//WiFi连接
    TLV_ECHO_CONTENT = 0x44,//回显内容
    
    // --- 波形数据 (0x60-0x6F) ---
    TLV_HEART_WAVEFORM = 0x60,  // 心跳波形 uint8, 原始int8+128偏移
    TLV_BREATH_WAVEFORM = 0x61, // 呼吸波形 uint8, 原始int8+128偏移

    
};

// 分层分域错误码定义 (高4位=模块，低4位=具体错误)
namespace ErrorCode {
    // 通用状态 (0x0_)
    constexpr uint8_t SUCCESS = 0x00;              // 成功
    constexpr uint8_t PROCESSING = 0x01;           // 已接收，处理中
    
    // 协议层错误 (0x1_)
    constexpr uint8_t ERR_PROTO_CMD_UNKNOWN = 0x13;    // 未知命令
    constexpr uint8_t ERR_PROTO_PARAM_MISSING = 0x14;  // 缺少参数
    constexpr uint8_t ERR_PROTO_PARAM_INVALID = 0x15;  // 参数非法
    constexpr uint8_t ERR_PROTO_BUSY = 0x16;           // 设备忙
    constexpr uint8_t ERR_PROTO_FRAME_TOO_LARGE = 0x18; // 命令帧过大（超过缓冲区限制）

        // WiFi错误 (0x2_)
    constexpr uint8_t ERR_WIFI_SCAN_TIMEOUT = 0x20;      // 扫描超时
    constexpr uint8_t ERR_WIFI_SSID_NOT_FOUND = 0x21;    // 找不到SSID
    constexpr uint8_t ERR_WIFI_WRONG_PASSWORD = 0x22;    // 密码错误
    constexpr uint8_t ERR_WIFI_SIGNAL_WEAK = 0x25;       // 信号太弱
    constexpr uint8_t ERR_WIFI_BUSY = 0x26;              // WiFi正在被其他操作占用
    
    // 设备/状态错误 (0x4_)
    constexpr uint8_t ERR_DEV_STATE_INVALID = 0x40;      // 当前状态不允许
    constexpr uint8_t ERR_DEV_STORAGE_FAIL = 0x41;       // 存储失败
    constexpr uint8_t ERR_DEV_QUEUE_FULL = 0x42;         // 队列已满
}

// 设备状态码（用于 b3 状态推送通道，与命令响应解耦）
namespace DeviceStatus {
    // WiFi 状态
    constexpr uint8_t WIFI_DISCONNECTED = 0x10;   // WiFi 断开
    constexpr uint8_t WIFI_CONNECTING = 0x11;     // WiFi 连接中
    constexpr uint8_t WIFI_CONNECTED = 0x12;      // WiFi 已连接
    constexpr uint8_t WIFI_FAILED = 0x13;         // WiFi 连接失败
    
    // MQTT 状态（加前缀避免与 PubSubClient 宏冲突）
    constexpr uint8_t DEV_MQTT_DISCONNECTED = 0x20;   // MQTT 断开
    constexpr uint8_t DEV_MQTT_CONNECTING = 0x21;     // MQTT 连接中
    constexpr uint8_t DEV_MQTT_CONNECTED = 0x22;      // MQTT 已连接
    constexpr uint8_t DEV_MQTT_FAILED = 0x23;         // MQTT 连接失败
    
    // 雷达状态
    constexpr uint8_t RADAR_SLEEP_QUERY_DISABLED = 0x30;  // 雷达睡眠/综合状态查询已关闭
    constexpr uint8_t RADAR_SLEEP_QUERY_ENABLED = 0x31;   // 雷达睡眠/综合状态查询已开启
}

// WiFi安全类型枚举
enum WifiSecurityType : uint8_t {
    WIFI_SEC_OPEN = 0,      // 开放网络
    WIFI_SEC_WEP = 1,      // WEP加密
    WIFI_SEC_WPA = 2,      // WPA加密
    WIFI_SEC_WPA2 = 3,     // WPA2加密
    WIFI_SEC_WPA3 = 4,     // WPA3加密
    WIFI_SEC_UNKNOWN = 255 // 未知类型
};

//数据帧结构体定义
struct Frame {
    uint8_t version = VERSION;//协议版本
    uint8_t cmd = 0;//命令在数据帧中表示具体的操作类型，如查询状态、配置WiFi等
    uint8_t seq = 0;//序列号
    std::vector<uint8_t> data;//数据
};

class FrameParser {
public:
    FrameParser();//构造函数
    bool input(const uint8_t* bytes, size_t len, Frame& outFrame);//输入数据并尝试解析出一帧
    void reset();//重置解析器状态

private:
    std::vector<uint8_t> buffer;//用于存储输入的字节数据，直到成功解析出一帧（接收蓝牙数据的缓冲区）
    bool tryParseOne(Frame& outFrame);//尝试从缓冲区解析出一帧
};

uint16_t crc16Ccitt(const uint8_t* data, size_t len);//计算CRC16-CCITT校验和

void appendU8(std::vector<uint8_t>& out, uint8_t value);//添加8位无符号整数
void appendU16(std::vector<uint8_t>& out, uint16_t value);//添加16位无符号整数
void appendI16(std::vector<uint8_t>& out, int16_t value);//添加16位有符号整数
void appendU32(std::vector<uint8_t>& out, uint32_t value);//添加32位无符号整数
void appendU64(std::vector<uint8_t>& out, uint64_t value);//添加64位无符号整数
void appendBytes(std::vector<uint8_t>& out, const uint8_t* data, size_t len);//添加字节数组
void appendString(std::vector<uint8_t>& out, const String& s);//添加字符串

void appendTlvU8(std::vector<uint8_t>& out, uint8_t type, uint8_t value);//添加8位无符号整数TLV
void appendTlvI8(std::vector<uint8_t>& out, uint8_t type, int8_t value);//添加8位有符号整数TLV
void appendTlvU16(std::vector<uint8_t>& out, uint8_t type, uint16_t value);//添加16位无符号整数TLV
void appendTlvI16(std::vector<uint8_t>& out, uint8_t type, int16_t value);//添加16位有符号整数TLV
void appendTlvU32(std::vector<uint8_t>& out, uint8_t type, uint32_t value);//添加32位无符号整数TLV
void appendTlvU64(std::vector<uint8_t>& out, uint8_t type, uint64_t value);//添加64位无符号整数TLV
void appendTlvString(std::vector<uint8_t>& out, uint8_t type, const String& value);//添加字符串TLV
void appendTlvBlock(std::vector<uint8_t>& out, uint8_t type, const std::vector<uint8_t>& value);//添加块TLV

bool readTlv(const std::vector<uint8_t>& data, size_t& offset, uint8_t& type, uint16_t& len, const uint8_t*& value);//从数据中读取一个TLV块

std::vector<uint8_t> encodeFrame(const Frame& frame);//编码帧为字节数组


} // namespace BleProto

#endif
