/**
 * @file wifi_manager.h
 * @brief Wi-Fi 配置、扫描、保存和自动重连管理接口。
 *
 * 支持当前 BLE 小程序配网，并在设备重启后恢复已保存网络。
 */
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <vector>

// WiFi扫描结果结构体
struct WiFiScanResult {
    String ssid;
    String password; // 仅保存已配置网络的密码，扫描结果中不包含密码
    int rssi;
    String security;// 安全类型字符串，如 "OPEN", "WEP", "WPA", "WPA2", "WPA3"
};

/**
 * @brief 最大WiFi网络配置数量
 * 定义设备可以保存的WiFi网络配置的最大数量
 */
#define MAX_WIFI_NETWORKS 10

/**
 * @brief 最小信号强度阈值
 * 定义WiFi网络信号强度的最低要求，低于此值的网络会被过滤
 * 单位：dBm
 */
#define MIN_RSSI_THRESHOLD -128

/**
 * @brief WiFi连接超时时间
 * 定义WiFi连接的最大等待时间，超过此时间认为连接失败
 * 单位：毫秒
 */
#define WIFI_CONNECT_TIMEOUT 10000

/**
 * @brief WiFi重连间隔时间
 * 定义WiFi断开后，尝试重新连接的时间间隔
 * 单位：毫秒
 */
#define WIFI_RECONNECT_INTERVAL 3000

/*****************************************测试用******************************* */
/**
 * @brief 默认WiFi名称
 */
#define DEFAULT_WIFI_SSID "Xiaomi_495B"
/**
 * @brief 默认WiFi密码
 */
#define DEFAULT_WIFI_PASSWORD "20221109"
/*****************************************测试用******************************* */

/**
 * @brief WiFi网络信息结构
 * 存储WiFi网络的详细信息，用于扫描和显示
 */
typedef struct {
    char ssid[32];       // WiFi网络名称
    char password[64];   // WiFi网络密码
    int rssi;            // 信号强度，单位：dBm
    uint8_t channel;     // WiFi通道
    uint8_t encryption;  // 加密类型
} WiFiNetworkInfo;

/**
 * @brief WiFi配置结构
 * 存储WiFi网络的配置信息，用于保存和加载
 */
typedef struct {
    char ssid[32];       // WiFi网络名称
    char password[64];   // WiFi网络密码
} WiFiConfig;

/**
 * @brief WiFi管理器状态枚举
 * 定义WiFi管理器的不同工作状态
 */
enum WiFiManagerState {
    WIFI_IDLE,           // 空闲状态
    WIFI_SCANNING,       // 扫描网络中
    WIFI_CONNECTING,     // 连接网络中
    WIFI_CONNECTED,      // 已连接
    WIFI_DISCONNECTED,   // 断开连接
    WIFI_CONFIGURING     // 配网模式
};

/**
 * @brief 网络状态枚举
 * 定义网络的不同状态，用于LED控制
 */
enum NetworkStatus {
    NET_INITIAL,      // 初始化/未连接 - 慢闪
    NET_CONNECTING,   // 连接中 - 快闪
    NET_CONNECTED,    // 已连接 - 呼吸灯
    NET_DISCONNECTED  // 断开连接 - 慢闪
};

/**
 * @brief WiFi管理器类
 * 负责WiFi网络的扫描、连接、配置和管理
 */
class WiFiManager {
private:
    Preferences preferences;              // 用于存储WiFi配置的Preferences对象
    WiFiConfig savedNetworks[MAX_WIFI_NETWORKS]; // 保存的WiFi网络配置数组
    int savedNetworkCount;                // 已保存的网络数量
    WiFiManagerState currentState;        // 当前WiFi管理器状态
    unsigned long lastReconnectAttempt;   // 上次尝试重连的时间
    bool isScanning;                     // 是否正在扫描
    bool manualConfigActive;             // 手动配置标志位（setWiFiConfig 配置时为True，暂停重连）
    bool lastScanHadAvailableNetwork;    // 上次扫描是否有可用的已保存网络
    
    // FreeRTOS资源
    TaskHandle_t reconnectTaskHandle;    // 重连任务句柄
    SemaphoreHandle_t wifiMutex;         // WiFi操作互斥锁（保护WiFi硬件操作）
    SemaphoreHandle_t stateMutex;        // 状态互斥锁
    volatile bool scanInProgress;        // 扫描进行标志
    
    // 重连任务函数（静态）
    static void reconnectTask(void* parameter);
    
    bool scanAndMatchNetworks();          // 扫描并匹配网络
    bool connectToNetwork(const char* ssid, const char* password); // 连接到指定网络
    bool saveWiFiConfig(const char* ssid, const char* password);   // 保存WiFi配置
    bool loadWiFiConfigs();               // 加载WiFi配置
    
public:

   
    WiFiManager();                        // 构造函数
    void begin();                         // 初始化WiFi管理器
    
    bool initializeWiFi();                // 初始化WiFi连接
    bool handleConfigurationData(const char* ssid, const char* password); // 处理配网数据
    void handleReconnect();               // 处理重连
    
    bool isConnected();                   // 检查是否已连接
    
    // 扫描接口（会阻塞重连任务）
    bool startScan(uint32_t timeoutMs = 30000);
    void scanAndSendResults();            // 扫描并发送结果
    bool addWiFiConfig(const char* ssid, const char* password); // 添加WiFi配置
    bool removeWiFiConfig(const char* ssid, bool* existed = nullptr); // 删除指定WiFi配置
    void clearAllConfigs();               // 清除所有配置
    int getSavedNetworkCount();           // 获取已保存的网络数量
    void getSavedNetworks();            // 获取已保存的WiFi网络列表
    
    // 简化update（不再处理重连，重连在独立任务）
    void update() {}                        // 更新WiFi管理器状态

};

#endif
