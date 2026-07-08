/**
 * @file tasks_manager.h
 * @brief FreeRTOS 任务、BLE 特征和设备运行状态的公共声明。
 *
 * 各硬件及业务模块的后台任务由任务管理器统一创建和协调。
 */
#ifndef TASKS_MANAGER_H
#define TASKS_MANAGER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Preferences.h>
#include "wifi_manager.h"
#include "data_processor.h"
#include "emotion_analyzer_simple.h"
#include "pir_manager.h"
#include "sleep_analyzer.h"
// 默认使用自有 HTTP 服务器；构建宏可切回 TuyaLink 或原平台 MQTT。
#ifdef USE_LEGACY_MQTT
#include "mqtt.h"
#elif defined(USE_TUYA_LINK)
#include "tuya_link.h"
#else
#include "server_link.h"
#endif

// ESP32 GPIO控制演示
#define BOOT_BUTTON_PIN 0   // Boot按钮引脚
#define NETWORK_LED_PIN 5    // 网络状态LED指示灯开发板48引脚，雷达板5引脚
#define CONFIG_CLEAR_PIN 4   // 配置清除指示灯
#define GPIO8 8              // 自定义GPIO8
#define GPIO9 9              // 自定义GPIO9

#define CLEAR_CONFIG_DURATION 3000 // 清除配置持续时间（毫秒）
#define SLOW_BLINK_INTERVAL 1000  // 慢闪间隔（毫秒）
#define FAST_BLINK_INTERVAL 200    // 快闪间隔（毫秒）
#define BREATHE_INTERVAL 40        // 呼吸灯更新间隔（毫秒）
#define BREATHE_MIN 0              // 呼吸灯最小亮度值
#define BREATHE_MAX 155            // 呼吸灯最大亮度值
#define BREATHE_STEP 5             // 呼吸灯亮度步进值

extern Preferences preferences;
extern WiFiManager wifiManager;
extern uint64_t device_sn;
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern BLEServer* pServer;
extern NetworkStatus currentNetworkStatus;
extern unsigned long lastBlinkTime;
extern bool ledState;
extern int breatheValue;
extern bool breatheIncreasing;
extern uint8_t WiFi_Connect_First_bit;
extern void sendRadarCommand(uint8_t cmd, uint8_t subCmd, uint8_t param);
extern void initR60ABD1();
extern void processBLEConfig();
extern void sendStatusToBLE();
extern bool sendSleepDataToInfluxDB(bool allowSessionEnd);
extern void setNetworkStatus(NetworkStatus status);
extern void clearStoredConfig();
extern void loadDeviceSN();
extern void saveDeviceSn();
extern std::string buildBLEManufacturerData();
extern void refreshBLEAdvertisingData();
extern String getDeviceMacAddress();
extern void updateDeviceInfo();
extern void updateRadarStatus();
extern void updateEmotionResult(const EmotionResult& result);
extern void clearEmotionResult();
extern bool getFreshEmotionResult(EmotionResult& result);

void initAllTasks();
void WiFiEvent(WiFiEvent_t event);

void bootButtonMonitorTask(void *parameter);
void ledControlTask(void *parameter);
void wifiMonitorTask(void *parameter);
void bleConfigTask(void *parameter);
void radarCmdTask(void *parameter);
void pirTask(void *parameter);
void emotionAnalysisTask(void *parameter);
void sleepAnalysisTask(void *parameter);

// 情绪分析结果（供 MQTT 上报读取）
extern EmotionResult g_lastEmotionResult;
extern bool g_hasEmotionResult;
extern unsigned long g_lastEmotionUpdateMs;

#endif
