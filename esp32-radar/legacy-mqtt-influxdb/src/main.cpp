#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "wifi_manager.h"
#include "radar_manager.h"
#include "data_processor.h"
#include "emotion_analyzer_simple.h"
#include "tasks_manager.h"
#include "OTA_manager.h"

Preferences preferences; // Flash存储对象
WiFiManager wifiManager; // WiFi管理器对象

void checkBootButton();//检查Boot按钮状态

/**
 * @brief 系统初始化函数
 * 初始化所有硬件外设、任务和通信模块
 */
void setup() {
    Serial.begin(115200);                   // 初始化串口通信
    checkBootButton();                      // 检查Boot按钮状态
    analogWrite(CONFIG_CLEAR_PIN, 0);        // 关闭配置清除指示灯
    WiFi.onEvent(WiFiEvent);                // 注册WiFi事件处理函数
    setNetworkStatus(NET_INITIAL);           // 初始化网络状态
    esp_task_wdt_init(30, true);            // 初始化看门狗定时器
    esp_task_wdt_add(NULL);                 // 将主任务添加到看门狗
    preferences.begin("radar_data", false);  // 初始化Flash存储
    initRadarManager();                     // 初始化雷达管理器
    initOtaManager();                       // 初始化OTA管理器
    initAllTasks();                         // 创建所有FreeRTOS任务
    if (WiFi.status() == WL_CONNECTED)    // 启动时发送睡眠数据
        sendSleepDataToInfluxDB();
    Serial.println("✅ 系统初始化完成");
}

/**
 * @brief 主循环函数
 * 主循环只负责看门狗重置，保持空闲
 */
void loop() {
    esp_task_wdt_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

/**
 * @brief 检查Boot按钮状态
 * 在启动时检查Boot按钮是否松开，等待松开后再启动避免频繁重启
 */
void checkBootButton() {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    delay(10);
    
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        Serial.println("⚠️ 检测到Boot按钮按下，请释放按钮后继续启动"); 
        while (digitalRead(BOOT_BUTTON_PIN) == LOW) {
            delay(50);
        }
        Serial.println("✅ Boot按钮已释放，正常启动");
    }
}


