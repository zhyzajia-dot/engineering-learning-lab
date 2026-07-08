/**
 * @file mqtt.h
 * @brief 原平台 MQTT 通信和 OTA 消息接口。
 *
 * 保留旧平台连接、属性上报、心跳以及 OTA 相关函数声明。
 */
#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <Preferences.h>

#undef MQTT_MAX_PACKET_SIZE // PubSubClient默认的最大包大小是256字节，增加到1024字节以支持更大的消息
#define MQTT_MAX_PACKET_SIZE 1024//最大MQTT包大小

class WiFiManager;

extern uint64_t device_sn;//设备序列号
extern Preferences preferences;//用于存储设备序列号等配置信息的Preferences对象
extern WiFiManager wifiManager;//WiFi管理器对象
extern bool continuousSendEnabled;//是否启用连续发送数据到MQTT的标志
extern unsigned long continuousSendInterval;//连续发送数据到MQTT的时间间隔，单位为毫秒

extern const char* mqttServer;//MQTT服务器地址
extern const int mqttPort;//MQTT服务器端口
extern const char* mqttProductKey;//MQTT产品Key
extern const char* mqttDeviceModel;//MQTT设备型号
extern const char* mqttProductSecret;//MQTT产品Secret
extern String deviceMacAddress;//设备MAC地址字符串

extern WiFiClient mqttWiFiClient;//MQTT使用的WiFi客户端对象
extern PubSubClient mqttClient;//MQTT客户端对象

extern TaskHandle_t mqttTaskHandle;//MQTT任务句柄

void mqttTask(void *parameter);//MQTT任务函数
String getMqttDeviceName();//获取MQTT设备名称
String getMqttClientId();//获取MQTT客户端ID
String getMqttSubscribeTopic();//获取MQTT订阅主题
String getMqttPropertyPostTopic();//获取MQTT属性发布主题
String makeMqttPassword(const String& clientId);//生成MQTT密码，用于认证
String buildReplyTopic(const char* requestTopic);//构建回复MQTT主题，用于接收回复消息。
bool publishMqttReply(const char* requestTopic, const char* requestId, const char* requestMethod, int code, JsonVariant data);//发布MQTT回复消息
void mqttMessageCallback(char* topic, byte* payload, unsigned int length);//MQTT消息回调函数
void initMQTT();//初始化MQTT
void reconnectMQTT();//重新连接MQTT服务器
void checkMQTTStatus();//检查MQTT连接状态
void sendDailyDataToMQTT();//发送每日数据到MQTT
bool sendSleepDataToMQTT(bool allowSessionEnd = false);//发送睡眠数据到MQTT
void sendHeartbeatToMQTT();//发送心跳包

// OTA相关函数
String getOtaUpgradeTopic();//获取OTA升级主题
String getOtaProgressTopic();//获取OTA进度主题
String getOtaVersionReportTopic();//获取OTA版本报告主题
String getOtaResultInformTopic();//获取OTA结果通知主题
bool publishOtaVersionReport();//发布OTA版本报告
bool publishOtaResultInform(const char* version, const char* module);//发布OTA结果通知
bool publishOtaProgress(const char* requestId, int step, const char* desc, const char* module);//发布OTA进度信息
bool handleOtaUpgradeMessage(const char* topic, const String& payload);//处理OTA升级消息

#endif
