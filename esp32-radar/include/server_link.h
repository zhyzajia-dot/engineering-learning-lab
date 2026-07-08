/**
 * @file server_link.h
 * @brief 自有服务器 HTTP 上传模块对外接口。
 *
 * 保持原 MQTT/TuyaLink 函数签名，使任务管理和睡眠算法无需感知
 * 当前云端传输方式。
 */
#ifndef SERVER_LINK_H
#define SERVER_LINK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern TaskHandle_t mqttTaskHandle;

void mqttTask(void* parameter);
void sendDailyDataToMQTT();
bool sendSleepDataToMQTT(bool allowSessionEnd = false);
void sendHeartbeatToMQTT();

#endif
