/**
 * @file tuya_link.h
 * @brief TuyaLink 云端通信模块对外接口。
 *
 * 本模块保持原工程 MQTT 函数名不变，使睡眠分析任务无需关心
 * 当前使用旧平台还是涂鸦平台。默认构建使用 TuyaLink，Legacy
 * 构建环境仍可切回原 mqtt.cpp。
 */
#ifndef TUYA_LINK_H
#define TUYA_LINK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// TuyaLink MQTT 后台任务句柄，由 initAllTasks() 创建任务时写入。
extern TaskHandle_t mqttTaskHandle;

/**
 * @brief TuyaLink MQTT 后台任务。
 *
 * 负责 TLS/MQTT 初始化、断线重连、MQTT loop 以及周期性 DP 上报。
 */
void mqttTask(void* parameter);

/**
 * @brief 上报日常雷达数据和当前睡眠状态。
 *
 * 包含 DP 102~110 中当前可用的数据，不包含暂未接入的 DP 101。
 */
void sendDailyDataToMQTT();

/**
 * @brief 上报睡眠状态和睡眠评分。
 * @param allowSessionEnd 保留原接口语义；会话结束时调用方传 true。
 * @return true 发布成功；false 未联网、时间未同步或 MQTT 发布失败。
 */
bool sendSleepDataToMQTT(bool allowSessionEnd = false);

/**
 * @brief 无人状态下周期上报存在、床状态和睡眠状态。
 *
 * MQTT 协议自身的 Keep Alive 负责连接保活，这里仅同步业务状态。
 */
void sendHeartbeatToMQTT();

#endif
