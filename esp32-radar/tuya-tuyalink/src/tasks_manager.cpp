/**
 * @file tasks_manager.cpp
 * @brief 系统后台任务的实现和统一创建入口。
 *
 * 协调 BLE、睡眠分析、红外检测、Wi-Fi、LED 和雷达命令等任务。
 */
#include "tasks_manager.h"
#include "wifi_manager.h"
#include "radar_manager.h"
#include "data_processor.h"
#include "emotion_analyzer_simple.h"
#include <BLEDevice.h>
#include <esp_task_wdt.h>
#include <freertos/portmacro.h>

// 外部常量声明
extern const unsigned long SENSOR_TIMEOUT; // 传感器超时时间

NetworkStatus currentNetworkStatus = NET_INITIAL;//当前网络状态，初始为初始网络状态
unsigned long lastBlinkTime = 0;//上次闪烁时间
bool ledState = false;//LED状态
int breatheValue = 0;//呼吸值
bool breatheIncreasing = true;//呼吸值是否递增
uint8_t WiFi_Connect_First_bit = 1;//WiFi连接状态位
uint64_t device_sn = 0;//设备SN，初始为0，后续从Flash中加载

SimpleEmotionAnalyzer* emotionAnalyzer;//情感分析器

// 情绪分析结果缓存（供 MQTT 上报读取）
EmotionResult g_lastEmotionResult = {};
bool g_hasEmotionResult = false;
unsigned long g_lastEmotionUpdateMs = 0;
static portMUX_TYPE emotionResultMux = portMUX_INITIALIZER_UNLOCKED;
static const unsigned long EMOTION_RESULT_TTL_MS = 5000;

bool clearConfigRequested = false;//是否请求清除配置
bool forceLedOff = false;//是否强制关闭LED

void updateEmotionResult(const EmotionResult& result) {
    unsigned long now = millis();

    taskENTER_CRITICAL(&emotionResultMux);
    g_lastEmotionResult = result;
    g_hasEmotionResult = result.isValid;
    g_lastEmotionUpdateMs = now;
    taskEXIT_CRITICAL(&emotionResultMux);
}

void clearEmotionResult() {
    taskENTER_CRITICAL(&emotionResultMux);
    g_lastEmotionResult = {};
    g_hasEmotionResult = false;
    g_lastEmotionUpdateMs = 0;
    taskEXIT_CRITICAL(&emotionResultMux);
}

bool getFreshEmotionResult(EmotionResult& result) {
    bool hasResult = false;
    bool isValid = false;
    unsigned long updatedAt = 0;
    unsigned long now = millis();

    taskENTER_CRITICAL(&emotionResultMux);
    result = g_lastEmotionResult;
    hasResult = g_hasEmotionResult;
    isValid = g_lastEmotionResult.isValid;
    updatedAt = g_lastEmotionUpdateMs;
    taskEXIT_CRITICAL(&emotionResultMux);

    return hasResult && isValid && (now - updatedAt <= EMOTION_RESULT_TTL_MS);
}

/**
 * @brief 加载设备SN
 * 从Flash中读取保存的设备SN，如果没有则保持为0
 * 设备SN用于MQTT设备标识和BLE广播，优先使用SN作为设备标识，如果SN为0则使用MAC地址作为设备标识
 */
void loadDeviceSN() {
    device_sn = preferences.getULong64("deviceSn", 0);
    Serial.printf("从Flash加载设备SN: %llu\n", device_sn);
}

/**
 * @brief 保存设备SN
 * 将设备SN保存到Flash中
 */
void saveDeviceSn() {
    preferences.putULong64("deviceSn", device_sn);
    Serial.printf("设备SN已保存到Flash: %llu\n", device_sn);
}

/**
 * @brief 构建BLE厂商数据
 * 构造BLE广播厂商数据
 * @return 9字节厂商数据字符串
 */
std::string buildBLEManufacturerData() {
    std::string manufacturerData;//厂商数据字符串
    manufacturerData.reserve(4);

    manufacturerData.push_back(static_cast<char>(0xFF));//BLE厂商数据标志（固定）
    manufacturerData.push_back(static_cast<char>(0xFF));//厂商ID
    manufacturerData.push_back(0x01);//设备类型（Radar）
    manufacturerData.push_back(0x00);//设备类型（Radar）

    return manufacturerData;
}

/**
 * @brief 刷新BLE广播数据
 * 更新BLE广播的厂商数据和设备名称，使用设备SN或MAC地址（如果SN为0）作为设备名
 */
void refreshBLEAdvertisingData() {
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();//获取BLE广播对象
    if (pAdvertising == nullptr) {
        Serial.println("⚠️ [BLE] 广播对象为空，无法刷新广播数据");
        return;
    }

    char snName[32];
    if (device_sn > 0) {
        snprintf(snName, sizeof(snName), "Radar_%llu", device_sn);//使用设备SN作为设备名
    } else {
        String macAddr = getDeviceMacAddress();
        macAddr.replace(":", "");
        snprintf(snName, sizeof(snName), "Radar_%s", macAddr.c_str());//使用设备MAC地址作为设备名
    }
    
    BLEAdvertisementData advertisementData;//广告数据
    advertisementData.setFlags(0x06);//设置广告标志（扫描响应）
    advertisementData.setCompleteServices(BLEUUID(DEVICE_CONFIG_SERVICE_UUID));//设置设备配置服务UUID
    advertisementData.setManufacturerData(buildBLEManufacturerData());//设置厂商数据

    BLEAdvertisementData scanResponseData;
    scanResponseData.setName(snName);

    // 手动添加 TX Power Level 字段
    // 0x02: 本段长度 (类型1字节 + 数据1字节)
    // 0x0A: AD Type (TX Power Level)
    // 0x09: 数据值 (+9 dBm，ESP32 默认发射功率)
    std::string txPowerField;
    txPowerField.push_back(0x02);
    txPowerField.push_back(0x0A);
    txPowerField.push_back(0x09);
    scanResponseData.addData(txPowerField);

    pAdvertising->setAdvertisementData(advertisementData);//设置广告数据
    pAdvertising->setScanResponseData(scanResponseData);//设置扫描响应数据
    pAdvertising->setScanResponse(true);//设置扫描响应
    pAdvertising->setMinPreferred(0x06);//设置最小优先级
    pAdvertising->setMinPreferred(0x12);//设置最小优先级                                                          

    Serial.printf("📡 [BLE] 已刷新广播 ManufacturerData, device_sn=%llu\n", device_sn);
}

/**
 * @brief 获取设备MAC地址
 * 读取WiFi STA接口的MAC地址并格式化为字符串
 * @return MAC地址字符串，格式为 XX:XX:XX:XX:XX:XX
 */
String getDeviceMacAddress() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

/**
 * @brief 更新设备信息特征
 * 当设备ID或其他静态信息变化时更新设备信息特征
 * 通过 b3 (DEVICE_INFO_CHAR_UUID) notify 通道分包发送
 */
void updateDeviceInfo() {
    if (deviceInfoCharacteristic == nullptr) {
        return;
    }
    
    // 构造设备信息TLV帧
    BleProto::Frame infoFrame;
    infoFrame.version = BleProto::VERSION;
    infoFrame.cmd = BleProto::CMD_DEVICE_INFO_PUSH;  // 主动推送设备信息
    infoFrame.seq = 0;
    infoFrame.data.clear();
    
    // 添加设备信息TLV字段
    BleProto::appendTlvU8(infoFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
    BleProto::appendTlvString(infoFrame.data, BleProto::TLV_PROTOCOL_VERSION, "1.0.0");
    BleProto::appendTlvString(infoFrame.data, BleProto::TLV_FIRMWARE_VERSION, "2.1.0");
    BleProto::appendTlvString(infoFrame.data, BleProto::TLV_DEVICE_TYPE, "Radar");
    
    String macAddress = getDeviceMacAddress();
    if (macAddress.length() > 0) {
        BleProto::appendTlvString(infoFrame.data, BleProto::TLV_MAC_ADDRESS, macAddress);
    }
    
    if (device_sn > 0) {
        BleProto::appendTlvU64(infoFrame.data, BleProto::TLV_DEVICE_SN, device_sn);
    }
    
    // 编码TLV帧
    std::vector<uint8_t> frameData = BleProto::encodeFrame(infoFrame);
    
    // 通过 notify 分包发送（b3 仅支持 NOTIFY，不支持 READ）
    if (deviceConnected) {
        sendFrameToBLE(infoFrame, deviceInfoCharacteristic);
        Serial.printf("📋 [BLE] 设备信息已通过 b3 notify 发送，长度: %u 字节\n", static_cast<unsigned>(frameData.size()));
    } else {
        Serial.printf("📋 [BLE] 设备信息帧已构建，长度: %u 字节（等待连接后 notify）\n", static_cast<unsigned>(frameData.size()));
    }
}

/**
 * @brief 更新x,y,z坐标和存在状态的BLE特征
 * 根据当前传感器数据更新雷达状态特征，包含存在状态、运动状态、距离和坐标等信息，以TLV格式发送给BLE客户端
 * 通过 a2 (RADAR_STATUS_CHAR_UUID) notify 通道分包发送
 */
void updateRadarStatus() {
    if (radarStatusCharacteristic == nullptr || !deviceConnected) {
        return;
    }
    
    // 构造雷达状态TLV帧
    BleProto::Frame statusFrame;
    statusFrame.version = BleProto::VERSION;
    statusFrame.cmd = BleProto::CMD_RADAR_STATUS_PUSH;  // 主动推送雷达状态
    statusFrame.seq = 0;
    statusFrame.data.clear();

    BleProto::appendTlvU16(statusFrame.data, BleProto::TLV_DISTANCE_CM, sensorData.distance);// 距离
    BleProto::appendTlvI16(statusFrame.data, BleProto::TLV_POS_X_MM, sensorData.pos_x);// X坐标
    BleProto::appendTlvI16(statusFrame.data, BleProto::TLV_POS_Y_MM, sensorData.pos_y);// Y坐标
    BleProto::appendTlvI16(statusFrame.data, BleProto::TLV_POS_Z_MM, sensorData.pos_z);// Z坐标
    BleProto::appendTlvU8(statusFrame.data, BleProto::TLV_BODY_MOVEMENT, sensorData.body_movement);// 身体运动状态

    // 通过 a2 notify 分包发送
    sendFrameToBLE(statusFrame, radarStatusCharacteristic);
    
    Serial.printf("📊 [BLE] 雷达状态已通过 a2 notify 发送，长度: %u 字节\n", 
                  static_cast<unsigned>(BleProto::encodeFrame(statusFrame).size()));
}

/**
 * @brief 设置网络状态
 * 更新当前网络状态，并重置呼吸灯参数
 * @param status 网络状态
 */
void setNetworkStatus(NetworkStatus status) {
    currentNetworkStatus = status;//更新当前网络状态

    if (status == NET_CONNECTED) {
        breatheValue = BREATHE_MIN;//重置呼吸灯值为最小
        breatheIncreasing = true;//设置呼吸灯增加
    }
}

/**
 * @brief 清除存储的配置
 * 清除Flash中保存的WiFi配置和设备ID，重置WiFi连接状态
 */
void clearStoredConfig() {
    Serial.println("🧹 开始清除存储的配置...");

    preferences.remove("wifi_first");

    wifiManager.clearAllConfigs();//清除所有WiFi配置

    Serial.println("✅ 配置已清除完成");

    WiFi_Connect_First_bit = 1;//设置WiFi连接首次标志位为1

    WiFi.disconnect(true);//断开WiFi连接
    setNetworkStatus(NET_DISCONNECTED);//设置网络状态为断开

    Serial.println("🔄 已清除Flash与内存中的配置，请重新配置WiFi");

    if (deviceConnected) {
        updateDeviceInfo();  // b3: 设备信息推送（替代已废弃的 sendStatusToBLE）
    }
}

/**
 * @brief BOOT按钮监控任务
 * 监控BOOT按钮按下事件，长按3秒清除存储的配置
 * @param parameter 任务参数（未使用）
 */
void bootButtonMonitorTask(void *parameter) {
    Serial.println("🔍 启动BOOT按钮监控任务...");

    pinMode(CONFIG_CLEAR_PIN, OUTPUT);//设置清除配置引脚为输出模式
    digitalWrite(CONFIG_CLEAR_PIN, LOW);//设置清除配置引脚为低电平

    unsigned long buttonPressStartTime = 0;//按钮按下开始时间
    bool buttonPressed = false;//按钮是否按下

    while (1) {
        int buttonState = digitalRead(BOOT_BUTTON_PIN);

        if (buttonState == LOW && !buttonPressed) {
            buttonPressed = true;//按钮按下
            buttonPressStartTime = millis();//记录按下时间
            Serial.println("⚠️ 检测到BOOT按钮按下，长按3秒将清除配置");

            digitalWrite(CONFIG_CLEAR_PIN, HIGH);
        }
        else if (buttonState == HIGH && buttonPressed) {
            if (!clearConfigRequested) {
                digitalWrite(CONFIG_CLEAR_PIN, LOW);
                Serial.println("❌ 按钮释放，取消清除操作");
            }
            buttonPressed = false;
        }

        if (buttonPressed && (millis() - buttonPressStartTime >= CLEAR_CONFIG_DURATION)) {
            if (!clearConfigRequested) {
                clearConfigRequested = true;
                forceLedOff = true;

                clearStoredConfig();//清除存储的配置

                Serial.println("🔄 系统即将重启...");

                vTaskDelay(1000 / portTICK_PERIOD_MS);
                digitalWrite(CONFIG_CLEAR_PIN, LOW);
                ledcWrite(0, 0);
                ESP.restart();
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

/**
 * @brief 睡眠分析任务
 * 每秒更新生理数据、运行睡眠状态机、输出睡眠状态到串口
 * @param parameter 任务参数（未使用）
 */
void sleepAnalysisTask(void *parameter) {
    SleepAnalyzer* sleepAnalyzer = new SleepAnalyzer();
    PhysioDataProcessor* sleepPhysioProcessor = new PhysioDataProcessor();

    static unsigned long lastSleepAnalysisTime = 0;
    const unsigned long SLEEP_ANALYSIS_INTERVAL = 1000;
    static unsigned long lastStatsPrintTime = 0;
    const unsigned long STATS_PRINT_INTERVAL = 30000;

    // 会话结束检测：追踪"会话结束"状态
    static bool sessionReportSent = false;
    static bool sessionInfluxReportSent = false;
    static bool sessionMqttReportSent = false;
    static unsigned long lastSessionReportAttemptTime = 0;
    const unsigned long SESSION_REPORT_RETRY_INTERVAL = 5000;

    while (1) {
        unsigned long currentTime = millis();

        if (currentTime - lastSleepAnalysisTime >= SLEEP_ANALYSIS_INTERVAL) {
            lastSleepAnalysisTime = currentTime;

            // 检查雷达睡眠模式是否启用
            if (!radarSleepQueryEnabled) {
                Serial.println("[SleepAnalyzer] 雷达睡眠模式已禁用，跳过算法分析");
                continue;
            }

            if (sensorData.heart_valid || sensorData.breath_valid) {
                float hr = sensorData.heart_valid ? sensorData.heart_rate : 0;
                float rr = sensorData.breath_valid ? sensorData.breath_rate : 0;

                if (hr > 0 || rr > 0) {
                    sleepPhysioProcessor->update(hr, rr,
                        sensorData.heart_valid ? 80 : 0,
                        sensorData.breath_valid ? 80 : 0);

                    HeartRateData hrData = sleepPhysioProcessor->getHeartRateData();//获取心率数据
                    RespirationData rrData = sleepPhysioProcessor->getRespirationData();//获取呼吸数据
                    HRVEstimate hrvData = sleepPhysioProcessor->getHRVEstimate();//获取HRV数据

                    BodyMovementData movementData;
                    memset(&movementData, 0, sizeof(BodyMovementData));//初始化运动数据结构体
                    movementData.movement = sensorData.body_movement;
                    movementData.movementSmoothed = sensorData.body_movement;
                    movementData.movementMean = sensorData.body_movement;
                    movementData.activityLevel = sensorData.body_movement / 100.0f;
                    movementData.isValid = (sensorData.body_movement >= 0 && sensorData.body_movement <= 100);
                    movementData.timestamp = currentTime;

                    sleepAnalyzer->update(hrData, rrData, hrvData, movementData,
                                          sensorData.bed_status);

                    // 更新全局睡眠分析快照
                    SleepState state = sleepAnalyzer->getCurrentState();//获取当前睡眠状态
                    SleepStatistics stats = sleepAnalyzer->getStatistics();//获取睡眠统计信息
                    SleepScore score = sleepAnalyzer->getScore();//获取睡眠评分
                    SleepCycle cycle = sleepAnalyzer->getCycle();//获取睡眠周期信息
                    SleepAnalysisSnapshot snapshot = {0};

                    snapshot.algorithm_state = (int)state;
                    snapshot.current_sleepiness = sleepAnalyzer->getSleepiness();//获取当前睡眠iness

                    snapshot.total_sleep_time = stats.totalSleepTime;//获取总睡眠时间
                    snapshot.deep_sleep_time = stats.deepSleepTime;//获取深度睡眠时间
                    snapshot.light_sleep_time = stats.lightSleepTime;//获取浅睡眠时间
                    snapshot.rem_sleep_time = stats.remSleepTime;//获取REM睡眠时间
                    snapshot.awake_time = stats.awakeTime;//获取清醒时间
                    snapshot.out_of_bed_time = stats.outOfBedTime;//获取离床时间
                    snapshot.sleep_latency = stats.sleepLatency;//获取睡眠延迟
                    snapshot.wake_count = stats.wakeCount;//获取唤醒次数
                    snapshot.sleep_cycles = stats.sleepCycles;//获取睡眠周期数
                    snapshot.session_start_time = stats.sessionStartTime;//获取会话开始时间
                    snapshot.sleep_start_time = stats.sleepStartTime;//获取睡眠开始时间
                    snapshot.last_wake_time = stats.lastWakeTime;//获取上次唤醒时间

                    snapshot.duration_score = score.durationScore;//获取睡眠持续时间评分
                    snapshot.deep_score = score.deepScore;//获取深度睡眠评分
                    snapshot.continuity_score = score.continuityScore;//获取睡眠连续性评分
                    snapshot.physiology_score = score.physiologyScore;//获取生理评分
                    snapshot.latency_score = score.latencyScore;//获取睡眠延迟评分
                    snapshot.efficiency_score = score.efficiencyScore;//获取睡眠效率评分
                    snapshot.cycle_score = score.cycleScore;//获取睡眠周期评分
                    snapshot.total_score = score.totalScore;//获取总睡眠评分

                    snapshot.cycle_count = cycle.cycleCount;//获取睡眠周期数
                    snapshot.cycle_start_time = cycle.cycleStartTime;//获取睡眠周期开始时间
                    snapshot.in_deep_phase = cycle.inDeepPhase;//是否在深度睡眠阶段
                    snapshot.in_rem_phase = cycle.inRemPhase;//是否在REM睡眠阶段

                    snapshot.updated_at = currentTime;//更新时间
                    snapshot.valid = true;//设置为有效
                    updateSleepAnalysisSnapshot(snapshot);//更新睡眠分析快照

                    // 会话结束检测：监听 SLEEP_SESSION_END 状态，立即发送睡眠报告
                    if (state == SLEEP_SESSION_END) {
                        // Session-end reports are retried until both storage paths succeed.
                        if (!sessionReportSent &&
                            currentTime - lastSessionReportAttemptTime >= SESSION_REPORT_RETRY_INTERVAL) {
                            lastSessionReportAttemptTime = currentTime;
                            Serial.println("[SleepAnalyzer] 会话结束，生成睡眠报告");
                            if (!sessionInfluxReportSent) {
                                sessionInfluxReportSent = sendSleepDataToInfluxDB(true);
                            }
                            if (!sessionMqttReportSent) {
                                sessionMqttReportSent = sendSleepDataToMQTT(true);
                            }
                            sessionReportSent = sessionInfluxReportSent && sessionMqttReportSent;
                            if (!sessionReportSent) {
                                Serial.println("[SleepAnalyzer] 睡眠报告发送失败，等待重试");
                            }
                        }
                    } else {
                        sessionReportSent = false;
                        sessionInfluxReportSent = false;
                        sessionMqttReportSent = false;
                        lastSessionReportAttemptTime = 0;
                    }

                    sleepAnalyzer->printState();

                    if (currentTime - lastStatsPrintTime >= STATS_PRINT_INTERVAL) {
                        lastStatsPrintTime = currentTime;
                        sleepAnalyzer->printStatistics();
                    }
                }
            }
        }

        invalidateSleepAnalysisSnapshotIfStale();

        vTaskDelay(50 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

/**
 * @brief LED控制任务
 * 根据网络状态控制LED显示：断开时慢闪、连接中快闪、已连接时呼吸灯效果
 * @param parameter 任务参数（未使用）
 */
void ledControlTask(void *parameter) {
    Serial.println("💡 启动LED控制任务...");

    pinMode(NETWORK_LED_PIN, OUTPUT);
    digitalWrite(NETWORK_LED_PIN, LOW);
    ledcSetup(0, 5000, 8);//设置LED引脚为PWM模式，频率5000Hz，分辨率8位
    ledcAttachPin(NETWORK_LED_PIN, 0);//将LED引脚与PWM通道0关联

    while (1) {
        if (forceLedOff) {
            ledcWrite(0, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        switch (currentNetworkStatus) {
            case NET_INITIAL://初始状态，LED慢闪
            case NET_DISCONNECTED://断开状态，LED慢闪
                if (millis() - lastBlinkTime >= SLOW_BLINK_INTERVAL) {
                    ledState = !ledState;//切换LED状态
                    if(ledState) {
                        ledcWrite(0, 255);//LED亮起
                    } else {
                        ledcWrite(0, 0);//LED熄灭
                    }
                    lastBlinkTime = millis();
                }
                break;

            case NET_CONNECTING://连接中状态，LED快闪
                if (millis() - lastBlinkTime >= FAST_BLINK_INTERVAL) {
                    ledState = !ledState;
                    if(ledState) {
                        ledcWrite(0, 255);
                    } else {
                        ledcWrite(0, 0);
                    }
                    lastBlinkTime = millis();
                }
                break;

            case NET_CONNECTED://已连接状态，LED呼吸灯效果
                if (millis() - lastBlinkTime >= BREATHE_INTERVAL) {
                    ledcWrite(0, breatheValue);//根据breatheValue设置LED亮度

                    if (breatheIncreasing) {
                        breatheValue += BREATHE_STEP;//增加breatheValue
                        if (breatheValue >= BREATHE_MAX) {
                            breatheValue = BREATHE_MAX;
                            breatheIncreasing = false;
                        }
                    } else {
                        breatheValue -= BREATHE_STEP;
                        if (breatheValue <= BREATHE_MIN) {
                            breatheValue = BREATHE_MIN;
                            breatheIncreasing = true;
                        }
                    }
                    lastBlinkTime = millis();
                }
                break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_START://WiFi启动事件
            setNetworkStatus(NET_INITIAL);//设置网络状态为初始状态
            break;

        case ARDUINO_EVENT_WIFI_STA_CONNECTED://WiFi连接成功事件
            setNetworkStatus(NET_CONNECTING);//设置网络状态为连接中状态
            break;

        case ARDUINO_EVENT_WIFI_STA_GOT_IP://WiFi获取IP地址事件
            setNetworkStatus(NET_CONNECTED);//设置网络状态为已连接状态
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED://WiFi断开事件
            setNetworkStatus(NET_DISCONNECTED);//设置网络状态为断开状态
            break;

        case ARDUINO_EVENT_WIFI_STA_STOP://WiFi停止事件
            setNetworkStatus(NET_DISCONNECTED);//设置网络状态为断开状态
            break;
    }
}

/**
 * @brief WiFi监控任务
 * 初始化WiFi并定期打印连接状态和信号强度
 * @param parameter 任务参数（未使用）
 */
void wifiMonitorTask(void *parameter) {
    Serial.println("📡 WiFi监控任务启动");

    wifiManager.begin();

    if (wifiManager.getSavedNetworkCount() > 0) {
        Serial.printf("💾 检测到 %d 个已保存的WiFi配置，尝试连接...\n", wifiManager.getSavedNetworkCount());
        if (wifiManager.initializeWiFi()) {
            Serial.println("✅ WiFi连接成功！");
        } else {
            Serial.println("❌ WiFi连接失败，请通过BLE重新配置");
        }
    } else {
        Serial.println("⚠️ 未检测到WiFi配置，请通过BLE进行网络配置");
    }

    size_t wifi_first_len = preferences.getBytes("wifi_first", &WiFi_Connect_First_bit, sizeof(WiFi_Connect_First_bit));
    if (wifi_first_len == sizeof(WiFi_Connect_First_bit)) {
        Serial.printf("从Flash读取 WiFi_Connect_First_bit: %u\n", WiFi_Connect_First_bit);
    } else {
        Serial.println("Flash中无 wifi_first 条目，保留内存中原始值");
    }

    if(WiFi_Connect_First_bit == 0)
    {
        unsigned long wifiWaitStart = millis();
        unsigned long lastWifiWaitPrint = 0;
        const unsigned long WIFI_WAIT_TIMEOUT = 15000;
        while (WiFi.status() != WL_CONNECTED && (millis() - wifiWaitStart) < WIFI_WAIT_TIMEOUT) {
            if (millis() - lastWifiWaitPrint >= 1000) {
                lastWifiWaitPrint = millis();
            }
            yield();
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    Serial.println("📡 WiFi初始化完成，开始监控...");

    while(1) {
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 30000) {
            Serial.printf("📡 WiFi状态: %d, RSSI: %d dBm\n",
                WiFi.status(), WiFi.RSSI());
            lastPrint = millis();
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief BLE配置处理任务
 * 处理BLE配置命令，监控设备连接状态并管理广播
 * @param parameter 任务参数（未使用）
 */
void bleConfigTask(void *parameter) {
    Serial.println("📡 BLE配置处理任务启动");

    char snName[32];
    if (device_sn > 0) {
        snprintf(snName, sizeof(snName), "Radar_%llu", device_sn);//设置设备名称为Radar_设备序列号
    } else {
        String macAddr = getDeviceMacAddress();
        macAddr.replace(":", "");//将MAC地址中的":"替换为空字符串
        snprintf(snName, sizeof(snName), "Radar_%s", macAddr.c_str());//设置设备名称为Radar_设备地址
    }
    BLEDevice::init(snName);//初始化BLE设备
#if BLE_FIXED_20_BYTE_MODE
    // 固定 20 字节兼容模式：不发起 MTU 协商
    Serial.println("[BLE] 固定 20 字节兼容模式，跳过 MTU 协商");
#else
    BLEDevice::setMTU(TARGET_ATT_MTU);//设置目标MTU值，ESP32会尝试与客户端协商一个合适的MTU大小
    Serial.printf("[BLE] 请求目标 MTU: %u\n", TARGET_ATT_MTU);
#endif
    
    pServer = BLEDevice::createServer();//创建BLE服务器
    pServer->setCallbacks(new MyServerCallbacks());//设置BLE服务器回调函数

    // 创建双服务
    radarDataService = pServer->createService(RADAR_DATA_SERVICE_UUID);//创建雷达数据服务
    deviceConfigService = pServer->createService(DEVICE_CONFIG_SERVICE_UUID);//创建设备配置服务

    // Radar Data Service
    radarStreamCharacteristic = radarDataService->createCharacteristic(
        RADAR_STREAM_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    radarStreamCharacteristic->addDescriptor(new BLE2902());//添加通知描述符，用于客户端订阅雷达数据流

    radarStatusCharacteristic = radarDataService->createCharacteristic(
        RADAR_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY  // a2 仅支持 NOTIFY
    );
    radarStatusCharacteristic->addDescriptor(new BLE2902());//添加通知描述符，用于客户端订阅雷达状态更新

    // Device Config Service
    deviceCommandCharacteristic = deviceConfigService->createCharacteristic(
        DEVICE_COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    deviceCommandCharacteristic->setCallbacks(new MyCallbacks());

    deviceResultCharacteristic = deviceConfigService->createCharacteristic(
        DEVICE_RESULT_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    deviceResultCharacteristic->addDescriptor(new BLE2902());

    deviceInfoCharacteristic = deviceConfigService->createCharacteristic(
               DEVICE_INFO_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY  // b3 仅支持 NOTIFY
    );
    deviceInfoCharacteristic->addDescriptor(new BLE2902());//添加通知描述符，用于客户端订阅设备信息更新
    
    // 启动服务（必须在广播前调用）
    radarDataService->start();
    deviceConfigService->start();
    Serial.println("✅ [BLE] 雷达数据服务和设备配置服务已启动");
    
    refreshBLEAdvertisingData();//刷新BLE广播数据
    BLEDevice::startAdvertising();//启动BLE广播

    Serial.println(String("✅ BLE已启动，设备名称: ") + snName);

    static unsigned long lastRadarStatusUpdate = 0;
    const unsigned long RADAR_STATUS_UPDATE_INTERVAL = 200; // 每200毫秒更新一次雷达状态

    while(1) {
        processBLEConfig();//处理BLE配置命令

        // 定期更新雷达状态特征（受 continuousSendEnabled 控制）
        unsigned long currentTime = millis();
        if (deviceConnected && continuousSendEnabled && (currentTime - lastRadarStatusUpdate >= RADAR_STATUS_UPDATE_INTERVAL)) {
            updateRadarStatus();
            lastRadarStatusUpdate = currentTime;
        }

        if (!deviceConnected && oldDeviceConnected) {//如果设备断开连接且之前连接过
            vTaskDelay(500 / portTICK_PERIOD_MS);
            pServer->startAdvertising();//启动BLE广播
            Serial.println("开始BLE广播");
            oldDeviceConnected = deviceConnected;
        }
        if (deviceConnected && !oldDeviceConnected) {//如果设备连接且之前未连接过
            // 连接后不自动推送，等待客户端发送 CMD_START_CONTINUOUS 命令
            oldDeviceConnected = deviceConnected;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 雷达命令发送任务
 * 每2秒轮流向雷达模组发送11条不同命令，查询心率、呼吸率、睡眠状态等数据
 * @param parameter 任务参数（未使用）
 */
void radarCmdTask(void *parameter) {
    Serial.println("📡 雷达命令发送任务启动");
    initR60ABD1();//初始化雷达模组

    static const uint8_t radar_cmds[][3] = {
        {0x84, 0x81, 0x0F},  // 0x81: 查询心率/呼吸率
        {0x84, 0x8D, 0x0F},  // 0x8D: 查询睡眠状态（受 radarSleepQueryEnabled 控制）
        {0x84, 0x8F, 0x0F},  // 0x8F: 查询体动数据
        {0x84, 0x8E, 0x0F},  // 0x8E: 查询人员存在
        {0x84, 0x91, 0x0F},  // 0x91: 查询呼吸波形
        {0x84, 0x92, 0x0F},  // 0x92: 查询呼吸波形(备用)
        {0x84, 0x83, 0x0F},  // 0x83: 查询心跳波形
        {0x84, 0x84, 0x0F},  // 0x84: 查询心跳波形(备用)
        {0x84, 0x85, 0x0F},  // 0x85: 查询心跳波形(扩展)
        {0x84, 0x86, 0x0F},  // 0x86: 查询心跳波形(扩展)
        {0x84, 0x90, 0x0F}   // 0x90: 查询综合状态（受 radarSleepQueryEnabled 控制）
    };
    // 受控命令索引：0x8D(索引1) 和 0x90(索引10)
    static const size_t SLEEP_CMD_INDICES[] = {1, 10};
    static const size_t SLEEP_CMD_COUNT = sizeof(SLEEP_CMD_INDICES) / sizeof(SLEEP_CMD_INDICES[0]);

    static size_t cmdIndex = 0;//当前命令索引
    static unsigned long lastCmdMillis = 0;//上次发送命令的时间戳
    const unsigned long CMD_INTERVAL = 2000UL;//命令发送间隔

    while (1) {
        unsigned long now = millis();//获取当前时间戳

        if (now - lastCmdMillis >= CMD_INTERVAL) {
            // 检查当前命令是否为受控命令（0x8D/0x90），且开关未开启
            bool isSleepCmd = false;
            for (size_t i = 0; i < SLEEP_CMD_COUNT; i++) {
                if (cmdIndex == SLEEP_CMD_INDICES[i]) {
                    isSleepCmd = true;
                    break;
                }
            }
            if (isSleepCmd && !radarSleepQueryEnabled) {
                // 跳过受控命令，不发送
                cmdIndex++;
                if (cmdIndex >= sizeof(radar_cmds) / sizeof(radar_cmds[0])) {
                    cmdIndex = 0;
                }
                continue;
            }

            sendRadarCommand(
                radar_cmds[cmdIndex][0],
                radar_cmds[cmdIndex][1],
                radar_cmds[cmdIndex][2]
            );

            lastCmdMillis = now;
            cmdIndex++;
            if (cmdIndex >= sizeof(radar_cmds) / sizeof(radar_cmds[0])) {
                cmdIndex = 0;
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

/**
 * @brief 情绪分析任务
 * 每秒更新生理数据、校准基线、分析情绪并输出结果
 * @param parameter 任务参数（未使用）
 */
void emotionAnalysisTask(void *parameter) {
    PhysioDataProcessor* emotionPhysioProcessor = new PhysioDataProcessor();
    emotionAnalyzer = new SimpleEmotionAnalyzer(60);

    static unsigned long lastEmotionAnalysisTime = 0;
    const unsigned long EMOTION_ANALYSIS_INTERVAL = 1000;

    while (1) {
        unsigned long currentTime = millis();

        if (currentTime - lastEmotionAnalysisTime >= EMOTION_ANALYSIS_INTERVAL) {
            lastEmotionAnalysisTime = currentTime;

            if (sensorData.heart_valid || sensorData.breath_valid) {
                float hr = sensorData.heart_valid ? sensorData.heart_rate : 0;
                float rr = sensorData.breath_valid ? sensorData.breath_rate : 0;

                if (hr > 0 || rr > 0) {
                    emotionPhysioProcessor->update(hr, rr,
                        sensorData.heart_valid ? 80 : 0,
                        sensorData.breath_valid ? 80 : 0);

                    HeartRateData hrData = emotionPhysioProcessor->getHeartRateData();
                    RespirationData rrData = emotionPhysioProcessor->getRespirationData();
                    HRVEstimate hrvData = emotionPhysioProcessor->getHRVEstimate();

                    BodyMovementData movementData;
                    memset(&movementData, 0, sizeof(BodyMovementData));
                    movementData.movement = sensorData.body_movement;
                    movementData.movementSmoothed = sensorData.body_movement;
                    movementData.movementMean = sensorData.body_movement;
                    movementData.activityLevel = sensorData.body_movement / 100.0f;
                    movementData.isValid = (sensorData.body_movement >= 0 && sensorData.body_movement <= 100);
                    movementData.timestamp = currentTime;

                    if (hrData.isValid && rrData.isValid) {
                        emotionAnalyzer->calibrateBaseline(hrData, rrData, movementData);
                    }

                    EmotionResult emotionResult = emotionAnalyzer->analyze(hrData, rrData, hrvData, movementData);

                    if (emotionResult.isValid) {
                        // 更新全局缓存
                        updateEmotionResult(emotionResult);

                        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                        Serial.printf("主要情绪:%s (置信度: %.1f%%);",
                            EMOTION_NAMES[emotionResult.primaryEmotion],
                            emotionResult.confidence * 100);
                        Serial.printf("次要情绪: %s倾向\n",
                            EMOTION_NAMES[emotionResult.secondaryEmotion]);
                        Serial.printf("情绪强度:%.1f%% ", emotionResult.intensity * 100);
                        Serial.printf("效价:%.2f(负面到正面) ", emotionResult.valence);
                        Serial.printf("唤醒度:%.2f(平静到激动)\n", emotionResult.arousal);
                        Serial.printf("压力水平:%.1f ", emotionResult.stressLevel);
                        Serial.printf("焦虑水平:%.1f ", emotionResult.anxietyLevel);
                        Serial.printf("放松水平:%.1f ", emotionResult.relaxationLevel);
                        Serial.printf("交感神经活动:%.2f ", emotionResult.sympatheticActivity);
                        Serial.printf("副交感神经活动:%.2f\n", emotionResult.parasympatheticActivity);
                    } else {
                        // 分析结果无效，清除缓存
                        clearEmotionResult();
                    }
                } else {
                    // hr 和 rr 都为 0，清除缓存
                    clearEmotionResult();
                }
            } else {
                // 传感器数据无效，清除缓存
                clearEmotionResult();
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

/**
 * @brief 初始化所有FreeRTOS任务
 * 创建并启动所有后台任务：BOOT按钮监控、LED控制、WiFi监控、MQTT、BLE配置、雷达命令发送、情绪分析、睡眠分析
 */
void initAllTasks() {
    loadDeviceSN();//加载设备序列号

    xTaskCreate(bootButtonMonitorTask, "Boot Button Monitor Task", 2048, NULL, 1, NULL);//创建BOOT按钮监控任务
    xTaskCreate(ledControlTask, "LED Control Task", 2048, NULL, 1, NULL);//创建LED控制任务
    xTaskCreate(wifiMonitorTask, "WiFi Monitor Task", 4096, NULL, 2, NULL);//创建WiFi监控任务
    xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 8192, NULL, 2, &mqttTaskHandle, 1);//创建MQTT任务
    xTaskCreate(bleConfigTask, "BLE Config Task", 4096, NULL, 1, NULL);//创建BLE配置任务
    xTaskCreate(radarCmdTask, "Radar Cmd Task", 2048, NULL, 2, NULL);//创建雷达命令发送任务
    xTaskCreate(pirTask, "PIR Task", 2048, NULL, 1, NULL);//创建红外人体检测任务
    xTaskCreate(emotionAnalysisTask, "Emotion Analysis Task", 4096, NULL, 1, NULL);//创建情绪分析任务
    xTaskCreate(sleepAnalysisTask, "Sleep Analysis Task", 4096, NULL, 1, NULL);//创建睡眠分析任务
}
