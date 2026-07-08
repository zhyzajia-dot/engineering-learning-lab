/**
 * @file wifi_manager.cpp
 * @brief Wi-Fi 扫描、BLE 配网、凭据持久化和断线重连实现。
 *
 * 接收小程序下发的网络信息，并维护设备后续的自动联网状态。
 */
#include "wifi_manager.h"
#include <vector>
#include "ble_tlv_protocol.h"

// 外部变量和函数声明
extern bool deviceConnected;// 设备是否已连接到WiFi网络标志
void setNetworkStatus(NetworkStatus status);// 设置网络状态

// 设备状态推送函数（b3 通道，带去重）
void pushDeviceStatusIfChanged(uint8_t status);

// WiFi专用TLV发送函数声明（简化签名：只传 resultCode）
// 发送WiFi配置结果到BLE
void sendWiFiConfigResultToBLE(uint8_t resultCode,
                               const String& ssid = "", const String& ipAddress = "");
// 发送WiFi扫描结果到BLE
void sendWiFiScanResultToBLE(uint8_t resultCode,
                             const std::vector<WiFiScanResult>& networks = {});
// 发送已保存网络列表结果到BLE
void sendSavedNetworksResultToBLE(bool success, const std::vector<WiFiScanResult>& networks = {});

static String getWiFiSecurityString(int encryptionType) {
    switch (encryptionType) {
        case WIFI_AUTH_OPEN:
            return "OPEN";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2-EAP";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return "WPA2/WPA3";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief WiFi管理器构造函数
 * 初始化WiFi管理器的成员变量
 */
WiFiManager::WiFiManager() {
    savedNetworkCount = 0;
    currentState = WIFI_IDLE;
    lastReconnectAttempt = 0;
    isScanning = false;
    manualConfigActive = false;
    scanInProgress = false;
    reconnectTaskHandle = NULL;
    
    // 创建WiFi操作互斥锁
    wifiMutex = xSemaphoreCreateMutex();
    if (wifiMutex == NULL) {
        Serial.println("❌ 创建WiFi互斥锁失败");
    }
    
    // 创建状态互斥锁
    stateMutex = xSemaphoreCreateMutex();//创建状态互斥锁
    if (stateMutex == NULL) {
        Serial.println("❌ 创建状态互斥锁失败");
    }
}

/**
 * @brief 初始化WiFi管理器
 * 开启Preferences存储，并加载保存的WiFi配置
 */
void WiFiManager::begin() {
    preferences.begin("wifi_manager", false);
    loadWiFiConfigs();


/***************************************测试用*******************************************/
    // 强制写入默认 WiFi 凭据到 Flash（每次上电都执行）
    // 已有同名 SSID 时只会更新密码，不会重复占用槽位
    saveWiFiConfig(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASSWORD);
    Serial.printf("📌 [WiFi] 已写入默认凭据: %s\n", DEFAULT_WIFI_SSID);
    loadWiFiConfigs();
/***************************************测试用*******************************************/




    // 注册WiFi事件监听器
    WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
            case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
                Serial.println("⚠️ [WiFi事件] WiFi断开连接");
                // 根据当前状态推送不同的状态码
                if (currentState == WIFI_CONNECTED) {
                    currentState = WIFI_DISCONNECTED;
                    setNetworkStatus(NET_DISCONNECTED);
                    // 推送 WiFi 断开状态（b3 通道）
                    pushDeviceStatusIfChanged(BleProto::DeviceStatus::WIFI_DISCONNECTED);
                    Serial.println("🔄 [WiFi事件] 已触发重连标志");
                } else if (currentState == WIFI_CONNECTING || currentState == WIFI_IDLE) {
                    // 连接中或空闲时断开，视为连接失败
                    currentState = WIFI_DISCONNECTED;
                    setNetworkStatus(NET_DISCONNECTED);
                    // 推送 WiFi 连接失败状态（b3 通道）
                    pushDeviceStatusIfChanged(BleProto::DeviceStatus::WIFI_FAILED);
                }
                break;
                
            case ARDUINO_EVENT_WIFI_STA_CONNECTED:
                // 推送 WiFi 连接中状态（b3 通道）
                pushDeviceStatusIfChanged(BleProto::DeviceStatus::WIFI_CONNECTING);
                break;
                
            case ARDUINO_EVENT_WIFI_STA_GOT_IP:
                currentState = WIFI_CONNECTED;
                setNetworkStatus(NET_CONNECTED);
                // 推送 WiFi 已连接状态（b3 通道）
                pushDeviceStatusIfChanged(BleProto::DeviceStatus::WIFI_CONNECTED);
                break;
                
            case ARDUINO_EVENT_WIFI_STA_LOST_IP:
                Serial.println("⚠️ [WiFi事件] 丢失IP地址");
                // 推送 WiFi 断开状态（b3 通道）
                pushDeviceStatusIfChanged(BleProto::DeviceStatus::WIFI_DISCONNECTED);
                break;
                
            default:
                break;
        }
    });
    
    // 创建重连任务（低优先级，会被扫描任务抢占）
    if (reconnectTaskHandle == NULL) {
        xTaskCreate(
            reconnectTask,
            "WiFi Reconnect Task",
            8192,
            this,
            1,  // 低优先级
            &reconnectTaskHandle
        );
        Serial.println("✅ WiFi重连任务已创建");
    }
}

/**
 * @brief 加载保存的WiFi配置
 * 从Flash中读取之前保存的WiFi网络配置
 * @return 是否成功加载到配置
 */
bool WiFiManager::loadWiFiConfigs() {
    savedNetworkCount = 0;
    
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        String key = "wifi_" + String(i);// 构建WiFi配置键名
        String configStr = preferences.getString(key.c_str(), "");// 从Preferences中读取字符串
        
        if (configStr.length() > 0) {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, configStr);// 反序列化JSON字符串
            
            if (!error) {
                const char* ssid = doc["ssid"];
                const char* password = doc["password"];
                
                if (ssid && password) {
                    strncpy(savedNetworks[savedNetworkCount].ssid, ssid, 31);// 保存SSID
                    savedNetworks[savedNetworkCount].ssid[31] = '\0';
                    strncpy(savedNetworks[savedNetworkCount].password, password, 63);
                    savedNetworks[savedNetworkCount].password[63] = '\0';
                    savedNetworkCount++;
                    
                    Serial.printf("📶 加载WiFi配置 %d: %s\n", savedNetworkCount, ssid);
                }
            }
        }
    }
    
    Serial.printf("✅ 共加载 %d 个WiFi配置\n", savedNetworkCount);
    return savedNetworkCount > 0;
}

/**
 * @brief 保存WiFi配置
 * 将WiFi网络配置保存到Flash中
 * @param ssid WiFi网络名称
 * @param password WiFi网络密码
 * @return 是否保存成功
 */
bool WiFiManager::saveWiFiConfig(const char* ssid, const char* password) {
    // 检查是否已存在该网络配置
    for (int i = 0; i < savedNetworkCount; i++) {
        // 已存在相同的SSID
        if (strcmp(savedNetworks[i].ssid, ssid) == 0) {
            // 更新现有配置
            strncpy(savedNetworks[i].password, password, 63);// 更新密码
            savedNetworks[i].password[63] = '\0';
            
            JsonDocument doc;
            doc["ssid"] = ssid;
            doc["password"] = password;
            String configStr;
            serializeJson(doc, configStr);// 序列化JSON文档
            
            String key = "wifi_" + String(i);
            preferences.putString(key.c_str(), configStr);// 保存到Preferences
            
            Serial.printf("🔄 更新WiFi配置: %s\n", ssid);
            return true;
        }
    }
    
    // 未找到相同的SSID，添加新的配置
    if (savedNetworkCount < MAX_WIFI_NETWORKS) {
        strncpy(savedNetworks[savedNetworkCount].ssid, ssid, 31);// 保存SSID
        savedNetworks[savedNetworkCount].ssid[31] = '\0';
        strncpy(savedNetworks[savedNetworkCount].password, password, 63);// 保存密码
        savedNetworks[savedNetworkCount].password[63] = '\0';
        
        JsonDocument doc;
        doc["ssid"] = ssid;
        doc["password"] = password;
        String configStr;
        serializeJson(doc, configStr);
        
        String key = "wifi_" + String(savedNetworkCount);
        preferences.putString(key.c_str(), configStr);
        savedNetworkCount++;
        
        Serial.printf("➕ 新增WiFi配置: %s (总计: %d)\n", ssid, savedNetworkCount);
        return true;
    }
    
    Serial.println("❌ WiFi配置已满，无法添加");
    return false;
}

/**
 * @brief 连接到指定WiFi网络
 * 尝试连接到给定的WiFi网络
 * @param ssid WiFi网络名称
 * @param password WiFi网络密码
 * @return 是否连接成功
 */
bool WiFiManager::connectToNetwork(const char* ssid, const char* password) {
    currentState = WIFI_CONNECTING;
    setNetworkStatus(NET_CONNECTING);
    
    WiFi.mode(WIFI_STA);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    WiFi.begin(ssid, password);
    
    unsigned long startTime = millis();
    unsigned long lastStatusPrint = 0;
    
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_CONNECT_TIMEOUT) {
        if (millis() - lastStatusPrint >= 500) {
            lastStatusPrint = millis();
        }
        yield();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ [WiFi] 连接成功！");
        Serial.printf("🌐 IP地址: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("🔒 信号强度: %d dBm\n", WiFi.RSSI());
        
        currentState = WIFI_CONNECTED;
        setNetworkStatus(NET_CONNECTED);
        
        // 清除手动配置标志位，恢复WiFi重连机制
        if (manualConfigActive) {
            manualConfigActive = false;
            Serial.println("🔧 [WiFi] 手动配置完成，WiFi重连机制已恢复");
        }
        
        return true;
    } else {
        Serial.println("❌ [WiFi] 连接超时");
        currentState = WIFI_DISCONNECTED;
        setNetworkStatus(NET_DISCONNECTED);
        
        return false;
    }
}

/**
 * @brief 扫描并匹配WiFi网络
 * 扫描附近的WiFi网络，并尝试匹配已保存的配置
 * 如果找到多个匹配的网络，按信号强度从强到弱依次尝试连接
 * @return 是否成功连接到匹配的网络
 */
bool WiFiManager::scanAndMatchNetworks() {
    // ========== 变量声明区 ==========
    struct CandidateNetwork {
        const char* ssid;
        const char* password;
        int rssi;
    };
    
    CandidateNetwork availableNetworks[MAX_WIFI_NETWORKS];
    int availableCount = 0;
    
    // 尝试获取WiFi互斥锁，最多等待100ms
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("⏸️ [scanAndMatchNetworks] WiFi正在被其他操作占用，跳过扫描");
        return false;
    }
    
    // 首先检查是否有已保存的WiFi网络
    if (savedNetworkCount == 0) {
        Serial.println("⚠️ [WiFi] 没有已保存的WiFi网络，跳过重连扫描");
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    if (isScanning) {
        Serial.println("⏳ [WiFi] 正在扫描中，等待扫描完成...");
        int waitCount = 0;
        while (isScanning && waitCount < 100) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            waitCount++;
        }
        
        if (isScanning) {
            Serial.println("⚠️ [WiFi] 等待超时，跳过本次扫描");
            xSemaphoreGive(wifiMutex);
            return false;
        }
        
        Serial.println("✅ [WiFi] 扫描已完成，开始新的扫描");
    }
    
    Serial.println("🔍 [WiFi] 开始扫描WiFi网络...");
    currentState = WIFI_SCANNING;
    isScanning = true;
    
    // 先完全断开并重置WiFi
    Serial.println("🔄 [WiFi] 重置WiFi硬件...");
    WiFi.disconnect(true);  // true = 关闭WiFi radio
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // 重新初始化WiFi
    WiFi.mode(WIFI_STA);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // 扫描重试机制
    int n = -1;
    int scanRetryCount = 0;
    const int MAX_SCAN_RETRIES = 5;
    
    while (n <= 0 && scanRetryCount < MAX_SCAN_RETRIES && !manualConfigActive) {
        if (scanRetryCount > 0) {
            Serial.printf("🔄 [WiFi] 扫描重试 %d/%d...\n", scanRetryCount, MAX_SCAN_RETRIES);
            // 重试前再次重置WiFi
            WiFi.disconnect(true);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            WiFi.mode(WIFI_STA);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        
        n = WiFi.scanNetworks(false, true, false, 1000);  // 增加扫描时间到1秒
        Serial.printf("🔍 扫描结果: %d 个WiFi网络 (尝试 %d/%d), WiFi状态: %d\n", 
                     n, scanRetryCount + 1, MAX_SCAN_RETRIES, WiFi.status());
        scanRetryCount++;
    }
    
    // 检查是否被蓝牙配网中断
    if (manualConfigActive) {
        Serial.println("🔧 [WiFi] 重连扫描被蓝牙配网中断");
        currentState = WIFI_DISCONNECTED;
        isScanning = false;
        WiFi.scanDelete();
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    if (n <= 0) {
        Serial.println("❌ 多次重试后仍未扫描到WiFi网络，WiFi硬件可能异常");
        currentState = WIFI_DISCONNECTED;
        isScanning = false;
        // 清理WiFi状态
        WiFi.scanDelete();
        // 尝试完全重置WiFi
        WiFi.mode(WIFI_OFF);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        WiFi.mode(WIFI_STA);
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    // 发送扫描结果到蓝牙（重连时的扫描也发送）
    // 再次检查是否被中断
    if (manualConfigActive) {
        Serial.println("🔧 [WiFi] 重连扫描被蓝牙配网中断");
        currentState = WIFI_DISCONNECTED;
        isScanning = false;
        WiFi.scanDelete();
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    if (deviceConnected) {
        // 构建WiFi网络列表
        std::vector<WiFiScanResult> networks;
        for (int i = 0; i < n; ++i) {
            if (WiFi.RSSI(i) >= MIN_RSSI_THRESHOLD) {
                WiFiScanResult network;
                network.ssid = WiFi.SSID(i);
                network.rssi = WiFi.RSSI(i);
                network.security = getWiFiSecurityString(WiFi.encryptionType(i));//获取WiFi安全类型
                networks.push_back(network);
            }
        }
        
        sendWiFiScanResultToBLE(BleProto::ErrorCode::SUCCESS, networks);
        Serial.printf("📱 [BLE] 发送重连扫描结果，共 %d 个网络\n", static_cast<int>(networks.size()));
    }
    
    // 遍历已保存的网络，寻找匹配的网络
    for (int i = 0; i < savedNetworkCount; i++) {
        for (int j = 0; j < n; j++) {
            if (WiFi.SSID(j) == String(savedNetworks[i].ssid)) {
                int rssi = WiFi.RSSI(j);
                
                // 检查信号强度是否符合要求
                if (rssi >= MIN_RSSI_THRESHOLD) {
                    // 添加到可用网络列表
                    if (availableCount < MAX_WIFI_NETWORKS) {
                        availableNetworks[availableCount].ssid = savedNetworks[i].ssid;
                        availableNetworks[availableCount].password = savedNetworks[i].password;
                        availableNetworks[availableCount].rssi = rssi;
                        availableCount++;
                    }
                } else {
                    Serial.printf("⚠️ 信号强度过低，跳过\n");
                }
                break; // 找到匹配后跳出内层循环
            }
        }
    }
    
    // 如果没有找到任何可用网络
    if (availableCount == 0) {
        Serial.println("❌ 未找到匹配的WiFi网络或信号过弱");
        WiFi.scanDelete();
        currentState = WIFI_DISCONNECTED;
        isScanning = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    // 按信号强度从强到弱排序（冒泡排序）
    for (int i = 0; i < availableCount - 1; i++) {
        for (int j = 0; j < availableCount - i - 1; j++) {
            if (availableNetworks[j].rssi < availableNetworks[j + 1].rssi) {
                CandidateNetwork temp = availableNetworks[j];
                availableNetworks[j] = availableNetworks[j + 1];
                availableNetworks[j + 1] = temp;
            }
        }
    }
    
    Serial.printf("📋 找到 %d 个可用网络，按信号强度排序:\n", availableCount);
    for (int i = 0; i < availableCount; i++) {
        Serial.printf("  %d. %s (信号: %d dBm)\n", i + 1, 
                     availableNetworks[i].ssid, availableNetworks[i].rssi);
    }
    
    WiFi.scanDelete();
    
    // 依次尝试连接所有可用网络
    for (int i = 0; i < availableCount; i++) {
        // 检查是否被蓝牙配网中断
        if (manualConfigActive) {
            Serial.println("🔧 [WiFi] 重连被蓝牙配网中断");
            currentState = WIFI_DISCONNECTED;
            isScanning = false;
            xSemaphoreGive(wifiMutex);
            return false;
        }
        
        Serial.printf("🔄 [%d/%d] 尝试连接: %s (信号: %d dBm)\n", 
                     i + 1, availableCount,
                     availableNetworks[i].ssid, 
                     availableNetworks[i].rssi);
        
        vTaskDelay(300 / portTICK_PERIOD_MS);
        
        if (connectToNetwork(availableNetworks[i].ssid, availableNetworks[i].password)) {
            Serial.printf("✅ 成功连接到: %s\n", availableNetworks[i].ssid);
            isScanning = false;
            xSemaphoreGive(wifiMutex);
            return true;
        }
        
        // 连接失败，继续尝试下一个
        Serial.printf("❌ %s 连接失败\n", availableNetworks[i].ssid);
        
        // 如果不是最后一个，准备尝试下一个
        if (i < availableCount - 1) {
            Serial.println("➡️ 尝试下一个网络...");
        }
    }
    
    Serial.println("❌ 所有可用网络均连接失败");
    currentState = WIFI_DISCONNECTED;// 扫描失败，设置为断开连接状态
    isScanning = false;
    
    xSemaphoreGive(wifiMutex);
    return false;
}

/**
 * @brief 初始化WiFi连接
 * 启动WiFi初始化过程，尝试连接到已保存的网络
 * @return 是否初始化成功
 */
bool WiFiManager::initializeWiFi() {
    Serial.println("🚀 [WiFi] 初始化WiFi连接...");
    
    if (savedNetworkCount == 0) {
        Serial.println("⚠️ 未保存的WiFi配置");
        currentState = WIFI_IDLE;
        return false;
    }
    
    if (scanAndMatchNetworks()) {
        Serial.println("✅ WiFi初始化成功");
        return true;
    } else {
        Serial.println("❌ WiFi初始化失败");
        currentState = WIFI_DISCONNECTED;
        return false;
    }
}

/**
 * @brief 扫描WiFi网络并发送结果
 * 扫描附近的WiFi网络，过滤信号弱的网络，将结果通过BLE发送给客户端
 */
void WiFiManager::scanAndSendResults() {
    // 尝试获取WiFi互斥锁，最多等待100ms
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("⏸️ [scanAndSendResults] WiFi正在被其他操作占用，跳过扫描");
        if (deviceConnected) {
            sendWiFiScanResultToBLE(BleProto::ErrorCode::ERR_WIFI_BUSY);
        }
        return;
    }
    
    if (isScanning) {
        Serial.println("⏳ [WiFi] 正在扫描中，等待扫描完成...");
        int waitCount = 0;
        while (isScanning && waitCount < 50) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            waitCount++;
        }
        
        if (isScanning) {
            Serial.println("⚠️ [WiFi] 等待超时，跳过本次扫描");
            if (deviceConnected) {
                sendWiFiScanResultToBLE(BleProto::ErrorCode::ERR_WIFI_SCAN_TIMEOUT);
            }
            xSemaphoreGive(wifiMutex);
            return;
        }
        
        Serial.println("✅ [WiFi] 扫描已完成，开始新的扫描");
    }
    
    Serial.println("📱 [BLE-WiFi] 开始WiFi扫描...");
    isScanning = true;
    
    // 如果已连接，先断开以确保扫描质量
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("📶 WiFi已连接，断开后进行同步扫描");
        WiFi.disconnect(true);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // 使用同步扫描，确保扫描完整完成，避免时序竞争
    Serial.println("🔍 [WiFi] 使用同步扫描...");
    int n = WiFi.scanNetworks(false, true, false, 300); // false = 同步扫描
    
    // 如果同步扫描失败，使用主动扫描重试
    if (n <= 0) {
        Serial.println("🔄 同步扫描失败，重试主动扫描...");
        
        int retryCount = 3;
        while (n <= 0 && retryCount > 0) {
            Serial.printf("🔍 同步扫描WiFi网络 (尝试 %d/3)...\n", 4 - retryCount);
            
            // 使用主动扫描重试，增加扫描时间
            n = WiFi.scanNetworks(false, true, false, 800);
            
            if (n <= 0) {
                Serial.printf("❌ 扫描失败，返回值: %d, 剩余重试: %d\n", n, retryCount - 1);
                retryCount--;
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
    
    Serial.printf("🔍 最终扫描到 %d 个WiFi网络\n", n);
    
    if (n <= 0) {
        Serial.println("❌ 未扫描到任何WiFi网络或扫描失败");
        isScanning = false;
        if (deviceConnected) {
            sendWiFiScanResultToBLE(BleProto::ErrorCode::ERR_WIFI_SCAN_TIMEOUT);
        }
        xSemaphoreGive(wifiMutex);
        return;
    }
    
    // 构建WiFi网络列表
    std::vector<WiFiScanResult> networks;
    for (int i = 0; i < n; ++i) {
        if (WiFi.RSSI(i) >= MIN_RSSI_THRESHOLD) {
            WiFiScanResult network;
            network.ssid = WiFi.SSID(i);
            network.rssi = WiFi.RSSI(i);
            network.security = getWiFiSecurityString(WiFi.encryptionType(i));
            networks.push_back(network);
        }
    }
    
    Serial.printf("✅ 发送WiFi扫描结果，包含 %d 个可用网络\n", static_cast<int>(networks.size()));
    
    WiFi.scanDelete();
    isScanning = false;
    
    if (deviceConnected) {
        sendWiFiScanResultToBLE(BleProto::ErrorCode::SUCCESS, networks);
    }
    
    xSemaphoreGive(wifiMutex);
}

/**
 * @brief 处理配网数据
 * 处理从客户端收到的WiFi配网信息，先扫描WiFi是否有匹配的网络，再尝试连接并保存
 * @param ssid WiFi网络名称
 * @param password WiFi网络密码
 * @return 是否配置成功
 */
bool WiFiManager::handleConfigurationData(const char* ssid, const char* password) {
    // ⚠️ 关键修改：在获取锁之前就设置标志位，让重连任务立即暂停
    manualConfigActive = true;
    Serial.println("🔧 [WiFi] 手动配置模式已激活，立即停止所有 WiFi 操作");
    
    // 尝试获取 WiFi 互斥锁，增加等待时间到 5 秒
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("⏸️ [handleConfigurationData] WiFi 正在被其他操作占用，跳过配网");
        manualConfigActive = false;  // 重置标志位
        if (deviceConnected) {
            sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_BUSY);
        }
        return false;
    }
    
    // 立即断开当前 WiFi 连接
    WiFi.disconnect(true);
    
    // 如果正在扫描，立即停止扫描
    if (isScanning || scanInProgress) {
        Serial.println("🛑 [WiFi] 检测到正在扫描，立即停止");
        WiFi.scanDelete();
        isScanning = false;
        scanInProgress = false;
        currentState = WIFI_DISCONNECTED;
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    Serial.printf("📱 [BLE-WiFi] 收到配网信息: SSID='%s'\n", ssid);
    
    if (ssid == nullptr || strlen(ssid) == 0) {
        Serial.println("❌ 配网参数无效：SSID为空");
        manualConfigActive = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    // 如果密码为空，尝试从已保存的WiFi中查找密码
    const char* actualPassword = password;
    char savedPassword[64] = {0};
    bool foundSavedPassword = false;
    
    if (password == nullptr || strlen(password) == 0) {
        Serial.println("🔑 密码为空，尝试从已保存的WiFi中查找...");
        
        for (int i = 0; i < savedNetworkCount; i++) {
            if (strcmp(savedNetworks[i].ssid, ssid) == 0) {
                strncpy(savedPassword, savedNetworks[i].password, 63);
                savedPassword[63] = '\0';
                actualPassword = savedPassword;
                foundSavedPassword = true;
                Serial.printf("✅ 找到已保存的WiFi密码: %s\n", ssid);
                break;
            }
        }
        
        if (!foundSavedPassword) {
            Serial.println("⚠️ 未找到已保存的密码，将尝试无密码连接");
            actualPassword = "";
        }
    }
    
    if (isScanning) {
        Serial.println("⏳ [WiFi] 正在扫描中，等待扫描完成...");
        int waitCount = 0;
        while (isScanning && waitCount < 50) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            waitCount++;
        }
        
        if (isScanning) {
            Serial.println("⚠️ [WiFi] 等待超时");
            if (deviceConnected) {
                sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_SCAN_TIMEOUT);
            }
            manualConfigActive = false;
            xSemaphoreGive(wifiMutex);
            return false;
        }
        
        Serial.println("✅ [WiFi] 扫描已完成，开始新的扫描");
    }
    
    // 先扫描WiFi网络，检查是否存在匹配的网络
    Serial.println("🔍 [WiFi] 扫描WiFi网络，检查是否存在匹配的网络...");
    currentState = WIFI_SCANNING;
    isScanning = true;
    
    // 重新初始化WiFi
    WiFi.mode(WIFI_STA);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // 扫描重试机制
    int n = -1;
    int scanRetryCount = 0;
    const int MAX_SCAN_RETRIES = 3;
    
    while (n <= 0 && scanRetryCount < MAX_SCAN_RETRIES && manualConfigActive) {
        if (scanRetryCount > 0) {
            Serial.printf("🔄 [WiFi] 扫描重试 %d/%d...\n", scanRetryCount, MAX_SCAN_RETRIES);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        
        n = WiFi.scanNetworks(false, true, false, 500);
        Serial.printf("🔍 扫描结果: %d 个WiFi网络 (尝试 %d/%d)\n", n, scanRetryCount + 1, MAX_SCAN_RETRIES);
        scanRetryCount++;
    }
    
    // 检查是否被中断
    if (!manualConfigActive) {
        Serial.println("⚠️ [WiFi] 配网被中断");
        WiFi.scanDelete();
        isScanning = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    if (n <= 0) {
        Serial.println("❌ 未扫描到任何WiFi网络");
        WiFi.scanDelete();
        isScanning = false;
        currentState = WIFI_DISCONNECTED;
        
        if (deviceConnected) {
            sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_SSID_NOT_FOUND);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        manualConfigActive = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    bool networkFound = false;
    bool signalTooWeak = false;
    int foundRssi = 0;
    
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == String(ssid)) {
            foundRssi = WiFi.RSSI(i);
            Serial.printf("📶 找到匹配网络: %s, 信号: %d dBm\n", ssid, foundRssi);
            
            if (foundRssi >= MIN_RSSI_THRESHOLD) {
                Serial.printf("✅ 信号强度符合要求，准备连接...\n");
                networkFound = true;
                break;
            } else {
                Serial.printf("⚠️ 信号强度过低，跳过\n");
                signalTooWeak = true;
            }
        }
    }
    
    if (!networkFound) {
        String errorMsg;
        
        if (signalTooWeak) {
            Serial.printf("❌ 目标WiFi信号过弱: %d dBm (阈值: %d dBm)\n", foundRssi, MIN_RSSI_THRESHOLD);
            if (deviceConnected) {
                sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_SIGNAL_WEAK);
            }
        } else {
            Serial.println("❌ 未找到目标WiFi网络");
            if (deviceConnected) {
                sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_SSID_NOT_FOUND);
            }
        }
        
        WiFi.scanDelete();
        isScanning = false;
        currentState = WIFI_DISCONNECTED;
        
        if (manualConfigActive) {
            manualConfigActive = false;
            Serial.println("🔧 [WiFi] 手动配置失败，WiFi重连机制已恢复");
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        manualConfigActive = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }
    
    WiFi.scanDelete();
    isScanning = false;
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // 尝试连接到指定网络
    if (connectToNetwork(ssid, actualPassword)) {
        // 连接成功后保存配置
        if (saveWiFiConfig(ssid, actualPassword)) {
            Serial.println("✅ WiFi配置成功并已保存");
            
            if (deviceConnected) {
                sendWiFiConfigResultToBLE(BleProto::ErrorCode::SUCCESS,
                                          ssid, WiFi.localIP().toString());
            }
            manualConfigActive = false;
            xSemaphoreGive(wifiMutex);
            return true;
        }
    }
    
    Serial.println("❌ WiFi配置失败");
    isScanning = false;
    currentState = WIFI_DISCONNECTED;
    
    if (manualConfigActive) {
        manualConfigActive = false;
        Serial.println("🔧 [WiFi] 手动配置失败，WiFi重连机制已恢复");
    }
    
    if (deviceConnected) {
        sendWiFiConfigResultToBLE(BleProto::ErrorCode::ERR_WIFI_WRONG_PASSWORD);
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    manualConfigActive = false;
    xSemaphoreGive(wifiMutex);
    return false;
}

/**
 * @brief 处理WiFi重连
 * 检查WiFi连接状态，当断开连接时尝试重连
 */
void WiFiManager::handleReconnect() {
    // 尝试获取WiFi互斥锁，最多等待100ms
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("⏸️ [handleReconnect] WiFi正在被其他操作占用，跳过重连");
        return;
    }

    bool needFallbackScan = false;

    // 手动配置模式下暂停自动重连
    if (manualConfigActive) {
        Serial.println("⏸️ [handleReconnect] 手动配置模式，跳过重连");
        xSemaphoreGive(wifiMutex);
        return;
    }

    // 扫描进行中，暂停重连
    if (scanInProgress) {
        Serial.println("⏸️ [handleReconnect] 扫描进行中，跳过重连");
        xSemaphoreGive(wifiMutex);
        return;
    }

    // 检查是否有已保存的WiFi网络
    if (savedNetworkCount == 0) {
        Serial.println("⚠️ [handleReconnect] 没有已保存的WiFi网络，跳过重连");
        xSemaphoreGive(wifiMutex);
        return;
    }

    // 检查当前是否已连接
    if (currentState == WIFI_CONNECTED) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("✅ [handleReconnect] WiFi已连接，无需重连");
            xSemaphoreGive(wifiMutex);
            return;
        }
        // 连接已断开
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            currentState = WIFI_DISCONNECTED;
            setNetworkStatus(NET_DISCONNECTED);
            xSemaphoreGive(stateMutex);
        }
        Serial.println("⚠️ [handleReconnect] WiFi连接断开，开始重连");
    }

    // 处理重连逻辑
    if (currentState == WIFI_DISCONNECTED) {
        Serial.println("🔄 [handleReconnect] 开始快速重连...");
        
        // 直接尝试重连已保存的网络（不扫描，更快速）
        for (int i = 0; i < savedNetworkCount; i++) {
            Serial.printf("🔄 [handleReconnect] 尝试重连: %s\n", savedNetworks[i].ssid);
            
            // 使用简化的重连方式
            WiFi.mode(WIFI_STA);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            WiFi.begin(savedNetworks[i].ssid, savedNetworks[i].password);
            
            // 等待连接
            unsigned long startTime = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 8000) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                // 检查是否被中断（蓝牙配网或扫描）
                if (manualConfigActive) {
                    Serial.println("🔧 [handleReconnect] 被蓝牙配网中断");
                    xSemaphoreGive(wifiMutex);
                    return;
                }
                if (scanInProgress) {
                    Serial.println("🔍 [handleReconnect] 被 WiFi 扫描中断");
                    xSemaphoreGive(wifiMutex);
                    return;
                }
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("✅ [handleReconnect] 重连成功: %s\n", savedNetworks[i].ssid);
                Serial.printf("🌐 IP地址: %s\n", WiFi.localIP().toString().c_str());
                Serial.printf("📶 信号强度: %d dBm\n", WiFi.RSSI());
                
                currentState = WIFI_CONNECTED;
                setNetworkStatus(NET_CONNECTED);
                xSemaphoreGive(wifiMutex);
                return;
            }
            
            Serial.printf("❌ [handleReconnect] 重连失败: %s (状态:%d)\n", 
                         savedNetworks[i].ssid, WiFi.status());
            WiFi.disconnect(false);  // false = 不关闭radio
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        
        needFallbackScan = true;
    }

    xSemaphoreGive(wifiMutex);

    if (needFallbackScan) {
        Serial.println("🔍 [handleReconnect] 尝试扫描匹配...");
        if (scanAndMatchNetworks()) {
            Serial.println("✅ [handleReconnect] 扫描匹配成功");
        } else {
            Serial.println("❌ [handleReconnect] 扫描匹配失败，稍后重试");
        }
    }
}

/**
 * @brief 检查WiFi是否已连接
 * @return WiFi是否已成功连接
 */
bool WiFiManager::isConnected() {
    return currentState == WIFI_CONNECTED && WiFi.status() == WL_CONNECTED;
}

/**
 * @brief 添加WiFi配置
 * 向保存的配置中添加新的WiFi网络
 * @param ssid WiFi网络名称
 * @param password WiFi网络密码
 * @return 是否添加成功
 */
bool WiFiManager::addWiFiConfig(const char* ssid, const char* password) {
    return saveWiFiConfig(ssid, password);
}

/**
 * @brief 删除指定WiFi配置
 * 根据SSID精确删除已保存的WiFi配置，并重新整理Flash中的索引
 * 
 * 行为约束：
 * - 精确按 SSID 删除（大小写敏感）
 * - 删除成功后如果当前连接的就是该 SSID，则立即断开
 * - 删除失败时不修改内存和 Flash
 * - 整个过程受 wifiMutex 保护，阻止重连逻辑并发访问
 * 
 * @param ssid 要删除的WiFi网络名称
 * @param existed 输出参数，可选，表示删除前是否存在该SSID
 * @return 是否删除成功（存在且持久化成功）
 */
bool WiFiManager::removeWiFiConfig(const char* ssid, bool* existed) {
    if (existed != nullptr) {
        *existed = false;
    }

    // 参数校验
    if (ssid == nullptr || strlen(ssid) == 0) {
        Serial.println("❌ [WiFi] 删除失败：SSID 为空");
        return false;
    }

    // 获取互斥锁，保护整个删除过程
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("❌ [WiFi] 删除失败：无法获取互斥锁");
        return false;
    }

    // 阻止重连逻辑继续使用当前列表
    manualConfigActive = true;

    // 查找目标 SSID
    int removeIndex = -1;
    for (int i = 0; i < savedNetworkCount; i++) {
        if (strcmp(savedNetworks[i].ssid, ssid) == 0) {
            removeIndex = i;
            break;
        }
    }

    // 未找到目标 SSID
    if (removeIndex < 0) {
        Serial.printf("⚠️ [WiFi] 未找到要删除的配置: %s\n", ssid);
        manualConfigActive = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }

    if (existed != nullptr) {
        *existed = true;
    }

    // 判断是否删除当前连接中的 SSID
    bool isCurrentConnectedTarget = false;
    if (WiFi.status() == WL_CONNECTED) {
        String currentSSID = WiFi.SSID();
        if (currentSSID.equals(ssid)) {
            isCurrentConnectedTarget = true;
            Serial.printf("🔗 [WiFi] 删除的是当前连接的 WiFi: %s\n", ssid);
        }
    }

    // 构造“删除后新列表”到临时数组（不直接改 savedNetworks）
    WiFiConfig newConfigs[MAX_WIFI_NETWORKS];
    int newCount = 0;
    for (int i = 0; i < savedNetworkCount; i++) {
        if (i != removeIndex) {
            newConfigs[newCount] = savedNetworks[i];
            newCount++;
        }
    }

    // 先完整写 Flash，再提交内存
    // 清空所有旧的 wifi_N 键
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        String key = "wifi_" + String(i);
        preferences.remove(key.c_str());
    }

    // 按新列表重写 Flash
    bool persistSuccess = true;
    for (int i = 0; i < newCount; i++) {
        JsonDocument doc;
        doc["ssid"] = newConfigs[i].ssid;
        doc["password"] = newConfigs[i].password;

        String configStr;
        serializeJson(doc, configStr);

        String key = "wifi_" + String(i);
        if (preferences.putString(key.c_str(), configStr) == 0) {
            Serial.printf("❌ [WiFi] 持久化失败: %s\n", newConfigs[i].ssid);
            persistSuccess = false;
            break;
        }
    }

    // 如果持久化失败，不更新内存，直接返回
    if (!persistSuccess) {
        Serial.println("❌ [WiFi] 删除失败：持久化写入失败，内存未修改");
        manualConfigActive = false;
        xSemaphoreGive(wifiMutex);
        return false;
    }

    // 持久化成功，提交内存状态
    memcpy(savedNetworks, newConfigs, sizeof(WiFiConfig) * newCount);
    savedNetworkCount = newCount;
    // 清空尾部剩余项
    for (int i = newCount; i < MAX_WIFI_NETWORKS; i++) {
        memset(&savedNetworks[i], 0, sizeof(WiFiConfig));
    }

    // 如果删的是当前连接 WiFi，立即断开
    if (isCurrentConnectedTarget) {
        Serial.printf("🔌 [WiFi] 断开当前连接: %s\n", ssid);
        WiFi.disconnect(false);
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            currentState = WIFI_DISCONNECTED;
            setNetworkStatus(NET_DISCONNECTED);
            xSemaphoreGive(stateMutex);
        }
    }

    // 恢复自动重连
    manualConfigActive = false;
    xSemaphoreGive(wifiMutex);

    Serial.printf("🗑️ [WiFi] 已删除配置: %s，剩余 %d 个%s\n", 
                  ssid, savedNetworkCount, 
                  isCurrentConnectedTarget ? "（已断开连接）" : "");
    return true;
}

/**
 * @brief 清除所有WiFi配置
 * 删除所有保存的WiFi网络配置，受互斥锁保护
 */
void WiFiManager::clearAllConfigs() {
    // 获取互斥锁
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("❌ [WiFi] 清空失败：无法获取互斥锁");
        return;
    }

    // 阻止重连逻辑
    manualConfigActive = true;

    // 清空 Flash
    for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
        String key = "wifi_" + String(i);
        preferences.remove(key.c_str());
    }

    // 清空内存
    savedNetworkCount = 0;
    memset(savedNetworks, 0, sizeof(savedNetworks));

    // 断开当前连接（如果连接的是保存的网络）
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("🔌 [WiFi] 断开当前连接");
        WiFi.disconnect(false);
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            currentState = WIFI_DISCONNECTED;
            setNetworkStatus(NET_DISCONNECTED);
            xSemaphoreGive(stateMutex);
        }
    }

    // 恢复自动重连
    manualConfigActive = false;
    xSemaphoreGive(wifiMutex);

    Serial.println("🗑️ 已清除所有WiFi配置");
}

/**
 * @brief 获取已保存的网络数量
 * @return 已保存的WiFi网络配置数量
 */
int WiFiManager::getSavedNetworkCount() {
    return savedNetworkCount;
}

/**
 * @brief 获取已保存的WiFi网络列表
 * 将保存的WiFi网络配置通过BLE发送给客户端
 */
void WiFiManager::getSavedNetworks() {
    Serial.printf("📋 [WiFi] 获取已保存的WiFi网络列表，共 %d 个\n", savedNetworkCount);
    
    // 构建保存的网络列表
    std::vector<WiFiScanResult> networks;
    for (int i = 0; i < savedNetworkCount; i++) {
        WiFiScanResult network;
        network.ssid = String(savedNetworks[i].ssid);// SSID直接使用保存的值
        network.password = String(savedNetworks[i].password);// 密码直接使用保存的值
        network.rssi = 0; // 保存的网络不显示RSSI
        network.security = ""; // 保存的网络不显示安全类型
        networks.push_back(network);
    }
    
    if (deviceConnected) {
        sendSavedNetworksResultToBLE(true, networks);
    }
    
    Serial.printf("📤 [WiFi] 发送已保存的WiFi网络列表，共 %d 个\n", static_cast<int>(networks.size()));
}

/**
 * @brief 重连任务 - 后台持续运行
 * 优先级低，会被扫描任务抢占
 * @param parameter WiFiManager实例指针
 */
void WiFiManager::reconnectTask(void* parameter) {
    WiFiManager* manager = (WiFiManager*)parameter;// 获取WiFiManager实例指针
    
    Serial.println("📡 [重连任务] 启动");
    Serial.printf("📡 [重连任务] 初始状态: %d, manualConfigActive: %d\n", 
                 manager->currentState, manager->manualConfigActive);
    
    while (true) {
        // ⚠️ 方案 C：在循环开头检查 BLE 配网标志位
        // 如果 BLE 正在配网，立即跳过本次重连尝试
        if (manager->manualConfigActive) {
            Serial.println("🔵 [重连任务] 检测到 BLE 配网，暂停重连");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        // ⚠️ 检查是否正在扫描
        if (manager->scanInProgress) {
            Serial.println("🔍 [重连任务] 检测到 WiFi 扫描，暂停重连");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        // 检查是否正在扫描或手动配置中（双重保险）
        if (manager->scanInProgress || manager->manualConfigActive) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }
        
        // 执行正常重连逻辑
        if (manager->currentState == WIFI_DISCONNECTED) {
            
            unsigned long currentTime = millis();
            unsigned long timeSinceLastAttempt = currentTime - manager->lastReconnectAttempt;
            
            // 断开后立即尝试重连（前 3 次），之后按间隔重连
            static int quickRetryCount = 0;
            bool shouldRetry = false;
            
            if (quickRetryCount < 3) {
                // 前 3 次快速重连（每次 1 秒间隔）
                shouldRetry = (timeSinceLastAttempt >= 1000);
                if (shouldRetry) {
                    quickRetryCount++;
                    Serial.printf("🚀 [重连任务] 快速重连 #%d\n", quickRetryCount);
                }
            } else {
                // 之后按正常间隔重连
                shouldRetry = (timeSinceLastAttempt >= WIFI_RECONNECT_INTERVAL);
            }
            
            if (shouldRetry) {
                manager->lastReconnectAttempt = currentTime;
                Serial.println("🔄 [重连任务] 尝试重连...");
                Serial.printf("🔄 [重连任务] 距上次尝试: %lu ms, WiFi状态: %d\n", 
                             timeSinceLastAttempt, WiFi.status());
                manager->handleReconnect();
                
                // 如果重连成功，重置快速重连计数
                if (manager->currentState == WIFI_CONNECTED) {
                    quickRetryCount = 0;
                }
            }
        }
        
        // 检查实际WiFi状态
        if (manager->currentState == WIFI_CONNECTED && WiFi.status() != WL_CONNECTED) {
            if (xSemaphoreTake(manager->stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                manager->currentState = WIFI_DISCONNECTED;
                setNetworkStatus(NET_DISCONNECTED);
                xSemaphoreGive(manager->stateMutex);
                Serial.println("⚠️ [重连任务] 检测到连接断开");
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 启动扫描 - 抢占式
 * 通过设置 scanInProgress 标志通知重连任务暂停，然后执行扫描
 * @param timeoutMs 扫描超时时间（毫秒）
 * @return 是否成功启动扫描
 */
bool WiFiManager::startScan(uint32_t timeoutMs) {
    Serial.println("🔔 [startScan] 进入扫描启动函数");
    Serial.printf("🔔 [startScan] scanInProgress: %d, currentState: %d\n", 
                 scanInProgress, currentState);
    
    // 检查是否已在扫描中
    if (scanInProgress) {
        Serial.println("⏳ [startScan] 扫描已在进行中");
        return false;
    }
    
    Serial.println("🔔 [startScan] 设置扫描标志...");
    
    // ⚠️ 关键修改：在获取锁之前就设置标志位，让重连任务立即暂停
    scanInProgress = true;
    Serial.println("✅ [startScan] scanInProgress 已设置为 true");
    
    // 获取 WiFi 互斥锁，增加等待时间到 5 秒
    if (xSemaphoreTake(wifiMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("⏸️ [startScan] WiFi 正在被其他操作占用，扫描失败");
        scanInProgress = false;  // 重置标志位
        return false;
    }
    
    // 获取状态锁，修改状态
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        currentState = WIFI_SCANNING;
        xSemaphoreGive(stateMutex);
        Serial.println("✅ [startScan] 状态已设置为 WIFI_SCANNING");
    }
    
    Serial.println("🔍 [startScan] 开始扫描...");
    
    // 执行实际扫描
    scanAndSendResults();
    
    // 扫描完成，释放锁
    xSemaphoreGive(wifiMutex);
    
    // 扫描完成，清除标志，恢复重连任务
    scanInProgress = false;
    Serial.println("✅ [startScan] scanInProgress 已设置为 false");
    
    // 恢复状态
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (currentState == WIFI_SCANNING) {
            currentState = (WiFi.status() == WL_CONNECTED) ? WIFI_CONNECTED : WIFI_DISCONNECTED;
        }
        xSemaphoreGive(stateMutex);
        Serial.printf("✅ [startScan] 状态已恢复为: %d, WiFi状态: %d\n", 
                     currentState, WiFi.status());
    }
    
    Serial.println("✅ [startScan] 扫描完成");
    return true;
}
