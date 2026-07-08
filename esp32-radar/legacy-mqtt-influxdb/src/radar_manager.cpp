#include "radar_manager.h"
#include "tasks_manager.h"
#include "wifi_manager.h"
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <vector>

extern Preferences preferences; // Flash存储对象
extern WiFiManager wifiManager; // WiFi管理器对象

const int BAUD_RATE = 115200; // 串口波特率
const int UART1_RX = 11; // UART1接收新板引脚
const int UART1_TX = 10; // UART1发送新板引脚
//const int UART1_RX = 3; // UART1接收旧板引脚
//const int UART1_TX = 2; // UART1发送旧板引脚

const uint32_t PHASE_SEND_INTERVAL = 1; // 相位数据发送间隔（毫秒）
const uint32_t VITAL_SEND_INTERVAL = 10; // 生命体征数据发送间隔（毫秒）

const unsigned long SENSOR_TIMEOUT = 40000; // 传感器超时时间（毫秒）
static uint32_t packetCounter = 0; // 数据包计数器
static bool shouldSendOtherData = false; // 是否发送其他数据标志

const unsigned long SLEEP_DATA_INTERVAL = 5000; // 睡眠数据发送间隔（毫秒）

static int dnsFailCount = 0; // DNS解析失败计数器
const int DNS_FAIL_RESET_THRESHOLD = 3; // DNS失败重置网络阈值

void resetWiFiConnection(); // WiFi连接重置函数声明
void syncAllDeviceStatusToBLE(); // BLE连接时同步所有设备状态

//const char* influxDBHost = "8.134.11.76"; // InfluxDB服务器公网地址
const char* influxDBHost = "www.lmhrt.cn";  // InfluxDB服务器域名地址
const int influxDBPort = 8086; // InfluxDB服务器端口
const char* influxDBToken = "KuTa5ZsqoHIhi2IglOO06zExUYw1_mJ6K0mcA9X1y6O6CJDog3_Cgr8mUw1SwpuCCKRElqxa6wAhrrhsYPytkg=="; // InfluxDB访问令牌
const char* influxDBOrg = "gzlg"; // InfluxDB组织名称
const char* influxDBBucket = "gzlg"; // InfluxDB存储桶名称

uint8_t presence_Bit = 1; // 存在标志位

// 当前设备状态（用于 b3 去重推送，各状态机独立跟踪）
static uint8_t currentWiFiStatus = 0;          // WiFi 状态
static uint8_t currentMqttStatus = 0;          // MQTT 状态
static uint8_t currentRadarSleepStatus = 0;    // 雷达睡眠查询状态

SensorData sensorData; // 传感器数据结构体
HardwareSerial mySerial1(1); // 硬件串口1对象

QueueHandle_t phaseDataQueue; // 相位数据队列句柄
QueueHandle_t vitalDataQueue; // 生命体征数据队列句柄
QueueHandle_t uartQueue; // UART数据队列句柄
TaskHandle_t bleSendTaskHandle = NULL; // BLE发送任务句柄
TaskHandle_t vitalSendTaskHandle = NULL; // 生命体征发送任务句柄
TaskHandle_t uartProcessTaskHandle = NULL; // UART处理任务句柄

BLEServer* pServer = NULL; // BLE服务器指针

// Radar Data Service
BLEService* radarDataService = NULL;
BLECharacteristic* radarStreamCharacteristic = NULL;
BLECharacteristic* radarStatusCharacteristic = NULL;// 雷达状态特征

// Device Config Service
BLEService* deviceConfigService = NULL;
BLECharacteristic* deviceCommandCharacteristic = NULL;// 设备命令特征
BLECharacteristic* deviceResultCharacteristic = NULL;// 设备结果特征
BLECharacteristic* deviceInfoCharacteristic = NULL;// 设备信息特征

bool deviceConnected = false; // 设备连接状态
bool oldDeviceConnected = false; // 旧设备连接状态
QueueHandle_t bleCommandQueue = nullptr; // BLE命令队列

bool continuousSendEnabled = false; // 持续发送使能标志
unsigned long continuousSendInterval = 500; // 持续发送间隔（毫秒）
bool radarSleepQueryEnabled = false; // 雷达睡眠/综合状态查询开关（0x8D/0x90）
BLEFlowController bleFlow(500); // BLE流控制器对象
SemaphoreHandle_t bleSendMutex; // BLE发送互斥锁

BleProto::FrameParser bleFrameParser; // BLE帧解析器（TLV协议）
uint8_t bleSequenceCounter = 0;       // BLE出站帧序列号计数器

static bool isBleNotifySubscribed(BLECharacteristic* pChar) {
    if (pChar == nullptr) {
        return false;
    }

    BLEDescriptor* descriptor = pChar->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    if (descriptor == nullptr) {
        return false;
    }

    return static_cast<BLE2902*>(descriptor)->getNotifications();
}

// BLE MTU 协商相关变量
size_t g_blePayloadSize = FALLBACK_PAYLOAD;

// BLE请求上下文：跟踪异步命令的 seq，确保回包 seq 与请求一致
BleRequestContext wifiConfigRequestCtx; // WiFi配置请求上下文
BleRequestContext wifiScanRequestCtx; // WiFi扫描请求上下文
BleRequestContext savedNetworksRequestCtx;// 其他命令的请求上下文可以继续添加

// BLE命令处理常量
static const size_t MAX_LEGACY_JSON_LEN = sizeof(BleCommandMessage::raw) - 1; // legacy JSON兼容队列限制，不是TLV协议限制

unsigned long lastSensorUpdate = 0; // 上次传感器更新时间
LastSentData lastSentData = {0}; // 上次发送的数据，初始化为0
unsigned long lastCheckTime = 0; // 上次检测时间，初始化为0

/**
 * @brief BLE流控制器构造函数
 * 初始化BLE数据流控制参数，限制数据发送速率
 * @param maxBps 最大每秒发送字节数
 */
BLEFlowController::BLEFlowController(size_t maxBps) : maxBytesPerSecond(maxBps), bytesSent(0) {
    lastResetTime = millis();
    lastSendTime = 0;
}

/**
 * @brief 检查是否可以发送数据
 * 检查当前是否满足发送条件，包括速率限制和时间间隔
 * @param dataSize 要发送的数据大小
 * @return 是否可以发送数据
 */
bool BLEFlowController::canSend(size_t dataSize) {
    unsigned long currentTime = millis();
    
    if(currentTime - lastResetTime >= 1000) {
        bytesSent = 0;
        lastResetTime = currentTime;
    }
    
    if((bytesSent + dataSize) > maxBytesPerSecond) {
        return false;
    }
    
    if(currentTime - lastSendTime < 5) {
        return false;
    }
    
    return true;
}

/**
 * @brief 检查发送时间间隔
 * 检查距离上次发送是否满足最小时间间隔
 * @return 是否满足发送条件
 */
bool BLEFlowController::check() {
    unsigned long currentTime = millis();
    if(currentTime - lastSendTime < 5) {
        return false;
    }
    return true;
}

/**
 * @brief 记录数据发送
 * 更新已发送字节数和最后发送时间
 * @param dataSize 已发送的数据大小
 */
void BLEFlowController::recordSend(size_t dataSize) {
    bytesSent += dataSize;
    lastSendTime = millis();
}

/**
 * @brief 重置流控制器
 * 重置已发送字节数和时间戳
 */
void BLEFlowController::reset() {
    bytesSent = 0;
    lastResetTime = millis();
    lastSendTime = 0;
}

/**
 * @brief BLE服务器连接回调
 * 当客户端连接时触发
 * @param pServer BLE服务器指针
 */
void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("✅ [BLE] 客户端已连接");
    syncAllDeviceStatusToBLE();// 连接后同步所有设备状态到BLE客户端，确保客户端状态与设备当前状态一致
}
/**
 * @brief BLE服务器连接回调（包含参数）
 * 当客户端连接时触发，包含连接参数
 * @param pServer BLE服务器指针
 * @param param 连接事件参数，包含连接ID、MTU等信息
 */
void MyServerCallbacks::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
#if BLE_FIXED_20_BYTE_MODE
    g_blePayloadSize = FALLBACK_PAYLOAD;// 固定兼容模式，忽略MTU协商结果
    Serial.printf("✅ [BLE] 连接建立, conn_id=%u, 固定 20 字节兼容模式\n",
                  param->connect.conn_id);
#else
    uint16_t mtu = pServer->getPeerMTU(param->connect.conn_id);
    if (mtu >= DEFAULT_ATT_MTU) {
        g_blePayloadSize = mtu - ATT_HEADER_SIZE;
    } else {
        g_blePayloadSize = FALLBACK_PAYLOAD;
    }
    if (g_blePayloadSize < FALLBACK_PAYLOAD) {
        g_blePayloadSize = FALLBACK_PAYLOAD;
    }
    Serial.printf("✅ [BLE] 连接建立, conn_id=%u, MTU=%u, payload=%u\n",
                  param->connect.conn_id, mtu, (unsigned)g_blePayloadSize);
#endif
}

/**
 * @brief BLE服务器MTU更新回调
 * 当MTU协商完成或更新时触发
 * @param pServer BLE服务器指针
 * @param param MTU更新事件参数，包含连接ID和新的MTU值
 */
void MyServerCallbacks::onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
#if BLE_FIXED_20_BYTE_MODE
    g_blePayloadSize = FALLBACK_PAYLOAD;// 固定兼容模式，忽略MTU协商结果
    Serial.printf("📏 [BLE] MTU 协商事件(已忽略), conn_id=%u, 固定 20 字节兼容模式\n",
                  param->mtu.conn_id);
#else
    uint16_t mtu = pServer->getPeerMTU(param->mtu.conn_id);
    if (mtu >= DEFAULT_ATT_MTU) {
        g_blePayloadSize = mtu - ATT_HEADER_SIZE;
    } else {
        g_blePayloadSize = FALLBACK_PAYLOAD;
    }
    if (g_blePayloadSize < FALLBACK_PAYLOAD) {
        g_blePayloadSize = FALLBACK_PAYLOAD;
    }
    Serial.printf("📏 [BLE] MTU 已更新, conn_id=%u, MTU=%u, payload=%u\n",
                  param->mtu.conn_id, mtu, (unsigned)g_blePayloadSize);
#endif
}

/**
 * @brief BLE服务器断开连接回调
 * 当客户端断开连接时触发
 * @param pServer BLE服务器指针
 */
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    continuousSendEnabled = false;       // 断开时停止连续推送，重连后需重新发 CMD_START_CONTINUOUS
    g_blePayloadSize = FALLBACK_PAYLOAD; // 重置为固定 20 字节
    Serial.println("🔴 [BLE] 客户端已断开, 连续推送已停止, payload 恢复为 20");
}

/**
 * @brief BLE特征值写入回调
 * 当客户端写入数据时触发
 * @param pCharacteristic BLE特征值指针
 */
void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.empty()) {
        return;
    }

    // 队列未初始化，直接丢弃
    // 检查命令队列是否已初始化
    if (bleCommandQueue == nullptr) {
        Serial.println("[BLE] ❌ 命令队列未初始化，拒绝处理");
        
        // 尝试解析帧头以获取 seq，用于返回错误响应
        if (deviceConnected && deviceResultCharacteristic != nullptr && value.size() >= 5) {
            uint8_t seq = value.data()[4];
            
            sendCommandErrorResponse(
                BleProto::CMD_ERROR_RESP,
                seq,
                BleProto::ErrorCode::ERR_DEV_STATE_INVALID
            );
        }
        return;
    }

    // 检查命令长度是否超限
    if (value.size() > sizeof(BleCommandMessage::raw)) {
        Serial.printf("[BLE] ❌ 命令长度超限：收到 %u 字节，最大支持 %u 字节，拒绝处理\n",
                      static_cast<unsigned>(value.size()),
                      static_cast<unsigned>(sizeof(BleCommandMessage::raw)));
        
        // 尝试解析帧头以获取 seq，用于返回错误响应
        if (deviceConnected && deviceResultCharacteristic != nullptr && value.size() >= 5) {
            // 尝试读取 seq（假设帧格式：AA 55 version cmd  seq ...）
            uint8_t seq = value.data()[4];
            
            sendCommandErrorResponse(
                BleProto::CMD_ERROR_RESP,
                seq,
                BleProto::ErrorCode::ERR_PROTO_FRAME_TOO_LARGE
            );
        }
        return;
    }

    BleCommandMessage msg = {};
    msg.len = value.size();  // 不再使用 min()，因为已经检查过长度
    memcpy(msg.raw, value.data(), msg.len);
    
    // 检查队列是否已满
    if (xQueueSend(bleCommandQueue, &msg, 0) != pdTRUE) {
        Serial.println("[BLE] ❌ 命令队列已满，拒绝处理");
        
        // 尝试解析帧头以获取 seq，用于返回错误响应
        if (deviceConnected && deviceResultCharacteristic != nullptr && value.size() >= 5) {
            uint8_t seq = value.data()[4];
            
            sendCommandErrorResponse(
                BleProto::CMD_ERROR_RESP,
                seq,
                BleProto::ErrorCode::ERR_DEV_QUEUE_FULL
            );
        }
        return;
    }
    
    // 命令已成功入队
    Serial.printf("[BLE] ✅ 命令已入队，长度: %u 字节\n", static_cast<unsigned>(value.size()));
}

/**
 * @brief 初始化雷达管理器
 * 初始化队列和FreeRTOS任务
 */
void initRadarManager() {
    Serial.println("🔧 初始化雷达管理器...");

    phaseDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(PhaseData));
    vitalDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(VitalData));
    uartQueue = xQueueCreate(2048, sizeof(char));

    if (phaseDataQueue == NULL || vitalDataQueue == NULL || uartQueue == NULL) {
        Serial.println("❌ 队列创建失败");
    } else {
        Serial.println("✅ FreeRTOS队列创建成功");
    }

    bleSendMutex = xSemaphoreCreateMutex();
    if (bleSendMutex == NULL) {
        Serial.println("❌ BLE发送互斥锁创建失败");
    } else {
        Serial.println("✅ BLE发送互斥锁创建成功");
    }

    bleCommandQueue = xQueueCreate(10, sizeof(BleCommandMessage));
    if (bleCommandQueue == NULL) {
        Serial.println("❌ BLE命令队列创建失败");
    } else {
        Serial.println("✅ BLE命令队列创建成功");
    }
    // 创建FreeRTOS任务
    // BLE数据发送任务，优先级较高，确保及时响应BLE命令和发送数据
    xTaskCreatePinnedToCore(
        bleSendTask,
        "BleSendTask",
        TASK_STACK_SIZE,
        NULL,
        3,
        &bleSendTaskHandle,
        1
    );
    
    // 生命体征数据发送任务，优先级较高，确保及时发送
    xTaskCreatePinnedToCore(
        vitalSendTask,
        "VitalSendTask",
        TASK_STACK_SIZE,
        NULL,
        2,
        &vitalSendTaskHandle,
        1
    );
    
    // 雷达数据处理任务，优先级较低，避免阻塞其他关键任务
    xTaskCreatePinnedToCore(
        radarDataTask,
        "RadarProcessTask",
        4096,
        NULL,
        4,
        NULL,
        1
    );
    
    // UART数据处理任务，优先级较低，避免阻塞其他关键任务
    xTaskCreatePinnedToCore(
        uartProcessTask,
        "UartProcessTask",
        4096,
        NULL,
        5,
        &uartProcessTaskHandle,
        1
    );
    
    Serial.println("✅ 雷达管理器初始化完成");
}

/**
 * @brief 初始化R60ABD1雷达模组
 * 发送初始化命令激活雷达数据上报功能
 */
void initR60ABD1() {
    Serial.println("🔧 初始化R60ABD1雷达模组...");
    
    Serial.println("📡 发送查询指令以激活数据上报...");
    uint8_t queryPresenceCmd[] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0x00, 0x7D, 0x54, 0x43};
    mySerial1.write(queryPresenceCmd, sizeof(queryPresenceCmd));

    Serial.println("📡 开启核心监测功能...");
    
    sendRadarCommand(0x80, 0x00, 0x01);
    delay(50);
    sendRadarCommand(0x81, 0x00, 0x01);
    delay(50);
    sendRadarCommand(0x85, 0x00, 0x01);
    delay(50);
    sendRadarCommand(0x84, 0x00, 0x01);
    delay(50);
    
    Serial.println("📡 尝试开启波形数据...");
    sendRadarCommand(0x81, 0x0C, 0x01);
    delay(50);
    sendRadarCommand(0x85, 0x0A, 0x01);
    delay(50);

    sendRadarCommand(0x84, 0x13, 0x01);
    delay(50);
    sendRadarCommand(0x84, 0x14, 0x01);
    delay(50);
    
    Serial.println("🔍 查询当前状态...");
    sendRadarCommand(0x80, 0x80, 0x0F);
    delay(50);
    sendRadarCommand(0x81, 0x80, 0x0F);
    delay(50);
    sendRadarCommand(0x85, 0x80, 0x0F);
    delay(50);
    sendRadarCommand(0x84, 0x80, 0x0F);
    
    Serial.println("✅ R60ABD1雷达初始化完成");
}

/**
 * @brief 解析R60ABD1雷达数据帧
 * 解析雷达返回的数据帧并提取传感器数据
 * @param frame 数据帧指针
 * @param frameLen 数据帧长度
 * @return 是否解析成功
 */
bool parseR60ABD1Frame(uint8_t *frame, uint16_t frameLen) {
    if(frameLen < 8) return false;
    
    if(frame[0] != FRAME_HEADER1 || frame[1] != FRAME_HEADER2 ||
       frame[frameLen-2] != FRAME_TAIL1 || frame[frameLen-1] != FRAME_TAIL2) {
        return false;
    }
    
    uint8_t checksum = 0;
    for(int i = 0; i < frameLen-3; i++) {
        checksum += frame[i];
        if(i % 50 == 0) {
            esp_task_wdt_reset();
        }
    }
    if(checksum != frame[frameLen-3]) {
        Serial.println("❌ R60ABD1帧校验和错误");
        return false;
    }
    
    for(int i = 0; i < frameLen && i < 20; i++) {
    }
    if(frameLen > 20) Serial.print("... ");
    
    uint8_t ctrlByte = frame[2];
    uint8_t cmdByte = frame[3];
    uint16_t dataLen = (frame[4] << 8) | frame[5];

    switch(ctrlByte) {
        case CTRL_PRESENCE:
            switch(cmdByte) {
                // 开关人体存在监测
                case 0x00:
                case 0x80:
                    if(dataLen >= 1) {
                        if(frame[6] == 0x01) {
                            sendRadarCommand(0x84, 0x00, 0x01);  // 睡眠监测
                        }
                    }
                    break;

                case 0x01:
                    if(dataLen >= 1) {
                        sensorData.presence = frame[6];  // 0:无人, 1:有人
                        Serial.printf("👤 人体存在: %s\n", sensorData.presence ? "有人" : "无人");
                    }
                    break;
                
                case 0x02:
                    if(dataLen >= 1) {
                        sensorData.motion = frame[6];  // 0:无, 1:静止, 2:活跃
                        const char* states[] = {"无", "静止", "活跃"};
                        Serial.printf("🏃 运动状态: %s\n", states[sensorData.motion]);
                    }
                    break;
                
                case 0x03:
                    if(dataLen >= 1) {
                        sensorData.body_movement = frame[6];  // 0-100
                        Serial.printf("📊体动参数: %d\n", sensorData.body_movement);
                    }
                    break;
                
                case 0x04:
                    if(dataLen >= 2) {
                        sensorData.distance = ((uint16_t)frame[6] << 8) | frame[7];
                        Serial.printf("📏人体距离: %d cm\n", sensorData.distance);
                    }
                    break;
                
                case 0x05:
                    if(dataLen >= 6) {
                        uint16_t x_raw = ((uint16_t)frame[6] << 8) | frame[7];
                        sensorData.pos_x = parseSignedCoordinate(x_raw);
                        
                        uint16_t y_raw = ((uint16_t)frame[8] << 8) | frame[9];
                        sensorData.pos_y = parseSignedCoordinate(y_raw);
                        
                        uint16_t z_raw = ((uint16_t)frame[10] << 8) | frame[11];
                        sensorData.pos_z = parseSignedCoordinate(z_raw);
                    }
                    break;
                
                default:
                    Serial.printf("❓未知的0x80命令字: 0x%02X\n", cmdByte);
                    break;
            }
            break;
            
        case CTRL_BREATH:
            switch(cmdByte) {
                case 0x00:
                case 0x80:
                    if(dataLen >= 1) {
                    }
                    break;
                   
                case 0x01:
                    if(dataLen >= 1) {
                        sensorData.breath_status = frame[6];
                    }
                    break;
                
                case 0x02:
                    if(dataLen >= 1) {
                        sensorData.breath_rate = (float)frame[6];
                        sensorData.breath_valid = (sensorData.breath_rate >= 0.0f && 
                                                 sensorData.breath_rate <= 35.0f);
                    }
                    break;
                
                case 0x05:
                    if(dataLen >= 5) {
                        for(int i = 0; i < 5 && i < dataLen; i++) {
                            sensorData.breath_waveform[i] = (int8_t)(frame[6+i] - 128);
                        }
                    }
                    break;
                
                default:
                    break;
            }
            break;
            
        case CTRL_HEARTRATE:
            switch(cmdByte) {
                case 0x00:
                case 0x80:
                    if(dataLen >= 1) {
                    }
                    break;
                case 0x02:
                    if(dataLen >= 1) {
                        sensorData.heart_rate = (float)frame[6];
                        sensorData.heart_valid = (sensorData.heart_rate >= 60.0f && 
                                                sensorData.heart_rate <= 120.0f);
                    }
                    break;
                
                case 0x05:
                    if(dataLen >= 5) {
                        for(int i = 0; i < 5 && i < dataLen; i++) {
                            sensorData.heart_waveform[i] = (int8_t)(frame[6+i] - 128);
                        }
                    }
                    break;
                
                default:
                    break;
            }
            break;
            
        case CTRL_SLEEP:
            switch(cmdByte) {
                case 0x00:
                case 0x80:
                    if(dataLen >= 1) {
                    }
                    break;  
                
                case 0x01:
                case 0x81:
                    if(dataLen >= 1) {
                        sensorData.bed_status = frame[6];
                    }
                    break;

                case 0x03:
                case 0x83:
                    if(dataLen >= 2) {
                        sensorData.awake_time = ((uint16_t)frame[6] << 8) | (uint16_t)frame[7];
                    }
                    break;
                
                case 0x04:
                case 0x84:
                    if(dataLen >= 2) {
                        sensorData.light_sleep_time = ((uint16_t)frame[6] << 8) | (uint16_t)frame[7];
                    }
                    break;
                
                case 0x05:
                case 0x85:
                    if(dataLen >= 2) {
                        sensorData.deep_sleep_time = ((uint16_t)frame[6] << 8) | (uint16_t)frame[7];
                    }
                    break;
                
                case 0x06:
                    if(dataLen >= 1) {
                        sensorData.sleep_score = frame[6];
                    }
                    break;

                case 0x86:
                    if(dataLen >= 2) {
                        sensorData.sleep_score = frame[6];
                    }
                    break;

                case 0x0C:
                case 0x8D:
                    if(dataLen >= 8) {
                        sensorData.presence = frame[6];
                        sensorData.sleep_state = frame[7];
                        sensorData.avg_breath_rate = frame[8];
                        sensorData.avg_heart_rate = frame[9];
                        sensorData.turnover_count = frame[10];
                        sensorData.large_move_ratio = frame[11];
                        sensorData.small_move_ratio = frame[12];
                        sensorData.apnea_count = frame[13];
                    }
                    break;
                
                case 0x0D:
                case 0x8F:
                    if(dataLen >= 12) {
                        sensorData.sleep_score = frame[6];
                        sensorData.sleep_total_time = ((uint16_t)frame[7] << 8) | (uint16_t)frame[8];
                        
                        sensorData.awake_ratio = frame[9];
                        sensorData.light_sleep_ratio = frame[10];
                        sensorData.deep_sleep_ratio = frame[11];
                        
                        sensorData.bed_Out_Time = frame[12];
                        sensorData.turn_count = frame[13];
                        sensorData.turnover_count = frame[14];
                        sensorData.avg_breath_rate = frame[15];
                        sensorData.avg_heart_rate = frame[16];
                        sensorData.apnea_count = frame[17];
                    }
                    break;
                
                case 0x0E:
                case 0x8E:
                    if(dataLen >= 1) {
                        sensorData.abnormal_state = frame[6];
                    }
                    break;
                
                case 0x10:
                case 0x90:
                    if(dataLen >= 1) {
                        sensorData.sleep_grade = frame[6];
                    }
                    break;
                
                case 0x11:
                case 0x91:
                    if(dataLen >= 1) {
                        sensorData.struggle_alert = frame[6];
                    }
                    break;
                
                case 0x12:
                case 0x92:
                    if(dataLen >= 1) {
                        sensorData.no_one_alert = frame[6];
                    }
                    break;
                
                default:
                    break;
            }
            break;
            
        case 0x07:
            if(dataLen >= 1) {
            }
            break;
            
        default:
            Serial.printf("❓未知控制字: 0x%02X\n", ctrlByte);
            break;
    }

    lastSensorUpdate = millis();
    
    sensorData.heart_valid = (sensorData.heart_rate > 0 && sensorData.heart_rate < 200);
    sensorData.breath_valid = (sensorData.breath_rate >= 0.1f && sensorData.breath_rate <= 60.0f);

    if( sensorData.heart_valid ==1 && sensorData.heart_valid == 1 && presence_Bit == 1 )
    {
        sensorData.presence = 1;
        presence_Bit = 0;
    }
    
    return true;
}
// 解析带符号的坐标值
int16_t parseSignedCoordinate(uint16_t raw_value) {
    bool is_negative = (raw_value & 0x8000) != 0;
    uint16_t magnitude = raw_value & 0x7FFF;
    
    int16_t result = (int16_t)magnitude;
    if (is_negative) {
        result = -result;
    }
    
    return result;
}

/**
 * @brief 发送雷达命令
 * 构造并发送控制命令到雷达模组
 * @param ctrl 控制字节
 * @param cmd 命令字节
 * @param value 值字节
 */
void sendRadarCommand(uint8_t ctrl, uint8_t cmd, uint8_t value) {
    uint8_t command[10];
    command[0] = 0x53;
    command[1] = 0x59;
    command[2] = ctrl;
    command[3] = cmd;
    command[4] = 0x00;
    command[5] = 0x01;
    command[6] = value;
    
    uint8_t checksum = 0;
    for(int i = 0; i < 7; i++) {
        checksum += command[i];
    }
    command[7] = checksum;
    
    command[8] = 0x54;
    command[9] = 0x43;
    
    mySerial1.write(command, 10);
}

void IRAM_ATTR serialRxCallback() {
    if (uartQueue != NULL) {
        while(mySerial1.available()) {
            char c = mySerial1.read();
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(uartQueue, &c, &xHigherPriorityTaskWoken);
            
            if(xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/**
 * @brief 检查数据是否发生变化
 * 比较当前传感器数据与上次发送的数据，判断是否需要发送更新
 * @return 是否发生变化
 */
bool isDataChanged() {
    // 心率变化阈值：0.5
    if (fabs(sensorData.heart_rate - lastSentData.heart_rate) > 0.5) {
        return true;
    }
    
    // 呼吸率变化阈值：0.5
    if (fabs(sensorData.breath_rate - lastSentData.breath_rate) > 0.5) {
        return true;
    }
    
    // 存在状态变化
    if (sensorData.presence != lastSentData.presence) {
        return true;
    }
    
    // 运动状态变化
    if (sensorData.motion != lastSentData.motion) {
        return true;
    }
    
    // 睡眠状态变化
    if (sensorData.sleep_state != lastSentData.sleep_state) {
        return true;
    }
    
    return false;
}

/**
 * @brief BLE数据发送任务
 * 实现基于数据变化和定时检测的发送机制
 * 1. 定时检测数据变化（基于continuousSendInterval）
 * 2. 检测到变化时立即发送数据
 * 3. 发送后更新lastSentData为当前数据
 * @param parameter 任务参数（未使用）
 */
void bleSendTask(void *parameter) {
    Serial.println("🔁 R60ABD1 BLE data send task started (纯TLV模式)");
    
    while (1) {
        esp_task_wdt_reset();
        
        // 检查是否需要进行数据检测
        if (continuousSendEnabled && deviceConnected) {
            unsigned long currentTime = millis();
            
            // 按照设定的时间间隔进行检测
            if (currentTime - lastCheckTime >= continuousSendInterval) {
                lastCheckTime = currentTime;
                
                // 检查数据是否发生变化
                if (isDataChanged()) {
                    // 构建TLV帧
                    BleProto::Frame frame;// 创建一个新的TLV帧
                    frame.version = BleProto::VERSION;// 设置协议版本
                    frame.cmd = BleProto::CMD_CONTINUOUS_PUSH;// 设置命令类型为连续推送
                    frame.seq = bleSequenceCounter++;// 设置序列号并自增
                    frame.data.clear();// 清空数据部分以准备添加新的TLV数据
                    
                    
                    if (sensorData.presence > 0) {
                        // 有人存在时的完整数据
                        BleProto::appendTlvU8(frame.data, BleProto::TLV_PRESENCE, sensorData.presence);// 存在状态
                        BleProto::appendTlvU16(frame.data, BleProto::TLV_HEART_RATE_X10, 
                                             static_cast<uint16_t>(sensorData.heart_rate * 10.0f + 0.5f));
                        BleProto::appendTlvU16(frame.data, BleProto::TLV_BREATH_RATE_X10, 
                                             static_cast<uint16_t>(sensorData.breath_rate * 10.0f + 0.5f));
                        BleProto::appendTlvU8(frame.data, BleProto::TLV_MOTION, sensorData.motion);// 运动状态
                        //BleProto::appendTlvU8(frame.data, BleProto::TLV_SLEEP_STATE, sensorData.sleep_state);
                        // 波形数据（取当前最新一点）
                        // BleProto::appendTlvU8(frame.data, BleProto::TLV_HEART_WAVEFORM,
                        //                      static_cast<uint8_t>(static_cast<int8_t>(sensorData.heart_waveform[0]) + 128));
                        // BleProto::appendTlvU8(frame.data, BleProto::TLV_BREATH_WAVEFORM,
                        //                      static_cast<uint8_t>(static_cast<int8_t>(sensorData.breath_waveform[0]) + 128));
                    } else {
                        // 无人时的基础数据
                        BleProto::appendTlvU8(frame.data, BleProto::TLV_PRESENCE, 0);
                        BleProto::appendTlvU16(frame.data, BleProto::TLV_HEART_RATE_X10, 0);
                        BleProto::appendTlvU16(frame.data, BleProto::TLV_BREATH_RATE_X10, 0);
                        BleProto::appendTlvU8(frame.data, BleProto::TLV_MOTION, 0);
                     //  BleProto::appendTlvU8(frame.data, BleProto::TLV_SLEEP_STATE, 0);
                    }
                    
                    // 发送TLV帧到雷达数据流特征
                    sendFrameToBLE(frame, radarStreamCharacteristic);// 发送数据到BLE

                    // 更新最后发送的数据
                    lastSentData.heart_rate = sensorData.heart_rate;
                    lastSentData.breath_rate = sensorData.breath_rate;
                    lastSentData.presence = sensorData.presence;
                    lastSentData.motion = sensorData.motion;
                    lastSentData.sleep_state = sensorData.sleep_state;
                }
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

bool isInRefusedCooldown();
unsigned long getRefusedCooldownRemaining();
void updateRefusedTime();

/**
 * @brief 生命体征数据发送任务
 * 从队列中获取生命体征数据并发送到InfluxDB数据库
 * @param parameter 任务参数（未使用）
 */
static void appendInfluxField(String& line, bool& firstField, const String& field) {
    if (!firstField) {
        line += ",";
    }
    line += field;
    firstField = false;
}

static bool sendVitalDailyDataToInfluxDB(const VitalData& data) {
    String macAddress = getDeviceMacAddress();
    String dailyDataLine = "daily_data,deviceId=" + macAddress + ",dataType=daily ";
    bool firstField = true;

    if (data.heart_rate > 0) {
        appendInfluxField(dailyDataLine, firstField, "heartRate=" + String(data.heart_rate, 1));
    }

    if (data.breath_rate > 0) {
        appendInfluxField(dailyDataLine, firstField, "breathingRate=" + String(data.breath_rate, 1));
    }

    appendInfluxField(dailyDataLine, firstField, "personDetected=" + String(data.presence) + "i");
    appendInfluxField(dailyDataLine, firstField, "humanActivity=" + String(data.motion) + "i");
    appendInfluxField(dailyDataLine, firstField, "bodyMovement=" + String((int)data.body_movement) + "i");

    if (data.distance > 0) {
        appendInfluxField(dailyDataLine, firstField, "humanDistance=" + String(data.distance) + "i");
    }

    appendInfluxField(dailyDataLine, firstField, "sleepState=" + String(data.sleep_state) + "i");
    appendInfluxField(dailyDataLine, firstField, "humanPositionX=" + String(data.pos_x) + "i");
    appendInfluxField(dailyDataLine, firstField, "humanPositionY=" + String(data.pos_y) + "i");
    appendInfluxField(dailyDataLine, firstField, "humanPositionZ=" + String(data.pos_z) + "i");
    appendInfluxField(dailyDataLine, firstField, "heartbeatWaveform=" + String((int)data.heartbeat_waveform) + "i");
    appendInfluxField(dailyDataLine, firstField, "breathingWaveform=" + String((int)data.breathing_waveform) + "i");
    appendInfluxField(dailyDataLine, firstField, "abnormalState=" + String(data.abnormal_state) + "i");
    appendInfluxField(dailyDataLine, firstField, "bedStatus=" + String(data.bed_status) + "i");
    appendInfluxField(dailyDataLine, firstField, "struggleAlert=" + String(data.struggle_alert) + "i");
    appendInfluxField(dailyDataLine, firstField, "noOneAlert=" + String(data.no_one_alert) + "i");

    return !firstField && sendDailyDataToInfluxDB(dailyDataLine);
}

void vitalSendTask(void *parameter) {
    Serial.println("🔁🔁 生命体征数据发送任务启动（WiFi数据库传输）");
    
    unsigned long lastSleepDataTime = 0;
    const unsigned long SLEEP_DATA_INTERVAL = 5000;
    
    VitalData cachedData = {0};
    bool hasCachedData = false;
    unsigned long lastSendTime = 0;
    const unsigned long FORCE_SEND_INTERVAL = 60000; // 每分钟强制发送一次
    const unsigned long MIN_SEND_INTERVAL = 1500; // 最小发送间隔1秒
    VitalData pendingData = {0};
    bool hasPendingData = false;
    
    while (1) {
        VitalData vitalData;
        
        if (xQueueReceive(vitalDataQueue, &vitalData, portMAX_DELAY) == pdTRUE) {
            esp_task_wdt_reset();
            
            if (WiFi.status() == WL_CONNECTED) {
                if (vitalData.heart_rate == 0 || vitalData.breath_rate == 0) {
                    Serial.printf("⚠️ 心率=%.1f, 呼吸率=%.1f，跳过发送数据到数据库\n", 
                        vitalData.heart_rate, vitalData.breath_rate);
                    continue;
                }
                
                if (vitalData.heart_rate > 200 || vitalData.breath_rate > 50 || 
                    vitalData.heart_rate < 0 || vitalData.breath_rate < 0) {
                    Serial.printf("❌ 数据异常 - 心率:%.1f, 呼吸:%.1f，丢弃\n",
                        vitalData.heart_rate, vitalData.breath_rate);
                    continue;
                }
                
                bool dataChanged = false;
                bool shouldSend = false;
                
                if (!hasCachedData) {
                    dataChanged = true;
                    shouldSend = true;
                    Serial.printf("📊 首次数据 - 心率:%.1f, 呼吸:%.1f, 距离:%d\n",
                        vitalData.heart_rate, vitalData.breath_rate, vitalData.distance);
                } else {
                    if (memcmp(&vitalData, &cachedData, sizeof(VitalData)) != 0) {
                        dataChanged = true;
                        shouldSend = true;
                        
                        if (fabs(vitalData.heart_rate - cachedData.heart_rate) >= 1.0f ||
                            fabs(vitalData.breath_rate - cachedData.breath_rate) >= 1.0f) {
                            Serial.printf("📊 数据变化: 心率 %.1f->%.1f, 呼吸 %.1f->%.1f, 距离 %d->%d, 状态 %d->%d\n",
                                cachedData.heart_rate, vitalData.heart_rate,
                                cachedData.breath_rate, vitalData.breath_rate,
                                cachedData.distance, vitalData.distance,
                                cachedData.sleep_state, vitalData.sleep_state);
                        } else {
                            Serial.printf("📊 数据变化: 距离 %d->%d, 状态 %d->%d, 运动 %d->%d, 异常 %d->%d\n",
                                cachedData.distance, vitalData.distance,
                                cachedData.sleep_state, vitalData.sleep_state,
                                cachedData.motion, vitalData.motion,
                                cachedData.abnormal_state, vitalData.abnormal_state);
                        }
                    }
                }
                
                unsigned long currentTime = millis();
                if (!shouldSend && (currentTime - lastSendTime >= FORCE_SEND_INTERVAL)) {
                    shouldSend = true;
                    Serial.printf("⏰ 定时发送触发 (距上次 %d 秒)\n", (currentTime - lastSendTime) / 1000);
                }
                
                cachedData = vitalData;
                hasCachedData = true;
                
                if (shouldSend) {
                    unsigned long timeSinceLastSend = millis() - lastSendTime;
                    
                    if (isInRefusedCooldown()) {
                        Serial.printf("⏳ 连接被拒绝冷却中，等待 %d 秒\n", getRefusedCooldownRemaining() / 1000);
                        if (timeSinceLastSend < MIN_SEND_INTERVAL && lastSendTime > 0) {
                            pendingData = vitalData;
                            hasPendingData = true;
                        }
                    } else if (timeSinceLastSend < MIN_SEND_INTERVAL && lastSendTime > 0) {
                        pendingData = vitalData;
                        hasPendingData = true;
                        Serial.printf("⏳ 发送间隔不足 (%d ms)，缓存最新数据等待 %d ms\n", 
                            (int)timeSinceLastSend, MIN_SEND_INTERVAL - (int)timeSinceLastSend);
                    } else {
                        Serial.printf("📡 发送数据 - 心率:%.1f, 呼吸:%.1f, 距离:%d\n",
                            vitalData.heart_rate, vitalData.breath_rate, vitalData.distance);
                    
                    {
                        bool sendSuccess = sendVitalDailyDataToInfluxDB(vitalData);
                        esp_task_wdt_reset();
                        
                        if (sendSuccess) {
                            lastSendTime = millis();
                        } else {
                            Serial.println("❌ 发送失败");
                        }
                    }
                    
                    unsigned long currentTime = millis();
                    if (currentTime - lastSleepDataTime >= SLEEP_DATA_INTERVAL) {
                        sendSleepDataToInfluxDB();
                        lastSleepDataTime = currentTime;
                    }
                    
                    hasPendingData = false;
                    }
                } else {
                    Serial.printf("💾 缓存数据未变化 - 心率:%.1f, 呼吸:%.1f\n",
                        vitalData.heart_rate, vitalData.breath_rate);
                    
                    if (hasPendingData && (millis() - lastSendTime >= MIN_SEND_INTERVAL) && !isInRefusedCooldown()) {
                        Serial.printf("⏰ 发送缓存数据 - 心率:%.1f, 呼吸:%.1f\n",
                            pendingData.heart_rate, pendingData.breath_rate);
                        
                        {
                            bool sendSuccess = sendVitalDailyDataToInfluxDB(pendingData);
                            esp_task_wdt_reset();
                            
                            if (sendSuccess) {
                                lastSendTime = millis();
                            } else {
                                Serial.println("❌ 缓存数据发送失败");
                            }
                        }
                        
                        hasPendingData = false;
                    }
                }
            } else {
                UBaseType_t queueItems = uxQueueMessagesWaiting(vitalDataQueue);
                if (queueItems > 0) {
                    VitalData tempData;
                    while (xQueueReceive(vitalDataQueue, &tempData, 0) == pdTRUE) {
                    }
                }
            }
            
            esp_task_wdt_reset();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 发送日常数据到InfluxDB数据库
 * 通过HTTP协议将数据写入InfluxDB时序数据库
 * @param dailyDataLine 数据行字符串
 * @return 是否发送成功
 */
bool isInRefusedCooldown() {
    static unsigned long lastRefusedTime = 0;
    const unsigned long REFUSED_COOLDOWN = 10000;
    
    if (lastRefusedTime == 0) return false;
    
    unsigned long currentTime = millis();
    return (currentTime - lastRefusedTime < REFUSED_COOLDOWN);
}

unsigned long getRefusedCooldownRemaining() {
    static unsigned long lastRefusedTime = 0;
    const unsigned long REFUSED_COOLDOWN = 10000;
    
    if (lastRefusedTime == 0) return 0;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - lastRefusedTime;
    if (elapsed >= REFUSED_COOLDOWN) return 0;
    return REFUSED_COOLDOWN - elapsed;
}

void updateRefusedTime() {
    static unsigned long lastRefusedTime = 0;
    lastRefusedTime = millis();
}

void resetWiFiConnection() {
    Serial.println("🔄 开始重置WiFi连接...");
    WiFi.disconnect(true);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_OFF);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_STA);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    Serial.println("✅ WiFi连接已重置，将触发重连");
    wifiManager.handleReconnect();
}

bool sendDailyDataToInfluxDB(String dailyDataLine) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi未连接，跳过发送");
        return false;
    }
    
    int rssi = WiFi.RSSI();
    if (rssi < -85) {
        Serial.printf("⚠️ WiFi信号过弱 (%d dBm)，跳过发送\n", rssi);
        return false;
    }
    
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        Serial.println("❌ WiFi没有获取到IP地址，跳过发送");
        return false;
    }
    
    if (isInRefusedCooldown()) {
        Serial.printf("⚠️ 连接被拒绝冷却中，等待 %d 秒\n", getRefusedCooldownRemaining() / 1000);
        return false;
    }
    
    IPAddress resolvedIP;
    if (!WiFi.hostByName(influxDBHost, resolvedIP)) {
        Serial.printf("❌ DNS解析失败: %s\n", influxDBHost);
        dnsFailCount++;
        Serial.printf("⚠️ DNS解析失败计数: %d/%d\n", dnsFailCount, DNS_FAIL_RESET_THRESHOLD);
        
        if (dnsFailCount >= DNS_FAIL_RESET_THRESHOLD) {
            Serial.println("⚠️ DNS解析连续失败次数过多，重置WiFi连接...");
            dnsFailCount = 0;
            resetWiFiConnection();
        }
        return false;
    } else {
        dnsFailCount = 0;
    }

    String url = String("http://") + resolvedIP.toString() + ":" + String(influxDBPort) + "/api/v2/write?org=" + String(influxDBOrg) + "&bucket=" + String(influxDBBucket);

    HTTPClient http;
    http.setTimeout(15000);
    http.setConnectTimeout(8000);
    http.setReuse(false);

    http.begin(url);
    http.addHeader("Authorization", String("Token ") + String(influxDBToken));
    http.addHeader("Content-Type", "text/plain; charset=utf-8");
    http.addHeader("Connection", "close");
    http.addHeader("Host", String(influxDBHost));

    int httpResponseCode = http.POST(dailyDataLine);
    String errorMsg = http.errorToString(httpResponseCode);
    
    if (httpResponseCode == 204) {
        Serial.println("✅ 日常数据已保存到InfluxDB");
        http.end();
        return true;
    }
    
    Serial.printf("❌ HTTP响应: %d (%s)\n", httpResponseCode, errorMsg.c_str());
    http.end();
    
    if (errorMsg.indexOf("connection refused") >= 0 || httpResponseCode == -1) {
        updateRefusedTime();
        Serial.println("⚠️ 服务器拒绝连接，进入冷却期");
    }
    
    return false;
}

/**
 * @brief 发送睡眠数据到InfluxDB数据库
 * 将睡眠相关的统计数据发送到InfluxDB时序数据库
 */
void sendSleepDataToInfluxDB() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }
    
    if (sensorData.sleep_total_time == 0) {
        return;
    }
    
    HTTPClient http;
    http.setTimeout(5000);  // 增加超时到5秒
    http.setConnectTimeout(5000);  // 连接超时5秒
    
    String url = String("http://") + String(influxDBHost) + ":" + String(influxDBPort) + "/api/v2/write?org=" + String(influxDBOrg) + "&bucket=" + String(influxDBBucket);
    
    http.begin(url);
    http.addHeader("Authorization", String("Token ") + String(influxDBToken));
    http.addHeader("Content-Type", "text/plain; charset=utf-8");
    
    String macAddress = getDeviceMacAddress();
    String lineProtocol = String("sleep_data,deviceId=") + macAddress + ",dataType=sleep ";
    
    String fields = "";
    fields += String("sleepQualityScore=") + String((int)sensorData.sleep_score) + "i";
    fields += ",sleepQualityGrade=" + String((int)sensorData.sleep_grade) + "i";
    fields += ",totalSleepDuration=" + String((int)sensorData.sleep_total_time) + "i";
    fields += ",awakeDurationRatio=" + String((int)sensorData.awake_ratio) + "i";
    fields += ",lightSleepRatio=" + String((int)sensorData.light_sleep_ratio) + "i";
    fields += ",deepSleepRatio=" + String((int)sensorData.deep_sleep_ratio) + "i";
    fields += ",outOfBedDuration=" + String((int)sensorData.bed_Out_Time) + "i";
    fields += ",outOfBedCount=" + String((int)sensorData.turn_count) + "i";
    fields += ",turnCount=" + String((int)sensorData.turnover_count) + "i";
    fields += ",avgBreathingRate=" + String((int)sensorData.avg_breath_rate) + "i";
    fields += ",avgHeartRate=" + String((int)sensorData.avg_heart_rate) + "i";
    fields += ",apneaCount=" + String((int)sensorData.apnea_count) + "i";
    fields += ",abnormalState=" + String((int)sensorData.abnormal_state) + "i";
    fields += ",breathStatus=" + String((int)sensorData.breath_status) + "i";
    fields += ",sleepState=" + String((int)sensorData.sleep_state) + "i";
    fields += ",largeMoveRatio=" + String((int)sensorData.large_move_ratio) + "i";
    fields += ",smallMoveRatio=" + String((int)sensorData.small_move_ratio) + "i";
    fields += ",struggleAlert=" + String((int)sensorData.struggle_alert) + "i";
    fields += ",noOneAlert=" + String((int)sensorData.no_one_alert) + "i";
    fields += ",awakeDuration=" + String((int)sensorData.awake_time) + "i";
    fields += ",lightSleepDuration=" + String((int)sensorData.light_sleep_time) + "i";
    fields += ",deepSleepDuration=" + String((int)sensorData.deep_sleep_time) + "i";
    
    lineProtocol += fields;
    
    Serial.println(String("🌙 发送睡眠数据到InfluxDB: ") + lineProtocol);

    // 重试机制
    int maxRetries = 3;
    int httpResponseCode = -1;
    
    for (int retry = 0; retry < maxRetries; retry++) {
        if (retry > 0) {
            Serial.printf("🔄 睡眠数据第%d次重试...\n", retry + 1);
            delay(500);
        }
        
        httpResponseCode = http.POST(lineProtocol);
        
        if (httpResponseCode == 204) {
            Serial.println(String("✅ 睡眠数据已保存到InfluxDB设备") + getDeviceMacAddress() + "上");
            http.end();
            return;
        }
        
        // 如果是连接错误，继续重试
        if (httpResponseCode == -1 || httpResponseCode == 0) {
            Serial.printf("⚠️ 睡眠数据连接失败 (错误码:%d)，准备重试...\n", httpResponseCode);
            http.end();
            http.begin(url);
            http.addHeader("Authorization", String("Token ") + String(influxDBToken));
            http.addHeader("Content-Type", "text/plain; charset=utf-8");
            continue;
        } else {
            break;
        }
    }
    
    Serial.printf("❌ 保存睡眠数据到InfluxDB失败 (重试%d次): %d - %s\n", 
        maxRetries, httpResponseCode,
        httpResponseCode == -1 ? "连接超时" : http.getString().c_str());
    
    http.end();
}

/**
 * @brief 雷达数据处理任务
 * 处理雷达数据并分发到相应的队列
 * @param parameter 任务参数（未使用）
 */
void radarDataTask(void *parameter) {
    Serial.println("🔁 雷达数据处理任务启动（最高优先级）");
    
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief UART数据处理任务
 * 从UART队列中读取数据并解析雷达数据帧
 * @param parameter 任务参数（未使用）
 */
void uartProcessTask(void *parameter) {
    Serial.println("🔧 初始化雷达UART串口...");
    mySerial1.setRxBufferSize(UART_RX_BUFFER_SIZE);
    mySerial1.begin(BAUD_RATE, SERIAL_8N1, UART1_RX, UART1_TX);
    mySerial1.onReceive(serialRxCallback);
    Serial.println("✅ UART1配置完成，缓冲区大小: 4096字节");

    uint8_t buffer[256];
    int bufferIndex = 0;
    bool inFrame = false;
    uint8_t prevByte = 0;

    Serial.println("✅ R60ABD1串口数据处理任务启动");
    
    while(1) {
        uint8_t c;
        if(xQueueReceive(uartQueue, &c, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            esp_task_wdt_reset();
            if(!inFrame) {
                if(prevByte == FRAME_HEADER1 && c == FRAME_HEADER2) {
                    buffer[0] = FRAME_HEADER1;
                    buffer[1] = FRAME_HEADER2;
                    bufferIndex = 2;
                    inFrame = true;
                }
            } else {
                if(bufferIndex < sizeof(buffer)) {
                    buffer[bufferIndex++] = c;
                    
                    if(bufferIndex >= 8 && 
                       buffer[bufferIndex-2] == FRAME_TAIL1 && 
                       buffer[bufferIndex-1] == FRAME_TAIL2) {
                        if(parseR60ABD1Frame(buffer, bufferIndex)) {
                            static uint32_t frameCounter = 0;
                            frameCounter++;
                            
                            lastSensorUpdate = millis();
                            
                            static uint32_t phasePacketCounter = 0;
                            static uint32_t vitalPacketCounter = 0;
                            
                            phasePacketCounter++;
                            if (phasePacketCounter >= PHASE_SEND_INTERVAL) {
                                PhaseData phaseData;
                                memset(&phaseData, 0, sizeof(PhaseData)); // 初始化为0
                                phaseData.heartbeat_waveform = sensorData.heartbeat_waveform;
                                phaseData.breathing_waveform = sensorData.breathing_waveform;
                                
                                if (xQueueSend(phaseDataQueue, &phaseData, 0) == pdTRUE) {
                                } else {
                                }
                                phasePacketCounter = 0;
                            }
                            
                            vitalPacketCounter++;
                            if (vitalPacketCounter >= VITAL_SEND_INTERVAL) {
                                VitalData vitalData;
                                memset(&vitalData, 0, sizeof(VitalData)); // 初始化为0
                                vitalData.heart_rate = sensorData.heart_rate;
                                vitalData.breath_rate = sensorData.breath_rate;
                                vitalData.presence = sensorData.presence;
                                vitalData.motion = sensorData.motion;
                                vitalData.distance = sensorData.distance;
                                vitalData.sleep_state = sensorData.sleep_state;
                                vitalData.sleep_score = sensorData.sleep_score;
                                vitalData.body_movement = sensorData.body_movement;
                                vitalData.breath_status = sensorData.breath_status;
                                vitalData.sleep_time = sensorData.sleep_time;
                                vitalData.bed_status = sensorData.bed_status;
                                vitalData.abnormal_state = sensorData.abnormal_state;
                                vitalData.avg_heart_rate = sensorData.avg_heart_rate;
                                vitalData.avg_breath_rate = sensorData.avg_breath_rate;
                                vitalData.turn_count = sensorData.turn_count;
                                vitalData.large_move_ratio = sensorData.large_move_ratio;
                                vitalData.small_move_ratio = sensorData.small_move_ratio;
                                vitalData.pos_x = sensorData.pos_x;
                                vitalData.pos_y = sensorData.pos_y;
                                vitalData.pos_z = sensorData.pos_z;
                                vitalData.deep_sleep_time = sensorData.deep_sleep_time;
                                vitalData.light_sleep_time = sensorData.light_sleep_time;
                                vitalData.awake_time = sensorData.awake_time;
                                vitalData.sleep_total_time = sensorData.sleep_total_time;
                                vitalData.deep_sleep_ratio = sensorData.deep_sleep_ratio;
                                vitalData.light_sleep_ratio = sensorData.light_sleep_ratio;
                                vitalData.awake_ratio = sensorData.awake_ratio;
                                vitalData.turnover_count = sensorData.turnover_count;
                                vitalData.struggle_alert = sensorData.struggle_alert;
                                vitalData.no_one_alert = sensorData.no_one_alert;
                                vitalData.apnea_count = sensorData.apnea_count;
                                vitalData.heartbeat_waveform = sensorData.heartbeat_waveform;
                                vitalData.breathing_waveform = sensorData.breathing_waveform;
                                
                                if (xQueueSend(vitalDataQueue, &vitalData, 0) == pdTRUE) {
                                } else {
                                }
                                vitalPacketCounter = 0;
                            }
                            
                            if(frameCounter % 100 == 0) {
                            }
                        }
                        
                        inFrame = false;
                    }
                } else {
                    Serial.println("⚠️ R60ABD1帧缓冲区溢出，重置接收状态");
                    inFrame = false;
                }
            }
            
            prevByte = c;
        }
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}





/**
 * @brief 发送TLV帧到BLE（通用二进制帧发送器）
 * 将TLV帧通过指定的BLE特征发送给客户端
 * @param frame TLV帧对象
 * @param pChar BLE特征指针
 */
void sendFrameToBLE(const BleProto::Frame& frame, BLECharacteristic* pChar) {
    if (xSemaphoreTake(bleSendMutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    if (!deviceConnected || pChar == nullptr) {//连接检查和特征指针检查(指针为空可能是未初始化或已断开连接)
        xSemaphoreGive(bleSendMutex);
        return;
    }
    if (!isBleNotifySubscribed(pChar)) {
        xSemaphoreGive(bleSendMutex);
        return;
    }

    std::vector<uint8_t> raw = BleProto::encodeFrame(frame);
#if BLE_FIXED_20_BYTE_MODE
    const size_t maxPacketSize = FALLBACK_PAYLOAD;//固定20字节模式，使用预定义的最大包大小
#else
    const size_t maxPacketSize = (g_blePayloadSize > 0) ? g_blePayloadSize : FALLBACK_PAYLOAD;
#endif
    size_t offset = 0;

    while (offset < raw.size()) {
        size_t chunkLen = min(maxPacketSize, raw.size() - offset);//计算当前发送的包大小
        
        // 流控：等待可以发送的时机
        int retryCount = 0;//重试次数
        const int MAX_RETRIES = 100;//最大重试次数
        while (!bleFlow.canSend(chunkLen) && retryCount < MAX_RETRIES) {//等待可以发送的时机
            vTaskDelay(5 / portTICK_PERIOD_MS);
            retryCount++;//重试次数增加
        }
        
        pChar->setValue(raw.data() + offset, chunkLen);//设置当前发送的包值
        pChar->notify();//通知客户端有新数据可读
        
        // 记录本次发送，更新流控计数器
        bleFlow.recordSend(chunkLen);//记录本次发送的包大小

        offset += chunkLen;
        if (offset < raw.size()) {
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    }

    Serial.printf("[BLE] 已发送TLV帧 CMD=0x%02X, 总长=%u, 分包=%u\n",
                  frame.cmd, 
                  static_cast<unsigned>(raw.size()),
                  static_cast<unsigned>(maxPacketSize));

    xSemaphoreGive(bleSendMutex);
}

/**
 * @brief 发送即时命令错误响应
 * 
 * 用于一问一答的即时命令失败场景，不包含 TLV_STATE/TLV_STEP。
 * 适用于：查询命令、设置命令、Ping 等即时响应命令。
 * 
 * @param respCmd 响应命令码（如 CMD_QUERY_STATUS、CMD_QUERY_RADAR）
 * @param seq 请求的序列号
 * @param resultCode 错误码（如 ERR_PROTO_PARAM_INVALID）
 */
void sendCommandErrorResponse(uint8_t respCmd, uint8_t seq, uint8_t resultCode) {
    if (!deviceConnected || deviceResultCharacteristic == nullptr) {
        return;
    }

    BleProto::Frame respFrame;
    respFrame.version = BleProto::VERSION;
    respFrame.cmd = respCmd;
    respFrame.seq = seq;
    
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, resultCode);

    sendFrameToBLE(respFrame, deviceResultCharacteristic);
    
    Serial.printf("[BLE] 已发送即时命令错误响应：CMD=0x%02X, SEQ=%u, RESULT_CODE=0x%02X\n",
                  respCmd, seq, resultCode);
}

/**
 * @brief 推送设备状态变化（b3 通道，带去重）
 * 只有状态真正变化时才推送，避免重复推送
 * WiFi 和 MQTT 状态独立去重
 * @param status 设备状态码（见 BleProto::DeviceStatus）
 */
void pushDeviceStatusIfChanged(uint8_t status) {
    // 根据 status 范围判断状态类型
    // WiFi 状态: 0x10-0x1F, MQTT 状态: 0x20-0x2F, 雷达状态: 0x30-0x3F
    bool isWiFiStatus = (status >= 0x10 && status < 0x20);
    bool isMqttStatus = (status >= 0x20 && status < 0x30);
    bool isRadarStatus = (status >= 0x30 && status < 0x40);
    
    // 状态未变化，不重复推送
    if (isWiFiStatus && status == currentWiFiStatus) {
        return;
    }
    if (isMqttStatus && status == currentMqttStatus) {
        return;
    }
    if (isRadarStatus && status == currentRadarSleepStatus) {
        return;
    }
    
    // 更新当前状态
    if (isWiFiStatus) {
        currentWiFiStatus = status;
    } else if (isMqttStatus) {
        currentMqttStatus = status;
    } else if (isRadarStatus) {
        currentRadarSleepStatus = status;
    }
    
    // 推送状态变化
    sendDeviceStatusToBLE(status);
}

/**
 * @brief 同步所有当前设备状态到 BLE（b3 通道）
 * 在 BLE 连接时调用，确保小程序连接后能拿到最新状态
 * 忽略去重检查，强制推送所有当前状态
 */
void syncAllDeviceStatusToBLE() {
    if (!deviceConnected) {
        return;
    }
    
    // 推送当前 WiFi 状态
    if (currentWiFiStatus != 0) {
        sendDeviceStatusToBLE(currentWiFiStatus);
    }
    
    // 推送当前 MQTT 状态
    if (currentMqttStatus != 0) {
        sendDeviceStatusToBLE(currentMqttStatus);
    }
    
    // 推送当前雷达睡眠查询状态
    if (currentRadarSleepStatus != 0) {
        sendDeviceStatusToBLE(currentRadarSleepStatus);
    }
}

/**
 * @brief 发送设备状态推送（b3 通道）
 * 用于设备状态变化时主动推送，与命令响应解耦
 * 适用场景：WiFi 断开/连接、MQTT 断开/连接等
 * @param status 设备状态码（见 BleProto::DeviceStatus）
 */
void sendDeviceStatusToBLE(uint8_t status) {
    if (!deviceConnected || deviceInfoCharacteristic == nullptr) {
        return;
    }

    BleProto::Frame statusFrame;
    statusFrame.version = BleProto::VERSION;
    statusFrame.cmd = BleProto::CMD_DEVICE_INFO_PUSH;
    statusFrame.seq = 0;  // 主动推送 seq 固定为 0

    BleProto::appendTlvU8(statusFrame.data, BleProto::TLV_DEVICE_STATUS, status);

    sendFrameToBLE(statusFrame, deviceInfoCharacteristic);

    Serial.printf("[BLE] 已发送设备状态推送：STATUS=0x%02X\n", status);
}

/**
 * @brief 处理查询雷达数据命令
 * 
 * 完整替代 RADAR_STATUS_CHAR_UUID (a2) 的 NOTIFY 功能，包含：
 * - 基础雷达数据（存在、心率、呼吸、运动、距离）
 * - 坐标数据（posX, posY, posZ）
 * - 体动数据（bodyMovement）
 */
bool processQueryRadarData(const BleProto::Frame& frame) {
    if (frame.cmd == BleProto::CMD_QUERY_RADAR) {
        Serial.println("收到查询雷达数据命令");
        
        // 注意：此命令无法在断连状态下回包，这是 BLE 协议的固有限制。
        // 雷达数据本身是内存快照，不存在"数据不可用"的失败态——
        // 即使传感器无数据，字段值为 0 也是合法的，客户端应自行判断有效性。
        if (!deviceConnected || deviceResultCharacteristic == nullptr) {
            Serial.println("[BLE] CMD_QUERY_RADAR：设备未连接，无法回包");
            return true;
        }

        BleProto::Frame respFrame;
        respFrame.version = BleProto::VERSION;
        respFrame.cmd = BleProto::CMD_QUERY_RADAR;
        respFrame.seq = frame.seq;
        
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
        BleProto::appendTlvU32(respFrame.data, BleProto::TLV_TIMESTAMP, millis());
        
        // 基础雷达数据
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_PRESENCE, sensorData.presence);
        BleProto::appendTlvU16(respFrame.data, BleProto::TLV_HEART_RATE_X10, static_cast<uint16_t>(sensorData.heart_rate * 10.0f + 0.5f));
        BleProto::appendTlvU16(respFrame.data, BleProto::TLV_BREATH_RATE_X10, static_cast<uint16_t>(sensorData.breath_rate * 10.0f + 0.5f));
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_MOTION, sensorData.motion);
        BleProto::appendTlvU16(respFrame.data, BleProto::TLV_DISTANCE_CM, sensorData.distance);
        
        // 坐标和体动数据
        BleProto::appendTlvI16(respFrame.data, BleProto::TLV_POS_X_MM, sensorData.pos_x);
        BleProto::appendTlvI16(respFrame.data, BleProto::TLV_POS_Y_MM, sensorData.pos_y);
        BleProto::appendTlvI16(respFrame.data, BleProto::TLV_POS_Z_MM, sensorData.pos_z);
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_BODY_MOVEMENT, sensorData.body_movement);
        
        sendFrameToBLE(respFrame, deviceResultCharacteristic);
        Serial.println("已发送完整雷达数据查询响应");
        return true;
    }
    return false;
}

/**
 * @brief 处理启动持续发送命令
 */
bool processStartContinuousSend(const BleProto::Frame& frame) {
    if (frame.cmd == BleProto::CMD_START_CONTINUOUS) {
        size_t offset = 0;
        uint8_t type = 0;
        uint16_t len = 0;
        const uint8_t* value = nullptr;

        uint32_t reqInterval = 0;
        bool foundInterval = false;

        while (BleProto::readTlv(frame.data, offset, type, len, value)) {
            if (type == BleProto::TLV_INTERVAL_MS && len == 2) {
                reqInterval = (static_cast<uint16_t>(value[0]) << 8) | value[1];
                foundInterval = true;
            }
        }
        
        // 检查参数是否缺失
        if (!foundInterval) {
            Serial.println("[错误] 缺少间隔参数");
            if (deviceConnected) {
                sendCommandErrorResponse(
                    BleProto::CMD_START_CONTINUOUS,
                    frame.seq,
                    BleProto::ErrorCode::ERR_PROTO_PARAM_MISSING
                );
            }
            return true;
        }
        
        // 检查参数范围
        if (reqInterval < 100 || reqInterval > 10000) {
            Serial.printf("[错误] 间隔参数超出范围: %lu ms，有效范围: 100-10000 ms\n", reqInterval);
            if (deviceConnected) {
                sendCommandErrorResponse(
                    BleProto::CMD_START_CONTINUOUS,
                    frame.seq,
                    BleProto::ErrorCode::ERR_PROTO_PARAM_INVALID
                );
            }
            return true;
        }
        
        // 检查是否已经在持续发送模式
        bool wasEnabled = continuousSendEnabled;
        continuousSendInterval = reqInterval;
        continuousSendEnabled = true;
        bleFlow.reset();
        
        if (wasEnabled) {
            Serial.printf("[BLE] START_CONTINUOUS already enabled, interval updated: %lu ms\n", continuousSendInterval);
        } else {
            Serial.printf("[BLE] START_CONTINUOUS enabled, interval: %lu ms\n", continuousSendInterval);
        }
        
        if (deviceConnected) {
            BleProto::Frame respFrame;
            respFrame.version = BleProto::VERSION;
            respFrame.cmd = BleProto::CMD_START_CONTINUOUS;
            respFrame.seq = frame.seq;
            
            BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
            BleProto::appendTlvU16(respFrame.data, BleProto::TLV_INTERVAL_MS, static_cast<uint16_t>(continuousSendInterval));

            if (deviceResultCharacteristic != nullptr) {
                sendFrameToBLE(respFrame, deviceResultCharacteristic);
            }
        }
        return true;
    }
    return false;
}

/**
 * @brief 处理停止持续发送命令
 * 
 * 幂等操作：无论当前是否在持续发送模式，都返回成功
 */
bool processStopContinuousSend(const BleProto::Frame& frame) {
    if (frame.cmd == BleProto::CMD_STOP_CONTINUOUS) {
        bool wasEnabled = continuousSendEnabled;
        continuousSendEnabled = false;
        
        if (wasEnabled) {
            Serial.println("🛑 停止持续发送模式");
        } else {
            Serial.println("🛑 持续发送模式未启动（幂等操作，返回成功）");
        }
        
        if (deviceConnected) {
            BleProto::Frame respFrame;
            respFrame.version = BleProto::VERSION;
            respFrame.cmd = BleProto::CMD_STOP_CONTINUOUS;
            respFrame.seq = frame.seq;
            BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
            
            if (deviceResultCharacteristic != nullptr) {
                sendFrameToBLE(respFrame, deviceResultCharacteristic);
            }
        }
        return true;
    }
    return false;
}

/**
 * @brief 处理BLE配置数据 — 根据命令码分派到对应处理函数
 */
void processBLEConfig() {
    if (bleCommandQueue == nullptr) {
        return;
    }

    BleCommandMessage msg;
    while (xQueueReceive(bleCommandQueue, &msg, 0) == pdTRUE) {
        BleProto::Frame frame;
        bool parsed = bleFrameParser.input(msg.raw, msg.len, frame);

        while (parsed) {
            bool processed = true;

            switch (frame.cmd) {
                case BleProto::CMD_PING:
                    processEchoRequest(frame);  // Ping 命令，原样回显 TLV_ECHO_CONTENT
                    break;
                case BleProto::CMD_QUERY_STATUS:
                    processQueryStatus(frame);  // 查询设备状态，无参数，始终返回成功
                    break;
                case BleProto::CMD_QUERY_RADAR:
                    processQueryRadarData(frame);  // 查询雷达快照，无参数，始终返回成功（字段值为 0 也合法）
                    break;
                case BleProto::CMD_START_CONTINUOUS:
                    processStartContinuousSend(frame);  // 启动持续发送，解析 TLV_INTERVAL_MS（可选）
                    break;
                case BleProto::CMD_STOP_CONTINUOUS:
                    processStopContinuousSend(frame);  // 停止持续发送，幂等操作
                    break;
                case BleProto::CMD_WIFI_CONFIG:
                    processWiFiConfigCommand(frame);  // WiFi 配网
                    break;
                case BleProto::CMD_WIFI_SCAN:
                    processScanWiFi(frame);  // WiFi 扫描命令，无参数，异步返回扫描结果
                    break;
                case BleProto::CMD_GET_SAVED_WIFI:
                    processGetSavedNetworks(frame);  // 获取已保存 WiFi 列表，无参数
                    break;
                case BleProto::CMD_DELETE_SAVED_WIFI:
                    processDeleteSavedNetwork(frame);  // 删除指定已保存 WiFi，解析 TLV_SSID
                    break;
                case BleProto::CMD_RADAR_SLEEP_QUERY:
                    processRadarSleepQuery(frame);  // 雷达睡眠/综合状态查询开关，解析 TLV_RADAR_SLEEP_ENABLED
                    break;
                default:
                    processed = false;
                    break;
            }

            if (!processed && deviceConnected) {
                sendCommandErrorResponse(
                    BleProto::CMD_ERROR_RESP,
                    frame.seq,
                    BleProto::ErrorCode::ERR_PROTO_CMD_UNKNOWN
                );
            }

            parsed = bleFrameParser.input(nullptr, 0, frame);
        }
    }
}

/* 
 * @brief 构建状态响应帧的 TLV 数据（processQueryStatus 专用）
 * 
 * 完整替代 DEVICE_INFO_CHAR_UUID (b3) 的 READ 功能，包含：
 * - 设备静态信息（ID、SN、MAC、协议版本、固件版本、设备类型）
 * - 运行状态（WiFi 配置状态、连接状态、IP 地址）
 */
static void buildStatusPayload(BleProto::Frame& respFrame) {
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
    
    // 添加设备 SN（仅当存在时）
    // TLV_DEVICE_SN 永远只发 uint64，没有 SN 就不发此字段
    extern uint64_t device_sn;
    extern String getDeviceMacAddress();
    
    if (device_sn > 0) {
        BleProto::appendTlvU64(respFrame.data, BleProto::TLV_DEVICE_SN, device_sn);
    }
    // 注意：不再用 MAC 字符串替代 TLV_DEVICE_SN
    
    // 补充静态设备信息（完整替代 b3 READ）
    BleProto::appendTlvString(respFrame.data, BleProto::TLV_PROTOCOL_VERSION, "1.0.0");
    BleProto::appendTlvString(respFrame.data, BleProto::TLV_FIRMWARE_VERSION, "2.1.0");
    BleProto::appendTlvString(respFrame.data, BleProto::TLV_DEVICE_TYPE, "Radar");
    
    String macAddress = getDeviceMacAddress();
    if (macAddress.length() > 0) {
        BleProto::appendTlvString(respFrame.data, BleProto::TLV_MAC_ADDRESS, macAddress);
    }
    
    // 运行状态
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_WIFI_CONFIGURED, wifiManager.getSavedNetworkCount() > 0 ? 1 : 0);
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_WIFI_CONNECTED, WiFi.status() == WL_CONNECTED ? 1 : 0);
    if (WiFi.status() == WL_CONNECTED) {
        BleProto::appendTlvString(respFrame.data, BleProto::TLV_IP_ADDRESS, WiFi.localIP().toString());
    }
    
    // WiFi 详细状态（查询响应中单独 TLV）
    if (currentWiFiStatus != 0) {
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_WIFI_STATUS, currentWiFiStatus);
    }
    
    // MQTT 状态（查询响应中单独 TLV）
    if (currentMqttStatus != 0) {
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_MQTT_STATUS, currentMqttStatus);
    }
    
    // 雷达睡眠查询状态（查询响应中单独 TLV）
    if (currentRadarSleepStatus != 0) {
        BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RADAR_SLEEP_STATUS, currentRadarSleepStatus);
    }
}

/**
 * @brief 处理查询状态命令（用请求的序列号回复）
 */
bool processQueryStatus(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_QUERY_STATUS) return false;

    // 注意：此命令无法在断连状态下回包，这是 BLE 协议的固有限制。
    // 客户端如果没收到响应，应在超时后重试，而不是等待错误帧。
    if (!deviceConnected || deviceResultCharacteristic == nullptr) {
        Serial.println("[BLE] CMD_QUERY_STATUS：设备未连接，无法回包");
        return true;
    }

    BleProto::Frame respFrame;
    respFrame.version = BleProto::VERSION;
    respFrame.cmd = BleProto::CMD_QUERY_STATUS;// 状态查询命令的响应
    respFrame.seq = frame.seq;
    buildStatusPayload(respFrame);// 构建状态响应的 TLV 数据
    sendFrameToBLE(respFrame, deviceResultCharacteristic);
    Serial.println("已发送设备状态信息");
    return true;
}

/**
 * @brief 处理WiFi配置命令
 */
bool processWiFiConfigCommand(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_WIFI_CONFIG) {
        return false;
    }

    Serial.println("📱 [BLE-WiFi] 收到WiFi配置命令");

    size_t offset = 0;
    uint8_t type = 0;
    uint16_t len = 0;
    const uint8_t* value = nullptr;
    String newSSID = "";
    String newPassword = "";

    while (BleProto::readTlv(frame.data, offset, type, len, value)) {
        if (type == BleProto::TLV_SSID) {
            newSSID = String((const char*)value, len);
        } else if (type == BleProto::TLV_PASSWORD) {
            newPassword = String((const char*)value, len);
        }
    }

    if (newSSID.length() == 0) {
        Serial.println("❌ [BLE-WiFi] SSID 不能为空");
        if (deviceConnected && deviceResultCharacteristic != nullptr) {
            sendCommandErrorResponse(
                BleProto::CMD_WIFI_CONFIG,
                frame.seq,
                BleProto::ErrorCode::ERR_PROTO_PARAM_MISSING
            );
        }
        return true;
    }

    // 保存本次请求的 seq
    wifiConfigRequestCtx.seq = frame.seq;
    wifiConfigRequestCtx.active = true;

    // 立即 ACK：告诉小程序"已收到，处理中"
    if (deviceConnected && deviceResultCharacteristic != nullptr) {
        BleProto::Frame ackFrame;
        ackFrame.version = BleProto::VERSION;
        ackFrame.cmd = BleProto::CMD_WIFI_CONFIG;
        ackFrame.seq = frame.seq;
        BleProto::appendTlvU8(ackFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::PROCESSING);
        BleProto::appendTlvString(ackFrame.data, BleProto::TLV_SSID, newSSID);// 可选：回显 SSID，方便小程序确认配置内容
        sendFrameToBLE(ackFrame, deviceResultCharacteristic);
    }

    // 调用 WiFiManager 处理配置数据，后续结果通过 wifiConfigResultHandler 回包
    wifiManager.handleConfigurationData(newSSID.c_str(), newPassword.c_str());
    return true;
}

/**
 * @brief 处理扫描WiFi命令
 */
bool processScanWiFi(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_WIFI_SCAN) {
        return false;
    }

    Serial.println("📱 [BLE-WiFi] 收到WiFi扫描命令");

    // 保存本次请求的 seq
    wifiScanRequestCtx.seq = frame.seq;
    wifiScanRequestCtx.active = true;

    // 立即回 processing
    if (deviceConnected && deviceResultCharacteristic != nullptr) {
        BleProto::Frame ackFrame;
        ackFrame.version = BleProto::VERSION;
        ackFrame.cmd = BleProto::CMD_WIFI_SCAN;
        ackFrame.seq = frame.seq;
        BleProto::appendTlvU8(ackFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::PROCESSING);
        sendFrameToBLE(ackFrame, deviceResultCharacteristic);
    }

    if (!wifiManager.startScan(30000)) {
        // 启动失败，立即回错误包
        if (deviceConnected && deviceResultCharacteristic != nullptr) {
            sendCommandErrorResponse(
                BleProto::CMD_WIFI_SCAN,
                frame.seq,
                BleProto::ErrorCode::ERR_WIFI_BUSY
            );
        }
        wifiScanRequestCtx.active = false;
    }

    return true; // 命令已识别，始终返回 true
}
/**
 * @brief 处理获取已保存WiFi网络命令
 */
/**
 * @brief 处理获取已保存WiFi网络命令
 * 
 * 即时命令：同步获取已保存的 WiFi 列表，直接回包，不走异步流程。
 * 回包格式遵循即时命令规则：只用 TLV_RESULT_CODE，不带 TLV_STATE/TLV_STEP。
 */
bool processGetSavedNetworks(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_GET_SAVED_WIFI) {
        return false;
    }
    Serial.println("📱 [BLE-WiFi] 收到获取已保存WiFi网络命令");
    
    if (!deviceConnected || deviceResultCharacteristic == nullptr) {
        Serial.println("[BLE] CMD_GET_SAVED_WIFI：设备未连接，无法回包");
        return true;
    }
    
    savedNetworksRequestCtx.seq = frame.seq;
    savedNetworksRequestCtx.active = true;
    wifiManager.getSavedNetworks();
    return true;
}

/**
 * @brief 处理删除指定已保存WiFi命令
 *
 * 即时命令：解析 TLV_SSID，删除对应保存的 WiFi 配置，直接回包。
 * 回包格式遵循即时命令规则：只用 TLV_RESULT_CODE，不带 TLV_STATE/TLV_STEP。
 */
bool processDeleteSavedNetwork(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_DELETE_SAVED_WIFI) {
        return false;
    }

    Serial.println("📱 [BLE-WiFi] 收到删除已保存WiFi命令");

    if (!deviceConnected || deviceResultCharacteristic == nullptr) {
        Serial.println("[BLE] CMD_DELETE_SAVED_WIFI：设备未连接，无法回包");
        return true;
    }

    size_t offset = 0;
    uint8_t type = 0;
    uint16_t len = 0;
    const uint8_t* value = nullptr;
    String targetSsid = "";

    while (BleProto::readTlv(frame.data, offset, type, len, value)) {
        if (type == BleProto::TLV_SSID) {
            targetSsid = String((const char*)value, len);
        }
    }

    if (targetSsid.length() == 0) {
        sendCommandErrorResponse(
            BleProto::CMD_DELETE_SAVED_WIFI,
            frame.seq,
            BleProto::ErrorCode::ERR_PROTO_PARAM_MISSING
        );
        return true;
    }

    bool existed = false;
    bool removed = wifiManager.removeWiFiConfig(targetSsid.c_str(), &existed);

    if (!existed) {
        sendCommandErrorResponse(
            BleProto::CMD_DELETE_SAVED_WIFI,
            frame.seq,
            BleProto::ErrorCode::ERR_WIFI_SSID_NOT_FOUND
        );
        return true;
    }

    if (!removed) {
        sendCommandErrorResponse(
            BleProto::CMD_DELETE_SAVED_WIFI,
            frame.seq,
            BleProto::ErrorCode::ERR_DEV_STORAGE_FAIL
        );
        return true;
    }

    BleProto::Frame respFrame;
    respFrame.version = BleProto::VERSION;
    respFrame.cmd = BleProto::CMD_DELETE_SAVED_WIFI;
    respFrame.seq = frame.seq;
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
    BleProto::appendTlvString(respFrame.data, BleProto::TLV_SSID, targetSsid);
    BleProto::appendTlvU16(respFrame.data, BleProto::TLV_WIFI_COUNT, static_cast<uint16_t>(wifiManager.getSavedNetworkCount()));
    sendFrameToBLE(respFrame, deviceResultCharacteristic);

    return true;
}

/**
 * @brief 处理雷达睡眠/综合状态查询开关命令
 * 通过 TLV_RADAR_SLEEP_ENABLED 参数控制 0x8D（睡眠状态）和 0x90（综合状态）查询的开启/关闭
 */
bool processRadarSleepQuery(const BleProto::Frame& frame) {
    if (frame.cmd != BleProto::CMD_RADAR_SLEEP_QUERY) {
        return false;
    }

    if (!deviceConnected) {
        Serial.println("[BLE] CMD_RADAR_SLEEP_QUERY：设备未连接，无法回包");
        return true;
    }

    // 解析 TLV_RADAR_SLEEP_ENABLED 参数
    size_t offset = 0;
    uint8_t type = 0;
    uint16_t len = 0;
    const uint8_t* value = nullptr;
    bool foundParam = false;
    uint8_t enabledValue = 0;

    while (BleProto::readTlv(frame.data, offset, type, len, value)) {
        if (type == BleProto::TLV_RADAR_SLEEP_ENABLED && len == 1) {
            enabledValue = value[0];
            foundParam = true;
        }
    }

    // 参数校验
    if (!foundParam) {
        sendCommandErrorResponse(
            BleProto::CMD_RADAR_SLEEP_QUERY,
            frame.seq,
            BleProto::ErrorCode::ERR_PROTO_PARAM_MISSING
        );
        return true;
    }

    if (enabledValue != 0 && enabledValue != 1) {
        sendCommandErrorResponse(
            BleProto::CMD_RADAR_SLEEP_QUERY,
            frame.seq,
            BleProto::ErrorCode::ERR_PROTO_PARAM_INVALID
        );
        return true;
    }

    // 设置开关状态（先保存旧状态用于比较）
    bool oldEnabled = radarSleepQueryEnabled;
    radarSleepQueryEnabled = (enabledValue == 1);
    Serial.printf("️ [BLE] 雷达睡眠查询开关: %s\n", radarSleepQueryEnabled ? "开启" : "关闭");
    
    // 状态变化时推送 b3
    if (oldEnabled != radarSleepQueryEnabled) {
        pushDeviceStatusIfChanged(
            radarSleepQueryEnabled
                ? BleProto::DeviceStatus::RADAR_SLEEP_QUERY_ENABLED
                : BleProto::DeviceStatus::RADAR_SLEEP_QUERY_DISABLED
        );
    }

    // 构建成功响应
    BleProto::Frame respFrame;
    respFrame.version = BleProto::VERSION;
    respFrame.cmd = BleProto::CMD_RADAR_SLEEP_QUERY;
    respFrame.seq = frame.seq;

    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
    BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RADAR_SLEEP_ENABLED, enabledValue);

    sendFrameToBLE(respFrame, deviceResultCharacteristic);
    return true;
}

/**
 * @brief 处理回显请求命令，类似于网络中的ping命令，可以携带任意字符串，设备原样返回，主要用于测试BLE通信的可靠性和时延
 * 类似于心跳包，但这个是由手机端主动发起的，设备收到后原样返回，可以携带任意字符串内容，手机端可以通过这个命令来测试BLE通信的可靠性和时延
 */
bool processEchoRequest(const BleProto::Frame& frame) {
    if (frame.cmd == BleProto::CMD_PING) {
        Serial.println("📱 [BLE] 收到回显请求");
        
        size_t offset = 0;
        uint8_t type = 0;
        uint16_t len = 0;
        const uint8_t* value = nullptr;
        String echoContent = "";

        while (BleProto::readTlv(frame.data, offset, type, len, value)) {
            if (type == BleProto::TLV_ECHO_CONTENT) {
                echoContent = String((const char*)value, len);
            }
        }
        
        if (deviceConnected) {
            BleProto::Frame respFrame;
            respFrame.version = BleProto::VERSION;
            respFrame.cmd = BleProto::CMD_PING;
            respFrame.seq = frame.seq;
            BleProto::appendTlvU8(respFrame.data, BleProto::TLV_RESULT_CODE, BleProto::ErrorCode::SUCCESS);
            if (echoContent.length() > 0) {
                BleProto::appendTlvString(respFrame.data, BleProto::TLV_ECHO_CONTENT, echoContent);
            }
            if (deviceResultCharacteristic != nullptr) {
                sendFrameToBLE(respFrame, deviceResultCharacteristic);// 发送回显响应
            }
        }
        return true;
    }
    return false;
}

/**
 * @brief 发送WiFi配置结果到BLE (纯TLV格式)
 * @param resultCode 具体错误码（BleProto::ErrorCode::*）
 * @param ssid       连接的 SSID
 * @param ipAddress  获取到的 IP 地址
 */
void sendWiFiConfigResultToBLE(uint8_t resultCode,
                               const String& ssid, const String& ipAddress) {
    BleProto::Frame frame;
    frame.version = BleProto::VERSION;
    frame.cmd = BleProto::CMD_WIFI_CONFIG;
    frame.seq = wifiConfigRequestCtx.active ? wifiConfigRequestCtx.seq : 0;
    frame.data.clear();

    // 命令响应只返回 RESULT_CODE
    BleProto::appendTlvU8(frame.data, BleProto::TLV_RESULT_CODE, resultCode);

    if (ssid.length() > 0) {
        BleProto::appendTlvString(frame.data, BleProto::TLV_SSID, ssid);
    }
    if (ipAddress.length() > 0) {
        BleProto::appendTlvString(frame.data, BleProto::TLV_IP_ADDRESS, ipAddress);
    }

    sendFrameToBLE(frame, deviceResultCharacteristic);

    // 最终结果后清除上下文（SUCCESS 或 ERR_xxx，不包括 PROCESSING）
    if (resultCode != BleProto::ErrorCode::PROCESSING) {
        wifiConfigRequestCtx.active = false;
    }
}

/**
 * @brief 发送WiFi扫描结果到BLE (纯TLV格式)
 * @param resultCode 具体错误码（BleProto::ErrorCode::*）
 * @param networks   扫描到的 WiFi 网络列表
 */
void sendWiFiScanResultToBLE(uint8_t resultCode,
                             const std::vector<WiFiScanResult>& networks) {
    BleProto::Frame frame;
    frame.version = BleProto::VERSION;
    frame.cmd = BleProto::CMD_WIFI_SCAN;
    frame.seq = wifiScanRequestCtx.active ? wifiScanRequestCtx.seq : 0;
    frame.data.clear();

    // 命令响应只返回 RESULT_CODE
    BleProto::appendTlvU8(frame.data, BleProto::TLV_RESULT_CODE, resultCode);

    if (!networks.empty()) {
        BleProto::appendTlvU16(frame.data, BleProto::TLV_WIFI_COUNT, static_cast<uint16_t>(networks.size()));
        for (const auto& network : networks) {
            std::vector<uint8_t> wifiItem;
            BleProto::appendTlvString(wifiItem, BleProto::TLV_SSID, network.ssid);
            BleProto::appendTlvI8(wifiItem, BleProto::TLV_RSSI, static_cast<int8_t>(network.rssi));
            BleProto::WifiSecurityType secType = BleProto::WIFI_SEC_UNKNOWN;
            if (network.security == "OPEN") secType = BleProto::WIFI_SEC_OPEN;
            else if (network.security == "WEP") secType = BleProto::WIFI_SEC_WEP;
            else if (network.security == "WPA" || network.security == "WPA/WPA2") secType = BleProto::WIFI_SEC_WPA;
            else if (network.security == "WPA2" || network.security == "WPA2-PSK" || network.security == "WPA2-EAP") secType = BleProto::WIFI_SEC_WPA2;
            else if (network.security == "WPA3" || network.security == "WPA2/WPA3") secType = BleProto::WIFI_SEC_WPA3;
            BleProto::appendTlvU8(wifiItem, BleProto::TLV_SECURITY, static_cast<uint8_t>(secType));
            BleProto::appendTlvBlock(frame.data, BleProto::TLV_WIFI_ITEM, wifiItem);
        }
    }

    sendFrameToBLE(frame, deviceResultCharacteristic);

    // 最终结果后清除上下文（SUCCESS 或 ERR_xxx，不包括 PROCESSING）
    if (resultCode != BleProto::ErrorCode::PROCESSING) {
        wifiScanRequestCtx.active = false;
    }
}

/**
 * @brief 发送保存的网络列表到BLE (纯TLV格式)
 * 
 * 即时命令回包：只用 TLV_RESULT_CODE，不带 TLV_STATE/TLV_STEP。
 * CMD_GET_SAVED_WIFI 是即时命令，getSavedNetworks() 同步返回结果，
 * 没有异步等待阶段，不需要 STATE/STEP 描述流程进度。
 */
void sendSavedNetworksResultToBLE(bool success, const std::vector<WiFiScanResult>& networks) {
    BleProto::Frame frame;
    frame.version = BleProto::VERSION;
    frame.cmd = BleProto::CMD_GET_SAVED_WIFI;
    frame.seq = savedNetworksRequestCtx.active ? savedNetworksRequestCtx.seq : 0;
    frame.data.clear();

    // 即时命令：只用 TLV_RESULT_CODE，不带 TLV_STATE/TLV_STEP
    BleProto::appendTlvU8(frame.data, BleProto::TLV_RESULT_CODE,
        success ? BleProto::ErrorCode::SUCCESS : BleProto::ErrorCode::ERR_DEV_STORAGE_FAIL);

    if (success) {
        BleProto::appendTlvU16(frame.data, BleProto::TLV_WIFI_COUNT, static_cast<uint16_t>(networks.size()));
        for (const auto& network : networks) {
            std::vector<uint8_t> wifiItem;
            BleProto::appendTlvString(wifiItem, BleProto::TLV_SSID, network.ssid);
            BleProto::appendTlvString(wifiItem, BleProto::TLV_PASSWORD, network.password);
            BleProto::appendTlvBlock(frame.data, BleProto::TLV_WIFI_ITEM, wifiItem);
        }
    }

    sendFrameToBLE(frame, deviceResultCharacteristic);
    savedNetworksRequestCtx.active = false;
}
