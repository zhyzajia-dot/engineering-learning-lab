/**
 * @file tuya_link.cpp
 * @brief 睡眠雷达接入 TuyaLink 的 TLS MQTT 实现。
 *
 * 将雷达、睡眠算法和红外状态映射为涂鸦 DP，并负责连接与属性上报。
 */
#include "tuya_link.h"

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_task_wdt.h>
#include <mbedtls/md.h>
#include <time.h>

#include "pir_manager.h"
#include "radar_manager.h"
#include "tasks_manager.h"
#include "tuya_credentials.h"

/**
 * 数据来源：
 * - DP 102~107 使用 R60ABD1 雷达解析得到的原始数据。
 * - DP 108~110 优先使用本地 SleepAnalyzer 快照；算法快照无效时，
 *   回退到雷达模块原始睡眠结果。
 * - DP 101 使用 HC-SR501 的 GPIO44 数字输入状态。
 *
 * 本文件仅替换云端 MQTT 链路，不修改现有 BLE 小程序配网、
 * 雷达串口解析、睡眠算法及 InfluxDB 存储链路。
 */
namespace {

// 涂鸦中国区 MQTT 接入配置，TuyaLink 强制使用 TLS 端口 8883。
constexpr char TUYA_MQTT_HOST[] = "m1.tuyacn.com";
constexpr uint16_t TUYA_MQTT_PORT = 8883;
constexpr uint16_t TUYA_KEEP_ALIVE_SECONDS = 60;

// 沿用原工程的上报节奏：日常 2 秒、睡眠 10 秒、无人状态 10 秒。
constexpr unsigned long RECONNECT_INTERVAL_MS = 5000;
constexpr unsigned long DAILY_DATA_INTERVAL_MS = 2000;
constexpr unsigned long SLEEP_DATA_INTERVAL_MS = 10000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 10000;

// 时间早于 2024-01-01 视为尚未完成 NTP 同步，不能用于涂鸦鉴权。
constexpr time_t MIN_VALID_UNIX_TIME = 1704067200;

// 涂鸦官方 IoT Core SDK 随附的 GoDaddy G2 根证书，用于校验服务端。
constexpr char TUYA_ROOT_CA[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDxTCCAq2gAwIBAgIBADANBgkqhkiG9w0BAQsFADCBgzELMAkGA1UEBhMCVVMx
EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxGjAYBgNVBAoT
EUdvRGFkZHkuY29tLCBJbmMuMTEwLwYDVQQDEyhHbyBEYWRkeSBSb290IENlcnRp
ZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5MDkwMTAwMDAwMFoXDTM3MTIzMTIz
NTk1OVowgYMxCzAJBgNVBAYTAlVTMRAwDgYDVQQIEwdBcml6b25hMRMwEQYDVQQH
EwpTY290dHNkYWxlMRowGAYDVQQKExFHb0RhZGR5LmNvbSwgSW5jLjExMC8GA1UE
AxMoR28gRGFkZHkgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIw
DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAL9xYgjx+lk09xvJGKP3gElY6SKD
E6bFIEMBO4Tx5oVJnyfq9oQbTqC023CYxzIBsQU+B07u9PpPL1kwIuerGVZr4oAH
/PMWdYA5UXvl+TW2dE6pjYIT5LY/qQOD+qK+ihVqf94Lw7YZFAXK6sOoBJQ7Rnwy
DfMAZiLIjWltNowRGLfTshxgtDj6AozO091GB94KPutdfMh8+7ArU6SSYmlRJQVh
GkSBjCypQ5Yj36w6gZoOKcUcqeldHraenjAKOc7xiID7S13MMuyFYkMlNAJWJwGR
tDtwKj9useiciAF9n9T521NtYJ2/LOdYq7hfRvzOxBsDPAnrSTFcaUaz4EcCAwEA
AaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYE
FDqahQcQZyi27/a9BUFuIMGU2g/eMA0GCSqGSIb3DQEBCwUAA4IBAQCZ21151fmX
WWcDYfF+OwYxdS2hII5PZYe096acvNjpL9DbWu7PdIxztDhC2gV7+AJ1uP2lsdeu
9tfeE8tTEH6KRtGX+rcuKxGrkLAngPnon1rpN5+r5N9ss4UXnT3ZJE95kTXWXwTr
gIOrmgIttRD02JDHBHNA7XIloKmf7J6raBKZV8aPEjoJpL1E/QYVN8Gb5DKj7Tjo
2GTzLH4U/ALqn83/B2gX2yKQOC16jdFU8WnjXzPKej17CuPKf1855eJ1usV2GDPO
LPAvTK33sefOT6jEm0pUBsV/fdUID+Ic/n4XuKxe9tQWskMJDE32p2u0mYRlynqI
4uJEvlz36hz1
-----END CERTIFICATE-----
)EOF";

// TLS 客户端作为 PubSubClient 的安全传输层，禁止使用 setInsecure()。
WiFiClientSecure tuyaTlsClient;
PubSubClient tuyaMqttClient(tuyaTlsClient);

// 运行期状态：消息序号、各类上报时间、NTP 和连接状态。
uint32_t messageSequence = 1;
unsigned long lastReconnectAttempt = 0;
unsigned long lastDailyReport = 0;
unsigned long lastSleepReport = 0;
unsigned long lastHeartbeatReport = 0;
bool ntpStarted = false;
bool wasConnected = false;
uint32_t lastPirRevisionReported = 0;

/**
 * @brief 构造当前设备的 TuyaLink MQTT 主题。
 * @param suffix 设备主题后缀，例如 /thing/property/report。
 * @return 完整主题 tylink/{deviceId}{suffix}。
 */
String topic(const char* suffix) {
    return String("tylink/") + TUYA_DEVICE_ID + suffix;
}

/**
 * @brief 获取 Unix 毫秒时间戳。
 * @return 当前 UTC 时间，单位为毫秒。
 */
uint64_t unixTimeMs() {
    return static_cast<uint64_t>(time(nullptr)) * 1000ULL;
}

/**
 * @brief 生成本次上报的唯一消息 ID。
 *
 * 使用 Unix 秒和本地递增序号组合，避免设备单次运行期间重复。
 */
String nextMessageId() {
    char value[33];
    snprintf(value, sizeof(value), "%lu-%lu",
             static_cast<unsigned long>(time(nullptr)),
             static_cast<unsigned long>(messageSequence++));
    return String(value);
}

/**
 * @brief 检查系统时间，并在需要时启动 NTP 同步。
 * @return true 时间已经可用于 TLS 和 TuyaLink 鉴权。
 *
 * 函数不阻塞等待 NTP；后台任务会在后续循环中继续检查，避免阻塞
 * 雷达、BLE 和其他 FreeRTOS 任务。
 */
bool ensureClockReady() {
    if (time(nullptr) >= MIN_VALID_UNIX_TIME) {
        return true;
    }

    if (!ntpStarted) {
        configTime(0, 0, "ntp.aliyun.com", "pool.ntp.org", "time.cloudflare.com");
        ntpStarted = true;
        Serial.println("[TuyaLink] NTP synchronization started");
    }

    return time(nullptr) >= MIN_VALID_UNIX_TIME;
}

/**
 * @brief 计算 TuyaLink 鉴权要求的 HMAC-SHA256 十六进制字符串。
 * @param content 待签名明文。
 * @param secret 当前设备的 DeviceSecret。
 * @return 64 字符小写十六进制签名；失败时返回空字符串。
 */
String hmacSha256Hex(const String& content, const char* secret) {
    unsigned char digest[32] = {0};
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (info == nullptr ||
        mbedtls_md_hmac(info,
                        reinterpret_cast<const unsigned char*>(secret),
                        strlen(secret),
                        reinterpret_cast<const unsigned char*>(content.c_str()),
                        content.length(),
                        digest) != 0) {
        return String();
    }

    char hex[65];
    for (size_t i = 0; i < sizeof(digest); ++i) {
        snprintf(hex + i * 2, 3, "%02x", digest[i]);
    }
    hex[64] = '\0';
    return String(hex);
}

/**
 * @brief 将雷达活动状态转换为涂鸦 DP 107 枚举。
 */
const char* motionStatusValue(uint8_t motion) {
    switch (motion) {
        case 1:
            return "1_static";
        case 2:
            return "2_active";
        default:
            return "0_unknown";
    }
}

/**
 * @brief 将本地 SleepAnalyzer 状态转换为涂鸦 DP 109 枚举。
 *
 * 本地状态：1 在床、2 清醒、3 浅睡、4 深睡、5 REM；
 * 其他状态均归为离床。涂鸦后台已增加 4_rem_sleep。
 */
const char* algorithmSleepStateValue(int state) {
    switch (state) {
        case 4:
            return "0_deep_sleep";
        case 3:
            return "1_light_sleep";
        case 1:
        case 2:
            return "2_awake";
        case 5:
            return "4_rem_sleep";
        default:
            return "3_out_of_bed";
    }
}

/**
 * @brief 将 R60ABD1 原始睡眠状态转换为涂鸦 DP 109 枚举。
 *
 * 仅在本地算法快照不可用时作为兜底数据源。
 */
const char* radarSleepStateValue(uint8_t state) {
    switch (state) {
        case 0:
            return "0_deep_sleep";
        case 1:
            return "1_light_sleep";
        case 2:
            return "2_awake";
        default:
            return "3_out_of_bed";
    }
}

/**
 * @brief 判断本地算法状态是否表示人体仍在床上。
 */
bool algorithmInBed(int state) {
    return state >= 1 && state <= 5;
}

/**
 * @brief 按 TuyaLink 属性格式向 data 对象写入一个 DP。
 *
 * 每个属性包含 value 和毫秒时间戳，模板允许 Bool、Value 和 Enum
 * 使用同一套构造逻辑。
 */
template <typename TValue>
void addProperty(JsonObject data, const char* code, const TValue& value, uint64_t timestamp) {
    JsonObject property = data[code].to<JsonObject>();
    property["value"] = value;
    property["time"] = timestamp;
}

/**
 * @brief 将当前雷达和睡眠算法数据转换为涂鸦 DP。
 * @param data TuyaLink payload 中的 data 对象。
 * @param includeVitals 是否包含心率、呼吸、体动、距离和活动状态。
 * @param includeSleep 是否包含床状态、睡眠阶段和睡眠评分。
 *
 * DP 映射：
 * - 101 pir_presence    -> HC-SR501 GPIO44
 * - 102 radar_presence  -> sensorData.presence
 * - 103 heart_rate      -> sensorData.heart_rate
 * - 104 breath_rate     -> sensorData.breath_rate
 * - 105 body_movement   -> sensorData.body_movement
 * - 106 distance        -> sensorData.distance
 * - 107 motion_status   -> sensorData.motion
 * - 108 bed_status      -> 本地算法优先，雷达数据兜底
 * - 109 sleep_state     -> 本地算法优先，雷达数据兜底
 * - 110 sleep_score     -> 本地算法优先，雷达数据兜底
 */
void appendCurrentProperties(JsonObject data, bool includeVitals, bool includeSleep) {
    const uint64_t timestamp = unixTimeMs();
    const SensorData radar = sensorData;

    // PIR完成上电稳定后才加入上报，避免初始化脉冲造成误报。
    PirSnapshot pir = {};
    if (getPirSnapshot(pir)) {
        addProperty(data, "pir_presence", pir.presence, timestamp);
    }

    addProperty(data, "radar_presence", radar.presence != 0, timestamp);

    if (includeVitals) {
        addProperty(data, "heart_rate",
                    constrain(static_cast<int>(radar.heart_rate + 0.5f), 0, 200),
                    timestamp);
        addProperty(data, "breath_rate",
                    constrain(static_cast<int>(radar.breath_rate + 0.5f), 0, 40),
                    timestamp);
        addProperty(data, "body_movement",
                    constrain(static_cast<int>(radar.body_movement), 0, 100),
                    timestamp);
        addProperty(data, "distance",
                    constrain(static_cast<int>(radar.distance), 0, 1000),
                    timestamp);
        addProperty(data, "motion_status", motionStatusValue(radar.motion), timestamp);
    }

    if (!includeSleep) {
        return;
    }

    // 使用并发安全的快照 API，避免在算法任务更新结构体时读到半包数据。
    SleepAnalysisSnapshot sleep = {};
    if (getFreshSleepAnalysisSnapshot(sleep)) {
        addProperty(data, "bed_status",
                    algorithmInBed(sleep.algorithm_state) ? "1_in_bed" : "0_out_of_bed",
                    timestamp);
        addProperty(data, "sleep_state",
                    algorithmSleepStateValue(sleep.algorithm_state),
                    timestamp);
        // 尚未形成有效睡眠时长时不上传 0 分，避免把“未评分”误作真实零分。
        if (sleep.total_sleep_time > 0) {
            addProperty(data, "sleep_score",
                        constrain(static_cast<int>(sleep.total_score + 0.5f), 0, 100),
                        timestamp);
        }
    } else {
        // 算法快照过期或尚未生成时，使用雷达模块自身结果保证基础状态可用。
        addProperty(data, "bed_status",
                    radar.bed_status ? "1_in_bed" : "0_out_of_bed",
                    timestamp);
        addProperty(data, "sleep_state", radarSleepStateValue(radar.sleep_state), timestamp);
        if (radar.sleep_score > 0) {
            addProperty(data, "sleep_score",
                        constrain(static_cast<int>(radar.sleep_score), 0, 100),
                        timestamp);
        }
    }
}

/**
 * @brief 构造并发布 TuyaLink 属性上报。
 * @return true MQTT 已成功接收待发送数据。
 *
 * 发布到 tylink/{deviceId}/thing/property/report。平台处理结果通过
 * report_response 主题异步返回。
 */
bool publishProperties(bool includeVitals, bool includeSleep) {
    if (!tuyaMqttClient.connected() || !ensureClockReady()) {
        return false;
    }

    JsonDocument document;
    document["msgId"] = nextMessageId();
    document["time"] = unixTimeMs();
    JsonObject data = document["data"].to<JsonObject>();
    appendCurrentProperties(data, includeVitals, includeSleep);

    String payload;
    serializeJson(document, payload);
    const String reportTopic = topic("/thing/property/report");

    const bool published = tuyaMqttClient.publish(reportTopic.c_str(), payload.c_str());
    if (!published) {
        Serial.printf("[TuyaLink] property report failed, state=%d\n", tuyaMqttClient.state());
    }
    return published;
}

/**
 * @brief 检测PIR状态版本并在变化时立即上报。
 *
 * 周期性日常/睡眠上报也会携带pir_presence；此函数用于缩短人体
 * 红外状态变化到达涂鸦平台的延迟。
 */
void reportPirChangeIfNeeded() {
    PirSnapshot pir = {};
    if (!getPirSnapshot(pir) || pir.revision == lastPirRevisionReported) {
        return;
    }

    if (publishProperties(false, false)) {
        lastPirRevisionReported = pir.revision;
        Serial.printf("[TuyaLink] PIR state reported: %s\n",
                      pir.presence ? "presence" : "clear");
    }
}

/**
 * @brief 响应平台的属性查询请求。
 *
 * 查询响应返回当前完整 DP 快照，不改变雷达或算法状态。
 */
void publishGetResponse(const JsonDocument& request) {
    JsonDocument response;
    response["msgId"] = request["msgId"] | "";
    response["time"] = unixTimeMs();
    response["code"] = 0;
    JsonObject data = response["data"].to<JsonObject>();
    appendCurrentProperties(data, true, true);

    String payload;
    serializeJson(response, payload);
    const String responseTopic = topic("/thing/property/get_response");
    tuyaMqttClient.publish(responseTopic.c_str(), payload.c_str());
}

/**
 * @brief 处理 TuyaLink 下行 MQTT 消息。
 *
 * 当前支持属性查询和属性上报响应。产品 DP 均为只上报属性，因此
 * 暂不处理属性设置命令。
 */
void mqttCallback(char* incomingTopic, byte* payload, unsigned int length) {
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; ++i) {
        message += static_cast<char>(payload[i]);
    }

    Serial.printf("[TuyaLink] received topic: %s\n", incomingTopic);

    JsonDocument document;
    const DeserializationError error = deserializeJson(document, message);
    if (error) {
        Serial.printf("[TuyaLink] invalid JSON: %s\n", error.c_str());
        return;
    }

    const String incoming(incomingTopic);
    if (incoming == topic("/thing/property/get")) {
        publishGetResponse(document);
    } else if (incoming == topic("/thing/property/report_response")) {
        const int code = document["code"] | -1;
        if (code != 0) {
            Serial.printf("[TuyaLink] cloud rejected property report, code=%d\n", code);
        }
    }
}

/**
 * @brief 使用一机一密凭证连接 TuyaLink。
 * @return true TLS/MQTT 鉴权成功并完成主题订阅。
 *
 * 鉴权规则：
 * - clientId: tuyalink_{deviceId}
 * - username: deviceId|signMethod=hmacSha256,timestamp=...,secureMode=1,accessType=1
 * - password: HMAC-SHA256(DeviceSecret, 签名明文)
 */
bool connectTuyaLink() {
    if (!WiFi.isConnected() || !ensureClockReady()) {
        return false;
    }

    const time_t timestamp = time(nullptr);
    const String clientId = String("tuyalink_") + TUYA_DEVICE_ID;
    const String username = String(TUYA_DEVICE_ID) +
                            "|signMethod=hmacSha256,timestamp=" +
                            String(static_cast<unsigned long>(timestamp)) +
                            ",secureMode=1,accessType=1";
    // 签名明文的字段顺序必须与涂鸦开放协议保持一致。
    const String signContent = String("deviceId=") + TUYA_DEVICE_ID +
                               ",timestamp=" +
                               String(static_cast<unsigned long>(timestamp)) +
                               ",secureMode=1,accessType=1";
    const String password = hmacSha256Hex(signContent, TUYA_DEVICE_SECRET);
    if (password.isEmpty()) {
        Serial.println("[TuyaLink] HMAC-SHA256 generation failed");
        return false;
    }

    Serial.printf("[TuyaLink] connecting to %s:%u\n", TUYA_MQTT_HOST, TUYA_MQTT_PORT);
    if (!tuyaMqttClient.connect(clientId.c_str(), username.c_str(), password.c_str())) {
        Serial.printf("[TuyaLink] connection failed, state=%d\n", tuyaMqttClient.state());
        pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_FAILED);
        return false;
    }

    // 订阅上报结果和属性查询；本产品当前没有可写 DP。
    tuyaMqttClient.subscribe(topic("/thing/property/report_response").c_str());
    tuyaMqttClient.subscribe(topic("/thing/property/get").c_str());
    pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_CONNECTED);
    Serial.println("[TuyaLink] connected");
    publishProperties(true, true);
    return true;
}

/**
 * @brief 维护 MQTT 连接并执行非阻塞重连。
 *
 * 重连最短间隔为 5 秒，连接状态继续复用原 BLE 状态通知通道，
 * 因此现有小程序仍能看到 MQTT 成功、失败和断开状态。
 */
void maintainConnection() {
    if (!WiFi.isConnected()) {
        return;
    }

    if (!tuyaMqttClient.connected()) {
        if (wasConnected) {
            wasConnected = false;
            pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_DISCONNECTED);
        }

        const unsigned long now = millis();
        if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
            lastReconnectAttempt = now;
            wasConnected = connectTuyaLink();
        }
        return;
    }

    wasConnected = true;
    tuyaMqttClient.loop();
}

}  // namespace

// 保持与原 mqtt.cpp 相同的任务句柄名称，避免修改任务管理代码。
TaskHandle_t mqttTaskHandle = nullptr;

/**
 * @brief 上报有人时的日常雷达数据。
 */
void sendDailyDataToMQTT() {
    if (publishProperties(true, true)) {
        Serial.println("[TuyaLink] daily radar data reported");
    }
}

/**
 * @brief 上报睡眠相关 DP。
 *
 * allowSessionEnd 为原工程睡眠会话结束重试接口保留参数。TuyaLink
 * 上报函数本身不限制睡眠状态，因此会话结束也可直接发布最终结果。
 */
bool sendSleepDataToMQTT(bool allowSessionEnd) {
    (void)allowSessionEnd;
    const bool result = publishProperties(false, true);
    if (result) {
        Serial.println("[TuyaLink] sleep data reported");
    }
    return result;
}

/**
 * @brief 无人时同步存在和睡眠状态。
 *
 * 不再发送旧平台自定义 heartbeat 字段，连接保活由 MQTT Keep Alive
 * 完成，避免向涂鸦上报产品模型中不存在的属性。
 */
void sendHeartbeatToMQTT() {
    if (publishProperties(false, true)) {
        Serial.println("[TuyaLink] no-person state reported");
    }
}

/**
 * @brief TuyaLink 后台任务主循环。
 *
 * 初始化 TLS 根证书和 MQTT 参数后，持续完成：
 * 1. WiFi 可用时同步时间并连接涂鸦；
 * 2. 调用 MQTT loop 维持连接和接收下行；
 * 3. 有人时每 2 秒上报日常数据；
 * 4. 无人时每 10 秒同步一次状态；
 * 5. 每 10 秒上报睡眠数据。
 */
void mqttTask(void* parameter) {
    (void)parameter;
    Serial.println("[TuyaLink] MQTT task started");

    // 使用官方根证书验证服务端身份，不跳过 TLS 证书校验。
    tuyaTlsClient.setCACert(TUYA_ROOT_CA);
    tuyaTlsClient.setHandshakeTimeout(15);
    tuyaMqttClient.setServer(TUYA_MQTT_HOST, TUYA_MQTT_PORT);
    tuyaMqttClient.setCallback(mqttCallback);
    tuyaMqttClient.setBufferSize(1536);
    tuyaMqttClient.setKeepAlive(TUYA_KEEP_ALIVE_SECONDS);
    tuyaMqttClient.setSocketTimeout(10);

    while (true) {
        esp_task_wdt_reset();
        maintainConnection();

        if (tuyaMqttClient.connected()) {
            reportPirChangeIfNeeded();

            const unsigned long now = millis();
            const SensorData radar = sensorData;
            // 与原工程判断保持一致：无人或生命体征均为零时进入无人分支。
            const bool noPerson = !radar.presence ||
                                  (radar.heart_rate <= 0 && radar.breath_rate <= 0);

            if (noPerson) {
                if (now - lastHeartbeatReport >= HEARTBEAT_INTERVAL_MS) {
                    sendHeartbeatToMQTT();
                    lastHeartbeatReport = now;
                }
            } else if (now - lastDailyReport >= DAILY_DATA_INTERVAL_MS) {
                sendDailyDataToMQTT();
                lastDailyReport = now;
            }

            if (now - lastSleepReport >= SLEEP_DATA_INTERVAL_MS) {
                sendSleepDataToMQTT();
                lastSleepReport = now;
            }
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
