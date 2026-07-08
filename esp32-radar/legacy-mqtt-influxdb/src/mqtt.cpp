#include "mqtt.h"
#include "wifi_manager.h"
#include "radar_manager.h"
#include "tasks_manager.h"
#include "OTA_manager.h"
#include "version.h"
#include <HTTPClient.h>
#include <Update.h>// 包含HTTPClient和OTA更新库
#include <WiFiClientSecure.h>
#include <mbedtls/md5.h>// 包含MD5头文件

extern uint64_t device_sn;// 设备序列号
extern Preferences preferences;
extern WiFiManager wifiManager;// WiFi管理器
extern String getDeviceMacAddress();// 获取设备MAC地址

// 设备状态推送函数（b3 通道，带去重）
void pushDeviceStatusIfChanged(uint8_t status);

TaskHandle_t mqttTaskHandle = NULL;// MQTT任务句柄

const char* mqttServer = "8.138.160.177";// MQTT服务器地址
const int mqttPort = 1883;// MQTT端口号
const char* mqttDeviceModel = "radar_1.0";// 设备型号
const char* mqttProductKey = "dEkr5BkkXTFZFBdR";// 产品标识
const char* mqttProductSecret = "2e7957febfcb48b08a1c69b8deb56738";// 产品密钥

String deviceMacAddress = "";// 设备MAC地址

WiFiClient mqttWiFiClient;// WiFi客户端
PubSubClient mqttClient(mqttWiFiClient);// MQTT客户端

static uint32_t mqttMessageId = 1;// MQTT消息ID
static bool otaExecutionRequested = false;// OTA请求执行
static bool otaBootResultChecked = false;// OTA引导结果检查

unsigned long lastSleepDataTime = 0;// 最后一次发送睡眠数据时间
const unsigned long SLEEP_DATA_INTERVAL = 10000;// 睡眠数据间隔
unsigned long lastDailyDataTime = 0;// 最后一次发送每日数据时间
const unsigned long DAILY_DATA_INTERVAL = 2000;// 每日数据间隔
unsigned long lastHeartbeatTime = 0;// 最后一次发送心跳时间
const unsigned long HEARTBEAT_INTERVAL = 10000;  // 10秒心跳间隔

/**
 * @brief 获取MQTT设备名称
 * 优先使用设备序列号，若序列号为空则使用设备MAC地址
 *
 * @return 设备名称字符串
 * @example "radar_2024_12345678"
 */
String getMqttDeviceName() {
    if (device_sn != 0) {
        return String((unsigned long long)device_sn);
    }

    String fallback = getDeviceMacAddress();
    fallback.replace(":", "");
    return fallback;
}

/**
 * @brief 获取MQTT客户端ID
 * 按照enjoy-iot规范构建客户端ID
 *
 * 格式：{productKey}_{deviceName}_{model}
 *
 * @return 客户端ID字符串
 * @example "radar_2024_12345678_v1"
 */
String getMqttClientId() {
    return String(mqttProductKey) + "_" + getMqttDeviceName() + "_" + String(mqttDeviceModel);
}

/**
 * @brief 获取MQTT下行订阅主题
 * 用于订阅平台下发的所有指令
 * 格式：/sys/{productKey}/{deviceName}/c/#
 * - /sys/ 系统主题前缀
 * - {productKey} 产品标识
 * - {deviceName} 设备名称
 * - /c/ 下行指令标识 (command)
 * - # 通配符，匹配所有子主题
 *
 * @return 订阅主题字符串
 * @example "/sys/radar_2024/12345678/c/#"
 */
String getMqttSubscribeTopic() {
    return String("/sys/") + mqttProductKey + "/" + getMqttDeviceName() + "/c/#";
}

/**
 * @brief 获取MQTT属性上报主题
 * 用于设备向平台上报属性数据
 * 格式：/sys/{productKey}/{deviceName}/s/event/property/post
 * - /sys/ 系统主题前缀
 * - {productKey} 产品标识
 * - {deviceName} 设备名称
 * - /s/ 上行事件标识 (status)
 * - event/property/post 属性上报事件
 *
 * @return 属性上报主题字符串
 * @example "/sys/radar_2024/12345678/s/event/property/post"
 */
String getMqttPropertyPostTopic() {
    return String("/sys/") + mqttProductKey + "/" + getMqttDeviceName() + "/s/event/property/post";
}

/**
 * @brief 获取MQTTOTA升级主题
 * 用于订阅平台下发的OTA升级指令
 * 格式：/ota/device/upgrade/{productKey}/{deviceName}
 * - /ota/device/upgrade/ OTA升级指令前缀
 * - {productKey} 产品标识
 * - {deviceName} 设备名称
 *
 * @return OTA升级主题字符串
 * @example "/ota/device/upgrade/radar_2024/12345678"
 */
String getOtaUpgradeTopic() {
    return String("/ota/device/upgrade/") + mqttProductKey + "/" + getMqttDeviceName();
}

/**
 * @brief 获取OTA升级进度上报主题
 * 用于设备向平台上报OTA升级进度
 * 格式：/ota/device/progress/{productKey}/{deviceName}
 * - /ota/device/progress/ OTA升级进度前缀
 * - {productKey} 产品标识
 * - {deviceName} 设备名称
 *
 * @return OTA升级进度主题字符串
 * @example "/ota/device/progress/radar_2024/12345678"
 */
String getOtaProgressTopic() {
    return String("/ota/device/progress/") + mqttProductKey + "/" + getMqttDeviceName();
}

/**
 * @brief 获取OTA升级结果上报主题
 * 用于设备向平台上报OTA升级结果
 * 格式：/ota/device/inform/{productKey}/{deviceName}
 * - /ota/device/inform/ OTA升级结果前缀
 * - {productKey} 产品标识
 * - {deviceName} 设备名称
 *
 * @return OTA升级结果主题字符串
 * @example "/ota/device/inform/radar_2024/12345678"
 */
String getOtaVersionReportTopic() {
    return String("/ota/device/inform/") + mqttProductKey + "/" + getMqttDeviceName();
}

String getOtaResultInformTopic() {
    return String("/ota/device/inform/") + mqttProductKey + "/" + getMqttDeviceName();
}

/**
 * @brief 生成下一个MQTT消息ID
 * MQTT消息ID是一个16位的整数，用于标识MQTT消息的唯一性
 * 每次调用该函数都会返回一个新的消息ID，范围从1到65535，超过范围后会重新从1开始
 *
 * @return 下一个MQTT消息ID字符串
 * @example "12345"
 */
static String nextMqttMessageId() {
    return String(mqttMessageId++);
}

/**
 * @brief 保存待处理OTA升级结果
 * 用于记录当前OTA升级任务的状态和结果
 * @param task OTA升级任务对象
 */
static void savePendingOtaResult(const OtaUpgradeTask& task) {
    Preferences otaPrefs;// 创建Preferences对象用于存储OTA状态
    otaPrefs.begin("ota_state", false);// 打开命名空间
    otaPrefs.putBool("pending", true);// 标记有待处理的OTA结果
    otaPrefs.putString("requestId", task.id);// 保存OTA请求ID
    otaPrefs.putString("version", task.version);// 保存OTA版本信息
    otaPrefs.putString("module", task.module);// 保存OTA模块信息
    otaPrefs.end();
}

/**
 * @brief 清除待处理OTA升级结果
 * 用于在OTA升级完成后清除记录
 */
static void clearPendingOtaResult() {
    Preferences otaPrefs;// 创建Preferences对象用于存储OTA状态
    otaPrefs.begin("ota_state", false);// 打开命名空间
    otaPrefs.clear();
    otaPrefs.end();
}

/**
 * @brief 检查并报告待处理OTA升级结果
 * 用于在OTA升级完成后检查并报告结果
 */
static void checkAndReportPendingOtaResult() {
    if (otaBootResultChecked || !mqttClient.connected()) {
        return;
    }

    otaBootResultChecked = true;// 标记已检查过OTA引导结果，避免重复检查

    Preferences otaPrefs;// 创建Preferences对象用于存储OTA状态
    otaPrefs.begin("ota_state", true);// 打开命名空间，使用只读模式
    bool pending = otaPrefs.getBool("pending", false);// 检查是否有待处理的OTA结果
    String requestId = otaPrefs.getString("requestId", "");
    String version = otaPrefs.getString("version", "");
    String module = otaPrefs.getString("module", OTA_MODULE_NAME);
    otaPrefs.end();

    if (!pending) {
        return;
    }

    if (version == APP_VERSION) {
        publishOtaResultInform(APP_VERSION, module.c_str());// 上报OTA升级结果
        publishOtaProgress(requestId.c_str(), 100, "OTA upgrade successful", module.c_str());// 上报OTA升级进度
    } else {
        publishOtaProgress(requestId.c_str(), -4, "OTA result version mismatch after reboot", module.c_str());// 上报OTA升级进度
    }

    clearPendingOtaResult();// 清除待处理OTA结果，避免重复上报
}

/**
 * @brief 执行HTTPS OTA升级任务
 * 用于在设备连接到WiFi网络时执行HTTPS OTA升级任务
 * @return 是否成功执行任务
 */
static bool executeHttpsOtaTask() {
    if (!hasExecutableOtaTask()) {
        return false;
    }

    const OtaUpgradeTask& task = getCurrentOtaTask();// 获取当前OTA任务对象

    if (!WiFi.isConnected()) {
        markOtaState(OTA_FAILED);
        publishOtaProgress(task.id.c_str(), -2, "WiFi is disconnected", task.module.c_str());// 上报OTA升级进度，指示WiFi未连接导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    if (!task.url.startsWith("https://")) {// 目前仅支持HTTPS OTA升级，其他协议不受支持
        markOtaState(OTA_UNSUPPORTED_PROTOCOL);
        publishOtaProgress(task.id.c_str(), -2, "Only HTTPS OTA url is supported", task.module.c_str());// 上报OTA升级进度，指示不支持的协议导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    WiFiClientSecure client;// 创建安全WiFi客户端用于HTTPS连接
    client.setInsecure();// 设置客户端为不验证服务器证书，适用于测试环境，生产环境建议使用setCACert等方法验证服务器证书

    HTTPClient https;
    if (!https.begin(client, task.url)) {
        markOtaState(OTA_FAILED);
        publishOtaProgress(task.id.c_str(), -2, "Failed to open OTA url", task.module.c_str());// 上报OTA升级进度，指示打开OTA URL失败导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    publishOtaProgress(task.id.c_str(), 5, "Starting HTTPS OTA download", task.module.c_str());// 上报OTA升级进度，指示开始下载OTA包

    int httpCode = https.GET();// 发送HTTP GET请求下载OTA包
    if (httpCode != HTTP_CODE_OK) {
        https.end();// 结束HTTP连接
        markOtaState(OTA_FAILED);
        publishOtaProgress(task.id.c_str(), -2, "HTTPS download request failed", task.module.c_str());// 上报OTA升级进度，指示HTTPS下载请求失败导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    int contentLength = https.getSize();// 获取OTA包的内容长度，单位为字节，如果服务器没有返回Content-Length头部则默认为-1，表示未知长度
    if (contentLength > 0 && task.size > 0 && static_cast<uint32_t>(contentLength) != task.size) {
        https.end();
        markOtaState(OTA_FAILED);
        publishOtaProgress(task.id.c_str(), -3, "Firmware size mismatch", task.module.c_str());// 上报OTA升级进度，指示固件大小不匹配导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    if (!Update.begin(task.size)) {// 初始化OTA更新，预分配足够的空间用于存储OTA包，如果初始化失败则无法继续OTA流程
        https.end();
        markOtaState(OTA_FAILED);// 标记OTA状态为失败
        publishOtaProgress(task.id.c_str(), -4, "Update.begin failed", task.module.c_str());// 上报OTA升级进度，指示Update.begin失败导致OTA失败
        otaExecutionRequested = false;
        return false;
    }

    if (!task.md5.isEmpty()) {// 如果平台返回了OTA包的MD5校验值，则设置MD5校验，Update库会在写入过程中自动验证下载的OTA包的完整性和正确性
        Update.setMD5(task.md5.c_str());// 如果平台返回了OTA包的MD5校验值，则设置MD5校验，Update库会在写入过程中自动验证下载的OTA包的完整性和正确性
    }

    markOtaState(OTA_DOWNLOADING);// 标记OTA状态为正在下载

    WiFiClient* stream = https.getStreamPtr();// 获取HTTP响应的流对象，用于读取下载的OTA包数据
    uint8_t buffer[4096];// 创建一个缓冲区用于存储从HTTP流中读取的数据，大小为4KB，可以根据需要调整这个大小以平衡内存使用和下载效率
    uint32_t writtenTotal = 0;// 记录已写入OTA包的总字节数，用于计算下载进度和验证下载的OTA包的大小是否正确
    int lastReported = 5;// 记录上次上报的下载进度百分比，初始值为5，表示已经上报了开始下载的进度

    while (https.connected() && (contentLength > 0 ? writtenTotal < task.size : true)) {
        size_t available = stream->available();// 检查HTTP流中是否有可用的数据，如果没有数据可读则继续循环等待，保持MQTT连接活跃，避免在OTA下载过程中MQTT连接超时断开
        if (available == 0) {
            mqttClient.loop();// 处理MQTT消息，保持MQTT连接活跃，避免在OTA下载过程中MQTT连接超时断开
            delay(1);
            continue;
        }

        size_t toRead = available > sizeof(buffer) ? sizeof(buffer) : available;
        size_t bytesRead = stream->readBytes(buffer, toRead);
        if (bytesRead == 0) {
            continue;
        }

        size_t written = Update.write(buffer, bytesRead);// 将下载的数据写入OTA更新分区
        if (written != bytesRead) {
            Update.abort();// 如果写入失败，终止OTA更新流程，避免安装一个不完整或损坏的OTA包
            https.end();// 结束HTTP连接
            markOtaState(OTA_FAILED);
            publishOtaProgress(task.id.c_str(), -4, "Update.write failed", task.module.c_str());// 上报OTA升级进度，指示Update.write失败导致OTA失败
            otaExecutionRequested = false;
            return false;
        }

        writtenTotal += written;// 更新已写入的总字节数
        esp_task_wdt_reset();
        mqttClient.loop();// 处理MQTT消息，保持MQTT连接活跃，避免在OTA下载过程中MQTT连接超时断开

        if (task.size > 0) {
            int progress = static_cast<int>((writtenTotal * 90UL) / task.size);
            progress = progress < 5 ? 5 : progress;
            progress = progress > 95 ? 95 : progress;
            if (progress >= lastReported + 5) {// 每当下载进度增加5%时上报一次OTA升级进度，避免过于频繁的上报导致MQTT消息过多，同时也能及时反映下载的状态
                lastReported = progress;// 更新上次上报的进度百分比
                publishOtaProgress(task.id.c_str(), progress, "Downloading OTA package", task.module.c_str());// 上报OTA升级进度，指示正在下载OTA包，并包含当前的下载进度百分比和模块信息
            }
        }

        if (contentLength <= 0 && !https.connected() && stream->available() == 0) {
            break;
        }
    }

    https.end();// 结束HTTP连接
    markOtaState(OTA_VERIFYING);// 标记OTA状态为正在验证

    if (task.size > 0 && writtenTotal != task.size) {
        Update.abort();// 如果下载完成后写入的总字节数与平台返回的OTA包大小不匹配，说明下载的OTA包不完整或损坏，终止OTA更新流程，避免安装一个不完整或损坏的OTA包
        markOtaState(OTA_FAILED);
        publishOtaProgress(task.id.c_str(), -3, "Firmware bytes received mismatch", task.module.c_str());
        otaExecutionRequested = false;
        return false;
    }

    markOtaState(OTA_WRITING);
    if (!Update.end()) {
        markOtaState(OTA_FAILED);// 如果结束OTA更新失败，说明写入过程中发生了错误，终止OTA更新流程，避免安装一个不完整或损坏的OTA包
        publishOtaProgress(task.id.c_str(), -4, "Update.end failed", task.module.c_str());
        otaExecutionRequested = false;
        return false;
    }

    if (!Update.isFinished()) {
        markOtaState(OTA_FAILED);// 如果OTA更新没有成功完成，说明写入的OTA包不完整或损坏，终止OTA更新流程，避免安装一个不完整或损坏的OTA包
        publishOtaProgress(task.id.c_str(), -4, "OTA image is incomplete", task.module.c_str());
        otaExecutionRequested = false;
        return false;
    }

    savePendingOtaResult(task);// 保存待处理的OTA结果，记录当前OTA任务的状态和结果，以便在设备重启后检查并上报OTA升级结果
    markOtaState(OTA_PENDING_REBOOT);
    publishOtaProgress(task.id.c_str(), 95, "OTA written successfully, rebooting", task.module.c_str());// 上报OTA升级进度，指示OTA包写入成功，即将重启设备，并包含当前的进度百分比和模块信息
    otaExecutionRequested = false;
    delay(500);
    ESP.restart();// 重启设备以应用新的固件
    return true;
}

/**
 * @brief 发布OTA版本信息
 * 向平台上报当前固件版本号
 * @return true 发布成功，false 发布失败
 */
bool publishOtaVersionReport() {
    checkMQTTStatus();

    if (!mqttClient.connected()) {
        Serial.println("[MQTT] 未连接，跳过OTA版本上报");
        return false;
    }

    JsonDocument doc;
    doc["id"] = nextMqttMessageId();
    
    JsonObject params = doc["params"].to<JsonObject>();
    params["version"] = APP_VERSION;
    params["module"] = OTA_MODULE_NAME;

    String jsonStr;
    serializeJson(doc, jsonStr);

    String topic = getOtaVersionReportTopic();
    bool result = mqttClient.publish(topic.c_str(), jsonStr.c_str());

    if (result) {
        Serial.println("[MQTT] OTA版本信息上报成功");
    } else {
        Serial.println("[MQTT] OTA版本信息上报失败");
    }

    return result;
}

/**
 * @brief 发布OTA结果信息
 * 向平台上报OTA升级结果或失败状态
 * @param version 版本号
 * @param module 模块名称
 * @return true 发布成功，false 发布失败
 */
bool publishOtaResultInform(const char* version, const char* module) {
    checkMQTTStatus();

    if (!mqttClient.connected()) {
        Serial.println("[MQTT] 未连接，跳过OTA结果上报");
        return false;
    }

    JsonDocument doc;
    doc["id"] = nextMqttMessageId();

    JsonObject params = doc["params"].to<JsonObject>();
    params["version"] = version ? version : APP_VERSION;
    params["module"] = module ? module : OTA_MODULE_NAME;

    String jsonStr;
    serializeJson(doc, jsonStr);

    String topic = getOtaResultInformTopic();
    bool result = mqttClient.publish(topic.c_str(), jsonStr.c_str());// 发布MQTT消息，主题为OTA结果主题，内容为序列化后的JSON字符串

    if (result) {
        Serial.println("[MQTT] OTA结果信息上报成功");
    } else {
        Serial.println("[MQTT] OTA结果信息上报失败");
    }

    return result;
}

/**
 * @brief 发布OTA进度信息
 * 向平台上报OTA升级进度或失败状态
 * @param requestId 请求ID
 * @param step 进度步骤，-4~-1表示失败，0表示成功
 * @param desc 描述信息
 * @param module 模块名称
 * @return true 发布成功，false 发布失败
 */
bool publishOtaProgress(const char* requestId, int step, const char* desc, const char* module) {
    checkMQTTStatus();// 确保MQTT客户端已初始化

    if (!mqttClient.connected()) {
        Serial.println("[MQTT] 未连接，跳过OTA进度上报");
        return false;
    }

    JsonDocument doc;
    doc["id"] = requestId;
    
    JsonObject params = doc["params"].to<JsonObject>();// 提取params对象，包含OTA升级进度的具体信息，强转为JsonObject类型，方便后续访问各个字段
    params["step"] = String(step);
    params["desc"] = desc;// 进度描述信息，通常由平台返回，描述当前OTA升级的状态或错误信息
    if (module != nullptr && strlen(module) > 0) {// 模块信息是可选的，如果提供了模块信息则上报，否则不包含模块字段
        params["module"] = module;
    }

    String jsonStr;
    serializeJson(doc, jsonStr);// 将JSON文档序列化为字符串，准备发送给MQTT服务器

    String topic = getOtaProgressTopic();
    bool result = mqttClient.publish(topic.c_str(), jsonStr.c_str());// 发布MQTT消息，主题为OTA进度主题，内容为序列化后的JSON字符串

    if (result) {
        Serial.printf("[MQTT] OTA进度上报成功: step=%d, desc=%s\n", step, desc);
    } else {
        Serial.println("[MQTT] OTA进度上报失败");
    }

    return result;
}

/**
 * @brief 处理OTA升级消息
 * 解析并处理平台下发的OTA升级指令
 *接收 MQTT 的 OTA 触发消息并统筹解析校验流程，包括消息格式校验、OTA任务合法性验证、状态更新和进度上报等
 * @param topic MQTT主题
 * @param payload 消息内容
 * @return true 处理成功，false 处理失败
 */
bool handleOtaUpgradeMessage(const char* topic, const String& payload) {
    (void)topic;// 目前topic不区分不同OTA模块，后续如果有多个模块需要区分时可以根据topic解析出模块信息
    Serial.println("[MQTT] 收到OTA升级消息");

    markOtaState(OTA_NOTIFIED);// 标记已收到OTA升级通知

    OtaUpgradeTask task; // 创建OTA升级任务对象
    String errorMsg;  // 错误消息字符串
    int errorStep = -1;

    if (!parseOtaUpgradeMessage(payload, task, errorMsg)) {// 解析OTA升级消息失败，可能是格式错误或缺少必要字段
        markOtaState(OTA_REJECTED);
        publishOtaProgress("", -1, errorMsg.c_str(), OTA_MODULE_NAME);
        return false;
    }

    storeOtaTask(task);// 存储OTA任务信息，供后续执行时使用
    markOtaState(OTA_VALIDATING);// 标记正在验证OTA任务合法性

    if (!validateOtaUpgradeTask(task, errorMsg, errorStep)) {// 验证OTA升级任务合法性失败，可能是URL不合法、版本不兼容等问题
        if (errorStep == -2) {
            markOtaState(OTA_UNSUPPORTED_PROTOCOL);// 不支持的协议，例如非HTTPS URL
        } else {
            markOtaState(OTA_REJECTED);// 其他验证失败都标记为拒绝
        }
        publishOtaProgress(task.id.c_str(), errorStep, errorMsg.c_str(), task.module.c_str());// 上报OTA升级进度，包含错误信息
        return false;
    }

    // 到这里说任务是合法的
    markOtaState(OTA_READY);// 标记OTA任务准备就绪
    otaExecutionRequested = true;
    publishOtaProgress(task.id.c_str(), 1, "OTA task accepted, waiting for HTTPS download", task.module.c_str());// 上报OTA升级进度，表示任务已接受，等待执行

    return true;
}

/**
 * @brief 计算MQTT连接密码
 * 按照enjoy-iot规范，使用MD5计算密码
 *
 * 公式：password = MD5(mqttProductSecret + clientId)
 *
 * @param clientId 客户端ID
 * @return 32位小写十六进制MD5字符
 * @example MD5("abc123" + "radar_2024_12345678_v1") -> "a1b2c3d4e5f6..."
 */
String makeMqttPassword(const String& clientId) {
    String raw = String(mqttProductSecret) + clientId;

    unsigned char digest[16];
    mbedtls_md5_context ctx;
    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts_ret(&ctx);
    mbedtls_md5_update_ret(&ctx, (const unsigned char*)raw.c_str(), raw.length());
    mbedtls_md5_finish_ret(&ctx, digest);
    mbedtls_md5_free(&ctx);

    char md5str[33];
    for (int i = 0; i < 16; i++) {
        sprintf(&md5str[i * 2], "%02x", digest[i]);
    }
    md5str[32] = '\0';
    return String(md5str);
}

/**
 * @brief 将情绪字段追加到 JSON 文档
 * 有结果且未过期（5秒内）才追加，否则静默跳过
 */
static void appendEmotionFields(JsonDocument& doc) {
    if (!g_hasEmotionResult || !g_lastEmotionResult.isValid) {
        return;
    }
    // 超过 5 秒没更新，认为情绪结果过期
    if (millis() - g_lastEmotionUpdateMs > 5000) {
        return;
    }
    // 主次情绪（枚举值，前端用 EMOTION_NAMES 映射）
    doc["primaryEmotion"]    = static_cast<int>(g_lastEmotionResult.primaryEmotion);
    doc["secondaryEmotion"]  = static_cast<int>(g_lastEmotionResult.secondaryEmotion);
    // 置信度和强度 0-1
    doc["emotionConfidence"] = g_lastEmotionResult.confidence;
    doc["emotionIntensity"]  = g_lastEmotionResult.intensity;
    // 情绪维度：效价 -1~+1，唤醒度 0-1
    doc["emotionValence"]    = g_lastEmotionResult.valence;
    doc["emotionArousal"]    = g_lastEmotionResult.arousal;
    // 压力评估 0-100
    doc["stressLevel"]       = g_lastEmotionResult.stressLevel;
    doc["anxietyLevel"]      = g_lastEmotionResult.anxietyLevel;
    doc["relaxationLevel"]   = g_lastEmotionResult.relaxationLevel;
}

/**
 * @brief 统一属性上报函数
 * 封装enjoy-iot规范的属性上报格式，被sendDailyDataToMQTT和sendSleepDataToMQTT调用
 *
 * 载荷格式： * {
 *   "id": "消息ID",
 *   "method": "thing.event.property.post",
 *   "params": {
 *     "deviceId": "设备ID",
 *     "reportType": "daily/sleep",
 *     ...业务字段...
 *   }
 * }
 *
 * @param params 业务参数JSON对象（调用者填充业务字段）
 * @param reportType 上报类型 ("daily" 或 "sleep")
 * @return true 发布成功，false 发布失败
 */
static bool publishPropertyReport(JsonDocument& params, const char* reportType) {
    JsonDocument payloadDoc;

    payloadDoc["id"] = nextMqttMessageId();

    payloadDoc["method"] = "thing.event.property.post";

    params["deviceId"] = getMqttDeviceName();
    params["reportType"] = reportType;

    payloadDoc["params"] = params;

    String topic = getMqttPropertyPostTopic();

    String payload;
    serializeJson(payloadDoc, payload);

    return mqttClient.publish(topic.c_str(), payload.c_str());
}

/**
 * @brief 构建回复主题
 * 将下行请求主题转换为上行回复主题
 *
 * 转换规则： * - /c/ 替换为 /s/
 * - 末尾添加 _reply
 *
 * @param requestTopic 请求主题
 * @return 回复主题字符 * @example "/sys/.../c/service/property/set" -> "/sys/.../s/service/property/set_reply"
 */
String buildReplyTopic(const char* requestTopic) {
    String topic = String(requestTopic);
    topic.replace("/c/", "/s/");
    topic += "_reply";
    return topic;
}

/**
 * @brief 发送MQTT回复
 * 统一处理平台下发指令的回复 *
 * 回复格式： * {
 *   "id": "原请求ID",
 *   "method": "原method_reply",
 *   "code": 0,  // 0=成功，其他值=失败
 *   "data": { ... }
 * }
 *
 * @param requestTopic 请求主题
 * @param requestId 请求ID
 * @param requestMethod 请求方法
 * @param code 状态码 (0=成功，其他值=失败)
 * @param data 回复数据
 * @return true 发送成功，false 发送失败
 */
bool publishMqttReply(const char* requestTopic,
                      const char* requestId,
                      const char* requestMethod,
                      int code,
                      JsonVariant data) {
    JsonDocument replyDoc;
    replyDoc["id"] = requestId ? requestId : "";
    replyDoc["method"] = String(requestMethod ? requestMethod : "") + "_reply";
    replyDoc["code"] = code;
    replyDoc["params"] = data;

    String replyTopic = buildReplyTopic(requestTopic);
    String payload;
    serializeJson(replyDoc, payload);

    Serial.printf("[MQTT] reply topic: %s\n", replyTopic.c_str());
    Serial.printf("[MQTT] reply payload: %s\n", payload.c_str());

    return mqttClient.publish(replyTopic.c_str(), payload.c_str());
}

/**
 * @brief MQTT消息回调函数
 * 处理平台下发的指令，支持属性设置、属性读取和自定义服务 *
 * 支持的method* - thing.service.property.set: 设置设备属性（如continuousSendEnabled、continuousSendInterval? * - thing.service.property.get: 读取设备属性（返回当前传感器数据和配置? * - thing.service.*: 自定义服务（预留扩展? *
 * @param topic 消息主题
 * @param payload 消息载荷
 * @param length 载荷长度
 */
void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.printf("[MQTT] 收到主题: %s\n", topic);
    Serial.printf("[MQTT] 收到内容: %s\n", message.c_str());

    // 先判断是否是OTA升级主题
    String topicStr = String(topic);
    String otaUpgradeTopic = getOtaUpgradeTopic();// 获取OTA升级主题字符串
    
    if (topicStr == otaUpgradeTopic) {
        Serial.println("[MQTT] 处理OTA升级消息");
        handleOtaUpgradeMessage(topic, message);// 处理OTA升级消息后直接返回，不再继续处理为属性设置或读取
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, message);
    if (err) {
        Serial.printf("[MQTT] JSON解析失败: %s\n", err.c_str());
        return;
    }

    const char* method = doc["method"] | "";
    const char* id = doc["id"] | "";
    JsonObject params = doc["params"].as<JsonObject>();

    if (strcmp(method, "thing.service.property.set") == 0) {
        bool ok = true;

        if (params["continuousSendEnabled"].is<bool>()) {
            continuousSendEnabled = params["continuousSendEnabled"].as<bool>();
        }

        if (params["continuousSendInterval"].is<unsigned long>()) {
            continuousSendInterval = params["continuousSendInterval"].as<unsigned long>();
        }

        JsonDocument replyData;
        replyData["success"] = ok;

        if (publishMqttReply(topic, id, method, ok ? 0 : -1, replyData.as<JsonVariant>())) {
            Serial.println("[MQTT] property.set reply 发送成功");
        } else {
            Serial.println("[MQTT] property.set reply 发送失败");
        }

    } else if (strcmp(method, "thing.service.property.get") == 0) {
        JsonDocument replyData;

        replyData["heartRate"] = sensorData.heart_rate;
        replyData["breathingRate"] = sensorData.breath_rate;
        replyData["personDetected"] = sensorData.presence;
        replyData["humanActivity"] = sensorData.motion;
        replyData["humanDistance"] = sensorData.distance;
        replyData["sleepState"] = sensorData.sleep_state;
        replyData["continuousSendEnabled"] = continuousSendEnabled;
        replyData["continuousSendInterval"] = continuousSendInterval;

        // 追加情绪字段
        appendEmotionFields(replyData);

        if (publishMqttReply(topic, id, method, 0, replyData.as<JsonVariant>())) {
            Serial.println("[MQTT] property.get reply 发送成功");
        } else {
            Serial.println("[MQTT] property.get reply 发送失败");
        }

    } else if (strncmp(method, "thing.service.", 14) == 0) {
        JsonDocument replyData;
        bool ok = true;

        replyData["success"] = ok;

        if (publishMqttReply(topic, id, method, ok ? 0 : -1, replyData.as<JsonVariant>())) {
            Serial.printf("[MQTT] service reply 发送成功 method=%s\n", method);
        } else {
            Serial.printf("[MQTT] service reply 发送失败 method=%s\n", method);
        }

    } else {
        Serial.printf("[MQTT] 暂不支持 method=%s\n", method);
    }
}

/**
 * @brief 初始化MQTT客户端
 * 配置MQTT服务器地址、端口、缓冲区大小和消息回调函数
 *
 * 初始化内容：
 * 1. 获取设备MAC地址
 * 2. 设置MQTT服务器地址和端口
 * 3. 设置消息缓冲区大小为1024字节
 * 4. 注册下行消息回调函数
 */
void initMQTT() {
    deviceMacAddress = getDeviceMacAddress();// 获取设备MAC地址，作为MQTT客户端ID的一部分
    mqttClient.setServer(mqttServer, mqttPort);// 设置MQTT服务器地址和端口，准备连接MQTT服务
    mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE);// 设置MQTT消息缓冲区大小，确保能够处理较大的消息载荷
    mqttClient.setCallback(mqttMessageCallback);// 注册MQTT消息回调函数，当收到MQTT消息时会调用该函数进行处理

    Serial.printf("[MQTT] broker: %s:%d\n", mqttServer, mqttPort);
    Serial.printf("[MQTT] clientId: %s\n", getMqttClientId().c_str());
    Serial.printf("[MQTT] username: %s\n", getMqttDeviceName().c_str());
}

/**
 * @brief 连接MQTT服务器
 * 使用enjoy-iot规范的身份认证方式连接，并订阅下行主题
 *
 * 注意：此函数同时承担首次连接和断线重连的职责
 * - 首次连接：checkMQTTStatus() 检测到未连接时调用
 * - 断线重连：checkMQTTStatus() 检测到断开时调用
 *
 * 连接流程： * 1. 检查WiFi是否已连接
 * 2. 计算clientId、username、password
 * 3. 连接MQTT服务
 * 4. 订阅下行主题 /sys/{productKey}/{deviceName}/c/#
 *
 * 密码计算：password = MD5(productSecret + clientId)
 */
void reconnectMQTT() {
    if (!WiFi.isConnected()) {
        return;
    }

    String clientId = getMqttClientId();
    String username = getMqttDeviceName();
    String password = makeMqttPassword(clientId);

    bool connected = mqttClient.connect(clientId.c_str(), username.c_str(), password.c_str());

    if (connected) {
        Serial.printf("[MQTT] 连接成功, clientId=%s\n", clientId.c_str());
        
        // 推送 MQTT 已连接状态（b3 通道）
        pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_CONNECTED);

        String subTopic = getMqttSubscribeTopic();
        mqttClient.subscribe(subTopic.c_str());
        Serial.printf("[MQTT] 已订�? %s\n", subTopic.c_str());

        // 订阅OTA升级主题
        String otaTopic = getOtaUpgradeTopic();
        mqttClient.subscribe(otaTopic.c_str());
        Serial.printf("[MQTT] 已订阅OTA主题: %s\n", otaTopic.c_str());

        // 上报当前版本信息
        publishOtaVersionReport();
    } else {
        Serial.printf("[MQTT] 连接失败, state=%d\n", mqttClient.state());
        // 推送 MQTT 连接失败状态（b3 通道）
        pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_FAILED);
    }
}

/**
 * @brief 检查MQTT连接状态
 * 如果未连接则尝试重连，并保持心跳
 *
 * 重连策略：* - 仅在WiFi已连接时尝试重连
 * - 5秒尝试一次重连，避免频繁重连
 * - 调用mqttClient.loop()保持心跳和处理消息
 */
void checkMQTTStatus() {
    static bool wasConnected = false; // 记录上一次的连接状态
    
    // 检测 MQTT 断开（不管 WiFi 状态）
    if (!mqttClient.connected()) {
        // 如果之前是连接状态，现在断开了，推送断开状态
        if (wasConnected) {
            pushDeviceStatusIfChanged(BleProto::DeviceStatus::DEV_MQTT_DISCONNECTED);
            wasConnected = false;
        }
        
        // 只有在 WiFi 已连接时才尝试重连
        if (WiFi.isConnected()) {
            static unsigned long lastReconnectAttempt = 0;
            unsigned long now = millis();
            if (now - lastReconnectAttempt > 5000) {
                lastReconnectAttempt = now;
                reconnectMQTT();// 尝试连接MQTT服务器，如果连接成功会订阅相关主题并上报版本信息，如果失败会在下一次检查时再次尝试连接
            }
        }
    }
    
    // 更新连接状态记录
    if (mqttClient.connected()) {
        wasConnected = true;
    }
    
    mqttClient.loop();// 处理MQTT消息，保持连接活跃，确保能够及时接收平台下发的指令和OTA升级消息
}

/**
    * @brief 发送日常数据到MQTT
    * 上报当前传感器数据和状态 * 上报字段： * - heartRate: 心率
 * - breathingRate: 呼吸率
 * - personDetected: 是否检测到人
 * - humanActivity: 人体活动状态
 * - humanDistance: 人体距离
 * - sleepState: 睡眠状态
 * - humanPositionX/Y/Z: 人体位置坐标
 * - heartbeatWaveform: 心跳波形数据（示例中仅上报第
 */
void sendDailyDataToMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    checkMQTTStatus();

    if (!mqttClient.connected()) {
        Serial.println("[MQTT] 未连接，跳过发送日常数据");
        return;
    }

    JsonDocument doc;

    if (sensorData.heart_rate > 0) {
        doc["heartRate"] = sensorData.heart_rate;
    }

    if (sensorData.breath_rate > 0) {
        doc["breathingRate"] = sensorData.breath_rate;
    }

    doc["personDetected"] = sensorData.presence;
    doc["humanActivity"] = sensorData.motion;

    if (sensorData.distance > 0) {
        doc["humanDistance"] = sensorData.distance;
    }

    doc["sleepState"] = sensorData.sleep_state;
    doc["humanPositionX"] = sensorData.pos_x;
    doc["humanPositionY"] = sensorData.pos_y;
    doc["humanPositionZ"] = sensorData.pos_z;
    doc["heartbeatWaveform"] = (int)sensorData.heart_waveform[0];
    doc["breathingWaveform"] = (int)sensorData.breath_waveform[0];
    doc["abnormalState"] = sensorData.abnormal_state;
    doc["bedStatus"] = sensorData.bed_status;
    doc["struggleAlert"] = sensorData.struggle_alert;
    doc["noOneAlert"] = sensorData.no_one_alert;
    
    // Add WiFi IP address
    doc["wifiIP"] = WiFi.localIP().toString();

    // 追加情绪字段（有有效结果且未过期时才追加）
    appendEmotionFields(doc);

    if (publishPropertyReport(doc, "daily")) {
        Serial.println("[MQTT] daily data report published");
    } else {
        Serial.printf("[MQTT] daily data report failed, state=%d\n", mqttClient.state());
    }
}

/**
 * @brief 发送睡眠数据到MQTT
 * 上报睡眠统计数据和质量评价 *
 * 上报字段： * - sleepQualityScore: 睡眠质量评分
 * - sleepQualityGrade: 睡眠质量等级
 * - totalSleepDuration: 总睡眠时长
 * - awakeDurationRatio: 清醒时长比例
 * - lightSleepRatio: 浅睡比例
 * - deepSleepRatio: 深睡比例
 * - outOfBedDuration: 离床时长
 * - outOfBedCount: 离床次数
 * - turnCount: 翻身次数
 * - avgBreathingRate: 平均呼吸率
 * - avgHeartRate: 平均心率
 * - apneaCount: 呼吸暂停次数
 * - awakeDuration: 清醒时长
 * - lightSleepDuration: 浅睡时长
 * - deepSleepDuration: 深睡时长
 *
 * 触发条件： * - mqttTask中每10秒调用一次
 * - 仅在sleep_state为0(深睡)或1(浅睡)时上报 */
void sendSleepDataToMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[MQTT] WiFi未连接，跳过发送睡眠数据");
        return;
    }

    checkMQTTStatus();

    if (!mqttClient.connected()) {
        Serial.println("[MQTT] MQTT未连接，跳过发送睡眠数据");
        return;
    }

    if (sensorData.sleep_state != 0 && sensorData.sleep_state != 1) {
        Serial.printf("[MQTT] 当前不是睡眠状态，sleep_state=%d\n", sensorData.sleep_state);
        return;
    }

    JsonDocument doc;
    doc["sleepQualityScore"] = sensorData.sleep_score;
    doc["sleepQualityGrade"] = sensorData.sleep_grade;
    doc["totalSleepDuration"] = sensorData.sleep_total_time;
    doc["awakeDurationRatio"] = sensorData.awake_ratio;
    doc["lightSleepRatio"] = sensorData.light_sleep_ratio;
    doc["deepSleepRatio"] = sensorData.deep_sleep_ratio;
    doc["outOfBedDuration"] = sensorData.bed_Out_Time;
    doc["outOfBedCount"] = sensorData.turn_count;
    doc["turnCount"] = sensorData.turnover_count;
    doc["avgBreathingRate"] = sensorData.avg_breath_rate;
    doc["avgHeartRate"] = sensorData.avg_heart_rate;
    doc["apneaCount"] = sensorData.apnea_count;
    doc["abnormalState"] = sensorData.abnormal_state;
    doc["bodyMovement"] = sensorData.body_movement;
    doc["breathStatus"] = sensorData.breath_status;
    doc["sleepState"] = sensorData.sleep_state;
    doc["largeMoveRatio"] = sensorData.large_move_ratio;
    doc["smallMoveRatio"] = sensorData.small_move_ratio;
    doc["struggleAlert"] = sensorData.struggle_alert;
    doc["noOneAlert"] = sensorData.no_one_alert;
    doc["awakeDuration"] = sensorData.awake_time;
    doc["lightSleepDuration"] = sensorData.light_sleep_time;
    doc["deepSleepDuration"] = sensorData.deep_sleep_time;

    // 追加情绪字段（睡眠期间的情绪状态）
    appendEmotionFields(doc);

    if (publishPropertyReport(doc, "sleep")) {
        Serial.println("[MQTT] 睡眠数据上报成功");
    } else {
        Serial.printf("[MQTT] 睡眠数据上报失败, state=%d\n", mqttClient.state());
    }
}

/**
 * @brief 发送心跳包到MQTT
 * 当无人时发送轻量心跳，防止平台判定设备离线
 * 
 * 上报字段： * - personDetected: 0 (无人)
 * - heartbeat: 1 (心跳标识)
 * - timestamp: 时间戳
 * - sleepState: 睡眠状态
 * - noOneAlert: 无人警报
 * 
 * 触发条件： * - mqttTask中心率和呼吸率都为空时，10秒发送一次 */

void sendHeartbeatToMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    checkMQTTStatus();

    if (!mqttClient.connected()) {
        return;
    }

    JsonDocument doc;
    doc["personDetected"] = 0;
    doc["heartbeat"] = 1;
    doc["timestamp"] = millis();
    doc["sleepState"] = sensorData.sleep_state;
    doc["noOneAlert"] = sensorData.no_one_alert;
    doc["wifiIP"] = WiFi.localIP().toString();

    // 追加情绪字段（无人时 g_hasEmotionResult 为 false，不会实际发出，保持接口一致）
    appendEmotionFields(doc);

    if (publishPropertyReport(doc, "heartbeat")) {
        Serial.println("[MQTT] heartbeat report published");
    } else {
        Serial.printf("[MQTT] heartbeat report failed, state=%d\n", mqttClient.state());
    }
}

/**
 * @brief MQTT任务
 * 在FreeRTOS任务中处理所有MQTT功能
 * - 保持MQTT连接心跳
 * - 处理MQTT消息回调
 * - 定时发送日常数据和睡眠数据
 * @param parameter 任务参数（未使用） */

void mqttTask(void *parameter) {
    Serial.println("📡 MQTT任务启动");

    initMQTT();// 初始化MQTT客户端配置

    while (1) {
        esp_task_wdt_reset();

        if (WiFi.status() == WL_CONNECTED) {
            checkMQTTStatus();// 检查MQTT连接状态，必要时重连，并处理MQTT消息回调

            if (mqttClient.connected()) {
                checkAndReportPendingOtaResult();// 检查是否有待上报的OTA结果，如果有则上报给平台

                if (otaExecutionRequested && hasExecutableOtaTask()) {// 如果收到OTA升级指令且有合法的OTA任务准备就绪，则执行OTA升级任务
                    executeHttpsOtaTask();// 执行OTA升级任务，连接MQTT服务器下载固件
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    continue;
                }

                unsigned long currentTime = millis();

                // 结合存在检测和生命体征检测结果
                bool isNoOne = !sensorData.presence || (sensorData.heart_rate == 0 && sensorData.breath_rate == 0);

                if (isNoOne) {
                    // 无人时发送心跳包，防止平台判定离线
                    if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
                        Serial.println("🔄 [MQTT] 检测到无人状态，发送心跳包");
                        sendHeartbeatToMQTT();
                        lastHeartbeatTime = currentTime;
                    }
                } else if (currentTime - lastDailyDataTime >= DAILY_DATA_INTERVAL) {
                    // 有人时发送日常数据
                    Serial.println("📊 [MQTT] 检测到有人状态，发送日常数据");
                    sendDailyDataToMQTT();
                    lastDailyDataTime = currentTime;
                }

                esp_task_wdt_reset();

                if (currentTime - lastSleepDataTime >= SLEEP_DATA_INTERVAL) {
                    sendSleepDataToMQTT();
                    lastSleepDataTime = currentTime;
                }
            } else {
                static unsigned long lastWifiCheck = 0;
                if (millis() - lastWifiCheck > 10000) {
                    lastWifiCheck = millis();
                }
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
