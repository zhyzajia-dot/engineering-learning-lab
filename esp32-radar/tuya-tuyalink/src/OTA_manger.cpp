/**
 * @file OTA_manger.cpp
 * @brief OTA 升级任务的状态保存、校验和生命周期管理实现。
 *
 * 本文件只管理升级任务状态，固件下载和写入流程由 MQTT 模块执行。
 */
#include "OTA_manager.h"
#include "version.h"

static OtaState currentOtaState = OTA_IDLE;// 当前OTA状态
static OtaUpgradeTask currentOtaTask;// 当前OTA升级任务

/**
 * @brief 检查OTA状态码是否被接受
 * @param codeValue OTA状态码
 * @return 是否被接受
 */
static bool isAcceptedOtaCode(const JsonVariantConst& codeValue) {
    if (codeValue.is<int>()) {// 有些平台返回的code是整数，有些是字符串，这里兼容两种情况
        int code = codeValue.as<int>();//它需要通过 as<int>() 进行类型转换/提取，而不是直接使用 as<int>() 的结果进行比较，因为 as<int>() 返回的是一个临时对象，而不是一个直接的整数值。通过将其赋值给一个变量，我们可以确保在比较时使用的是一个稳定的整数值，而不是一个临时对象。这也是C++中常见的类型转换和比较的正确方式。
        return code == 200 || code == 1000;
    }

    String code = codeValue.as<String>();
    return code == "200" || code == "1000";// 200表示有新版本，1000表示当前版本已是最新，无需升级
}

/**
 * @brief 保存待处理的OTA结果
 * @param task OTA升级任务对象
 */
void initOtaManager() {
    resetOtaTask();
    Serial.println("[OTA] manager initialized");
}

/**
 * @brief 重置OTA任务状态
 */
void resetOtaTask() {
    currentOtaState = OTA_IDLE;
    currentOtaTask = OtaUpgradeTask{};// 重置为默认构造的空任务对象
}

/**
 * @brief 检查是否有待处理的OTA任务
 * @return 是否有待处理的OTA任务
 */
bool hasPendingOtaTask() {
    return !currentOtaTask.id.isEmpty();// 通过检查当前OTA任务的ID是否为空来判断是否有待处理的OTA任务
}

/**
 * @brief 获取当前OTA状态
 * @return 当前OTA状态
 */
OtaState getOtaState() {
    return currentOtaState;
}

/**
 * @brief 获取当前OTA任务对象
 * @return 当前OTA任务对象的常量引用
 */
const OtaUpgradeTask& getCurrentOtaTask() {
    return currentOtaTask;
}

/**
 * @brief 检查是否有可执行的OTA任务
 * @return 是否有可执行的OTA任务
 */
bool hasExecutableOtaTask() {
    return currentOtaState == OTA_READY;// 只有当OTA状态为READY时才表示有可执行的OTA任务，其他状态如IDLE、RECEIVED、VALIDATED等都不表示可执行的状态
}

/**
 * @brief 标记OTA状态
 * @param state 要标记的OTA状态
 */
void markOtaState(OtaState state) {
    currentOtaState = state;
}

/**
 * @brief 存储OTA任务
 * @param task OTA升级任务对象
 */
void storeOtaTask(const OtaUpgradeTask& task) {
    currentOtaTask = task;// 存储OTA任务信息，供后续执行时使用
    currentOtaTask.receivedAt = millis();
}

/**
 * @brief 保存待处理的OTA结果
 * 用于在OTA升级完成后保存结果，等待设备重启后上报
 * 将收到的 JSON 字符串反序列化，提取出固件版本、下载 URL、文件大小、MD5 校验和等关键信息，并存入 OtaUpgradeTask 结构体中，以供后续验证和执行 OTA 升级使用
 * @param task OTA升级任务对象
 */
bool parseOtaUpgradeMessage(const String& payload, OtaUpgradeTask& task, String& errorMsg) {
    JsonDocument doc;// 创建一个JsonDocument对象用于存储解析后的JSON数据
    DeserializationError error = deserializeJson(doc, payload);// 反序列化JSON字符串到JsonDocument对象中，如果失败则返回错误信息
    if (error) {
        errorMsg = "OTA message JSON parse failed";
        return false;
    }

    if (doc["id"].isNull()) {
        errorMsg = "Missing OTA request id";
        return false;
    }

    JsonObject data = doc["data"].as<JsonObject>();// 提取data对象，包含OTA升级的具体信息，强转为JsonObject类型，方便后续访问各个字段
    if (data.isNull()) {
        errorMsg = "Missing OTA data field";
        return false;
    }

    task = OtaUpgradeTask{};// 初始化一个空的OTA任务对象，确保所有字段都有默认值，避免未初始化的字段导致的问题
    task.id = doc["id"].as<String>();// 提取OTA请求ID，唯一标识一次OTA升级请求,强转为String类型，兼容平台返回的不同类型的id字段
    if (doc["code"].is<int>()) {
        task.code = doc["code"].as<int>();
    } else {
        String code = doc["code"].as<String>();
        task.code = code.toInt();// 兼容平台返回的不同类型的code字段，尝试转换为整数，如果转换失败则默认为0
    }
    task.message = doc["message"].as<String>();// 提取OTA状态消息，通常由平台返回，描述OTA升级请求的状态或错误信息
    task.version = data["version"].as<String>();
    task.module = data["module"].as<String>();
    task.signMethod = data["signMethod"].as<String>();
    task.md5 = data["md5"].as<String>();
    task.sign = data["sign"].as<String>();
    task.url = data["url"].as<String>();
    task.extData = data["extData"].as<String>();
    task.size = data["size"] | 0;// 提取OTA升级包的大小，单位为字节，使用 | 0 来兼容平台返回的不同类型的size字段，如果size字段不存在或无法转换为整数，则默认为0
    task.isDiff = (data["isDiff"] | 0) == 1;// 有些平台可能返回布尔值，有些返回整数，这里兼容两种情况，最终转换为bool类型
    task.rawPayload = payload;
    task.receivedAt = millis();

    if (task.module.isEmpty()) {// 如果平台没有返回模块信息，默认使用主控固件模块
        task.module = OTA_MODULE_NAME;// 目前仅支持主控固件模块，后续如果有多个模块需要区分时可以根据平台返回的模块信息进行区分
    }

    return true;
}

/**
 * @brief 验证OTA任务的有效性
 * @param task OTA升级任务对象
 * @param errorMsg 验证失败时的错误消息输出参数
 * @param errorStep 验证失败时的错误步骤输出参数，-1表示通用错误，其他值表示特定验证步骤的错误
 * @return OTA任务是否有效
 */
bool validateOtaUpgradeTask(const OtaUpgradeTask& task, String& errorMsg, int& errorStep) {
    if (task.id.isEmpty()) {// OTA请求ID是必需的，如果缺失则无法识别和处理这个OTA任务，因此验证失败
        errorMsg = "Missing OTA request id";
        errorStep = -1;
        return false;
    }

    JsonDocument rawDoc;// 创建一个JsonDocument对象用于存储解析后的原始JSON数据，主要是为了验证code字段是否合法
    if (deserializeJson(rawDoc, task.rawPayload) || !isAcceptedOtaCode(rawDoc["code"])) {// 反序列化原始JSON字符串失败，或者code字段不是被接受的状态码（200或1000），都表示这个OTA任务无效，因此验证失败
        errorMsg = "Invalid OTA task status";
        errorStep = -1;
        return false;
    }

    if (!task.module.isEmpty() && task.module != OTA_MODULE_NAME) {
        errorMsg = "OTA module mismatch";
        errorStep = -1;
        return false;
    }

    if (task.version.isEmpty()) {
        errorMsg = "Missing target version";
        errorStep = -1;
        return false;
    }

    if (task.size == 0) {
        errorMsg = "Invalid firmware size";
        errorStep = -1;
        return false;
    }

    if (task.isDiff) {
        errorMsg = "Diff OTA is not supported";
        errorStep = -2;
        return false;
    }

    if (!task.signMethod.isEmpty() && task.signMethod != "MD5") {
        errorMsg = "Only MD5 verification is supported";
        errorStep = -3;
        return false;
    }

    if (task.md5.isEmpty() && task.sign.isEmpty()) {
        errorMsg = "Missing OTA checksum";
        errorStep = -3;
        return false;
    }

    if (task.url.isEmpty()) {
        errorMsg = "Missing OTA package url";
        errorStep = -2;
        return false;
    }

    return true;
}
