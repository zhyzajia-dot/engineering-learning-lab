/**
 * @file OTA_manager.h
 * @brief 设备远程固件升级的状态、任务结构和管理接口。
 *
 * MQTT 模块通过这些接口保存、查询并执行 OTA 升级任务。
 */
#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>

enum OtaState {
    OTA_IDLE = 0,// 空闲状态，没有OTA任务
    OTA_NOTIFIED,// 已收到OTA升级通知，但尚未验证
    OTA_VALIDATING,// 正在验证OTA升级任务的合法性和适配性
    OTA_READY,// OTA任务已验证通过，准备就绪，可以执行但尚未开始执行
    OTA_DOWNLOADING,// 正在下载OTA升级包
    OTA_VERIFYING,// 下载完成，正在验证OTA升级包的完整性和正确性
    OTA_WRITING,// 正在将OTA升级包写入设备
    OTA_PENDING_REBOOT,// OTA升级完成，等待重启以应用新固件
    OTA_SUCCESS,// OTA升级成功完成
    OTA_REJECTED,// OTA升级请求被拒绝，可能是因为格式错误、缺少必要字段、验证失败等原因
    OTA_UNSUPPORTED_PROTOCOL,// OTA升级请求使用了不受支持的协议，例如非HTTPS URL
    OTA_FAILED// OTA升级失败，可能是下载失败、验证失败、写入失败等原因
};

struct OtaUpgradeTask {
    String id;// OTA请求ID，唯一标识一次OTA升级请求
    int code;// OTA状态码，通常由平台返回，200表示有新版本，1000表示当前版本已是最新，无需升级
    String message;// OTA状态消息，通常由平台返回，描述OTA升级请求的状态或错误信息
    String version;// 目标固件版本号，平台下发的OTA升级请求中指定的版本号，设备可以根据这个版本号决定是否接受升级
    String module;// OTA升级任务所属模块，用于区分不同模块的OTA升级请求，例如主控固件、通信模块、传感器模块等
    String signMethod;// OTA升级包的签名验证方法，例如MD5、SHA256等，目前仅支持MD5
    String md5;// OTA升级包的MD5校验值，用于验证下载的OTA升级包的完整性和正确性
    String sign;// OTA升级包的签名值，通常是对OTA升级包进行签名后的结果，用于验证下载的OTA升级包的真实性，目前仅支持MD5签名，即signMethod为MD5时，sign应该与md5值相同
    String url;// OTA升级包的下载URL，设备将从这个URL下载OTA升级包，目前仅支持HTTPS URL
    String extData;// 扩展数据字段，平台下发的OTA升级请求中可能包含的其他信息，设备可以根据需要解析和使用这些信息，例如升级包的大小、发布时间等
    uint32_t size;// OTA升级包的大小，单位为字节，设备可以根据这个字段来预分配存储空间或验证下载的OTA升级包的大小是否正确
    bool isDiff;// 是否为差分OTA升级包，差分OTA升级包只包含与当前版本不同的部分，设备需要先合成完整的OTA升级包才能进行验证和安装，目前不支持差分OTA升级
    unsigned long receivedAt;// OTA升级请求的接收时间，单位为毫秒，可以用于计算OTA升级请求的处理时长、下载时长等指标
    String rawPayload;// OTA升级请求的原始JSON字符串，设备可以根据需要保存这个原始字符串以供后续分析或调试使用
};

void initOtaManager();//初始化OTA管理器
void resetOtaTask();//重置OTA任务
bool hasPendingOtaTask();//检查是否有待处理的OTA任务
OtaState getOtaState();//获取当前OTA状态
const OtaUpgradeTask& getCurrentOtaTask();//获取当前OTA任务
bool hasExecutableOtaTask();//检查是否有可执行的OTA任务

bool parseOtaUpgradeMessage(const String& payload, OtaUpgradeTask& task, String& errorMsg);//解析OTA升级消息
bool validateOtaUpgradeTask(const OtaUpgradeTask& task, String& errorMsg, int& errorStep);//验证OTA升级任务
void storeOtaTask(const OtaUpgradeTask& task);//存储OTA任务
void markOtaState(OtaState state);//标记OTA状态 

#endif
