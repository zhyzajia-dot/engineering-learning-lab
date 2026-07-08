# MQTT 协议说明文档

本文档描述固件与 IoT 平台之间的 MQTT 通信协议，覆盖连接配置、主题列表、上行数据上报、下行指令处理、OTA 升级及数据含义/范围说明。参考实现见 [src/mqtt.cpp](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp) 和 [include/mqtt.h](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/include/mqtt.h)。

## 1. 设计原则

1. 设备使用 enjoy-iot 风格的 `/sys/{productKey}/{deviceName}/...` 主题规范。
2. 设备只与一个产品（productKey）关联，通过 productSecret + clientId 派生密码进行身份认证。
3. 上行属性上报统一使用 `thing.event.property.post` 方法，通过 `params.reportType` 区分业务类型（`daily` / `sleep` / `heartbeat`）。
4. 下行请求与上行回复通过主题后缀 `/c/` ↔ `/s/..._reply` 配对。
5. OTA 升级与业务属性上报解耦，使用独立主题 `/ota/device/...`。
6. 单包最大 1024 字节（[mqtt.h:18-19](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/include/mqtt.h#L18-L19)），避免波形/大数据量字段撑爆缓冲。
7. 心跳（`heartbeat`）用于在无人时维持连接保活，与日常/睡眠数据互斥触发。

## 2. 连接配置

| 项目 | 值 / 说明 |
|------|----------|
| 服务器地址 | `8.138.160.177`（[mqtt.cpp:21](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L21)） |
| 端口 | `1883`（[mqtt.cpp:22](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L22)） |
| 设备型号 | `radar_1.0`（[mqtt.cpp:23](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L23)） |
| 产品标识 productKey | `dEkr5BkkXTFZFBdR`（[mqtt.cpp:24](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L24)） |
| 产品密钥 productSecret | `2e7957febfcb48b08a1c69b8deb56738`（[mqtt.cpp:25](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L25)） |
| 客户端ID clientId | `{productKey}_{deviceName}_{model}`（[mqtt.cpp:72-74](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L72-L74)） |
| 用户名 username | `deviceName`（优先 SN，否则 MAC 去冒号）|
| 密码 password | `MD5(productSecret + clientId)` 32位小写十六进制（[mqtt.cpp:539-558](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L539-L558)） |
| 缓冲区大小 | 1024 字节 |
| 重连间隔 | 5 秒 |
| 订阅 Qos | 0（默认）|
| 心跳包周期 | 10 秒（无人时）|
| 日常数据周期 | 2 秒（有人时）|
| 睡眠数据周期 | 10 秒（睡眠/会话结束时）|

> deviceName 规则：先取 `device_sn`（uint64 转字符串），若为 0 则取设备 MAC 地址并去除冒号。

## 3. 主题列表

| 方向 | 主题模板 | 用途 | 代码位置 |
|------|----------|------|----------|
| 订阅（下行）| `/sys/{productKey}/{deviceName}/c/#` | 平台下发的所有业务指令 | [getMqttSubscribeTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L84-L86) |
| 订阅（下行）| `/ota/device/upgrade/{productKey}/{deviceName}` | 平台下发的 OTA 升级指令 | [getOtaUpgradeTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L121-L123) |
| 发布（上行）| `/sys/{productKey}/{deviceName}/s/event/property/post` | 属性/数据上报 | [getMqttPropertyPostTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L106-L108) |
| 发布（上行）| `/ota/device/progress/{productKey}/{deviceName}` | OTA 升级进度上报 | [getOtaProgressTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L138-L140) |
| 发布（上行）| `/ota/device/inform/{productKey}/{deviceName}` | OTA 版本/结果上报 | [getOtaVersionReportTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L156-L158) |
| 发布（回复）| `/sys/{productKey}/{deviceName}/s/..._reply` | 下行指令的回复（替换 `/c/` 为 `/s/` 并加 `_reply`）| [buildReplyTopic](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L674-L678) |

## 4. 上行属性上报（公共格式）

所有业务类型（`daily` / `sleep` / `heartbeat`）统一格式（[publishPropertyReport](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L621-L637)）：

```json
{
  "id": "<消息ID>",
  "method": "thing.event.property.post",
  "params": {
    "deviceId": "<设备名称>",
    "reportType": "daily | sleep | heartbeat",
    "...业务字段..."
  }
}
```

| 顶层字段 | 类型 | 说明 |
|----------|------|------|
| `id` | string | MQTT 消息ID，自增 1~65535，超过归零（[nextMqttMessageId](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L173-L175)）|
| `method` | string | 固定 `thing.event.property.post` |
| `params.deviceId` | string | 设备名称（`getMqttDeviceName()`）|
| `params.reportType` | string | `daily` / `sleep` / `heartbeat` |

## 5. 日常数据上报（`reportType=daily`）

有人时（`presence=1` 或心率/呼吸率 > 0）每 2 秒上报一次。代码见 [sendDailyDataToMQTT](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L919-L963)。

| 字段 | 类型 | 范围/取值 | 数据来源 | 说明 |
|------|------|-----------|----------|------|
| `heartRate` | int | 0~200 BPM | `sensorData.heart_rate` | 心率；>0 才上报 |
| `breathingRate` | int | 0~60 次/分 | `sensorData.breath_rate` | 呼吸率；>0 才上报 |
| `personDetected` | int | 0=无人, 1=有人 | `sensorData.presence` | 是否检测到人 |
| `humanActivity` | int | 0=无, 1=静止, 2=活跃 | `sensorData.motion` | 人体活动状态 |
| `humanDistance` | int | 0~600 cm | `sensorData.distance` | 人体距离（>0 才上报）|
| `algorithmState` | int | 0~8 | `sleepAnalysisSnapshot.algorithm_state` | 算法睡眠状态（见 §11）|
| `algorithmSleepiness` | float | 0.0~1.0 | `sleepAnalysisSnapshot.current_sleepiness` | 困倦度；**1.0（100%）即判定为入睡**（入睡判定标准）|
| `humanPositionX` | int | 与雷达坐标系相关 | `sensorData.pos_x` | 人体X坐标 |
| `humanPositionY` | int | 与雷达坐标系相关 | `sensorData.pos_y` | 人体Y坐标 |
| `humanPositionZ` | int | 与雷达坐标系相关 | `sensorData.pos_z` | 人体Z坐标 |
| `wifiIP` | string | IPv4 字符串 | `WiFi.localIP().toString()` | 设备 IP |
| `primaryEmotion` | int | 0~8 | `g_lastEmotionResult.primaryEmotion` | 主情绪枚举（见 §13）|
| `secondaryEmotion` | int | 0~8 | `g_lastEmotionResult.secondaryEmotion` | 次情绪枚举 |

> 情绪字段仅在 5 秒内有有效结果时才追加（[appendEmotionFields](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L572-L585)）。

## 6. 睡眠数据上报（`reportType=sleep`）

仅在睡眠状态 0(深睡) / 1(浅睡) 时每 10 秒上报一次，会话结束(`sleep_state=8`)时支持强制上报。代码见 [sendSleepDataToMQTT](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L979-L1077)。

### 6.1 传感器统计字段

| 字段 | 类型 | 范围/取值 | 数据来源 | 说明 |
|------|------|-----------|----------|------|
| `outOfBedCount` | int | 0~255 | `sensorData.turn_count` | 离床次数 |
| `turnCount` | int | 0~255 | `sensorData.turnover_count` | 翻身次数 |
| `avgBreathingRate` | int | 0~60 次/分 | `sensorData.avg_breath_rate` | 平均呼吸率 |
| `avgHeartRate` | int | 0~200 BPM | `sensorData.avg_heart_rate` | 平均心率 |
| `apneaCount` | int | 0~255 | `sensorData.apnea_count` | 呼吸暂停次数 |
| `abnormalState` | int | 0=正常, 非0=异常 | `sensorData.abnormal_state` | 异常状态 |
| `breathStatus` | int | 0=正常, 1=呼吸过缓, 2=呼吸过快, 3=无呼吸 | `sensorData.breath_status` | 呼吸状态 |
| `largeMoveRatio` | int | 0~100 | `sensorData.large_move_ratio` | 大幅运动比例 (%) |
| `smallMoveRatio` | int | 0~100 | `sensorData.small_move_ratio` | 小幅运动比例 (%) |
| `struggleAlert` | int | 0=无, 1=有 | `sensorData.struggle_alert` | 挣扎警告 |

### 6.2 本地睡眠算法（SleepAnalyzer）字段

仅在 `sleepAnalysisSnapshot.valid = true` 时追加（[sendSleepDataToMQTT:1019-1050](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L1019-L1050)）。

| 字段 | 类型 | 范围/取值 | 数据来源 | 说明 |
|------|------|-----------|----------|------|
| `algorithmSleepiness` | float | 0~1 | `sleepAnalysisSnapshot.current_sleepiness` | 当前困倦度 |
| `algoTotalSleepTime` | long | 0~86400 s | `total_sleep_time` | 总睡眠时长 |
| `algoDeepSleepTime` | long | 0~86400 s | `deep_sleep_time` | 深睡时长 |
| `algoLightSleepTime` | long | 0~86400 s | `light_sleep_time` | 浅睡时长 |
| `algoRemSleepTime` | long | 0~86400 s | `rem_sleep_time` | REM 睡眠时长 |
| `algoAwakeTime` | long | 0~86400 s | `awake_time` | 清醒时长 |
| `algoOutOfBedTime` | long | 0~86400 s | `out_of_bed_time` | 离床时长 |
| `algoSleepLatency` | long | 0~7200 s | `sleep_latency` | 入睡潜伏期 |
| `algoWakeCount` | int | 0~100 | `wake_count` | 醒来次数 |
| `algoSleepCycles` | int | 0~10 | `sleep_cycles` | 睡眠周期数 |
| `algoTotalScore` | float | 0~100 | `total_score` | 总评分 |

## 7. 心跳包上报（`reportType=heartbeat`）

无人时（`presence=0` 或心率/呼吸率均为 0）每 10 秒发送一次，用于维持平台保活。代码见 [sendHeartbeatToMQTT](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L1090-L1117)。

| 字段 | 类型 | 值 | 说明 |
|------|------|------|------|
| `personDetected` | int | `0` | 固定 0 |
| `heartbeat` | int | `1` | 心跳标识 |
| `timestamp` | long | `millis()` | 设备启动后毫秒数 |

> 心跳包有意保持最小体积，避免被防火墙/平台判定为业务噪声。

## 8. 下行指令处理

平台向 `/sys/{productKey}/{deviceName}/c/...` 主题发布 JSON 消息，固件在 [mqttMessageCallback](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L699-L800) 中按 `method` 字段分发。

### 8.1 通用消息格式（入参）

```json
{
  "id": "<请求ID>",
  "method": "thing.service.property.set | thing.service.property.get | thing.service.<自定义>",
  "params": { ... }
}
```

### 8.2 通用回复格式（出参）

通过 [publishMqttReply](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L687-L713) 发出，主题为 `/sys/{productKey}/{deviceName}/s/<原路径>_reply`：

```json
{
  "id": "<原请求ID>",
  "method": "<原method>_reply",
  "code": 0,
  "params": { ... }
}
```

`code` 含义：`0`=成功，其他值=失败。

### 8.3 `thing.service.property.set` — 设置属性

| 入参字段 | 类型 | 范围/取值 | 说明 |
|----------|------|-----------|------|
| `continuousSendEnabled` | bool | true/false | 是否启用持续发送数据 |
| `continuousSendInterval` | unsigned long | ≥ 100（ms） | 持续发送间隔 |

成功回复示例：

```json
{
  "id": "123",
  "method": "thing.service.property.set_reply",
  "code": 0,
  "params": { "success": true }
}
```

### 8.4 `thing.service.property.get` — 读取属性

成功回复示例：

```json
{
  "id": "123",
  "method": "thing.service.property.get_reply",
  "code": 0,
  "params": {
    "heartRate": 75,
    "breathingRate": 18,
    "personDetected": 1,
    "humanActivity": 1,
    "humanDistance": 120,
    "sleepState": 3,
    "continuousSendEnabled": true,
    "continuousSendInterval": 2000,
    "primaryEmotion": 0,
    "secondaryEmotion": 8
  }
}
```

字段含义与 §5 一致，情绪字段在 5 秒内有效时才附加。

### 8.5 `thing.service.*` — 自定义服务（预留）

固件统一回复 `{ "success": true }`，code=0，不解析 params。平台可借此扩展自定义 RPC。

## 9. OTA 升级

OTA 使用独立主题集群 `/ota/device/...`，与业务属性上报完全解耦。代码见 [mqtt.cpp:400-530](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L400-L530)。

### 9.1 上报主题

| 主题 | 内容 |
|------|------|
| `/ota/device/inform/{productKey}/{deviceName}` | 版本信息、升级结果 |
| `/ota/device/progress/{productKey}/{deviceName}` | 升级进度 |

### 9.2 设备版本上报（连上 MQTT 后立即调用一次）

```json
{
  "id": "<msgId>",
  "params": {
    "version": "<APP_VERSION>",
    "module": "<OTA_MODULE_NAME>"
  }
}
```

### 9.3 平台下发 OTA 指令（订阅 `/ota/device/upgrade/...`）

```json
{
  "id": "<requestId>",
  "code": 200,
  "message": "...",
  "params": {
    "version": "1.0.2",
    "module": "app",
    "signMethod": "MD5",
    "md5": "<32位十六进制>",
    "sign": "<同上>",
    "url": "https://...",
    "size": 1048576,
    "isDiff": false,
    "extData": "..."
  }
}
```

| 字段 | 必填 | 范围/取值 | 说明 |
|------|------|-----------|------|
| `params.url` | 是 | `https://` 开头 | 固件下载地址，目前仅支持 HTTPS |
| `params.version` | 是 | 字符串 | 目标版本号 |
| `params.size` | 是 | 字节数 | 固件大小，与实际下载字节数校验 |
| `params.md5` | 否 | 32位十六进制 | 固件 MD5，校验完整性 |
| `params.module` | 否 | 字符串 | 升级模块标识 |
| `params.isDiff` | 否 | false | 当前不支持差分升级 |

### 9.4 设备上报进度

```json
{
  "id": "<requestId>",
  "params": {
    "step": "5",
    "desc": "Starting HTTPS OTA download",
    "module": "app"
  }
}
```

| `step` 取值 | 含义 |
|-------------|------|
| `-4` | OTA 写入失败（Update.begin/write/end 失败）|
| `-3` | 固件大小不匹配 |
| `-2` | 下载失败 / 仅支持 HTTPS / WiFi 断开 |
| `-1` | 消息解析失败 |
| `0` | 任务已被接受（保留）|
| `1` | 任务接受，等待 HTTPS 下载 |
| `5` | 开始下载 |
| `5~95` | 下载进度（每 5% 上报一次）|
| `95` | 写入成功，即将重启 |
| `100` | OTA 升级成功（设备重启后自检时上报）|

### 9.5 OTA 内部状态机

| `OtaState` | 数值 | 含义 |
|------------|------|------|
| `OTA_IDLE` | 0 | 空闲，无任务 |
| `OTA_NOTIFIED` | 1 | 收到升级通知 |
| `OTA_VALIDATING` | 2 | 正在验证 |
| `OTA_READY` | 3 | 验证通过，等待执行 |
| `OTA_DOWNLOADING` | 4 | 下载中 |
| `OTA_VERIFYING` | 5 | 校验中 |
| `OTA_WRITING` | 6 | 写入 Flash |
| `OTA_PENDING_REBOOT` | 7 | 写入成功，待重启 |
| `OTA_SUCCESS` | 8 | 升级成功 |
| `OTA_REJECTED` | 9 | 验证失败被拒绝 |
| `OTA_UNSUPPORTED_PROTOCOL` | 10 | 非 HTTPS 等不支持的协议 |
| `OTA_FAILED` | 11 | 升级失败 |

详见 [include/OTA_manager.h](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/include/OTA_manager.h)。

## 10. 数据范围与单位汇总

| 类别 | 字段 | 数据类型 | 取值范围 | 单位 |
|------|------|----------|----------|------|
| 生命体征 | `heartRate` / `avgHeartRate` | float → int | 0~200 | BPM（次/分）|
| 生命体征 | `breathingRate` / `avgBreathingRate` | float → int | 0~60 | 次/分 |
| 距离 | `humanDistance` | uint16 | 0~600 | cm |
| 位置 | `humanPositionX/Y/Z` | int16 | 与雷达坐标相关 | cm |
| 状态枚举 | `personDetected` | uint8 | 0/1 | - |
| 状态枚举 | `humanActivity` | uint8 | 0=无, 1=静止, 2=活跃 | - |
| 状态枚举 | `breathStatus` | uint8 | 0=正常, 1=过缓, 2=过快, 3=无 | - |
| 状态枚举 | `sleepState` | uint8 | 0=深睡, 1=浅睡, …（见 §11）| - |
| 状态枚举 | `algorithmState` | int | 0~8（见 §11）| - |
| 状态枚举 | `abnormalState` | uint8 | 0=正常, 非0=异常 | - |
| 状态枚举 | `struggleAlert` | uint8 | 0/1 | - |
| 比例 | `largeMoveRatio` / `smallMoveRatio` | uint8 | 0~100 | % |
| 计数 | `outOfBedCount` / `turnCount` / `apneaCount` | uint8 | 0~255 | 次 |
| 时长 | `algoTotalSleepTime` 等 | unsigned long | 0~86400 | s |
| 评分 | `algoTotalScore` | float | 0~100 | 分 |
| 情绪 | `primaryEmotion` / `secondaryEmotion` | int (enum) | 0~8（见 §13）| - |
| 困倦度 | `algorithmSleepiness` | float | 0~1 | - |

## 11. 睡眠状态枚举

`SleepState`（[include/sleep_analyzer.h](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/include/sleep_analyzer.h)）：

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | `SLEEP_NO_PERSON` | 无人 |
| 1 | `SLEEP_IN_BED` | 在床 |
| 2 | `SLEEP_AWAKE` | 清醒 |
| 3 | `SLEEP_LIGHT_SLEEP` | 浅睡 |
| 4 | `SLEEP_DEEP_SLEEP` | 深睡 |
| 5 | `SLEEP_REM_SLEEP` | REM 睡眠 |
| 6 | `SLEEP_OUT_OF_BED` | 离床 |
| 7 | `SLEEP_GETTING_UP` | 起床 |
| 8 | `SLEEP_SESSION_END` | 睡眠会话结束（无人且睡眠结束）|

## 12. 呼吸状态枚举（`breathStatus`）

| 值 | 含义 |
|----|------|
| 0 | 正常 |
| 1 | 呼吸过缓 |
| 2 | 呼吸过快 |
| 3 | 无呼吸 |

## 13. 情绪枚举（`EmotionType`，[include/config.h:113-136](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/include/config.h#L113-L136)）

| 值 | 名称 | 描述 |
|----|------|------|
| 0 | `EMOTION_CALM` | 平静，心情平静、情绪稳定 |
| 1 | `EMOTION_HAPPY` | 快乐，心情愉悦、积极正向 |
| 2 | `EMOTION_EXCITED` | 兴奋，情绪高涨、充满活力 |
| 3 | `EMOTION_ANXIOUS` | 焦虑，感到焦虑或不安 |
| 4 | `EMOTION_ANGRY` | 愤怒，情绪激动、可能有愤怒 |
| 5 | `EMOTION_SAD` | 悲伤，情绪低落 |
| 6 | `EMOTION_STRESSED` | 压力，压力较大 |
| 7 | `EMOTION_RELAXED` | 放松，身心放松 |
| 8 | `EMOTION_UNKNOWN` | 未知 / 无效结果 |

## 14. 状态推送（BLE b3 通道联动）

MQTT 模块在以下状态变化时通过 [pushDeviceStatusIfChanged](file:///c:/Users/Admin/Desktop/zmq/Radar-recover-code-1/src/mqtt.cpp#L12) 通知 BLE，便于在小程序端显示：

| 触发时机 | 推送状态 |
|----------|----------|
| MQTT 连接成功 | `DEV_MQTT_CONNECTED` |
| MQTT 连接失败 | `DEV_MQTT_FAILED` |
| MQTT 断开连接 | `DEV_MQTT_DISCONNECTED` |

## 15. 时序与触发条件

```text
[WiFi已连接]
   ↓
initMQTT() → setServer / setBufferSize / setCallback
   ↓
reconnectMQTT() ← checkMQTTStatus() 周期检查
   ├─ 5s 节流重连
   ├─ 成功 → subscribe(/sys/.../c/#) + subscribe(/ota/...) + publishOtaVersionReport
   └─ 失败 → 上报 BLE 状态 DEV_MQTT_FAILED
   ↓
mqttTask 主循环 (10ms tick)：
   ├─ checkMQTTStatus() 处理 loop()
   ├─ checkAndReportPendingOtaResult() 启动补报
   ├─ otaExecutionRequested → executeHttpsOtaTask() (HTTPS 下载升级)
   ├─ 无人 → sendHeartbeatToMQTT()  (每 10s)
   ├─ 有人 → sendDailyDataToMQTT()  (每 2s)
   └─ 睡眠中 → sendSleepDataToMQTT()  (每 10s)
```

## 16. 错误码与失败处理

| 现象 | 行为 |
|------|------|
| WiFi 断开 | 跳过所有上报，`checkMQTTStatus()` 不重连 |
| MQTT 断开 | 5 秒后重试；通过 BLE 推送 `DEV_MQTT_DISCONNECTED` |
| JSON 解析失败 | 丢弃该下行消息，串口打印错误 |
| 不支持的 `method` | 串口打印 `[MQTT] 暂不支持 method=...` |
| OTA url 非 HTTPS | 标记 `OTA_UNSUPPORTED_PROTOCOL`，上报 step=-2 |
| OTA size 与实际不符 | 标记 `OTA_FAILED`，上报 step=-3 |
| OTA Update 失败 | 标记 `OTA_FAILED`，上报 step=-4 |

## 17. 调试与串口日志

固件默认通过 `Serial.printf` 输出以下关键日志，可用于联调：

- `[MQTT] broker: %s:%d` — 服务器配置
- `[MQTT] clientId: %s` — 客户端 ID
- `[MQTT] 连接成功, clientId=...` — 鉴权通过
- `[MQTT] 已订阅: /sys/.../c/#` — 订阅下行
- `[MQTT] daily data report published` — 日常数据上报成功
- `[MQTT] 睡眠数据上报成功` — 睡眠数据上报成功
- `[MQTT] heartbeat report published` — 心跳上报成功
- `[MQTT] reply topic: ...` — 下行回复
- `[MQTT] 收到主题: ...` — 收到下行消息
- `[MQTT] OTA版本信息上报成功` — OTA 版本上报
- `[MQTT] OTA进度上报成功: step=...` — OTA 进度上报

## 18. 协议示例（curl 模拟平台下发）

```bash
# 设置属性
mosquitto_pub -h 8.138.160.177 -p 1883 \
  -t '/sys/dEkr5BkkXTFZFBdR/<deviceName>/c/service/property/set' \
  -m '{"id":"100","method":"thing.service.property.set","params":{"continuousSendEnabled":true,"continuousSendInterval":2000}}'

# 读取属性
mosquitto_pub -h 8.138.160.177 -p 1883 \
  -t '/sys/dEkr5BkkXTFZFBdR/<deviceName>/c/service/property/get' \
  -m '{"id":"101","method":"thing.service.property.get","params":{}}'
```

设备收到后会向 `/sys/dEkr5BkkXTFZFBdR/<deviceName>/s/service/property/get_reply` 推送回复，载荷参见 §8.4。
