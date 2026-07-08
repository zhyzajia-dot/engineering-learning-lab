# BLE TLV 协议说明

本文档描述固件与蓝牙小程序之间的 BLE GATT 通信协议。当前协议目标是保持通信层简单稳定：所有业务数据统一封装为 TLV 帧，通过 Notify 分包发送；命令响应只用 `TLV_RESULT_CODE` 表达结果，具体文案由小程序根据错误码或状态码映射。

## 1. 设计原则

1. 客户端必须先按 GATT 特征 UUID 分流，再按 `cmd` 解析。

2. `b1/b2` 是命令请求/响应通道：客户端写 `b1`，设备通过 `b2` notify 响应。

3. `a1/a2/b3` 是主动推送通道，不参与请求响应匹配。

4. Notify 数据可能超过单包大小，客户端必须按特征分别维护重组 buffer。

5. 命令响应只以 `TLV_RESULT_CODE` 作为结果事实来源。

6. 状态变化通过 `b3` 推送，和命令结果解耦。

7. 协议层不再使用 `flags`、`ACK`、`TLV_STATE`、`TLV_STEP`、`TLV_MESSAGE`、`TLV_ERROR_MESSAGE`；这些字段当前不再定义为有效业务字段。

8. `READ` 不作为主协议能力使用，客户端不要依赖 `readBLECharacteristicValue` 获取业务数据。

## 2. GATT 服务与特征

### 2.1 Radar Data Service

| 名称 | UUID | 属性 | 方向 | 职责 |
| --- | --- | --- | --- | --- |
| `RADAR_DATA_SERVICE_UUID` | `a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1a` | Service | - | 雷达数据服务 |
| `RADAR_STREAM_CHAR_UUID` (`a1`) | `beb5483e-36e1-4688-b7f5-ea07361b26a1` | `NOTIFY` | 设备 -> 客户端 | 连续雷达数据流 |
| `RADAR_STATUS_CHAR_UUID` (`a2`) | `beb5483e-36e1-4688-b7f5-ea07361b26a2` | `NOTIFY` | 设备 -> 客户端 | 雷达状态主动推送 |

### 2.2 Device Config Service

| 名称 | UUID | 属性 | 方向 | 职责 |
| --- | --- | --- | --- | --- |
| `DEVICE_CONFIG_SERVICE_UUID` | `a8c1e5c0-3d5d-4a9d-8d5e-7c8b6a4e2f1b` | Service | - | 设备配置服务 |
| `DEVICE_COMMAND_CHAR_UUID` (`b1`) | `beb5483e-36e1-4688-b7f5-ea07361b26b1` | `WRITE` | 客户端 -> 设备 | 命令写入 |
| `DEVICE_RESULT_CHAR_UUID` (`b2`) | `beb5483e-36e1-4688-b7f5-ea07361b26b2` | `NOTIFY` | 设备 -> 客户端 | 命令响应 |
| `DEVICE_INFO_CHAR_UUID` (`b3`) | `beb5483e-36e1-4688-b7f5-ea07361b26b3` | `NOTIFY` | 设备 -> 客户端 | 设备信息与状态主动推送 |

## 3. 二进制帧格式

所有业务数据统一封装为 TLV 帧：

```text
SOF1 SOF2 VERSION CMD SEQ LEN_H LEN_L PAYLOAD CRC_H CRC_L
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `SOF1` | 1 | 固定 `0xAA` |
| `SOF2` | 1 | 固定 `0x55` |
| `VERSION` | 1 | 当前 `0x01` |
| `CMD` | 1 | 命令码 |
| `SEQ` | 1 | 请求序列号；主动推送固定为 `0` |
| `LEN` | 2 | `PAYLOAD` 长度，大端 |
| `PAYLOAD` | N | TLV 数据区 |
| `CRC` | 2 | CRC16-CCITT，大端 |

CRC 计算范围：从 `VERSION` 到 `PAYLOAD` 末尾，不包含 `SOF1/SOF2`，也不包含 `CRC` 字段本身。

### 3.1 Notify 分包

固件通过 `sendFrameToBLE()` 发送 Notify。客户端不能假设一次 Notify 就是一帧完整数据，必须为每个 Notify 特征分别维护重组 buffer：

- `a1` 一个 buffer
- `a2` 一个 buffer
- `b2` 一个 buffer
- `b3` 一个 buffer

## 4. TLV 编码

TLV 编码格式：

```text
TYPE(1) LEN_H(1) LEN_L(1) VALUE(N)
```

`LEN` 为大端。

## 5. 命令码

### 5.1 系统命令

| 命令 | 值 | 通道 | 说明 |
| --- | --- | --- | --- |
| `CMD_PING` | `0x01` | `b1/b2` | Ping 请求/响应 |

### 5.2 雷达与状态命令

| 命令 | 值 | 通道 | 说明 |
| --- | --- | --- | --- |
| `CMD_QUERY_STATUS` | `0x10` | `b1/b2` | 查询设备概览 |
| `CMD_QUERY_RADAR` | `0x12` | `b1/b2` | 查询雷达快照 |
| `CMD_START_CONTINUOUS` | `0x14` | `b1/b2` | 启动连续数据推送 |
| `CMD_STOP_CONTINUOUS` | `0x16` | `b1/b2` | 停止连续数据推送 |
| `CMD_RADAR_SLEEP_QUERY` | `0x17` | `b1/b2` | 雷达睡眠/综合状态查询开关 |
| `CMD_CONTINUOUS_PUSH` | `0x18` | `a1` | 连续雷达数据主动推送 |
| `CMD_DEVICE_INFO_PUSH` | `0x19` | `b3` | 设备信息/状态主动推送 |
| `CMD_RADAR_STATUS_PUSH` | `0x1A` | `a2` | 雷达状态主动推送 |

### 5.3 WiFi 命令

| 命令 | 值 | 通道 | 说明 |
| --- | --- | --- | --- |
| `CMD_WIFI_SCAN` | `0x20` | `b1/b2` | WiFi 扫描 |
| `CMD_WIFI_CONFIG` | `0x22` | `b1/b2` | WiFi 配网 |
| `CMD_GET_SAVED_WIFI` | `0x24` | `b1/b2` | 查询已保存 WiFi |
| `CMD_DELETE_SAVED_WIFI` | `0x26` | `b1/b2` | 删除已保存 WiFi |

### 5.4 通用命令

| 命令 | 值 | 通道 | 说明 |
| --- | --- | --- | --- |
| `CMD_ERROR_RESP` | `0x7E` | `b2` | 协议层错误响应 |

## 6. TLV 类型

### 6.1 设备信息 TLV

| TLV | 值 | 类型 | 说明 |
| --- | --- | --- | --- |
| `TLV_RESULT_CODE` | `0x02` | `uint8` | 结果码 |
| `TLV_PROTOCOL_VERSION` | `0x05` | `string` | 协议版本 |
| `TLV_DEVICE_SN` | `0x06` | `uint64` | 设备序列号，仅存在时发送 |
| `TLV_FIRMWARE_VERSION` | `0x07` | `string` | 固件版本 |
| `TLV_DEVICE_TYPE` | `0x08` | `string` | 设备类型 |
| `TLV_MAC_ADDRESS` | `0x09` | `string` | MAC 地址 |

约束：

- `TLV_DEVICE_SN` 固定为 `uint64`。
- 没有设备 SN 时不发送 `TLV_DEVICE_SN`。
- 不使用 MAC 字符串替代 `TLV_DEVICE_SN`。
- 旧 `TLV_DEVICE_ID` / `CMD_SET_DEVICE_ID` 已移除；设备唯一标识优先使用 MAC，存在 SN 时可同时发送 SN。

### 6.2 雷达数据 TLV

| TLV | 值 | 类型 | 说明 |
| --- | --- | --- | --- |
| `TLV_HEART_RATE_X10` | `0x10` | `uint16` | 心率乘 10 |
| `TLV_BREATH_RATE_X10` | `0x11` | `uint16` | 呼吸率乘 10 |
| `TLV_PRESENCE` | `0x12` | `uint8` | 是否存在人体 |
| `TLV_MOTION` | `0x13` | `uint8` | 运动状态 |
| `TLV_DISTANCE_CM` | `0x15` | `uint16` | 距离，单位 cm |
| `TLV_POS_X_MM` | `0x16` | `int16` | X 坐标，单位 mm |
| `TLV_POS_Y_MM` | `0x17` | `int16` | Y 坐标，单位 mm |
| `TLV_POS_Z_MM` | `0x18` | `int16` | Z 坐标，单位 mm |
| `TLV_BODY_MOVEMENT` | `0x19` | `uint8` | 体动状态 |

### 6.3 WiFi TLV

| TLV | 值 | 类型 | 说明 |
| --- | --- | --- | --- |
| `TLV_SSID` | `0x20` | `string` | WiFi SSID |
| `TLV_PASSWORD` | `0x21` | `string` | WiFi 密码 |
| `TLV_WIFI_COUNT` | `0x22` | `uint16` | WiFi 数量 |
| `TLV_WIFI_ITEM` | `0x23` | `block` | WiFi 条目 |
| `TLV_RSSI` | `0x24` | `int8` | RSSI |
| `TLV_SECURITY` | `0x25` | `uint8` | 加密类型，见 `WifiSecurityType` |

### 6.4 控制与状态 TLV

| TLV | 值 | 类型 | 说明 |
| --- | --- | --- | --- |
| `TLV_INTERVAL_MS` | `0x31` | `uint16` | 推送间隔，单位 ms |
| `TLV_RADAR_SLEEP_ENABLED` | `0x32` | `uint8` | 雷达睡眠查询开关，`0` 关闭，`1` 开启 |
| `TLV_DEVICE_STATUS` | `0x33` | `uint8` | b3 状态推送使用 |
| `TLV_WIFI_STATUS` | `0x34` | `uint8` | `CMD_QUERY_STATUS` 查询响应中的 WiFi 状态 |
| `TLV_MQTT_STATUS` | `0x35` | `uint8` | `CMD_QUERY_STATUS` 查询响应中的 MQTT 状态 |
| `TLV_RADAR_SLEEP_STATUS` | `0x36` | `uint8` | `CMD_QUERY_STATUS` 查询响应中的雷达睡眠查询状态 |
| `TLV_IP_ADDRESS` | `0x41` | `string` | IP 地址 |
| `TLV_WIFI_CONFIGURED` | `0x42` | `uint8` | 是否保存过 WiFi |
| `TLV_WIFI_CONNECTED` | `0x43` | `uint8` | WiFi 是否连接 |
| `TLV_ECHO_CONTENT` | `0x44` | `string` | Echo 内容 |

### 6.5 波形 TLV

| TLV | 值 | 类型 | 说明 |
| --- | --- | --- | --- |
| `TLV_HEART_WAVEFORM` | `0x60` | `uint8` | 心跳波形，原始 `int8 + 128` |
| `TLV_BREATH_WAVEFORM` | `0x61` | `uint8` | 呼吸波形，原始 `int8 + 128` |

## 7. 结果码

| 结果码 | 值 | 客户端建议处理 |
| --- | --- | --- |
| `SUCCESS` | `0x00` | 成功 |
| `PROCESSING` | `0x01` | 命令已接收，等待最终结果 |
| `ERR_PROTO_CMD_UNKNOWN` | `0x13` | 未知命令 |
| `ERR_PROTO_PARAM_MISSING` | `0x14` | 参数缺失 |
| `ERR_PROTO_PARAM_INVALID` | `0x15` | 参数非法 |
| `ERR_PROTO_BUSY` | `0x16` | 设备忙，稍后重试 |
| `ERR_PROTO_FRAME_TOO_LARGE` | `0x18` | 请求帧过大 |
| `ERR_WIFI_SCAN_TIMEOUT` | `0x20` | WiFi 扫描超时 |
| `ERR_WIFI_SSID_NOT_FOUND` | `0x21` | 未找到 SSID |
| `ERR_WIFI_WRONG_PASSWORD` | `0x22` | WiFi 密码错误 |
| `ERR_WIFI_SIGNAL_WEAK` | `0x25` | WiFi 信号弱 |
| `ERR_WIFI_BUSY` | `0x26` | WiFi 忙 |
| `ERR_DEV_STATE_INVALID` | `0x40` | 当前设备状态不允许 |
| `ERR_DEV_STORAGE_FAIL` | `0x41` | 存储失败 |
| `ERR_DEV_QUEUE_FULL` | `0x42` | 队列已满 |

客户端 UI 文案应由结果码映射，不依赖固件下发字符串。

## 8. 设备状态码

`TLV_DEVICE_STATUS`、`TLV_WIFI_STATUS`、`TLV_MQTT_STATUS`、`TLV_RADAR_SLEEP_STATUS` 的 value 使用同一组状态码。

### 8.1 WiFi 状态

| 状态 | 值 | 说明 |
| --- | --- | --- |
| `WIFI_DISCONNECTED` | `0x10` | WiFi 断开 |
| `WIFI_CONNECTING` | `0x11` | WiFi 连接中 |
| `WIFI_CONNECTED` | `0x12` | WiFi 已连接 |
| `WIFI_FAILED` | `0x13` | WiFi 连接失败 |

### 8.2 MQTT 状态

| 状态 | 值 | 说明 |
| --- | --- | --- |
| `DEV_MQTT_DISCONNECTED` | `0x20` | MQTT 断开 |
| `DEV_MQTT_CONNECTING` | `0x21` | MQTT 连接中 |
| `DEV_MQTT_CONNECTED` | `0x22` | MQTT 已连接 |
| `DEV_MQTT_FAILED` | `0x23` | MQTT 连接失败 |

### 8.3 雷达睡眠查询状态

| 状态 | 值 | 说明 |
| --- | --- | --- |
| `RADAR_SLEEP_QUERY_DISABLED` | `0x30` | 雷达睡眠/综合状态查询关闭 |
| `RADAR_SLEEP_QUERY_ENABLED` | `0x31` | 雷达睡眠/综合状态查询开启 |

## 9. 命令响应模型

### 9.1 普通命令

客户端写 `b1`，设备从 `b2` 返回响应：

```text
cmd = 原命令码
seq = 请求 seq
payload = TLV_RESULT_CODE + 业务数据 TLV
```

失败时也使用原命令码响应，`TLV_RESULT_CODE` 为对应错误码。协议层无法归属到具体业务命令的入口级错误，使用 `CMD_ERROR_RESP (0x7E)`。

响应不发送：

- `flags`
- `TLV_MESSAGE`
- `TLV_ERROR_MESSAGE`
- `TLV_STATE`
- `TLV_STEP`

### 9.2 异步长命令

适用命令：

- `CMD_WIFI_SCAN`
- `CMD_WIFI_CONFIG`

收到命令后立即返回处理中：

```text
cmd = 原命令码
seq = 请求 seq
TLV_RESULT_CODE = PROCESSING
```

最终完成后再次返回：

```text
cmd = 原命令码
seq = 请求 seq
TLV_RESULT_CODE = SUCCESS 或 ERR_XXX
业务数据 TLV（如 SSID、IP、WiFi 列表）
```

流程进度不再通过 `TLV_STATE/TLV_STEP` 表达。WiFi、MQTT、雷达睡眠查询等运行状态通过 b3 状态推送表达。

## 10. 主要命令载荷

### 10.1 `CMD_QUERY_STATUS`

请求 payload 可为空。

响应 TLV：

- `TLV_RESULT_CODE`
- `TLV_PROTOCOL_VERSION`
- `TLV_FIRMWARE_VERSION`
- `TLV_DEVICE_TYPE`
- `TLV_MAC_ADDRESS`
- `TLV_DEVICE_SN`，仅设备 SN 存在时发送
- `TLV_WIFI_CONFIGURED`
- `TLV_WIFI_CONNECTED`
- `TLV_IP_ADDRESS`，仅 WiFi 已连接时发送
- `TLV_WIFI_STATUS`，当前 WiFi 状态存在时发送
- `TLV_MQTT_STATUS`，当前 MQTT 状态存在时发送
- `TLV_RADAR_SLEEP_STATUS`，当前雷达睡眠查询状态存在时发送

### 10.2 `CMD_QUERY_RADAR`

请求 payload 可为空。

响应 TLV：

- `TLV_RESULT_CODE`
- `TLV_PRESENCE`
- `TLV_HEART_RATE_X10`
- `TLV_BREATH_RATE_X10`
- `TLV_MOTION`
- `TLV_DISTANCE_CM`
- `TLV_POS_X_MM`
- `TLV_POS_Y_MM`
- `TLV_POS_Z_MM`
- `TLV_BODY_MOVEMENT`

### 10.3 `CMD_WIFI_SCAN`

收到请求后先返回：

- `TLV_RESULT_CODE = PROCESSING`

最终响应：

- `TLV_RESULT_CODE`
- `TLV_WIFI_COUNT`，成功且存在扫描结果时发送
- 多个 `TLV_WIFI_ITEM`

每个 `TLV_WIFI_ITEM` 内部包含：

- `TLV_SSID`
- `TLV_RSSI`
- `TLV_SECURITY`

### 10.4 `CMD_WIFI_CONFIG`

请求 TLV：

- `TLV_SSID`
- `TLV_PASSWORD`

收到请求后先返回：

- `TLV_RESULT_CODE = PROCESSING`
- `TLV_SSID`

最终响应：

- `TLV_RESULT_CODE`
- `TLV_SSID`，成功时发送
- `TLV_IP_ADDRESS`，成功时发送

### 10.5 `CMD_GET_SAVED_WIFI`

响应 TLV：

- `TLV_RESULT_CODE`
- `TLV_WIFI_COUNT`
- 多个 `TLV_WIFI_ITEM`，每个条目至少包含 `TLV_SSID`

### 10.6 `CMD_DELETE_SAVED_WIFI`

请求 TLV：

- `TLV_SSID`

响应 TLV：

- `TLV_RESULT_CODE`

### 10.7 `CMD_RADAR_SLEEP_QUERY`

请求 TLV：

- `TLV_RADAR_SLEEP_ENABLED`，`0` 关闭，`1` 开启

响应 TLV：

- `TLV_RESULT_CODE`
- `TLV_RADAR_SLEEP_ENABLED`

如果开关状态发生变化，设备另外通过 b3 推送：

- `TLV_DEVICE_STATUS = RADAR_SLEEP_QUERY_DISABLED` 或 `RADAR_SLEEP_QUERY_ENABLED`

## 11. 主动推送

### 11.1 b3 设备状态推送

通道：`b3`

帧：

```text
cmd = CMD_DEVICE_INFO_PUSH (0x19)
seq = 0
payload = TLV_DEVICE_STATUS
```

b3 状态推送与命令完全解耦。设备只在状态变化时推送；BLE 连接建立后可同步当前已知状态。

状态推送统一只发送状态码，不发送 `TLV_MESSAGE`。

### 11.2 b3 设备信息推送

通道：`b3`

帧：

```text
cmd = CMD_DEVICE_INFO_PUSH (0x19)
seq = 0
```

常见 TLV：

- `TLV_RESULT_CODE`
- `TLV_PROTOCOL_VERSION`
- `TLV_FIRMWARE_VERSION`
- `TLV_DEVICE_TYPE`
- `TLV_MAC_ADDRESS`
- `TLV_DEVICE_SN`，仅设备 SN 存在时发送

### 11.3 a2 雷达状态推送

通道：`a2`

帧：

```text
cmd = CMD_RADAR_STATUS_PUSH (0x1A)
seq = 0
```

常见 TLV：

- `TLV_DISTANCE_CM`
- `TLV_POS_X_MM`
- `TLV_POS_Y_MM`
- `TLV_POS_Z_MM`
- `TLV_BODY_MOVEMENT`

### 11.4 a1 连续雷达数据推送

通道：`a1`

帧：

```text
cmd = CMD_CONTINUOUS_PUSH (0x18)
seq = 设备侧自增或固定策略
```

常见 TLV：

- `TLV_TIMESTAMP`
- `TLV_PRESENCE`
- `TLV_HEART_RATE_X10`
- `TLV_BREATH_RATE_X10`
- `TLV_MOTION`
- `TLV_DISTANCE_CM`
- `TLV_HEART_WAVEFORM`
- `TLV_BREATH_WAVEFORM`

## 12. 客户端接入要求

1. 先按特征 UUID 分流。

2. 每个 Notify 特征维护独立重组 buffer。

3. 只在 `b2` 上按 `seq` 匹配请求响应。

4. `a1/a2/b3` 不做请求响应匹配。

5. 所有响应以 `TLV_RESULT_CODE` 为唯一结果判断来源。

6. 错误文案、状态文案由小程序根据错误码/状态码本地映射。

7. 不依赖 `TLV_MESSAGE`、`TLV_ERROR_MESSAGE`、`flags`、`ACK`、`TLV_STATE`、`TLV_STEP`。

## 13. 客户端伪代码

### 13.1 Notify 重组

```javascript
const buffers = {
  a1: new Uint8Array(0),
  a2: new Uint8Array(0),
  b2: new Uint8Array(0),
  b3: new Uint8Array(0),
}

function onNotify(characteristicId, chunk) {
  const channel = mapCharacteristicToChannel(characteristicId)
  buffers[channel] = concatBytes(buffers[channel], chunk)

  while (true) {
    const frame = tryExtractFrame(buffers[channel])
    if (!frame) break

    buffers[channel] = frame.remaining

    if (!verifyCrc(frame.bytes)) {
      continue
    }

    dispatchByChannel(channel, frame.decoded)
  }
}
```

### 13.2 通道分发

```javascript
function dispatchByChannel(channel, frame) {
  switch (channel) {
    case 'a1':
      handleContinuousPush(frame)
      break
    case 'a2':
      handleRadarStatusPush(frame)
      break
    case 'b3':
      handleDeviceInfoOrStatusPush(frame)
      break
    case 'b2':
      handleCommandResponse(frame)
      break
  }
}
```

### 13.3 请求响应匹配

```javascript
const pending = new Map()

function sendCommand(cmd, payload) {
  const seq = nextSeq()
  const frame = encodeFrame({ cmd, seq, payload })

  pending.set(seq, { cmd, createdAt: Date.now() })
  return writeB1(frame)
}

function handleCommandResponse(frame) {
  const req = pending.get(frame.seq)
  if (!req) {
    return
  }

  const resultCode = getTlvU8(frame.payload, TLV_RESULT_CODE)
  if (resultCode !== PROCESSING) {
    pending.delete(frame.seq)
  }

  routeCommandResult(req.cmd, resultCode, frame.payload)
}
```

## 14. 固件实现约束

1. 所有业务 Notify 必须通过 `sendFrameToBLE()` 发送。

2. 不允许直接 `setValue(fullFrame); notify();` 发送完整业务帧。

3. `DEVICE_INFO_CHAR_UUID (b3)` 只作为 notify 通道使用。

4. `RADAR_STATUS_CHAR_UUID (a2)` 只作为 notify 通道使用。

5. `CMD_QUERY_STATUS (0x10)` 用于 `b1` 请求和 `b2` 响应。

6. `CMD_QUERY_RADAR (0x12)` 用于 `b1` 请求和 `b2` 响应。

7. `CMD_DEVICE_INFO_PUSH (0x19)` 只用于 `b3`。

8. `CMD_RADAR_STATUS_PUSH (0x1A)` 只用于 `a2`。

9. `TLV_DEVICE_SN` 固定为 `uint64`，无 SN 时不发送。

10. `b1` 写入超过固件接收缓冲区上限时，必须返回 `ERR_PROTO_FRAME_TOO_LARGE`。
