/*
 * ESP8266_BRIDGE.cpp - 两块 ESP-01/ESP-01S 组成的无线串口桥
 *
 * 数据路径：
 *   PC 上位机 <-> PC 端 ESP8266 <-> ESP-NOW
 *              <-> 小车端 ESP8266 <-> MSPM0 UART
 *
 * PlatformIO 会把同一份源码编译成两个角色：
 *   BRIDGE_SIDE_PC=1 为 PC 端；BRIDGE_SIDE_PC=0 为小车端。
 * 桥接层只搬运原始字节，不解析 PID 命令，所以对上位机表现为普通串口。
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>

extern "C" {
#include <espnow.h>
#include <user_interface.h>
}

#include "bridge_config.h"

namespace {

/* 两端串口均为 115200 8-N-1，无线固定在信道 1。 */
constexpr uint32_t kSerialBaud = 115200;
constexpr uint8_t kWifiChannel = 1;
constexpr uint16_t kPacketMagic = 0x5049;
constexpr uint8_t kPacketVersion = 1;
constexpr size_t kPayloadSize = 200;
constexpr uint8_t kMaxRetries = 3;
constexpr uint32_t kSendCallbackTimeoutMs = 80;
/*
 * UART at 115200 baud needs about 8.7 ms for a 100-byte PID row.  Sending as
 * soon as the first byte arrives turns one row into dozens of ESP-NOW packets
 * and makes a single lost packet destroy the whole row.  Give UART a short
 * coalescing window so commands and compact tune results normally travel in
 * one radio packet, while a live PID row uses at most two packets.
 */
constexpr uint32_t kSerialCoalesceMs = 10;
constexpr uint8_t kDiagnosticCommand[] = "RADIOPING\r\n";
constexpr uint8_t kDiagnosticReply[] = "BRIDGE_RADIO_PONG\r\n";

/* 串口流按最多 200 字节拆包。
 * session 区分模块重启前后的数据，sequence 用于过滤无线重复包。 */
struct __attribute__((packed)) BridgePacket {
  uint16_t magic;
  uint8_t version;
  uint8_t length;
  uint32_t session;
  uint16_t sequence;
  uint8_t payload[kPayloadSize];
};

/* 回调与主循环共享的发送状态。回调写入的标志必须使用 volatile。 */
BridgePacket txPacket = {};
volatile bool sendBusy = false;
volatile bool sendFinished = false;
volatile bool sendSucceeded = false;
uint8_t retryCount = 0;
uint32_t sendStartedMs = 0;
uint32_t localSession = 0;
uint16_t nextSequence = 0;
uint32_t receivedSession = 0;
uint16_t receivedSequence = 0;
bool receivedAnyPacket = false;
bool peerConfigured = false;
uint8_t diagnosticCommandMatch = 0;
#if BRIDGE_SIDE_PC
/* 标记当前发送包是否包含 RADIOPING，便于输出无线发送结果。 */
bool diagnosticTxInFlight = false;
#else
/* 接收回调只置位，真正的诊断应答留给主循环发送。 */
volatile bool diagnosticReplyPending = false;
#endif

/* 全零 MAC 表示尚未配置配对设备。 */
bool isZeroMac(const uint8_t *mac) {
  for (size_t index = 0; index < 6; ++index) {
    if (mac[index] != 0) {
      return false;
    }
  }
  return true;
}

/* 用固定两位十六进制格式打印 MAC，方便首次配对时抄录。 */
void printMac(const uint8_t *mac) {
  for (size_t index = 0; index < 6; ++index) {
    if (index != 0) {
      Serial.print(':');
    }
    if (mac[index] < 0x10) {
      Serial.print('0');
    }
    Serial.print(mac[index], HEX);
  }
}

/* ESP-NOW 发送完成回调：只保存结果，不在 Wi-Fi 回调里执行阻塞操作。 */
void onSend(uint8_t *, uint8_t status) {
  sendSucceeded = status == 0;
  sendFinished = true;
}

/* 逐字节匹配 RADIOPING\r\n，不额外复制整行串口数据。 */
bool trackDiagnosticCommand(uint8_t value) {
  if (value == kDiagnosticCommand[diagnosticCommandMatch]) {
    ++diagnosticCommandMatch;
    if (diagnosticCommandMatch == sizeof(kDiagnosticCommand) - 1) {
      diagnosticCommandMatch = 0;
      return true;
    }
    return false;
  }

  diagnosticCommandMatch = value == kDiagnosticCommand[0] ? 1 : 0;
  return false;
}

/* 验证来源、协议和长度，过滤重复包，再把载荷原样写入本端 UART。 */
void onReceive(uint8_t *mac, uint8_t *data, uint8_t length) {
  if (!peerConfigured || memcmp(mac, kPeerMac, 6) != 0 ||
      length < offsetof(BridgePacket, payload)) {
    return;
  }

  BridgePacket packet = {};
  memcpy(&packet, data, length);
  if (packet.magic != kPacketMagic || packet.version != kPacketVersion ||
      packet.length > kPayloadSize ||
      length != offsetof(BridgePacket, payload) + packet.length) {
    return;
  }

  if (receivedAnyPacket && packet.session == receivedSession &&
      packet.sequence == receivedSequence) {
    return;
  }
  receivedAnyPacket = true;
  receivedSession = packet.session;
  receivedSequence = packet.sequence;

#if !BRIDGE_SIDE_PC
  /* 小车端本地识别 RADIOPING，让无线链路测试不依赖 MSPM0。 */
  bool localDiagnostic = false;
  for (size_t index = 0; index < packet.length; ++index) {
    if (trackDiagnosticCommand(packet.payload[index])) {
      diagnosticReplyPending = true;
      localDiagnostic = true;
    }
  }
  if (localDiagnostic) {
    return;
  }
#endif

  Serial.write(packet.payload, packet.length);
}

/* 启动一次异步发送，最终结果由 onSend() 回填。 */
void startSend() {
  const uint8_t packetLength =
      static_cast<uint8_t>(offsetof(BridgePacket, payload) + txPacket.length);
  sendFinished = false;
  sendSucceeded = false;
  sendBusy = true;
  sendStartedMs = millis();
  if (esp_now_send(const_cast<uint8_t *>(kPeerMac),
                   reinterpret_cast<uint8_t *>(&txPacket),
                   packetLength) != 0) {
    sendFinished = true;
  }
}

/* 处理异步结果；失败时有限重试，避免短暂射频干扰直接丢包。 */
void serviceSendResult() {
  if (!sendBusy) {
    return;
  }

  /* 极少数射频/SDK 异常不会触发发送回调。若一直等下去，sendBusy 会让
   * 整座串口桥永久停止收集新数据；把它按失败处理并走同一有限重试。 */
  if (!sendFinished && millis() - sendStartedMs >= kSendCallbackTimeoutMs) {
    sendSucceeded = false;
    sendFinished = true;
  }
  if (!sendFinished) {
    return;
  }

  if (sendSucceeded || retryCount >= kMaxRetries) {
#if BRIDGE_SIDE_PC
    if (diagnosticTxInFlight) {
      Serial.println(sendSucceeded ? "BRIDGE_RADIO_TX_OK"
                                   : "BRIDGE_RADIO_TX_FAILED");
      diagnosticTxInFlight = false;
    }
#endif
    sendBusy = false;
    sendFinished = false;
    retryCount = 0;
    return;
  }

  ++retryCount;
  delay(1);
  startSend();
}

/* 小车端返回桥接层自己的 PONG，不经过 MSPM0 UART。 */
void serviceDiagnosticReply() {
#if !BRIDGE_SIDE_PC
  if (!diagnosticReplyPending || sendBusy) {
    return;
  }

  diagnosticReplyPending = false;
  txPacket.magic = kPacketMagic;
  txPacket.version = kPacketVersion;
  txPacket.session = localSession;
  txPacket.sequence = nextSequence++;
  txPacket.length = sizeof(kDiagnosticReply) - 1;
  memcpy(txPacket.payload, kDiagnosticReply, txPacket.length);

  retryCount = 0;
  startSend();
#endif
}

/* 收集 UART 当前可用字节并封包；前一包未完成时不会覆盖 txPacket。 */
void collectSerialData() {
  if (!peerConfigured || sendBusy || Serial.available() <= 0) {
    return;
  }

  const uint32_t coalesceStartMs = millis();
  while (Serial.available() < static_cast<int>(kPayloadSize) &&
         millis() - coalesceStartMs < kSerialCoalesceMs) {
    /* Keep the Wi-Fi task alive while UART hardware collects the rest of the
     * current command/result line. */
    delay(0);
  }

  txPacket.magic = kPacketMagic;
  txPacket.version = kPacketVersion;
  txPacket.session = localSession;
  txPacket.sequence = nextSequence++;
  txPacket.length = 0;

  while (Serial.available() > 0 && txPacket.length < kPayloadSize) {
    const uint8_t value = static_cast<uint8_t>(Serial.read());
    txPacket.payload[txPacket.length++] = value;
#if BRIDGE_SIDE_PC
    if (trackDiagnosticCommand(value)) {
      diagnosticTxInFlight = true;
    }
#endif
  }

  retryCount = 0;
  startSend();
}

/* 初始化 STA/ESP-NOW、固定信道、注册回调并加入唯一对端。
 * STA 模式仅用于 ESP-NOW，不会连接路由器或影响电脑上网。 */
void initializeEspNow() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (!wifi_set_channel(kWifiChannel)) {
    Serial.println("BRIDGE_ERROR,SET_CHANNEL");
    return;
  }

  if (esp_now_init() != 0) {
    Serial.println("BRIDGE_ERROR,ESP_NOW_INIT");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  if (isZeroMac(kPeerMac)) {
    Serial.println("BRIDGE_WAITING_FOR_PEER_MAC");
    return;
  }

  if (esp_now_add_peer(const_cast<uint8_t *>(kPeerMac), ESP_NOW_ROLE_COMBO,
                       kWifiChannel, nullptr, 0) != 0) {
    Serial.println("BRIDGE_ERROR,ADD_PEER");
    return;
  }

  peerConfigured = true;
  Serial.print("BRIDGE_READY,");
  Serial.print(kSideName);
  Serial.print(",PEER=");
  printMac(kPeerMac);
  Serial.print(",CHANNEL=");
  Serial.print(wifi_get_channel());
  Serial.println();
}

}  // namespace

/* Arduino 启动入口：初始化串口、生成本次 session 并建立无线链路。 */
void setup() {
  Serial.begin(kSerialBaud);
  /* 车端在转发实时 PID 数据时，ESP-NOW 一包尚未完成的短窗口内 UART
   * 仍可能继续进字节。2 KiB 缓冲可覆盖该窗口，避免把一条 CSV 截成两段。 */
  Serial.setRxBufferSize(2048);
  delay(300);

  localSession = ESP.getChipId() ^ ESP.getCycleCount();
  Serial.println();
  Serial.print("PID_BRIDGE_SIDE,");
  Serial.println(kSideName);
  Serial.print("ESP8266_MAC,");
  Serial.println(WiFi.macAddress());

  initializeEspNow();
}

/* 主循环处理发送结果、诊断应答和新串口数据。
 * delay(0) 只把时间片交给 Wi-Fi 后台任务，不是实际延时。 */
void loop() {
  serviceSendResult();
  serviceDiagnosticReply();
  collectSerialData();
  delay(0);
}
