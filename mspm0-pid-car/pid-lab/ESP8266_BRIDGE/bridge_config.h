/*
 * bridge_config.h - ESP8266 WiFi 桥接模块的对端 MAC 地址与角色配置
 *
 * 部署场景：
 *   - 一台 ESP8266 烧录 BRIDGE_SIDE_PC = 1，接在 PC USB 上
 *   - 另一台 ESP8266 烧录 BRIDGE_SIDE_PC = 0，接在小车上
 *   - 两台 ESP8266 通过 ESP-NOW 互相转发串口数据，
 *     把 MSPM0 的 TTL UART 透明延伸到 PC 上位机
 *
 * 第一次配对时按实际烧录结果替换 kPcModuleMac / kCarModuleMac 即可。
 */
#pragma once

#include <Arduino.h>

// 首次配对时记录的对端 MAC 地址（按实际烧录结果修改）
constexpr uint8_t kPcModuleMac[6] = {0xF4, 0xCF, 0xA2, 0xFE, 0x6B, 0x36};
constexpr uint8_t kCarModuleMac[6] = {0xE0, 0x98, 0x06, 0x15, 0xE4, 0x3C};

// 根据本端角色选择“对端是谁”，同时也用于日志显示
#if BRIDGE_SIDE_PC
constexpr const uint8_t *kPeerMac = kCarModuleMac;
constexpr const char *kSideName = "PC";
#else
constexpr const uint8_t *kPeerMac = kPcModuleMac;
constexpr const char *kSideName = "CAR";
#endif
