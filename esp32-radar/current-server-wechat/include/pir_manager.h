/**
 * @file pir_manager.h
 * @brief HC-SR501 人体红外检测模块接口。
 *
 * 硬件连接：
 * - VCC -> 板载 5V
 * - OUT -> ESP32-S3 GPIO44
 * - GND -> 板载 GND
 *
 * GPIO44 原为 UART0 RX。应用启动后 UART0 被重新配置为仅保留
 * GPIO43 TX，因此 GPIO44 可作为普通数字输入读取红外模块状态。
 */
#ifndef PIR_MANAGER_H
#define PIR_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// HC-SR501 OUT 接入 GPIO44，高电平表示检测到人体活动。
constexpr uint8_t PIR_INPUT_PIN = 44;

// 模块上电后需要等待热释电传感器和内部电路稳定。
constexpr unsigned long PIR_WARMUP_TIME_MS = 30000;

// 连续稳定达到该时间后才确认状态变化，抑制边沿抖动和瞬态干扰。
constexpr unsigned long PIR_DEBOUNCE_TIME_MS = 150;

/**
 * @brief PIR 状态快照。
 *
 * revision 每次有效状态发生变化时递增，供 TuyaLink 判断是否需要
 * 立即上报，避免不同任务直接共享“已变化”标志而发生竞争。
 */
struct PirSnapshot {
    bool presence;
    bool valid;
    uint32_t revision;
    unsigned long updatedAt;
};

void initPirManager();
void pirTask(void* parameter);
bool getPirSnapshot(PirSnapshot& snapshot);

#endif
