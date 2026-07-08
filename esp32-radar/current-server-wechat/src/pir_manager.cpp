/**
 * @file pir_manager.cpp
 * @brief HC-SR501 红外人体检测的采样、预热和消抖实现。
 *
 * 后台任务读取 GPIO44，并向涂鸦通信模块提供线程安全的状态快照。
 */
#include "pir_manager.h"

#include <freertos/portmacro.h>

namespace {

// PIR 快照由 pirTask 写入，TuyaLink 任务读取，使用临界区保证整包一致。
PirSnapshot pirSnapshot = {false, false, 0, 0};
portMUX_TYPE pirSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief 更新全局 PIR 快照。
 * @param presence 当前是否检测到人体活动。
 * @param valid 模块是否已完成上电稳定。
 * @param stateChanged 是否为有效状态变化；变化时递增 revision。
 */
void updatePirSnapshot(bool presence, bool valid, bool stateChanged) {
    taskENTER_CRITICAL(&pirSnapshotMux);
    pirSnapshot.presence = presence;
    pirSnapshot.valid = valid;
    pirSnapshot.updatedAt = millis();
    if (stateChanged) {
        pirSnapshot.revision++;
    }
    taskEXIT_CRITICAL(&pirSnapshotMux);
}

}  // namespace

void initPirManager() {
    // HC-SR501 OUT 为主动输出的 0V/约3V数字信号，不启用内部上下拉。
    pinMode(PIR_INPUT_PIN, INPUT);
    updatePirSnapshot(false, false, false);
    Serial.printf("[PIR] GPIO%u initialized, waiting %lu ms for warm-up\n",
                  PIR_INPUT_PIN,
                  PIR_WARMUP_TIME_MS);
}

bool getPirSnapshot(PirSnapshot& snapshot) {
    taskENTER_CRITICAL(&pirSnapshotMux);
    snapshot = pirSnapshot;
    taskEXIT_CRITICAL(&pirSnapshotMux);
    return snapshot.valid;
}

void pirTask(void* parameter) {
    (void)parameter;
    Serial.println("[PIR] sampling task started");

    // 上电稳定期间不向云端上报，避免把传感器初始化脉冲当成有人。
    vTaskDelay(pdMS_TO_TICKS(PIR_WARMUP_TIME_MS));

    bool stableState = digitalRead(PIR_INPUT_PIN) == HIGH;
    bool candidateState = stableState;
    unsigned long candidateSince = millis();
    updatePirSnapshot(stableState, true, true);
    Serial.printf("[PIR] warm-up complete, initial state: %s\n",
                  stableState ? "presence" : "clear");

    while (true) {
        const bool sampledState = digitalRead(PIR_INPUT_PIN) == HIGH;
        const unsigned long now = millis();

        if (sampledState != candidateState) {
            candidateState = sampledState;
            candidateSince = now;
        }

        if (candidateState != stableState &&
            now - candidateSince >= PIR_DEBOUNCE_TIME_MS) {
            stableState = candidateState;
            updatePirSnapshot(stableState, true, true);
            Serial.printf("[PIR] state changed: %s\n",
                          stableState ? "presence" : "clear");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
