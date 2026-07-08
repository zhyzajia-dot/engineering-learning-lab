/**
 * @file server_link.cpp
 * @brief 将睡眠雷达数据通过 HTTP JSON 上传到自有服务器。
 */
#include "server_link.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <freertos/semphr.h>

#include "pir_manager.h"
#include "radar_manager.h"
#include "tasks_manager.h"

namespace {

constexpr char RADAR_UPLOAD_URL[] =
    "http://47.86.10.121:8080/api/radar/upload";
constexpr unsigned long DAILY_UPLOAD_INTERVAL_MS = 2000;
constexpr unsigned long IDLE_UPLOAD_INTERVAL_MS = 10000;
constexpr unsigned long RETRY_INTERVAL_MS = 5000;
constexpr uint16_t HTTP_CONNECT_TIMEOUT_MS = 3000;
constexpr uint16_t HTTP_TIMEOUT_MS = 5000;

unsigned long lastUploadAttempt = 0;
bool serverAvailable = false;
bool serverStatusKnown = false;
uint32_t lastPirRevisionUploaded = 0;
SemaphoreHandle_t uploadMutex = nullptr;

String getDeviceId() {
    if (device_sn > 0) {
        char value[24];
        snprintf(value, sizeof(value), "%llu",
                 static_cast<unsigned long long>(device_sn));
        return String(value);
    }

    String mac = getDeviceMacAddress();
    mac.replace(":", "");
    return mac;
}

const char* algorithmSleepState(int state) {
    switch (state) {
        case 2:
            return "awake";
        case 3:
            return "light_sleep";
        case 4:
            return "deep_sleep";
        case 5:
            return "rem_sleep";
        default:
            return "unknown";
    }
}

const char* radarSleepState(uint8_t state) {
    switch (state) {
        case 0:
            return "deep_sleep";
        case 1:
            return "light_sleep";
        case 2:
            return "awake";
        default:
            return "unknown";
    }
}

const char* emotionCode(EmotionType emotion) {
    switch (emotion) {
        case EMOTION_CALM:
            return "calm";
        case EMOTION_HAPPY:
            return "happy";
        case EMOTION_EXCITED:
            return "excited";
        case EMOTION_ANXIOUS:
            return "anxious";
        case EMOTION_ANGRY:
            return "angry";
        case EMOTION_SAD:
            return "sad";
        case EMOTION_STRESSED:
            return "stressed";
        case EMOTION_RELAXED:
            return "relaxed";
        default:
            return "unknown";
    }
}

void updateServerStatus(bool available) {
    if (serverStatusKnown && available == serverAvailable) {
        return;
    }

    serverStatusKnown = true;
    serverAvailable = available;
    pushDeviceStatusIfChanged(
        available
            ? BleProto::DeviceStatus::DEV_MQTT_CONNECTED
            : BleProto::DeviceStatus::DEV_MQTT_FAILED);
}

bool uploadCurrentData() {
    if (uploadMutex == nullptr ||
        xSemaphoreTake(uploadMutex, 0) != pdTRUE) {
        return false;
    }

    if (!WiFi.isConnected()) {
        if (serverAvailable) {
            serverAvailable = false;
            pushDeviceStatusIfChanged(
                BleProto::DeviceStatus::DEV_MQTT_DISCONNECTED);
        }
        xSemaphoreGive(uploadMutex);
        return false;
    }

    const SensorData radar = sensorData;
    PirSnapshot pir = {};
    const bool pirValid = getPirSnapshot(pir);

    SleepAnalysisSnapshot sleep = {};
    const bool algorithmValid = getFreshSleepAnalysisSnapshot(sleep);

    EmotionResult emotion = {};
    const bool emotionValid = getFreshEmotionResult(emotion);

    JsonDocument document;
    document["deviceId"] = getDeviceId();
    document["pirPresence"] = pirValid ? pir.presence : false;
    document["pirValid"] = pirValid;
    document["radarPresence"] = radar.presence != 0;
    document["heartRate"] =
        constrain(static_cast<int>(radar.heart_rate + 0.5f), 0, 200);
    document["breathRate"] =
        constrain(static_cast<int>(radar.breath_rate + 0.5f), 0, 60);
    document["distanceCm"] =
        constrain(static_cast<int>(radar.distance), 0, 1000);
    document["bodyMovement"] =
        constrain(static_cast<int>(radar.body_movement), 0, 100);
    document["motionStatus"] = radar.motion;
    document["bedStatus"] =
        algorithmValid
            ? ((sleep.algorithm_state >= 1 && sleep.algorithm_state <= 5)
                   ? "in_bed"
                   : "out_of_bed")
            : (radar.bed_status ? "in_bed" : "out_of_bed");
    document["sleepState"] =
        algorithmValid
            ? algorithmSleepState(sleep.algorithm_state)
            : radarSleepState(radar.sleep_state);
    document["sleepScore"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.total_score + 0.5f), 0, 100)
            : constrain(static_cast<int>(radar.sleep_score), 0, 100);
    document["sleepAlgorithmValid"] = algorithmValid;
    document["sleepinessPercent"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.current_sleepiness * 100.0f + 0.5f),
                        0,
                        100)
            : 0;
    document["totalSleepMinutes"] =
        algorithmValid ? sleep.total_sleep_time / 60000UL : 0;
    document["deepSleepMinutes"] =
        algorithmValid ? sleep.deep_sleep_time / 60000UL : 0;
    document["lightSleepMinutes"] =
        algorithmValid ? sleep.light_sleep_time / 60000UL : 0;
    document["remSleepMinutes"] =
        algorithmValid ? sleep.rem_sleep_time / 60000UL : 0;
    document["awakeMinutes"] =
        algorithmValid ? sleep.awake_time / 60000UL : 0;
    document["outOfBedMinutes"] =
        algorithmValid ? sleep.out_of_bed_time / 60000UL : 0;
    document["sleepLatencyMinutes"] =
        algorithmValid ? sleep.sleep_latency / 60000UL : 0;
    document["wakeCount"] = algorithmValid ? sleep.wake_count : 0;
    document["sleepCycles"] = algorithmValid ? sleep.sleep_cycles : 0;
    document["durationScore"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.duration_score + 0.5f), 0, 100)
            : 0;
    document["deepScore"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.deep_score + 0.5f), 0, 100)
            : 0;
    document["continuityScore"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.continuity_score + 0.5f), 0, 100)
            : 0;
    document["physiologyScore"] =
        algorithmValid
            ? constrain(static_cast<int>(sleep.physiology_score + 0.5f), 0, 100)
            : 0;

    document["emotionValid"] = emotionValid;
    document["primaryEmotion"] =
        emotionValid ? emotionCode(emotion.primaryEmotion) : "unknown";
    document["secondaryEmotion"] =
        emotionValid ? emotionCode(emotion.secondaryEmotion) : "unknown";
    document["emotionConfidence"] =
        emotionValid
            ? constrain(static_cast<int>(emotion.confidence * 100.0f + 0.5f),
                        0,
                        100)
            : 0;
    document["emotionIntensity"] =
        emotionValid
            ? constrain(static_cast<int>(emotion.intensity * 100.0f + 0.5f),
                        0,
                        100)
            : 0;
    document["stressLevel"] =
        emotionValid
            ? constrain(static_cast<int>(emotion.stressLevel + 0.5f), 0, 100)
            : 0;
    document["anxietyLevel"] =
        emotionValid
            ? constrain(static_cast<int>(emotion.anxietyLevel + 0.5f), 0, 100)
            : 0;
    document["relaxationLevel"] =
        emotionValid
            ? constrain(static_cast<int>(emotion.relaxationLevel + 0.5f), 0, 100)
            : 0;
    document["dataSource"] = algorithmValid ? "algorithm" : "radar";
    document["uptimeMs"] = millis();

    String payload;
    serializeJson(document, payload);

    HTTPClient http;
    http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.setReuse(false);

    if (!http.begin(RADAR_UPLOAD_URL)) {
        Serial.println("[Server] failed to initialize HTTP client");
        updateServerStatus(false);
        xSemaphoreGive(uploadMutex);
        return false;
    }

    http.addHeader("Content-Type", "application/json");
    http.addHeader("Connection", "close");

    esp_task_wdt_reset();
    const int statusCode = http.POST(payload);
    const String response = statusCode > 0 ? http.getString() : String();
    http.end();
    esp_task_wdt_reset();

    const bool success = statusCode >= 200 && statusCode < 300;
    updateServerStatus(success);

    if (success) {
        if (pirValid) {
            lastPirRevisionUploaded = pir.revision;
        }
        Serial.printf("[Server] upload succeeded, HTTP %d\n", statusCode);
    } else {
        Serial.printf("[Server] upload failed, HTTP %d, response=%s\n",
                      statusCode,
                      response.c_str());
    }
    xSemaphoreGive(uploadMutex);
    return success;
}

}  // namespace

TaskHandle_t mqttTaskHandle = nullptr;

void sendDailyDataToMQTT() {
    uploadCurrentData();
}

bool sendSleepDataToMQTT(bool allowSessionEnd) {
    (void)allowSessionEnd;
    return uploadCurrentData();
}

void sendHeartbeatToMQTT() {
    uploadCurrentData();
}

void mqttTask(void* parameter) {
    (void)parameter;
    Serial.printf("[Server] HTTP upload task started: %s\n", RADAR_UPLOAD_URL);
    uploadMutex = xSemaphoreCreateMutex();
    if (uploadMutex == nullptr) {
        Serial.println("[Server] failed to create upload mutex");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        esp_task_wdt_reset();

        if (WiFi.isConnected()) {
            const unsigned long now = millis();
            const SensorData radar = sensorData;
            const bool noPerson = !radar.presence ||
                                  (radar.heart_rate <= 0 &&
                                   radar.breath_rate <= 0);
            const unsigned long interval =
                noPerson ? IDLE_UPLOAD_INTERVAL_MS
                         : DAILY_UPLOAD_INTERVAL_MS;

            PirSnapshot pir = {};
            const bool pirChanged =
                getPirSnapshot(pir) &&
                pir.revision != lastPirRevisionUploaded;

            if (pirChanged ||
                now - lastUploadAttempt >=
                    (serverAvailable ? interval : RETRY_INTERVAL_MS)) {
                lastUploadAttempt = now;
                uploadCurrentData();
            }
        } else if (serverAvailable) {
            serverAvailable = false;
            pushDeviceStatusIfChanged(
                BleProto::DeviceStatus::DEV_MQTT_DISCONNECTED);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
