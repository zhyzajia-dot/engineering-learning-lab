# ESP32 Radar Firmware Versions

This folder keeps the important ESP32 radar firmware versions side by side.

## Folders

- `current-server-wechat/`: current main firmware. Default build uploads radar data to the self-hosted HTTP server for the WeChat mini program. It also keeps alternate PlatformIO environments for TuyaLink and legacy MQTT.
- `tuya-tuyalink/`: archived TuyaLink-focused firmware version.
- `legacy-mqtt-influxdb/`: archived legacy MQTT + InfluxDB firmware version.

## Current Main Build

Use `current-server-wechat/` for normal development.

```powershell
cd F:\engineering-learning-lab\esp32-radar\current-server-wechat

# Self-hosted server + WeChat mini program
platformio run -e freenove_esp32_s3_wroom

# TuyaLink build from the current codebase
platformio run -e tuya_freenove_esp32_s3_wroom

# Legacy MQTT + InfluxDB build from the current codebase
platformio run -e legacy_freenove_esp32_s3_wroom
```

The `tuya-tuyalink/` and `legacy-mqtt-influxdb/` folders are kept for comparing older versions and recovering code if needed.
