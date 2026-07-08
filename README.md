# Engineering Learning Lab

This repository collects learning and engineering projects.

## Projects

- `esp32-radar/`: ESP32-S3 radar vital-sign firmware. The default build uploads to the self-hosted server for the WeChat mini program, and alternate PlatformIO environments build the TuyaLink and legacy MQTT variants.
- `k230-pid-gimbal/`: planned location for the K230 visual gimbal PID project.
- `mspm0-pid-car/`: planned location for the MSPM0 PID car projects.

## ESP32 Builds

From `esp32-radar/`:

```powershell
# Self-hosted server + WeChat mini program
platformio run -e freenove_esp32_s3_wroom

# TuyaLink
platformio run -e tuya_freenove_esp32_s3_wroom

# Legacy MQTT + InfluxDB
platformio run -e legacy_freenove_esp32_s3_wroom
```

## Daily Git Flow

```powershell
git pull
git status
git add .
git commit -m "describe the change"
git push
```
