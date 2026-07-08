# Engineering Learning Lab

This repository collects learning and engineering projects.

## Projects

- `esp32-radar/`: ESP32-S3 radar vital-sign firmware. The default build uploads to the self-hosted server for the WeChat mini program, and alternate PlatformIO environments build the TuyaLink and legacy MQTT variants.
- `k230-pid-gimbal/`: K230 visual gimbal tracking and PID auto-tuning project, including the PC tuning GUI.
- `mspm0-pid-car/pid-lab/`: MSPM0G3507 PID lab firmware, host auto-tuning tool, and ESP8266 wireless serial bridge.
- `mspm0-pid-car/working-car/`: working MSPM0G3507 car firmware used as the race/project baseline.

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

## Notes

Generated folders such as `.pio/`, `Debug/`, `logs/`, `__pycache__/`, build outputs, and compressed archives are intentionally ignored. Rebuild dependencies locally with PlatformIO, Code Composer Studio, or the relevant Python requirements files.
