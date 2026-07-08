# ESP32 Radar Firmware

这里保存 ESP32 雷达固件的三个重要版本。

## 版本说明

- `current-server-wechat/`: 当前主线。默认编译为自有服务器 HTTP 上传 + 微信小程序版本，同时保留涂鸦和旧 MQTT 的编译环境。
- `tuya-tuyalink/`: 涂鸦 TuyaLink 阶段的归档版本。
- `legacy-mqtt-influxdb/`: 旧 MQTT + InfluxDB 阶段的归档版本。

日常开发优先改：

```powershell
F:\engineering-learning-lab\esp32-radar\current-server-wechat
```

## 当前主线怎么编译

进入目录：

```powershell
cd F:\engineering-learning-lab\esp32-radar\current-server-wechat
```

自有服务器 + 微信小程序版：

```powershell
platformio run -e freenove_esp32_s3_wroom
```

涂鸦 TuyaLink 版：

```powershell
platformio run -e tuya_freenove_esp32_s3_wroom
```

旧 MQTT + InfluxDB 版：

```powershell
platformio run -e legacy_freenove_esp32_s3_wroom
```

## 烧录和串口

烧录默认微信小程序版：

```powershell
platformio run -e freenove_esp32_s3_wroom -t upload
```

打开串口监视器：

```powershell
platformio device monitor -b 115200
```

## 什么时候看旧版本

`tuya-tuyalink/` 和 `legacy-mqtt-influxdb/` 主要用于：

- 对比以前怎么写的；
- 找回旧功能；
- 判断某个问题是不是新版本引入的；
- 必要时从旧版本复制代码到当前主线。

正常开发不要直接在归档版本里改新功能，优先改 `current-server-wechat/`。

## 注意

`include/tuya_credentials.h` 被 `.gitignore` 忽略，不会上传。朋友如果要编译涂鸦版本，需要自己在本地补对应凭据文件。
