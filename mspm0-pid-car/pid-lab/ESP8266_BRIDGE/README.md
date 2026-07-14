# ESP8266 ESP-NOW 串口桥

两块 ESP-01/ESP-01S 把 PC 上位机的串口透明延伸到小车 MSPM0。当前版本可以通信，但实测仍存在秒级积压、重复交付和诊断命令泄漏，**尚未达到自动调参所需的可靠性**。完整故障证据与验收流程见工程根目录 `README.md`。

## 角色与接线

- `pc_bridge`：插在电脑 USB 下载器/串口板上。
- `car_bridge`：装在车上，与 MSPM0 应用 UART 相连。

| ESP8266 | MSPM0G3507 | 说明 |
|---|---|---|
| TX / GPIO1 | PB16 / UART RX | ESP 发给 MCU |
| RX / GPIO3 | PB15 / UART TX | MCU 发给 ESP |
| GND | GND | 必须共地 |
| VCC、EN/CH_PD | 稳定 3.3V | 不可接 5V |

串口为 115200、8-N-1。`PA10/PA11` 只用于 MSPM0 BSL 烧录，不连接车端 ESP8266。

两块模块的 MAC 位于 `bridge_config.h`：

```cpp
kPcModuleMac
kCarModuleMac
```

更换模块后必须更新实际 MAC，并同时重建两个角色固件。

## 构建与烧录

```powershell
# 同时构建两个角色
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d .\ESP8266_BRIDGE

# 一键烧录电脑端
.\ESP8266_BRIDGE\upload_pc.bat

# 一键烧录小车端
.\ESP8266_BRIDGE\upload_car.bat
```

生成文件：

- `.pio/build/pc_bridge/firmware.bin`
- `.pio/build/car_bridge/firmware.bin`

ESP-01/ESP-01S 下载接线：稳定 3.3V 供电，`EN/CH_PD` 拉高，`GPIO0` 拉低后复位进入下载模式。烧录结束后释放 `GPIO0` 并重新上电。PC/车端角色不要烧反。

## 当前实现与已知限制

当前代码使用约 10 ms 的 UART 聚合窗口，再把字节块送入 ESP-NOW；另有发送忙看门限和有限重试。这能减少逐字节小包，但不能提供完整的端到端可靠性。

已观察到：

- 多次 `HELLO` 在 3～4 秒后成批返回。
- 同一 token 的回复重复出现。
- `RADIOPING` 虽返回 `BRIDGE_RADIO_PONG`，仍有残留字节到达 MSPM0，产生 `ERR COMMAND`。

因此不要仅靠继续调大 GUI 超时解决。推荐增加包含 `session + sequence + length + CRC` 的无线帧、ACK/超时重传、接收去重与按序交付；桥本地诊断使用独立帧类型，不能混入应用串口数据。

## 修复后的最低验收

1. 100 次 `RADIOPING`：零丢失，MSPM0 零 `ERR COMMAND`。
2. 100 次带 token 的 `HELLO`：零丢失、零重复、零乱序。
3. 请求不再数秒积压后成批到达。
4. 连续转发 10 分钟遥测，无粘行、拆行、乱码或永久发送忙。
5. `STOP` 始终优先传输，并得到确定回复。

桥接验证未通过前，不要使用无线链路运行自动轮速或循迹调参；可以先用 USB-TTL 直连 `PB15/PB16` 验证 MSPM0 与上位机。
