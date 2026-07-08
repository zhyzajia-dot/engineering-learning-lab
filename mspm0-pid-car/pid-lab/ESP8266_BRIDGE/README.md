# ESP-01 ESP-NOW UART Bridge

核心文件：

- `ESP8266_BRIDGE.cpp`：两端共用的无线串口桥主程序。
- `bridge_config.h`：PC/小车两块模块的 MAC 地址。
- `platformio.ini`：ESP-01 1 MB 配置及两个固件角色。
- `upload_pc.bat`：编译并烧录 PC 端角色。
- `upload_car.bat`：编译并烧录小车端角色。

两个脚本使用同一份 C++ 源码，只通过 `BRIDGE_SIDE_PC` 编译宏选择角色
和对端 MAC。烧错角色不会损坏 ESP8266，但两端不能按预期通信。

Two ESP-01/ESP-01S modules form a point-to-point wireless UART link. The PC
module stays in a USB programmer and appears as a normal COM port. The car
module connects to the MSPM0 UART.

Both UART sides use 115200 baud, 8-N-1.
Both radios are explicitly locked to Wi-Fi channel 1.

## Link diagnostics

Type `RADIOPING` followed by Enter in the PC serial monitor. This command is
recognized by the car ESP-01 itself and does not depend on the MSPM0 UART:

- `BRIDGE_RADIO_TX_FAILED`: the car ESP-01 did not acknowledge the wireless
  packet; check its power, firmware, MAC address and radio link.
- `BRIDGE_RADIO_TX_OK` followed by `BRIDGE_RADIO_PONG`: the ESP-NOW link works
  in both directions. If the normal `PING` command still has no `PONG`, check
  the car-side UART routing, TX/RX wiring and MSPM0 firmware.

## First pairing

1. Upload `pc_bridge` to the first module and record `ESP8266_MAC`.
2. Upload `car_bridge` to the second module and record `ESP8266_MAC`.
3. Put both addresses into `bridge_config.h`.
4. Upload `pc_bridge` again to the PC module.
5. Upload `car_bridge` again to the car module.

The supported USB programmer has an automatic-download circuit. Its only
button is RST; do not hold it during upload.

## Car wiring

| ESP-01S | MSPM0 |
| --- | --- |
| TXD | PA11 RX |
| RXD | PA10 TX |
| GND | GND |
| VCC/3V3 | Stable 3.3 V |
| EN/CH_PD | 3.3 V high |

The ESP-01 supply must be a stable 3.3 V source capable of at least 500 mA.
Never apply 5 V to ESP-01 VCC or UART pins.
