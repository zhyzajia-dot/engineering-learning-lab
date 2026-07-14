# MSPM0G3507 小车自动调参工程

## 交接状态（2026-07-15，换电脑前必读）

### 现在到底有哪两个 HEX

Debug 目录里有多个历史文件，但用途不同，不能混用：

| 文件 | 用途 | 现在是否烧录 |
|---|---|---|
| `Debug/i2c_address_scan.hex` | 诊断固件。等待约6秒后循环扫描 `0x08~0x77`，通过串口输出 `I2C_FOUND,<十进制地址>`；不会运行正常小车控制。 | 只有排查 IMU 地址时烧录 |
| `Debug/pid_lab_mspm0.hex` | 当前正常工程固件。包含修正后的 IMU repeated-START 读流程、5秒 IMU 上电等待、灰度/IMU诊断和 ESP8266 `RADIOPING` 处理。 | 日常使用烧录这个 |

`Debug/pid_lab_mspm0_repeated_start.hex` 是同一版正常固件的旧副本，不需要再选；`Debug/i2c_pin_selftest.hex` 是更早的 PB2/PB3 方波测试，只能在拔掉两个 I²C 模块时使用，也不是小车固件。

### 已经确认的事实

1. MSPM0 的 I²C 引脚配置为 `PB3=SDA`、`PB2=SCL`，灰度传感器和 IMU 共用 I²C1。
2. 灰度传感器地址 `0x5D` 已经在逻辑分析仪中得到完整 ACK，灰度模块通信正常。
3. IMU 例程使用的 7 位地址是 `0x23`；此前抓包中 `0x46`（`0x23<<1`）反复 NACK，说明地址阶段尚未得到 IMU 应答。
4. IMU 资料要求上电后约等待5秒；当前正常固件已加入等待，不会在启动阶段访问 IMU。
5. IMU 使用左侧 `5V` 供电是允许的。右侧 I²C 信号必须按名称连接：`PB3->SDA`、`PB2->SCL`、`GND->GND`。不要按排针物理顺序直接插线。
6. 灰度模块接回后 SDA 空闲恢复3.3V，说明灰度板提供了总线上拉；IMU单独连接时 SDA 约1.2V不能直接当作模块损坏证据。

### 明天的最短验证流程

1. 保持灰度模块和 IMU 都接好，先烧录 `Debug/i2c_address_scan.hex`。
2. 复位后等待6秒，在串口终端或上位机日志中查找：
   - `I2C_FOUND,93` = 灰度地址 `0x5D`；
   - `I2C_FOUND,35` = IMU 地址 `0x23`。
3. 如果只找到 `93`，把扫描输出或新的 `.sr` 文件保存下来；此时不要继续改寄存器和 PID，问题仍在 IMU 地址/接口应答层。
4. 如果找到 `35`，改烧 `Debug/pid_lab_mspm0.hex`，等待5秒后再执行上位机的“检查 IMU”。
5. 逻辑分析仪采集时使用公共 GND，`D0/CH0=SCL(PB2)`、`D1/CH1=SDA(PB3)`；不要拔掉灰度模块，否则可能同时拔掉总线上拉。

### 不要再做的事情

- 不要把 `i2c_address_scan.hex` 当作最终小车固件使用。
- 不要把 `i2c_pin_selftest.hex` 在模块仍连接时烧录；它会临时把 PB2/PB3 配成推挽 GPIO。
- 不要在没有测量依据时改 IMU 地址、寄存器地址或增加5V上拉；I²C上拉只能接3.3V。
- 不要在 I²C/串口尚未稳定时运行 PID 自动调参，先确认日志能持续得到 `SENSOR,OK` 和 `IMU,OK`。

### 换电脑后的构建

仓库已包含 `build_mspm0_hex.ps1`。在安装好 TI MSPM0 SDK、SysConfig 和 TI ARM Clang 的电脑上，在本目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1 -OutputHex 'Debug\pid_lab_mspm0.hex'
```

构建诊断扫描固件：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1 -OutputHex 'Debug\i2c_address_scan.hex' -Defines 'I2C_ADDRESS_SCAN_DIAG_ENABLE=1'
```

> 这是一份面向协作者的交接说明。当前版本还没有达到“整车自动调参稳定可用”的验收状态。请先恢复通信与传感器读取的确定性，再继续调整 PID 算法；否则调参结果没有可信度。

## 1. 当前结论与处理优先级

目前不是单一 PID 参数不合适，而是存在几个会直接破坏自动调参的数据链问题：

1. **ESP8266 无线串口桥会延迟、成批返回和重复返回。** 最近一次 `HELLO` 测试中，上位机约每 650 ms 重发一次，回复却在 3～4 秒后一次性返回多份，导致握手已超时但旧响应仍继续到达。
2. **同一次实测中灰度与 IMU 都返回过读取错误。** 两个器件共用 I²C1，因此应先检查总线初始化、上电等待、事务恢复和寄存器读时序，不能先认定两个硬件同时损坏。
3. **轮速自动调参依赖前两项稳定。** 只要命令/结果丢失、延迟或重复，上位机就会出现“无 STEP 结果”“没有足够速度点”“没有本地 PWM 结果”等异常，算法无法判断真实电机响应。

建议按这个顺序处理：

1. 用 USB-TTL 直连 `PB15/PB16` 绕过 ESP8266，证明 MSPM0 协议和电机测试本身稳定。
2. 单独修复并压测两块 ESP8266 的端到端通信。
3. 在电机停止状态下恢复 I²C 灰度与 IMU 的连续读取。
4. 校验编码器比例、方向和低速分辨率。
5. 最后再运行轮速 PI、循迹 PD 和方形赛道自动调参。

在第 1～4 项通过前，不要通过不断改变 PID 参数来掩盖通信或采样故障。

## 2. 系统结构

```text
Python 上位机
    │ USB 串口，115200
电脑端 ESP8266
    │ ESP-NOW
小车端 ESP8266
    │ UART：ESP TX -> PB16，ESP RX <- PB15
MSPM0G3507
    ├─ TB6612 -> 左右电机
    ├─ 编码器 -> PA14/PA15、PB12/PB13
    └─ I²C1 PB3/PB2 -> 8 路灰度 + IMU
```

正常应用通信与 BSL 烧录是两套接口：

| 用途 | 引脚 | 波特率 | 说明 |
|---|---|---:|---|
| 应用上位机通信 | PB15 TX / PB16 RX | 115200 | 接车端 ESP8266 或 USB-TTL |
| BSL 烧录 | PA10 BSL_TX / PA11 BSL_RX | 9600 | 只用于烧录，不接 ESP8266 |

## 3. 已确认的硬件事实

以下结论来自用户实测、器件资料和当前 PCB 接线，不应再次当成未知项：

### 8 路灰度巡线传感器

- 供电：**5V，当前实际接线也为 5V**。
- I²C 7 位地址：**`0x5D`，已经确认正确**。
- I²C 接口标注顺序：`SCL / SDA / GND / 5V`。
- 资料给出的工作电流约 85 mA，推荐检测高度 10～40 mm，适用线宽约 0.6～6.9 cm。
- 传感器必须在实际安装高度、实际地面和实际黑线条件下完成自学习标定。
- 用户已测得 SCL、SDA 空闲电压均为 **3.3V**。这与 3.3V 逻辑上拉一致；器件供电仍是 5V，两者不要混淆。

### IMU

- I²C 7 位地址：**`0x23`，已经确认正确**。
- 模块资料支持 I²C 与 UART；I²C 速率为 100 kHz，模块可使用 3.3V 或 5V 供电。
- 资料标出的启动时间为约 **5000 ms**，冷启动或整车重新上电后，首次访问前必须留出足够时间。
- 当前代码上电等待 5000 ms 后先读 `0x01` 的 3 字节固件版本，再用“写寄存器地址 + repeated START + 连续读”的事务从 `0x26` 读取 12 字节，并按 3 个 little-endian `float` 解释欧拉角；yaw 取最后 4 字节并由弧度换算为 0.1°。

### I²C 总线

- 当前 SysConfig：`PB3 = SDA`，`PB2 = SCL`，灰度和 IMU 共用 I²C1。
- SCL/SDA 空闲为 3.3V 说明线上存在上拉且没有持续被拉低，但不能单独证明每次 I²C 事务均成功。
- 旧工程今天早上曾正常工作，因此排查重点应放在新旧板迁移后的引脚配置、初始化顺序、上电等待、事务超时与失败恢复，不应直接判定传感器硬件损坏。

## 4. 当前故障证据

原始运行日志不作为普通代码提交上传；下面保留足够复现问题的摘要。

### 4.1 无线桥延迟与重复

最近一次失败流程：

```text
05:51:44.286 TX HELLO 895551582
05:51:44.941 TX HELLO 895551582
05:51:45.590 TX HELLO 895551582
05:51:46.251 TX HELLO 895551582
05:51:46.902 TX HELLO 895551582
05:51:47.596 TX STOP
05:51:48.019 RX OK STOP
05:51:48.019 RX HELLO,...   # 同一回复连续出现 3 次
05:51:49.243 RX HELLO,...   # 又连续出现 2 次
```

这证明 MSPM0 实际产生了正确回复，但桥接链路没有及时、单次、有序地交付。当前桥虽然增加了 UART 聚合窗口和发送看门限，仍没有完整的端到端序号、确认、去重和顺序保证。

`RADIOPING` 还出现过：

```text
RX BRIDGE_RADIO_TX_OK
RX BRIDGE_RADIO_PONG
RX ERR COMMAND; SEND HELP
```

最后一行说明桥的本地诊断命令仍有字节泄漏到 MSPM0。

### 4.2 同总线 I²C 错误

一次诊断返回：

```text
SENSOR,ERROR,0,4,0,2675,0
IMU,ERROR,0,0,2743
```

两个地址已经确认正确，灰度也确认使用 5V 供电。接下来应抓取 I²C 波形或增加总线扫描/分阶段错误码，定位是 NACK、超时、总线忙还是寄存器读取失败。

### 4.3 自动轮速调参失败表现

近期上位机记录过：

```text
No reliable HELLO response
L: no local PWM result for 55
L: no local STEP result
L: encoder did not provide enough usable speed points
```

这些错误目前首先指向通信结果未按时到达或编码器采样无有效点，不足以证明自动调参数学模型本身错误。

## 5. 给协作者的最短复现流程

### A. 先绕过 ESP8266

1. 两个驱动轮悬空，关闭电机主电源或保持紧急停止可用。
2. USB-TTL 只接 `GND`、`TX -> PB16`、`RX -> PB15`，不要接 TTL 的 VCC。
3. 以 115200、8-N-1 打开串口。
4. 依次发送 `PING`、`HELLO <任意数字>`、`STATUS`、`SENSOR`、`IMU`、`STOP`。
5. 连续执行 100 次 `PING`/`HELLO`，确认无丢失、无重复、无乱序，再短测 `PWM L 120`、`PWM R 120`。

若直连仍失败，先修 MSPM0 串口/协议；若直连稳定而无线失败，问题就限定在 ESP8266 桥。

### B. 单测 ESP8266 桥

1. PC 端与车端必须烧录同一次构建生成的对应角色固件。
2. 核对 `ESP8266_BRIDGE/bridge_config.h` 中两块实际 MAC。
3. 不接电机，连续运行 100 次 `RADIOPING`，再运行 100 次带 token 的 `HELLO`。
4. 记录每条请求的序号、发送时刻、接收时刻、重试次数和重复次数。

桥接最低验收标准：

- 100 次请求零丢失、零重复、零乱序。
- 普通请求绝大多数在 100 ms 内完成，任何一次不得拖到数秒后成批返回。
- `RADIOPING` 完全由桥消费，不得进入 MSPM0。
- 连续遥测 10 分钟不出现粘行、拆行、乱码或永久 `sendBusy`。

旧的 `uart_out_in2.zip` 使用的是 ESP32-C3、UART1 4800 baud、按行聚合后定长发送。它不能直接作为 ESP8266 固件烧录，但“完整行再发送、清晰分帧”的思路值得保留。

### C. 单测 I²C

1. 整车彻底断电 10 秒后重新上电，不只按复位键。
2. 确认灰度板 5V、两块模块共地，SDA/SCL 空闲仍约 3.3V。
3. 上电后至少等待 5 秒再访问 IMU。
4. 电机保持停止，分别连续读取灰度 `0x5D` 和 IMU `0x23` 各 1000 次，统计成功、NACK、BUSY、TIMEOUT 和恢复次数。
5. 若失败，使用逻辑分析仪确认 START、地址、ACK、寄存器字节、重复 START 与 STOP；不要只看静态电压。

建议把 I²C 访问统一到一个总线层：失败时先发 STOP，必要时将 SCL 切 GPIO 输出 9 个恢复脉冲，再重新初始化 I²C；灰度与 IMU 不应各自无协调地重置同一外设。

### D. 最后测电机与编码器

1. 两轮必须悬空。
2. 左、右轮分别以低 PWM 短测，核对正方向和编码器计数方向。
3. 从静止开始逐级升 PWM，确认速度随 PWM 基本单调。
4. 核对实际电机 PPR、减速比与轮径。当前换算为 13 脉冲/电机圈、20:1、48 mm 轮径、只计 A 相上升沿，即 `0.580 mm/count`。
5. 只有通信和计数稳定后，才运行整车轮速自动调参。

## 6. 推荐的修复设计

### 6.1 ESP8266 不再做无状态透明字节管道

建议给无线层增加小型帧协议：

```text
magic | version | type | session | sequence | length | payload | CRC
```

- 每个方向独立递增 `sequence`。
- 接收端校验 CRC、去重并按序交付。
- 发送端收到明确 ACK 才释放帧，超时只重发同一序号。
- 重连或重启改变 `session`，防止旧包在新任务中生效。
- ESP-NOW 回调只入队/更新状态，不在回调里阻塞打印或转发大量串口数据。
- `RADIOPING` 等桥诊断用独立 `type`，不得混入 MCU 应用数据。
- 命令与遥测分队列，`STOP` 具有最高优先级。

MSPM0 应用层现有 token/幂等机制继续保留，它能避免 ACK 丢失时重复启动测试或重复写 Flash；无线帧的序号与 CRC 解决的是更底层的丢包、重复和破帧问题，两者不是替代关系。

### 6.2 I²C 采用统一状态机

- 系统启动后明确等待 IMU 启动完成。
- 先扫描并记录 `0x5D`、`0x23` 是否 ACK，再开始周期采样。
- 记录“地址 NACK、数据 NACK、控制器忙、超时、数据校验/范围错误”而不是统一显示 ERROR。
- 单次传感器失败不得长时间阻塞 1 ms 主调度；恢复动作分步执行。
- 电机 PWM 切换时观察 I²C 错误率，以排除供电纹波、地弹和电机干扰。

### 6.3 自动调参必须有进入门槛

上位机启动自动任务前应强制检查：

- 可靠 `HELLO` 往返通过。
- `STATUS` 显示 `STBY,HIGH`。
- 编码器比例与固件采样周期一致。
- 灰度/IMU 在需要它们的任务中达到连续成功阈值。
- 可靠 `STOP` 已确认。

任一项失败都应停止任务并给出具体原因，不能继续试参数。

## 7. 构建、烧录与启动

### Python 上位机

```powershell
python -m pip install -r .\HOST\requirements.txt
.\HOST\start_autotune.bat
```

也可以直接运行：

```powershell
python .\HOST\pid_autotune_gui.py
```

连接的是 PC 端 ESP8266 的 COM 口，或接在 `PB15/PB16` 的 USB-TTL，不是 `PA10/PA11` 的 BSL 串口。

### 两块 ESP8266

安装 PlatformIO 后，一键分别烧录：

```powershell
.\ESP8266_BRIDGE\upload_pc.bat
.\ESP8266_BRIDGE\upload_car.bat
```

或先只构建：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d .\ESP8266_BRIDGE
```

固件位置：

- PC 端：`ESP8266_BRIDGE/.pio/build/pc_bridge/firmware.bin`
- 小车端：`ESP8266_BRIDGE/.pio/build/car_bridge/firmware.bin`

ESP-01/ESP-01S 烧录时使用稳定 3.3V 供电，`EN/CH_PD` 拉高，`GPIO0` 拉低后复位进入下载模式；正常运行时释放 `GPIO0` 后重新上电。两端角色不能烧反。

### MSPM0G3507

用 CCS 打开本工程并构建，成功后生成：

```text
Debug\pid_lab_mspm0.hex
```

如果当前电脑没有安装 CCS，也可以在工程根目录执行下面的脚本；它会调用本机 TI SysConfig、TI ARM Clang 和 Hex Converter，自动生成同一个文件：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1
```

生成的 `Debug\pid_lab_mspm0.hex` 就直接拖入立创 MSPM0 在线烧录工具。脚本使用 TI 的 ROM 初始化模型，HEX 只包含 Flash 地址，不会把 SRAM 地址交给 BSL。脚本要求 TI 工具安装在 `C:\ti` 默认目录，若安装位置不同再调整脚本开头的三个工具路径。

BSL 烧录步骤：

1. 关闭占用 BSL 串口的软件。
2. 按住 `BSL`，按下并松开 `RST`，再松开 `BSL`。
3. 在立创 MSPM0 在线烧录工具中连接对应串口。
4. 选择 `Debug\pid_lab_mspm0.hex`，保持 9600 baud 烧录。
5. 完成后启动应用或复位。

烧录网页底部的“操作日志”只显示 BSL 烧录进度，不显示应用层 IMU 数据。烧录完成并让 MSPM0 退出 BSL 后，在上位机连接的 **115200** 应用串口打开工程的 Python 界面，点击“检查 IMU”；返回行会显示在窗口下方日志中。烧录用的 BSL 串口是 **9600**，不要用它判断 `IMU,OK`。

改动 `main.c`、`LAB/`、`MOTOR/`、`SENSOR/`、`MPU/`、`SERIAL/` 或 `pid_lab.syscfg` 后都必须重新构建并烧录 MSPM0。修改 Python 界面本身不需要重烧下位机。

## 8. 引脚表

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| TB6612 左轮 | PA12、PA24、PA25 | PWMA、AIN1、AIN2 |
| TB6612 右轮 | PA13、PB9、PB8 | PWMB、BIN1、BIN2 |
| TB6612 STBY | PB22 | 高电平使能 |
| 左编码器 A/B | PA14 / PA15 | 当前只计 A 相上升沿 |
| 右编码器 A/B | PB12 / PB13 | 当前只计 A 相上升沿 |
| I²C1 SDA/SCL | PB3 / PB2 | 灰度与 IMU 共用 |
| 应用串口 TX/RX | PB15 / PB16 | 115200 |
| BSL TX/RX | PA10 / PA11 | 9600，只用于烧录 |
| 按键 SW1/SW2/SW3 | PB23 / PA8 / PB6 | 低电平有效 |
| LED | PB4 / PB5 | 低电平点亮 |
| 蜂鸣器接口 | PB10 | 默认高阻 |
| 预留 ADC | PB19 / A1_6 | 当前不采样 |

车端 ESP8266 接线：

| ESP8266 | MSPM0 | 说明 |
|---|---|---|
| TX / GPIO1 | PB16 / RX | ESP 发给 MCU |
| RX / GPIO3 | PB15 / TX | MCU 发给 ESP |
| GND | GND | 必须共地 |
| VCC、EN/CH_PD | 稳定 3.3V | ESP8266 不可接 5V |

## 9. 上位机功能与命令

正式功能包括：

- 通信与硬件诊断。
- 左轮、右轮和双轮直接 PWM 悬空短测。
- 整车轮速前馈、最小 PWM 与左右轮速度 PI 自动调参。
- 八路灰度循迹 PD 自动调参。
- 方形赛道/IMU 转弯验证与转弯参数调整。
- 参数写入 MCU Flash V2。

常用诊断命令：

```text
PING             # 期望 PONG
HELLO <token>    # 协议/硬件握手
STATUS           # 运行状态与 STBY
SENSOR           # 灰度状态和错误计数
IMU              # 启动/就绪状态、yaw、成功/错误计数与固件版本
RADIOPING        # 只测试 ESP 无线桥
PARAM            # 当前参数
STOP             # 立即停车
```

轮速自动调参时两个驱动轮必须悬空。循迹和方形验证必须使用封闭场地，并随时准备断开电机电源。

## 10. 目录与修改边界

- `main.c`：初始化、1 ms 时基、看门狗和调度。
- `MOTOR/`：TB6612 方向、PWM 与安全停止。
- `ENCODER/`：计数、10 ms 速度和里程换算。
- `SENSOR/`：灰度 I²C 读取与诊断。
- `MPU/`：IMU yaw 读取与相对转角。
- `SERIAL/`：UART 环形缓冲和行协议。
- `LAB/`：协议、轮速 PI、循迹 PD、方形状态机与 Flash 参数。
- `HOST/`：Python GUI、自动调参、协议追踪和离线测试。
- `ESP8266_BRIDGE/`：PC/车端 ESP-NOW 串口桥。
- `pid_lab.syscfg`：时钟、PWM、I²C、UART 与 GPIO 配置。

`Debug/ti_msp_dl_config.c/.h` 是 SysConfig 生成文件，不要手工修改；要改引脚或外设，应修改 `pid_lab.syscfg` 后重新生成。

## 11. 测试命令

```powershell
# Python 协议与算法离线回归
python -m unittest discover -s .\HOST -p "test_*.py" -v

# 两个 ESP8266 角色的编译
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d .\ESP8266_BRIDGE
```

MSPM0 仍需在 CCS/SysConfig 环境中构建并检查告警。硬件测试必须记录所用的三个固件版本：MSPM0、PC 端 ESP、车端 ESP。

## 12. 日志与 Git 协作

- `HOST/logs/` 下按日期生成的原始会话目录默认不提交，避免仓库被大量重复 CSV/trace 污染。
- 需要分享故障时，优先在 Issue/README 写最小复现、固件版本、关键几十行 trace 和测量条件。
- 如果某次日志确有长期价值，再显式挑选单个文件提交，不要整目录上传。
- 开始工作前执行 `git pull --rebase`，建议在 `fix/reliable-bridge` 或对应功能分支修改，完成测试后再合并。
- 不提交 `.pio/`、`Debug/`、Python 缓存和临时 PDF 渲染文件。

## 13. 完成标准

只有同时满足以下条件，才能把工程称为“整车自动调参可用”：

- 直连串口和无线桥各 100 次握手零丢失、零重复、零乱序。
- 无线连续遥测 10 分钟无粘包、拆包和秒级积压。
- 灰度与 IMU 冷启动、热复位后均能稳定识别，1000 次读取达到预期成功率并能从故障恢复。
- 左右编码器方向和比例经过实测，PWM—速度关系有足够单调有效点。
- 轮速调参可重复得到相近参数，且自动停车保护始终有效。
- 循迹调参与方形 IMU 转弯在封闭赛道重复通过，不依赖人工碰运气。
- 断电重启后 Flash 参数正确恢复，任意异常都能通过 `STOP` 或超时保护停车。

安全优先：上电、复位和异常默认零 PWM；轮速调参必须悬空，实车循迹必须在封闭区域进行。

## 14. 本地保存与明天上传

当前版本已保存在本地 Git 提交中（完整提交号可用 `git log -1` 查看）。本次 GitHub 推送因尚未登录而没有完成；不要把远端旧版本误认为当前版本。

本目录同时生成了两个本地交接文件：

- `pid_lab_handoff_20260715.zip`：源码、README、构建脚本和两个重要 HEX 的压缩包；
- `pid_lab_handoff_20260715.bundle`：包含完整 Git 历史和当前 `main` 提交的 Git 备份。

如果明天仍使用这个工作目录，登录 GitHub 后执行：

```powershell
git push origin main
```

如果换电脑只带走 bundle，可执行：

```powershell
git clone .\pid_lab_handoff_20260715.bundle pid-lab
cd .\pid-lab
git remote remove origin
git remote add origin https://github.com/zhyzajia-dot/engineering-learning-lab.git
git push -u origin main
```

上传前先确认 `git log -1` 显示 `Fix IMU I2C transaction and add handoff diagnostics`，并确认 `Debug\pid_lab_mspm0.hex` 和 `Debug\i2c_address_scan.hex` 存在。
