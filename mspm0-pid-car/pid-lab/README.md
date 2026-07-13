# MSPM0 小车 PID 上位机调参工程

## 启动上位机（先看这里）

在 PowerShell 或 VS Code 终端中进入本工程目录后执行：

```powershell
cd E:\zhongyaogc\mspm0_pid_lab\mspm0_pid_lab
.\HOST\start_autotune.bat
```

如果是第一次使用，先安装 Python 依赖：

```powershell
cd E:\zhongyaogc\mspm0_pid_lab\mspm0_pid_lab
python -m pip install -r .\HOST\requirements.txt
.\HOST\start_autotune.bat
```

也可以不使用批处理文件，直接启动图形上位机：

```powershell
python .\HOST\pid_autotune_gui.py
```

启动前请关闭其他占用同一个 COM 口的软件。上位机波特率为 **115200**，连接后点击“刷新串口”→选择串口→“连接”；程序会使用 `PING` 和 `PARAM` 自动确认下位机通信是否正常。

## 编译与天猛星烧录

工程已配置为在 CCS 成功构建后自动生成 Intel HEX 文件：

```text
Debug\pid_lab_mspm0.hex
```

文件名固定，每次成功构建都会覆盖旧版本，因此 `Debug/` 内只需要选择这一个 HEX 文件，不会累计多个版本。烧录时使用浏览器 MSPM0 BSL 工具，选择 `COM14`，烧录波特率保持默认 **9600**；它只用于 BSL 烧录，程序运行后上位机串口仍使用 **115200**。

烧录步骤：关闭占用 COM14 的软件 → 按住板上 `BSL` → 按一下 `RST` 并松开 → 松开 `BSL` → 在 10 秒内上传 `Debug\pid_lab_mspm0.hex` 并点击“一键烧录”。

## 工程用途

这是基于天猛星 **MSPM0G3507** 的小车 PID 学习、上位机调参与方框循迹工程。它用于：

- 测试左右电机的 PWM、死区和速度响应；
- 分别调节左右轮速度 PI 与前馈参数；
- 调节直线同步、八路灰度循迹 PD；
- 跑逆时针方框，验证 90° 左转精度；
- 通过上位机实时绘图、自动寻优、保存 CSV/JSON 日志；
- 将调好的参数保存到 MCU Flash，断电后仍可恢复使用。

本工程是“上位机调参版”。正常独立跑车工程在 `mspm0_working_car`，两者使用同一块新版 PCB 的引脚定义，但控制目标不同。

## 使用前确认

- 先烧录本工程，再上电；上电和复位后电机默认停止。
- USB-TTL：**PA10（MCU TX）接 USB-TTL RX，PA11（MCU RX）接 USB-TTL TX，GND 必须共地**。
- 板子已供电时，不要再接 USB-TTL 的 VCC。
- 也可以通过 `ESP8266_BRIDGE/` 的 ESP-NOW 无线串口桥连接；电脑端仍会显示为普通 COM 口。
- 第一次做单轮 PWM 或阶跃测试时，应让驱动轮悬空。

## 新版 PCB 引脚对应

| 功能 | MSPM0G3507 引脚 | 说明 |
|---|---|---|
| TB6612 左轮 | PA12、PA24、PA25 | PWMA、AIN1、AIN2 |
| TB6612 右轮 | PA13、PB9、PB8 | PWMB、BIN1、BIN2 |
| TB6612 待机 | PB22 | STBY |
| 左编码器 A/B | PA14 / PA15 | 当前以 A 相上升沿计数 |
| 右编码器 A/B | PB12 / PB13 | 当前以 A 相上升沿计数 |
| IMU、灰度 I2C1 | PB3 / PB2 | SDA / SCL，共用总线 |
| OLED I2C0 | PA0 / PA1 | SDA / SCL，与传感器总线独立 |
| UART0 | PA10 / PA11 | TX / RX，115200 baud |
| 三个按键 | PB23、PA8、PB6 | SW1、SW2、SW3，低电平有效 |
| LED | PB4 / PB5 | 低电平点亮 |
| H1 蜂鸣器接口 | PB10 | H1：GND、3V3、PB10；默认高阻 |
| 预留 ADC | PB19 / A1_6 | 当前不采样 |

IMU 地址为 `0x23`，欧拉角起始寄存器为 `0x26`；八路灰度传感器地址为 `0x5D`。I2C 外设必须使用 **3.3V 上拉**。

## 工程目录

- `main.c`：系统初始化、1ms 时基、看门狗和各模块调度。
- `MOTOR/`：TB6612 电机方向和 PWM 输出。
- `ENCODER/`：霍尔编码器计数、10ms 速度计算和里程换算。
- `SENSOR/`：八路灰度 I2C 读取、重探测和诊断信息。
- `MPU/`：IMU yaw 读取和相对转角。
- `SERIAL/`：UART 环形缓冲与按行命令收发。
- `LAB/`：串口协议、PI/PD 控制、方框状态机、Flash 参数和日志。
- `HOST/`：Python 图形上位机、自动调参、CSV/JSON 日志。
- `ESP8266_BRIDGE/`：两块 ESP-01S 的 ESP-NOW 无线串口桥。
- `pid_lab.syscfg`：新版板子的时钟、PWM、I2C、UART、GPIO 配置。

`Debug/ti_msp_dl_config.c/.h` 由 SysConfig 自动生成，不要手动修改；需要改引脚或外设时修改 `pid_lab.syscfg` 后重新生成。

## 上位机调参建议流程

1. 连接串口后先点击 `PING`，确认收到 `PONG`。
2. 用单轮 `PWM` / `STEP` 找到每个电机的最小启动 PWM 与基本响应。
3. 用左右轮阶跃测试调 PI、前馈；观察上位机曲线，先保证单轮稳定。
4. 用 `RUN` 调直线同步，必要时调整 `SYNC` 与 `BIAS`。
5. 在封闭黑线赛道运行循迹自动调参，先调 `LINEKP`，再调 `LINEKD`。
6. 用 `SQUARE speed laps` 测试方框左转；确认转角、IMU、重新捕线均正常。
7. 效果满意后执行 `SAVE`，把参数写入 Flash。

## 常用串口命令

命令以回车结尾；可在上位机命令框中发送，也可用普通串口终端发送。

```text
PING                         # 通信测试，期望返回 PONG
HELP                         # 显示帮助
STOP                         # 立即停车
STATUS                       # 当前模式、速度、PWM、圈数状态
PARAM                        # 读取当前全部参数
SENSOR                       # 灰度传感器状态与诊断码
IMU                          # IMU yaw、成功/失败计数

PWM L 120                    # 左轮开环 PWM
PWM R 120                    # 右轮开环 PWM
PWM B 120                    # 两轮开环 PWM
STEP L 300 3000              # 左轮 300mm/s、持续 3 秒闭环阶跃
STEP R 300 3000              # 右轮阶跃
RUN 250 5000                 # 直线同步闭环，250mm/s、5 秒
LINE 160 2500                # 循迹闭环，160mm/s、2.5 秒
SQUARE 160 1                 # 逆时针方框，160mm/s、1 圈

STREAM ON                    # 打开实时 PID 数据流
STREAM OFF                   # 关闭实时数据流
DUMP                         # 回放最近一次离线测试日志
SAVE                         # 保存当前参数到 Flash
LOAD                         # 从 Flash 重新加载参数
```

参数设置使用 x1000 定点数。例如 `SET LKP 580` 表示左轮 Kp=0.580。

```text
SET LKP 580 / SET LKI 0      # 左轮 PI
SET RKP 580 / SET RKI 0      # 右轮 PI
SET LFF 340 / SET RFF 340    # 左右轮前馈
SET LMIN 45 / SET RMIN 45    # 左右轮最小启动 PWM
SET SYNC 1000                # 直线同步增益
SET BIAS 0                   # 左右轮机械偏置补偿
SET LINEKP 6000              # 循迹比例增益
SET LINEKD 2500              # 循迹微分增益
SET TURNANGLE 900            # 左转目标角度：900 = 90.0°
SET TURNFAST 185             # 转向快速 PWM
SET TURNSLOW 140             # 接近目标时的慢速 PWM
SET TURNMARGIN 180           # 降速余量：180 = 18.0°
SET TURNEXIT 140             # 转弯后重新循迹的目标速度
SET TURNDIST 85              # 转弯编码器行程阈值，单位 mm
```

## 方框与脱机运行

方框模式为逆时针左转。转角完成采用三重保护：IMU 相对 yaw、左右轮编码器转动里程、转弯后灰度重新捕线。任一关键故障（持续丢线、转弯超时、无法重新捕线、IMU 不可用）会停车。

调参保存后可脱机运行：

- 空闲时短按任意板载按键，圈数会在 1~5 圈循环。
- 长按约 1.2 秒，松开后确认启动。
- 小车会等待 2 秒，再以 300mm/s 运行对应圈数的逆时针方框。

也可用 `ARM speed duration_ms` 先设置脱机直线测试；断开 USB-TTL 后按任意板载按键，等待 2 秒后开始运行。完成后重新连接上位机并使用 `DUMP` 读取 RAM 中的最近一次日志。

## 参数和日志

- `SAVE` 写入 Flash V2，保存轮速 PI、前馈、直线同步、循迹和转弯参数。
- `LOAD` 可读取当前 V2 参数，也兼容旧 V1 参数；下次 `SAVE` 会升级为 V2。
- 上位机的实时数据和自动调参结果保存在 `HOST/logs/`。
- 方框日志目录会包含 CSV 与 `run_summary.json`，便于回看候选参数、误差和转角状态。

## 安全规则

- 上电、复位和 `STOP` 都会输出零 PWM。
- 开环 PWM 最大 300，并在 2 秒未收到新命令后自动停止。
- 闭环速度测试范围是 80~600mm/s，测试时长是 500~15000ms。
- 转动的轮子附近不要触碰导线或电机；异常时优先断开电机电源或发送 `STOP`。
