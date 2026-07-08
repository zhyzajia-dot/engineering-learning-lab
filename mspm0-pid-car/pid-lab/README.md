# MSPM0 PID Lab

## 工程地图

这个工程当前专注于“逆时针正方形循迹”，各目录职责如下：

- `main.c`：上电初始化、5 ms 周期任务调度和独立看门狗。
- `MOTOR/`：TB6612 电机方向与 PWM 输出。
- `ENCODER/`：霍尔编码器计数、速度和累计里程。
- `SENSOR/`：8 路灰度模块的 I2C 读取、自动寻址和诊断。
- `MPU/`：IMU 偏航角读取，仅辅助正方形转弯判断。
- `SERIAL/`：MSPM0 UART 环形缓冲区和按行命令接口。
- `LAB/`：轮速 PI、循迹 PD、正方形状态机、命令协议和 Flash 参数。
- `HOST/`：PC 自动调参、方形测试、CSV/JSON 数据保存。
- `ESP8266_BRIDGE/`：两块 ESP-01S 构成的 ESP-NOW 无线串口桥。

`Debug/ti_msp_dl_config.c/.h` 是自动生成文件，不应手工维护；硬件引脚
和外设配置应修改 `pid_lab.syscfg` 后重新生成。

This is an independent PID learning project for the MSPM0G3507 line-following
car. The working race project remains unchanged at:

`F:\MSPM0GC\ckcar\car`

## Baseline

- CCS project name: `pid_lab_mspm0`
- MCU: MSPM0G3507
- Existing motor, encoder, I2C, key, display and timer pin assignments are
  inherited from the working car project.
- The baseline firmware initializes the motor and encoder, then keeps both
  motors stopped.
- The lab firmware includes an optional guarded counter-clockwise square
  state machine; it stays inactive until an explicit `SQUARE` command.

## Planned Experiments

1. Measure motor dead zone and PWM-to-speed feedforward.
2. Run left and right wheel forward step-response tests.
3. Tune independent wheel speed PI controllers.
4. Add low-speed line-following PD experiments.
5. Export telemetry to CSV for plotting and AI-assisted analysis.

## UART Lab Console

UART0 uses 115200 baud, 8 data bits, no parity, one stop bit and no flow
control. Connect PA10 to the USB-TTL RX pin, PA11 to USB-TTL TX, and connect
the grounds. Do not connect USB-TTL VCC when the board is already powered.

Commands are terminated by Enter:

```text
PING
HELP
STOP
STATUS
PARAM
STREAM ON
STREAM OFF
PWM L 120
PWM R 120
PWM B 120
STEP L 300 3000
STEP R 300 3000
RUN 250 5000
LINE 160 2500
SENSOR
IMU
SQUARE 160 1
SET LKP 580
SET LKI 100
SET RKP 580
SET RKI 100
SET SYNC 1000
SET BIAS 0
SET LINEKP 10000
SET LINEKD 6000
SET TURNANGLE 900
SET TURNFAST 185
SET TURNSLOW 140
SET TURNMARGIN 180
SET TURNEXIT 140
SET TURNDIST 85
SAVE
LOAD
ARM 200 3000
DUMP
```

PWM tests are forward-only and limited to 300. A nonzero motor command expires
after 2000 ms and stops both motors. Closed-loop tests accept 80 to 600 mm/s
and 500 to 15000 ms. `STEP` and `RUN` automatically output one PID CSV sample
every 20 ms and stop at the requested duration.

PID and feedforward settings use x1000 fixed-point values. For example,
`SET LKP 580` means a left-wheel Kp of 0.580. `SYNC` corrects accumulated
left/right encoder-count difference during `RUN`. `BIAS` is the desired
left-minus-right encoder ratio in 1/10000 units; positive values make the
left encoder travel slightly farther to compensate a physical left drift.
`LINEKP` and `LINEKD` are the outer line-following PD
gains. Saved runtime settings are restored after reset.
Flash V2 protects every wheel, line-following and square-turn parameter with
a full-structure checksum. Existing V1 settings remain readable and migrate
automatically on the next successful `SAVE`.

## PC AutoTune

The `HOST` directory contains a Windows upper-computer application. It connects
directly to the USB-TTL COM port, identifies each motor's feedforward model,
tests multiple PI candidates, applies the best result, validates it and stores
all results under `HOST/logs`.

After tuning, choose a straight-test speed and duration in the GUI.
`Arm Untethered Test` saves the gains to the last 1 KB main-flash sector and
arms the requested run. Disconnect USB-TTL, place the car on the floor, then
press either the configured PB21 SET input or the LaunchPad S1 button (PB23).
After the key press, the MCU waits 2 seconds and ramps to the requested speed
over 1.5 seconds. It stores up to 600 samples (12 seconds at 20 ms) in RAM. Reconnect
USB-TTL without resetting the MCU and click `Read Last Run` to plot and save
the captured CSV. The default 200 mm/s for 8 seconds travels about 1.6 m.

For line-following tuning, place the car on a closed black-line track, confirm
that the area is clear, and click `Auto Tune Line`. The application tests line
Kp first and then Kd at 160 mm/s, scores sensor error and line-loss rate, saves
all candidate CSV files, validates the winner, and writes it to MCU flash.

This car's S7 (eighth) digital line channel is ignored because its indicator
channel is faulty. The remaining S0-S6 channels use rebalanced edge weights.
Brief invalid frames slow the car to 100 mm/s; a continuous 600 ms loss stops
the motors.

Line autotuning applies a limited 45 mm/s, 220 ms steering probe in both
directions for every candidate. Scoring includes recovery time from a large
offset, overshoot, sign-changing oscillation, center error and line loss.
The controller reduces proportional gain near the center, boosts it for large
errors, uses lighter derivative action while moving away, and stronger
derivative damping while returning to the center.

For the counter-clockwise square task, the line sensor follows each straight
edge and detects a left corner. The car approaches briefly and pivots left.
Outgoing-line recapture and encoder turn travel are the primary turn-complete
guards; continuous, physically plausible IMU yaw is used for early slowdown.
An IMU discontinuity is ignored instead of extending the turn. `SQUARE speed
laps` stops after four corners per lap. `TURNANGLE` and `TURNMARGIN` use
0.1-degree units; `TURNDIST` is the per-wheel pivot travel limit in millimetres.
The firmware learns a bounded turn distance from successful outgoing-line
captures and automatically saves it after a completed square. The GUI exposes
all square parameters and saves yaw, turn travel, corner count and state
telemetry under `HOST/logs/square_*`. An unavailable IMU, lost line, turn
timeout or failed line reacquisition stops the motors immediately.
Each square log directory also contains `run_summary.json`, recording the
requested settings, active parameter snapshot and basic line-error statistics.

After tuning has been saved, the PC application is not required for a race.
Power the car and place it at the start. Each short SET/S1 press cycles the
selected lap count through 1, 2, 3, 4, 5 and back to 1. Hold SET/S1 for about
1.2 seconds, then release it to confirm. The firmware waits two more seconds
and runs the selected number of 300 mm/s counter-clockwise square laps using
the PI, line PD and turn parameters restored from flash.

Install its dependency once:

```text
python -m pip install -r HOST\requirements.txt
```

Then close any other serial terminal and run:

```text
HOST\start_autotune.bat
```

## ESP-01 Wireless UART

The `ESP8266_BRIDGE` directory contains the current wireless solution. One
ESP-01S stays in a USB programmer at the PC and the other connects to the
MSPM0 UART on the car. They communicate directly with ESP-NOW, so the PC
internet connection is unaffected. The upper-computer application continues
to use a normal COM port.

## Safety Rules

- Reset and power-on always command zero PWM.
- Motor tests require an explicit start command.
- A stop command and communication timeout stop both motors.
- Initial single-wheel tests are performed with the driven wheels raised.
