# Current handoff (2026-07-18, Guard25)

## 当前行动版：V4 式完整一圈自动调参

`gimbal-auto` 现在采用 V4 的闭环搜索：每个 `LINEKP/LINEKD` 候选必须完成逆时针正方形的四条边，评分包含出弯恢复、P95 误差、蛇形/反向次数、目标差速跳变和丢线比例；随后交替复测 incumbent/challenger，只有完整一圈的中位数至少改善 3% 才保存。一次调参会话会跑多圈，这是为了得到可重复的整圈冠军参数。

会话开始时会把旧的 `TURNDIST<98`（例如把 `TURN CAPTURE` 的 3/4 距离误存成 93）恢复为 V4 的完整转弯基准 `98 mm`。固件的 `TURN CAPTURE/CENTER/LEARN` 继续用于在线观察转弯几何；失真的绝对陀螺仪角度不作为唯一转弯完成条件。最近的固定参数验证发生在第一弯出弯后的同一条直线上振荡，日志中没有第二次 `TURN CAPTURE`，所以问题不是“第二弯识别到了却被限制停下”。

The current source and default firmware are Guard25. Guard24's second-corner run showed the actual track pattern was `mask=13` followed by `mask=7` (three left sensors), so even the four-sensor detector never started the turn. Guard25 admits only these two three-sensor masks after at least one corner has completed, while still rejecting the historical straight-line `mask=14` false turn. The valid-edge PID handoff, slower GIMBAL pair (`TURNFAST=165`, `TURNSLOW=110`), `1/2×TURNDIST` taper, and short counter-torque remain.

The host-side post-corner target-differential guard is intentionally set to `260 mm/s`, above the firmware's normal `200 mm/s` GIMBAL correction envelope. A large but valid outer-sensor error is therefore handed back to grayscale PID; only malformed telemetry beyond the physical envelope is stopped by the host.

Guard25 host completion no longer treats a late score fluctuation as a line-loss fault or ends the run after the second corner. After two valid centered corners, the incumbent PID is kept and turn geometry is learned; the vehicle continues under firmware grayscale PID until the firmware emits `SQUARE DONE`. A temporarily worse validation window is reported, while emergency stops for serial loss, IMU faults, and a genuinely lost line remain enabled.

Turn-distance learning uses the correct geometry: `TURN CAPTURE` is emitted at the 3/4-distance capture gate, so the host converts the median capture travel back to the nominal `TURNDIST` with `×4/3`. Without this conversion, the previous run incorrectly changed `98` to `72`, causing the next run to capture the first corner too early and lose the line before the second corner.

For recovery, `gimbal-auto` also detects an old collapsed value below `80 mm` and starts that session from the proven `98 mm` baseline; a successful two-corner session then replaces it with the corrected learned value.

The three-corner run proved that absolute IMU yaw is cumulative across track edges: after three successful `TURN CENTER` events the car still had `mask=48/error=6/line_valid=1`, while yaw crossed `-60°`. The host hard-yaw backstop therefore no longer stops on yaw alone. It requires the grayscale line to be absent as well, sustained for `15` compact samples; a valid gray line always remains under firmware PID ownership.

The full-lap tuner intentionally requests multiple firmware laps, as V4 did, because each candidate needs a complete four-edge measurement. After `GIMBAL COMPLETE`, run a separate fixed-parameter square for physical validation; `SQUARE DONE` remains the normal course boundary for that validation run.

Physical validation at `2026-07-18 13:11` completed all four corners and reached `GIMBAL COMPLETE`. Captures occurred at `76/71/68/65 mm`; all four produced valid `TURN CENTER` and `TURN LEARN` events. The corrected `×4/3` learner saved `TURNDIST=93`, while the proven line pair remained `LINEKP=8250/LINEKD=2250`. Final readback was `IDLE/SAFE`, `mask=48`, `GUARDVER=25`, `PROFILE=1`.

The design deliberately returns to the proven V4 ownership model instead of stacking independent recovery controllers:

- Straight-line steering has one owner: grayscale error → PD → left/right speed targets → wheel PI/PWM. The heavy-gimbal automatic run starts at the historical champion `LINEKP=8250`, `LINEKD=2250` (the old `6750/2000` start could not reach that champion with local ±100 trials). D is active outside the small center deadband; candidates that worsen the same straight-edge score are rolled back.
- Turning follows V4 geometry: 50 ms approach, slower heavy-gimbal turn, geometry-based pre-brake from `1/2×TURNDIST`, about `70 PWM` at the capture window, then a short `45 PWM/40 ms` counter-torque pulse. Once capture is confirmed, any valid black line—including sensor 6/7 edge masks—is handed to grayscale PID alignment; it does not wait for an initially centered line.
- The loaded profile now starts with V4 turn parameters `TURNFAST=185`, `TURNSLOW=140`, `TURNMARGIN=180`, `TURNEXIT=140`, `TURNDIST=98`; wheel response remains separate (`LMIN=90/RMIN=85`, `LFF=540/RFF=520`) so LIGHT is not changed.
- IMU yaw is relative and diagnostic/safety evidence. The same mounting location as V4 does not guarantee the same Euler yaw after the high gimbal changes roll/pitch vibration and chassis motion. Logs measured a physical 90° rotation as about 68.7°, and a Guard16 turn peaked near 51° while encoder travel exceeded 290 mm. Therefore yaw cannot be the only completion authority; encoder geometry and real line return are primary.

The Guard18 run is the reason for Guard19: the vehicle crossed `mask=48` at travel `75..91 mm`, but Guard18 waited until `98 mm`, then saw `mask=96/64` and stopped with `LINE NOT CAPTURED`. Guard19 moved capture back into the V4-style sweep. Guard20/21 reduced turn speed and coast; Guard22/23 proved the capture and PID handoff; Guard24's second-corner `mask=13/7` detector miss is the sole Guard25 change.

Current HEX: 110,765 bytes, SHA-256 `AE8EEBAB79A09863F7D56CAB1D653797C3CB8843EAE2B629DD7978957BE46D44` (run `Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0.hex` before flashing). This build adds measured wheel-speed damping, continuous P/D gain scheduling, curvature/derivative-based forward-speed scheduling, and smooth nonlinear steering compression for the heavy gimbal platform.

The heavy-platform correction is deliberately continuous: when the line error and filtered error velocity point outward, a bounded portion of forward speed is exchanged for capture margin; it never enters a fixed stop or a one-sided turn mode. The PD command is then passed through a soft saturation curve instead of being hard-clipped, so a large error still produces a strong correction without instantly commanding one wheel near zero. The measured left/right wheel-speed difference is fed back as damping, reducing the alternating overshoot caused by chassis inertia. Flash this HEX before the next run and record the telemetry directory; do not compare a run made with the previous HEX against this controller. An immutable copy is kept at `Debug/pid_lab_mspm0_guard26_adaptive_damping_20260718.hex`.

## Guard19 换电脑交接与烧录注意事项

本节优先于后面的历史 Guard9～Guard18 复盘。历史内容保留是为了追溯故障，不代表当前应烧录的版本。

### 在新电脑上准备

所有工程和工具尽量放在 D 盘：

```powershell
# 工程
D:\engineering-learning-lab\mspm0-pid-car\pid-lab

# 当前机器已准备好的便携 Git
D:\Tools\MinGit\cmd\git.exe

# 当前机器的 TI 工具
D:\TI\mspm0_sdk_2_10_00_04
D:\TI\SysConfig\sysconfig_cli.bat
D:\TI\tiarmclang\bin\tiarmclang.exe
```

如果 VS Code 提示没有 Git，在设置中把 `git.path` 指向 `D:\Tools\MinGit\cmd\git.exe`；也可以直接在 PowerShell 中使用上面的完整路径。换电脑后先安装 MSPM0 SDK、SysConfig、TI ARM Clang，再运行构建脚本，不要把旧电脑的 `tmp/` 目录当作编译结果复制过去。

### 建议的恢复顺序

```powershell
Set-Location D:\engineering-learning-lab\mspm0-pid-car\pid-lab
$env:PYTHONPATH='D:\engineering-learning-lab\mspm0-pid-car\pydeps'
$py='C:\Users\<用户名>\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'

& $py -m unittest discover -s .\HOST -p 'test_*.py'
powershell -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0.hex
```

把 `$py` 换成新电脑实际的 Python 路径；若已安装 Python 和 `pyserial`，可直接使用 `python`。烧录后连接串口，先只读确认：

```powershell
& $py .\HOST\pid_lab_cli.py diagnose --port COM3
& $py .\HOST\pid_lab_cli.py params --port COM3
```

必须看到 `GUARDVER=19`、`FLASHVER=3`、`PROFILE=0/1` 且 `STATUS ... IDLE ... SAFE`。重云台运行前由 `gimbal-auto` 自动选择 `PROFILE GIMBAL`；不要把 LIGHT 参数保存覆盖到 GIMBAL，也不要在开放场地测试。

### Guard19 唯一实车入口

先把车放在逆时针封闭方形赛道起点，确认两轮和云台供电正常、车轮前方无人，再运行：

```powershell
& $py .\HOST\pid_lab_cli.py gimbal-auto --port COM3 --speed 120 --track-safe
```

该流程会从 `8250/2250` 开始直线评分，D 优先做有界试探；明显变差立即回滚。转弯完成必须出现 `TURN CAPTURE`/`TURN CENTER`，失败时主机会先 `STOP` 并恢复本次任务前的 GIMBAL RAM 参数，不会把失败候选 `SAVE` 到 Flash。第一次 Guard19 实车只跑 120 mm/s，确认首弯和最终直边后再考虑更高速度。

Build tools used on this computer are installed under `D:\TI`: MSPM0 SDK 2.10.00.04, SysConfig 1.28.0.4712, and TI ARM Clang 5.1.1.LTS. The build script now searches these D: locations and also accepts `MSPM0_SDK`, `SYSCONFIG_CLI`, and `TI_ARMCLANG` environment variables.

Current HEX: 108,917 bytes, SHA-256 `11E78C61AC6862FA89C3F94D93AFE85DAD22488410155CC4C8949A09A1B11CEF` (run `Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0.hex` before flashing).

# MSPM0 循迹小车：V4 基线与重云台适配交接

> 更新时间：2026-07-17。当前候选恢复 V4 的两层闭环：灰度 PD 只分配目标速度，编码器轮速 PI+前馈直接决定 PWM。GIMBAL 运行前使用重载轮速参数 `LMIN=90 / RMIN=85 / LFF=540 / RFF=520`，不再运行任何启动托举或启动同步状态机。历史 Guard9/Guard8 记录仅用于复盘，不代表当前 HEX。

## 0. 2026-07-17 最新交接：Guard11 已构建，等待首次受控实车验证

本节是最新状态，优先级高于后文仍保留的历史过程说明。

- 15:35 实车失败的直接证据：启动同步已把目标推到左/右 `150/90 mm/s`，但实际左轮仍只有 `13～58 mm/s`、右轮 `47～104 mm/s`；旧稳定门始终未释放，灰度 PD 无法接管，所以车辆向左脱线。这不是自动调参没有算出结果，而是控制所有权设计错误。
- 当前 Guard11 已删除 GIMBAL 直线中的启动同步转向、稳定确认门、共同启动门、破静摩擦 PWM 覆盖和专用堵转状态。灰度 PD 从第一帧分配左右速度目标，编码器轮速 PI 使用 GIMBAL 独立的 `MIN/FF/KP/KI` 生成 PWM，与 V4 数据流一致。
- 16:05 实车显示起点正确、灰度 `mask=24`，但沿用轻载 `LMIN=32/RMIN=26` 时左轮在 `340～1200 ms` 几乎没有速度。当前候选不再增加托举状态机，而是在自动流程开始时直接装入重载 `LMIN=90/RMIN=85/LFF=540/RFF=520`；在首个 `40 mm/s` 目标处，两轮正常 PI 公式均约输出 `125 PWM`。
- 上位机 GIMBAL 自动调参从 `LINEKP=3500 / LINEKD=1000` 起步，局部步长缩小到 `250 / 200`；明显变差的候选累计够最少样本后立即回滚，不再等待车辆重新居中。
- 当前 HEX 已通过 TI ARM Clang 的 `-Wall -Wextra -Werror` 构建和全部 62 项测试；尚未进行本版实车验证，必须重新烧录后再运行。

- 2026-07-17 最新实车日志确认首条直线在 `mask=0` 后仍等速前冲，旧版的 `100 mm/s` 首边限速和 `25°` 提前航向停车已撤回/放宽。当前候选恢复为 `120 mm/s` 首边速度、`500 ms` 起步斜坡、保留最后转向追回丢线，且 PID 差速上限回到可调范围；待烧录 `Debug/pid_lab_mspm0.hex` 与不可变归档 `Debug/pid_lab_mspm0_guard9_recovery_pid_20260717.hex` 均为 `109,196 B`，SHA-256 均为 `6BC9485047A9B100AE7E3450632F1C8508180D3CE5906B3C9581E1CBBE51F4D1`。旧版归档仅保留作对比，不要再烧录。
- Guard9 不改 LIGHT、轮速 PI、PWM 和基本循迹 PD，只收紧 GIMBAL 转弯捕获并收窄重载直线差速：直线差速由原先最大 `±75` 改为 `±45 mm/s`，每周期变化限制为 `10 mm/s`；转弯仍使用有符号逆时针 yaw、IMU 失效/野点立即停车、强制 `RIGHT_DEPART → 连续 gap → LEFT_ENTRY`、左侧 0～2 路且 `error <= -6`、`75°～110°` 和 `0.75～1.8 × TURNDIST` 窗口，刹停/ALIGN 无线继续 fail-stop。
- 历史正确序列 `mask1→3→2 @81.5°～88.7°`、`073722` 的 `90.2°/mask0`、单独 `mask24`、IMU 中途失效和 `160°` 再遇原边均已加入回归。`python -m unittest discover -s .\HOST -p "test_*.py"` 共 62 项通过；当前工程已通过 `-Wall -Wextra -Werror` 构建。冻结 LIGHT 工程也已重新构建并逐字节验证为原哈希 `E8EA86F3...A36A93`。
- Guard9 目前是“软件检查通过、第三版待实车验证”的候选，不应称为最终成品。下一步先烧录当前 HEX，空闲时回读 `PROFILE=1 / FLASHVER=3 / GUARDVER=9`，再在可随时断电的封闭赛道上仅以 `120 mm/s` 运行一次 `gimbal-auto`；通过首边、至少四个真实 `TURN CENTER` 和最终直边验证后，主机才会执行唯一一次 `SAVE`。
- 本次完整实车日志为 `HOST/logs/gimbal_auto_guard8_20260717_110537`：首边 541 帧全部有线，但 `|error|` 均值 `10.10`、P95 `23`、最大 `24`，yaw 摆动 `-23.8°～+11.8°`，目标多次达到 `140/20` 或 `20/140 mm/s`；首弯在 `74.2°/travel=138 mm/mask64` 看到右侧旧边，随后 `74.8°～82.4°` 全部 `mask=0`，没有任何左侧新边证据，最终按 `176 mm` 安全停车。没有候选保存、没有 `SAVE`；失败后的 GIMBAL RAM 已回滚。

- 车辆已经由上位机明确发送 `STOP`，最后诊断为 `STATUS ... IDLE ... SAFE ... STBY,HIGH`。换电脑后不要直接发送任何运动命令。
- `Debug/pid_lab_mspm0_guard8_gray_online_20260716.hex` 仍保留为 Guard8 实车失败检查点，不是成品，也不应再烧录。它为 `108,365 B`，SHA-256 为 `132F616C025C352AF4809C2E3EE649D737AD97669A1BBEE6F1061CEC4913020D`。
- 2026-07-16 08:30 的第一次 Guard8 运行暴露了上位机启动竞态：`start_gimbal_auto()` 创建线程后，CLI 曾在工作线程把 `running` 置真之前错误退出并关闭串口。固件已经收到 `SQUARE`，因此车辆继续自主运动，但主机日志只记录到 MCU `t=440 ms`。
- 该短日志尚未到 `1400 ms` 评分门，也没有形成基线，没有发送任何在线候选 `SET`，没有进入日志可见的首弯，没有 `SAVE`，也没有完成失败回滚。不能把本轮表现称为“自动调参得到的结果”。
- 用户在主机日志中断后观察到：车辆从起步开始仍呈蛇形；第一个逆时针弯转得过多，甚至转到起步直线并沿原路返回，走一会后停止。这是可靠的人工实车现象，但没有对应的完整串口遥测，必须如实区分。
- CLI 生命周期竞态已经在 `HOST/pid_lab_cli.py` 修复：现在持有本次工作线程并等待 `join`，排空最终状态后才关闭串口；无 `COMPLETE/FAILED` 会明确报错，不再静默退出。该修复当时 61 项测试通过；加入 Guard9 回归后当前共 62 项通过。
- Guard8 的固件转弯门禁曾有四个缺口：IMU 失去可靠性后用里程降级、yaw 取绝对值、`CAPTURE_BRAKE` 后不重新确认线、`CAPTURE_ALIGN` 最多可在无线时继续前进约 1.2 秒。Guard9 已按以下边界修复，并保持轮速 PI、PWM 与循迹 PD 不变：
  1. GIMBAL 转弯过程中 IMU 一旦不可靠立即安全停止，不再用仅里程完成捕获；
  2. 使用有符号的逆时针 yaw 进度，候选角度限制在 `75°～110°`；
  3. 强制完整顺序 `RIGHT_DEPART → 连续 gap → LEFT_ENTRY`；
  4. `LEFT_ENTRY` 首次证据必须来自左侧 0～2 路且 `error <= -6`，单独 `mask=24/error=0` 不能建立新出边；
  5. 捕获里程限制约为 `0.75～1.8 × TURNDIST`；
  6. 刹停期间丢线不得进入向前 `CAPTURE_ALIGN`，ALIGN 无效线超过 `150～200 ms` 必须停止或回到慢转搜索。
- 已补回归：历史正确序列 `mask1→3→2 @81.5°～88.7°` 允许捕获；`073722` 的 `90.2°/mask0` 被拒绝；合成“右外→gap→无新线→160°～180°重新遇到原边”在捕获前停止；IMU 中途不可靠、单独 `mask24`、刹停时丢线均不得进入向前 ALIGN。
- 设备在烧录 Guard9 前的最后一次 GIMBAL RAM 读回为：`SYNC=500 BIAS=80 GSTART=10 LINEKP=3000 LINEKD=800 TURNANGLE=927 TURNFAST=150 TURNSLOW=100 TURNMARGIN=300 TURNEXIT=100 TURNDIST=98 PROFILE=1 GUARDVER=8`。这些是未保存成功的临时/固件学习值，不是验收参数；烧录或复位后必须重新 `PARAM` 回读，不能假设仍然存在。
- 轻载 V4 基线没有改变：`pid_lab_mspm0_v4_turnguard.hex` SHA-256 为 `E8EA86F3A9CD9BA089644B90D8C7E6922221B08E38C6136FD7068A4648A36A93`；`pid_lab_mspm0_v4_exact.hex` 为 `39AB0AF11D68C6541B7C159AD4590EF7AFE632C9B79B4F9DA43B3ED5CF7B5AA1`。

换电脑后的安全恢复顺序：

```powershell
git pull --ff-only
cd .\mspm0-pid-car\pid-lab
python -m pip install -r .\HOST\requirements.txt
python -m unittest discover -s .\HOST -p "test_*.py"
powershell -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1
python .\HOST\pid_lab_cli.py ports
python .\HOST\pid_lab_cli.py diagnose --port COM12
```

上述转弯门禁修改、测试、无警告构建、新 HEX 归档和 SHA-256 已完成。仍不得用 Guard8 运行；只有烧录并回读 `GUARDVER=9` 后，才可按第 6 节从 `120 mm/s gimbal-auto` 继续。

## 1. 当前结论

- 轻载/原车状态下，截图所指的 V4 已在 `300 / 340 / 380 mm/s` 各完成一圈，用户评价整体可用，380 仍有轻微晃动。
- 当前源代码是 **V4 灰度循迹 + 最小 TurnGuard + LIGHT/GIMBAL 双参数档 + TB6612 ADC + Guard9 灰度单主控/有符号顺序捕线/同场在线小步调参**。LIGHT 仍逐字节保持原 V4-TurnGuard 控制路径；GIMBAL 的直线与 `CAPTURE_ALIGN` 只由灰度误差控制，按误差连续调度转向权限和公共速度。IMU 不再参与直线转向，只用于遥测评分、转弯角度测量和 `25°` 直线硬保护。原 V4-TurnGuard 固件已经单独保留，随时可回退。
- V4-TurnGuard 在 300 mm/s 完成过一圈，平均绝对循迹误差 `1.411`、P95 `6`。
- 用户随后给车安装了一个很重的云台，明确反馈 V4 已不能正常工作。重载会同时改变轮速响应、可用加速度、轮胎附着、重心和转弯惯量，因此下一步必须建立独立的重载参数档，不能覆盖轻载 V4 基线，也不能只盲调循迹 `Kp/Kd`。
- **目前仍没有“重云台循迹与转弯全部实车标定完成”的 HEX。** `GUARDVER=5 exit6` 的 120 mm/s 实车资格轮已经证明：首条直边 499 帧全部在线，稳态平均绝对误差 `2.03`、P95 `6`、实际均速约 `117.9/120 mm/s`，启动、轮速和直线循迹已通过；失败发生在首弯，旧逻辑于 yaw `65.1°`、`mask=64/error=36` 时把正在离开的旧入边右外侧误判为“新边已捕获”。Guard6 随后加入旧边/gap/新边捕线、刹停和低速对中，但 `06:58:43` 的实车轮在到达首弯前触发了错误的固定 RECOVER 超时：262 帧全部 `line_valid=1`，误差已经按 `11→6→2→0` 回到中心，停车前约 33 帧持续 `mask=24/error=0`，用户也确认传感器仍在黑线上。
- `07:37:22` 的 Guard7 实车轮进一步证明 Guard7 的分层恢复方向不对：首条直边 701 帧全部有效，平均绝对误差 `5.137`、P95 `11`，但 yaw 在 `-17.3°～+6.3°` 之间摆动，灰度误差和 yaw 多次换向，并产生 5 次恢复过程、15 条恢复阶段消息，符合灰度、IMU 航向和恢复状态机互相争夺转向权的特征。首弯到 `90.2°` 时仍为 `mask=0/line_valid=0`，却被角度条件错误报告 `TURN CAPTURE`；随后 `CAPTURE_ALIGN` 连续 60 帧全部无效线，仍以 `111/89 mm/s` 前进 72 mm，把 yaw 从 `91.1°` 拉回到 `72.8°`，最终由真正的 `LINE LOST` 安全停止。该轮没有进入循迹 A/B，也没有 `SAVE`。
- Guard8 删除了多控制器冲突，但首次实车仍因主机竞态丢失完整遥测，并暴露首弯过转回原边。当前 `GUARDVER=9` 在不改直线控制器的前提下补上有符号 yaw、IMU fail-stop、完整右出/gap/左入来源、角度/里程双窗口及捕获连续性；`90°/mask0` 只会保持低速搜索，`115°`、`1.8×TURNDIST` 或 `3.2 s` 会安全中止。Guard9 已通过软件测试与构建，但尚待 120 mm/s 实车验收。
- 第 8 路灰度传感器已继续屏蔽：有效掩码为 `0x7F`。这是已知硬件约束，不要重新启用第 8 路。

## 2. 当前固件与回退件

| 文件 | 用途 | 大小 | SHA-256 |
| --- | --- | ---: | --- |
| **当前版本标记** | 以 `recovery_pid` 归档和 `6BC94850...BE51F4D1` 哈希为准；下方旧版归档仅供对比 | 109,196 B | `6BC9485047A9B100AE7E3450632F1C8508180D3CE5906B3C9581E1CBBE51F4D1` |
| `Debug/pid_lab_mspm0.hex` | 当前待烧录的 `GUARDVER=9` 候选；首边恢复 `120 mm/s`、起步斜坡 `500 ms`，丢线短暂保留最后转向，GIMBAL 直线差速重新交给 PID/自动调参；软件检查通过，待实车验收 | 109,196 B | `6BC9485047A9B100AE7E3450632F1C8508180D3CE5906B3C9581E1CBBE51F4D1` |
| `Debug/pid_lab_mspm0_guard9_recovery_pid_20260717.hex` | 与当前 HEX 逐字节一致的恢复版归档；针对最新实车“起步丢线后等速前冲”修正，不再使用 `100 mm/s` 首边硬限速 | 109,196 B | `6BC9485047A9B100AE7E3450632F1C8508180D3CE5906B3C9581E1CBBE51F4D1` |
| `Debug/pid_lab_mspm0_guard9_lineenvelope_20260717.hex` | 上一版 Guard9 直线包络归档；实车已证明它仍会在首条直边积累过大的航向偏差，仅作对比，不要再烧录 | 108,965 B | `D4F0EB1B54DF0F7BD4DAFE638C84BCC54E881AEA4B9B0765886E2AB7A8066313` |
| `Debug/pid_lab_mspm0_guard9_signed_turngate_20260717.hex` | 与当前 HEX 逐字节一致的 Guard9 不可变候选归档，用于本轮实车验证和回退对比 | 108,994 B | `CD7A871ACB4CA7B851BB9E7B76F17171EA5DD34182DF0B17DEDBBD0E2A236777` |
| `Debug/pid_lab_mspm0_guard8_gray_online_20260716.hex` | Guard8 首次实车失败归档，用于复盘和回退对比，不是验收成品，也不再与当前 HEX 相同 | 108,365 B | `132F616C025C352AF4809C2E3EE649D737AD97669A1BBEE6F1061CEC4913020D` |
| `Debug/pid_lab_mspm0_guard7_progress_recovery_20260716.hex` | Guard8 前的直接回退件；对应 `07:37:22` 首边蛇形、多控制器冲突，并在 `90.2°/mask0` 错误 CAPTURE 后真正失线的 Guard7 实车固件 | 115,526 B | `FE2B03B21D8FF85442C41C2CD1356290D7C1C8EAE67B6FBC27570AF1BFF06622` |
| `Debug/pid_lab_mspm0_guard6_capture_align_20260716.hex` | Guard7 前的直接回退件；对应 `06:58:43` 全程在线、已经回到中心却被固定 RECOVER 超时误停的 Guard6 实车固件 | 112,581 B | `D08A499C30243324D3167B5760725A28E71AFAE11378F41648BFF0AC463AD873` |
| `Debug/pid_lab_mspm0_pre_gimbal_turn_capture_20260716.hex` | Guard6 转角捕线重构前的直接回退件；与 exit6 成品字节相同 | 105,776 B | `395F634491880E4124BBEB00CC89C0D6B2436CB0E0E627C5956DB02829A6036A` |
| `Debug/pid_lab_mspm0_guard5_recovery_exit6_20260716.hex` | exit6 成品的不可变归档；对应 06:12 首弯把旧边误判为完成的实车固件 | 105,776 B | `395F634491880E4124BBEB00CC89C0D6B2436CB0E0E627C5956DB02829A6036A` |
| `Debug/pid_lab_mspm0_pre_gimbal_recovery_exit_20260716.hex` | 本次恢复退出修改前的直接回退件；对应 05:37 首次 GUARD5 实车资格测试固件 | 105,776 B | `27AE85B83E8F217BE0CD2CB978DF8F1E9AA45AE1CA093CD4DA057DA4793636E2` |
| `Debug/pid_lab_mspm0_guard5_pre_vehicle_20260716.hex` | 首次 GUARD5 实车测试前的不可变归档；与上一行字节相同，但保留“实车前检查点”的独立语义 | 105,776 B | `27AE85B83E8F217BE0CD2CB978DF8F1E9AA45AE1CA093CD4DA057DA4793636E2` |
| `Debug/pid_lab_mspm0_pre_gimbal_imu_recovery_20260716.hex` | 增加自适应启动、IMU 航向保持和 RECOVER 前的 `GUARDVER=4` 回退件；实车仍在线且已经回正，却因旧 `mask=96/error=23` 上位机门限提前停车 | 98,416 B | `A3B9D7D8D61834A956A17A690A26231C1AC6D2E34B8ABFD7153BB071AFE1FF1A` |
| `Debug/pid_lab_mspm0_pre_gimbal_seed_sync_20260716.hex` | 增加持久 GSTART、增量学习和连续 A/B 共享保护前的回退件；对应首个低速脉冲把 trim 暂时学反、随后由保护安全停车的固件 | 97,412 B | `F6D1D2AF4CF941D43CB766789014BC81390DB4A4FAEAC39210F0FCF6CB77AE33` |
| `Debug/pid_lab_mspm0_pre_gimbal_line_balance_20260716.hex` | 增加 GIMBAL 首边轮间补偿、真实直边 yaw 和启动逃逸保护前的回退件；对应“启动后直接向左出线”的实车失败 | 96,472 B | `081E885F077932A3FD6999A4AC8C8C2A34BF5DE05E6C60FFF11840CE8698D7EC` |
| `Debug/pid_lab_mspm0_pre_gimbal_bootstrap_capture_20260716.hex` | 增加保守 GIMBAL 自动调参入口和外侧捕线前的回退件；它能阻止此前的 180° 掉头，但实车仍会蛇形并在首弯脱线 | 95,904 B | `A6F47635F9BC3E75A77A86CF51AA93737C438109CA5CD65A3E14D2A91FD9FBAC` |
| `Debug/pid_lab_mspm0_pre_gimbal_line_guard_20260716.hex` | 增加 GIMBAL 低速循迹/航向保护前的回退件；对应发生首弯后约 180° 掉头的固件 | 94,717 B | `49FF43F7E841D8EE5C7A96FBEA22D37756908FCFDFFC19BCA26FA393677729F5` |
| `Debug/pid_lab_mspm0_pre_run_yaw_20260716.hex` | 增加 RUN 连续相对 yaw 前、已通过实车 ADC 通信检查的回退件 | 94,518 B | `C6EA9260F188BA82AAC1256E49FAE1D644A5B173D526F72858E26C57D3776D20` |
| `Debug/pid_lab_mspm0_pre_adc_20260715.hex` | 增加 ADC 采集前的双档固件回退件 | 90,485 B | `B59DA8C7BEB6D740B520E1090105A060198C4498C952AB5A9FA5006B22FDEDDD` |
| `Debug/pid_lab_mspm0_v4_turnguard.hex` | 修改前已验证的轻载 V4-TurnGuard 回退件 | 87,405 B | `E8EA86F3A9CD9BA089644B90D8C7E6922221B08E38C6136FD7068A4648A36A93` |
| `Debug/pid_lab_mspm0_v4_exact.hex` | 截图版本的精确 V4 回退件；没有 50° 转弯保护 | 87,158 B | `39AB0AF11D68C6541B7C159AD4590EF7AFE632C9B79B4F9DA43B3ED5CF7B5AA1` |

完整轻载 V4 工程另存为 `releases/V4_LIGHT_TURNGUARD_PROJECT_20260715.zip`（201,581 B，SHA-256 `33F8D90520A3CA89A686D871E32120E8A1BCFE5A19B1CA64F1F6C1DF83D12BAD`）。根目录 `build_v4_light_hex.ps1` 会先校验该冻结工程归档，再解压到独立临时目录构建并强制验证结果与 V4-TurnGuard 回退件逐字节一致；它不会拿当前持续演进的 GIMBAL/ADC 源码冒充轻载冻结源码。归档说明和独立固件副本位于 `releases/V4_LIGHT_TURNGUARD_20260715/`。

不要再根据文件名猜版本，烧录前用 PowerShell 校验：

```powershell
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard9_lineenvelope_20260717.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard9_signed_turngate_20260717.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard8_gray_online_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard7_progress_recovery_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard6_capture_align_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_turn_capture_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard5_recovery_exit6_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_recovery_exit_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_guard5_pre_vehicle_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_imu_recovery_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_seed_sync_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_line_balance_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_bootstrap_capture_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_gimbal_line_guard_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_run_yaw_20260716.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_pre_adc_20260715.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_v4_turnguard.hex
Get-FileHash -Algorithm SHA256 .\Debug\pid_lab_mspm0_v4_exact.hex
```

普通烧录后，Flash 中已有的运行参数可能不会被 HEX 覆盖。连接上位机后必须执行一次 `restore-v4`，它会先显式选择 `LIGHT`，再写入、保存并回读 V4 的 `LINEKP / LINEKD / TURNDIST`：

```powershell
python .\HOST\pid_lab_cli.py restore-v4 --port COM12
```

把 `COM12` 换成实际串口。只有命令最后明确报告回读一致，才算恢复完成。

## 3. V4 基线参数

以下既是 `app_config.h` 的默认值，也是 `restore-v4` 应恢复到 `LIGHT` 的基线。修改源码默认值不会自动覆盖芯片里已经有效的 Flash 参数。Flash V3 分别保存两套完整参数和当前档位；旧 V1/V2 数据加载时作为 `LIGHT` 保留，下一次 `SAVE` 才迁移为 V3。新建的 `GIMBAL` 槽最初复制这些数值，但后续 `SAVE` 只更新当前槽，不会覆盖另一槽。

| 参数 | 数值 | 含义 |
| --- | ---: | --- |
| `LKP / LKI` | `350 / 100` | 左轮 PI，系数按 x1000 |
| `RKP / RKI` | `500 / 350` | 右轮 PI，系数按 x1000 |
| `LFF / RFF` | `460 / 437` | 左右轮前馈，按 x1000 |
| `LMIN / RMIN` | `32 / 26` | 最小有效 PWM |
| `SYNC / BIAS` | `1000 / 50` | 直线同步与右轮目标补偿 |
| `LINEKP / LINEKD` | `6750 / 2000` | V4 循迹 PD，按 x1000 |
| `TURNANGLE` | `900` | 90.0° |
| `TURNFAST / TURNSLOW` | `185 / 140` | 转弯快、慢段 PWM |
| `TURNMARGIN` | `180` | 距目标 18.0° 切换慢转 |
| `TURNEXIT` | `140` | 出弯目标速度 mm/s |
| `TURNDIST` | `98` | V4 基准转弯里程 mm |

编码器换算为 `0.580 mm/count`，控制采样周期为 `10 ms`。硬件接线、器件规格、I²C 地址和原始资料位置见 [`docs/HARDWARE_REFERENCES.md`](docs/HARDWARE_REFERENCES.md)。

## 4. 当前 V4 控制到底包含什么

### 4.1 直线循迹

V4 直线循迹代码位于 `LAB/lab_ctrl.c`：

- 高速分界为 `340 mm/s`；
- 高速中心/中区/远区修正上限为 `45 / 90 / 125 mm/s`；
- 中心/中区/远区修正变化率为 `5 / 8 / 18 mm/s` 每控制周期；
- `abs(error) <= 4` 为中心区，`abs(error) <= 2` 时关闭 D 项；
- `abs(error) >= 10` 为远区；
- 公共目标速度保持不变，没有主动共模降速，所以已消除此前的顿感来源；
- 有效灰度通道掩码为 `0x7F`，第 8 路不参与误差计算和丢线判断。

### 4.2 唯一附加的 TurnGuard

精确 V4 只要转弯里程超过 `TURNDIST / 2`，看到灰度线就可能提前结束转弯。实车日志证明它曾在 38.8° 和 34.9° 捕获旧边，随后车头方向错误而丢线。

控制算法中的 V4-TurnGuard 只增加：

- IMU 可靠时，偏航角没有达到 50.0° 前禁止灰度提前捕线；
- IMU 不可靠时，至少走到预计转弯里程的 3/4 才允许灰度捕线；
- 90° 目标、快慢转 PWM、循迹 PD、轮速 PI 和编码器滤波均未因这个保护改变。

对应常量是 `LAB_TURN_LINE_CAPTURE_MIN_YAW_X10 = 500`。

双档基础设施本身只负责参数存储和选择：`PROFILE LIGHT` / `PROFILE GIMBAL` 在电机空闲时切换槽位；切档会丢弃当前槽未 `SAVE` 的试验值；`PARAM` 用数值字段 `PROFILE=0/1` 和 `FLASHVER=3` 提供回读。LIGHT 没有调整 V4 的循迹区间、增益、限幅、TurnGuard 或第 8 路掩码。

2026-07-16 的重载实车失败后，当前源码只为 GIMBAL 增加以下 Guard9 控制和安全包络，LIGHT 不进入这些分支：

- GIMBAL 仍使用独立参数槽和可保存的 `GSTART`；启动共同门槛、左右破静摩擦托举、1.0 秒斜坡、轮速 PI、`SYNC/BIAS` 与堵转检测继续保留，不写入或覆盖 LIGHT；
- 直线只保留一个转向主控：灰度误差。Guard7 的有效线 `SEEK/SETTLE` 状态机和 IMU 直线航向分量不再参与左右目标生成，避免三个控制器对同一横向误差反复反向修正；
- 只要灰度有效，外侧误差就仍交给同一个灰度控制器；转向权限和公共速度按误差大小连续变化，大误差获得更强回线量并自动降速，不再因为跨过某个阈值切换到另一套控制器；
- IMU 在直线段只记录相对 yaw、变化趋势和质量状态，供日志、候选评分和故障定位使用；连续达到 `25.0°` 仍是硬停止条件，但不会为了“保持绝对笔直”与灰度循迹对抗；
- 真正无有效灰度、轮速堵转、串口任务中止和硬 yaw 越界仍立即停车。这些属于保留的硬安全限制，不是正常循迹的软门槛；
- 转弯记录旧入边离开、连续 gap、新出边首见和中心稳定。`TURNANGLE=90.0°` 只表示进入低速 `TURN SEARCH` 的角度参考；若此时 `mask=0`，必须继续受控逆时针搜索，不能发送 `TURN CAPTURE`；GIMBAL 的 yaw 必须保持有符号逆时针进度，IMU 失效或野点立即停车；
- 只有 `RIGHT_DEPART → 连续 gap → LEFT_ENTRY` 完整成立，且新边来自左侧 0～2 路、`error <= -6`、yaw 在 `75°～110°`、里程在 `0.75～1.8×TURNDIST` 内，才能进入 `CAPTURE_BRAKE`。刹停后必须再次确认线仍存在；`CAPTURE_ALIGN` 无线时原地停车，连续 `180 ms` 未恢复即中止；
- 转弯 `115°` yaw 硬上限、3.2 秒总上限、真正失线、堵转和刹停超时继续保留。它们防止无限旋转，不把 90° 误当作已经找到线；
- 每个成功转角仍发送 `TURN CAPTURE / TURN CENTER / TURN LEARN`，用真实新边首见里程学习 `TURNDIST`，用中心稳定 yaw 学习 `TURNANGLE`。学习值只在完整主机任务成功后保存，失败会恢复任务前 GIMBAL 参数；
- 当前固件在 `PARAM` 回报 `GUARDVER=9` 和 `GSTART`；所有 GIMBAL 方框任务在车轮动作前必须读到 Guard9，Guard8 及更旧 HEX 会被上位机拒绝启动。第 8 路灰度继续以 `0x7F` 掩码屏蔽。

## 5. 实车证据

保留日志只包含可复现基线、故障定位和最新状态；大量重复试跑与已否决调参日志已经删除。

| 日志 | 固件/速度 | 结果 | 关键数据 |
| --- | --- | --- | --- |
| `HOST/logs/square_20260715_070335` | 精确 V4，300 | 完成 4 弯 | 均值 `3.433`，P95 `9` |
| `HOST/logs/square_20260715_070408` | 精确 V4，340 | 完成 4 弯 | 均值 `3.360`，P95 `18` |
| `HOST/logs/square_20260715_070438` | 精确 V4，380 | 完成 4 弯 | 均值 `3.916`，P95 `6`；用户仍感觉轻微晃动 |
| `HOST/logs/square_20260715_080531` | 精确 V4，300 | 2 弯后丢线 | 定位到约 38.8° 提前捕获旧边 |
| `HOST/logs/square_20260715_080558` | 精确 V4，300 | 3 弯后丢线 | 定位到约 34.9° 提前捕获旧边 |
| `HOST/logs/square_20260715_082029` | V4-TurnGuard，300 | 完成 4 弯 | 均值 `1.411`，P95 `6`，学习里程 `90 mm` |
| `HOST/logs/square_20260715_082445` | 当时开发版，340 | 起步后丢线 | 均值 `8.017`，P95 `30` |
| `HOST/logs/square_20260715_082505` | 当时开发版，300 | 1 弯后丢线 | 均值 `9.566`，P95 `30` |
| `HOST/logs/square_20260715_082526` | 当时开发版，300 | 起步后丢线 | 均值 `5.321`，P95 `25` |
| `HOST/logs/square_cli_20260716_022840` | 重云台、旧保护前 HEX，120 | 首弯后约 180° 掉头，用户提车 | 转弯状态在 yaw `90.0°`、里程 `142 mm` 后完成角 1；随后 LINE 状态目标达到 `20/220 mm/s`，差速 `200 mm/s`，最终才报 `LINE LOST`；未进入循迹调参、未保存候选参数 |
| `HOST/logs/square_cli_20260716_025901` | 重云台、`GUARDVER=1`，120 | 安全停止，无再次掉头；首弯脱线 | 首边 452 个有效样本平均绝对误差 `12.14`、P95 `23`，`55.5%` 位于远区，平均绝对循迹修正 `36.47 mm/s`、最大达到旧限幅 `45 mm/s`；旧入边在 yaw `68.7°～80.9°` 由 `mask96/64` 向右外扫出，约 `85.7°` 后进入 gap，旧逻辑在 yaw `90.4°` 直接进入 30 帧无有效线的 EXIT。该轮只是资格验证，未开始自动调参、未保存候选参数 |
| `HOST/logs/square_cli_20260716_032458` | 重云台、`GUARDVER=2` 保守启动档，120 | 安全停止；启动后直接向左出线 | 全程仍是首边 LINE、没有进入转弯或 A/B。灰度保持居中到 `1420 ms`，但此时计数差已为 `-11`；`1440 ms` 左右实际轮速为 `14/60 mm/s`。随后控制器按正确方向把目标逐步拉到 `155/85 mm/s`，总修正达到安全上限 `35 mm/s`，但计数差仍恶化到 `-66`，`2560 ms` 首次失线。`1500 ms` 后 53 个有效样本平均绝对误差 `19.74`、P95 `36`。根因是 LINE/SQUARE 未使用已保存的 `SYNC=500、BIAS=80`，不是先盲调 LINEKP/LINEKD 能解决的问题 |
| `HOST/logs/square_cli_20260716_035838` | 重云台、`GUARDVER=3`，120 | 上位机启动逃逸保护安全停车 | 全程 117 个样本均为首边 LINE。`80 ms` 左轮先出现一个脉冲，旧绝对累计算法从 `280 ms` 起把目标错误偏向右轮，最大反向 trim `-4 mm/s`；`1540 ms` 目标 `120/120` 时实际已为 `57/101 mm/s`。trim 后来转正且总目标最终达到 `155/85`，但右轮累计已领先 69 脉冲、yaw 达 `13.0°`。旧 `12°/5帧` 保护发出 STOP；没有进入转弯、没有保存参数。停车后灰度回读 mask `64`，证明不是保护误报 |
| `HOST/logs/square_cli_20260716_042526` | 重云台、`GUARDVER=4`，120 | 旧上位机门限提前停车；全程仍有有效线 | 120 帧全部 `line_valid=1`。右轮约 `60 ms` 首脉冲，左轮约 `680 ms`，启动相差约 `620 ms`；`1680 ms` 后目标达到旧上限 `155/85`。yaw 在 `2040 ms` 达峰 `7.1°` 后回落，`2280 ms` 起为 `mask=96/error=23`，停车末端 yaw 已降至 `5.4°`，说明车辆正在回正。旧主机把连续 3 帧有效 6/7 路误判为逃逸；没有转弯、没有保存参数。目录已补载荷元数据和汇总 |
| `HOST/logs/square_cli_20260716_053734` | 重云台、首次 `GUARDVER=5`，120 | 共同起步成功；RECOVER 持续回线但旧退出条件超时 | 178 帧全部 `line_valid=1`；左右轮首次运动为 `360/380 ms`，只差 `20 ms`。`1080 ms` 因 yaw `5.1°` 进入 RECOVER，yaw 峰值 `6.1°` 后回落，计数差由最差 `-32` 收回到末端 `-3`，最大线误差仅 `11`，没有进入转弯。`mask=48/error=6` 在 `1100～2980 ms` 持续存在，yaw 在 `1940 ms` 已回到 `4.0°`；旧 `error<=4` 使确认计数始终为 0，最终在约 107 mm 恢复里程处触发 2.5 秒超时。未开始 A/B、未保存候选，参数已回滚；目录已补载荷元数据和汇总 |
| `HOST/logs/square_cli_20260716_061232` | 重云台、exit6 `GUARDVER=5`，120 | 首条直边通过；首弯旧 EXIT 捕错边 | 首边 499 帧全部有效线，1 秒后平均绝对误差 `2.03`、P95 `6`、实际均速 `117.9 mm/s`，未进入 RECOVER。首弯在 yaw `50.3°/76 mm` 降为慢转；旧入边依次扫过 `mask48→32→96→64`，旧固件在 `65.1°/101 mm、mask64` 直接进入 EXIT。随后 40 帧/780 ms 始终为最外侧 `mask64/error36`、中心确认 0，最终 `LINE NOT CAPTURED`。A 相编码器把左轮残余反转也报告为正速度，立即切前进 PI 又放大了交接迟滞。未开始 A/B、未保存候选，参数已回滚；目录含载荷元数据和汇总 |
| `HOST/logs/square_cli_20260716_065843` | 重云台、`GUARDVER=6`，120；实际试跑 `LINEKP=3000/LINEKD=800` | 首弯前错误触发 `GIMBAL RECOVERY TIMEOUT` | 262 帧全部 `line_valid=1`，掩码只出现 `16/24/32/48`，误差只出现 `0/2/6/11`；恢复过程中误差按 `11→6→2→0` 改善。yaw 从约 `+5.2°` 过纠到 `-10.0°` 后已回升到停车前约 `-7.7°`；`4603 ms` 后约 33 帧持续 `mask=24/error=0` 直到 STOP，用户确认传感器仍在黑线上。固定 `2.5 s / 200 mm` 恢复超时属于误停；未到首弯、未开始 A/B、未保存候选，参数已回滚 |
| `HOST/logs/square_cli_20260716_073722` | 重云台、`GUARDVER=7`，120；实际试跑 `LINEKP=3000/LINEKD=800` | 首边蛇形；首弯在无有效线时错误 CAPTURE，随后真实失线 | 首边 701 帧全部有效，平均绝对误差 `5.137`、P95 `11`，yaw 范围 `-17.3°～+6.3°`；5 次恢复过程产生 15 条 `SEEK/SETTLE/CENTER` 消息，误差与 yaw 多次换向。转弯到 `90.2°/142 mm` 时为 `mask=0/error=0/line_valid=0`，角度条件却发送 `TURN CAPTURE`。`CAPTURE_ALIGN` 随后 60 帧全部无效线，以 `111/89 mm/s` 前进 72 mm，yaw 从 `91.1°` 回落到 `72.8°`，最终 `SQUARE ERROR,LINE LOST`。未进入 A/B、未保存，主机已恢复任务前参数；目录含载荷元数据和汇总 |

最后三份历史日志本身没有记录是否安装云台，不能仅凭文件断言载荷状态；但用户的实车观察已经明确确认“安装重云台后 V4 不行”。这些历史文件不补写猜测值。新的落地直线日志会在驱动车轮前写入独立 `load_metadata.json`，并在 `summary.json` 中重复保存同一份载荷快照。

另外保留：

- `HOST/logs/20260715_022256`：当前轻载轮速参数的完整调参依据；
- `HOST/logs/imu_manual_synced_20260715_033241`：静态 IMU 基线；
- `HOST/logs/imu_motor_20260715_032546`：电机运行时 IMU/通信基线。

第一轮重云台 120 mm/s 落地直线辨识也已保留：

| 日志 | 临时轮速参数 | 实车观察与数据结论 |
| --- | --- | --- |
| `HOST/logs/straight_20260715_232933` | 无，保存的 GIMBAL=V4 | 基本直线、轻微左偏；末端计数差 `-5`，偏航变化 `-0.1°` |
| `HOST/logs/straight_20260715_233418` | 无，重复验证 | 起步即明显左偏；末端计数差 `-9`，偏航变化 `-1.4°`，说明重复性不足 |
| `HOST/logs/straight_20260715_234432` | 未保存 `LMIN=40` | 左偏减轻但仍存在，左右响应仍不一致；试验后已自动恢复 `LMIN=32` |
| `HOST/logs/straight_20260715_235024` | 未保存 `LMIN=40,LFF=500` | 几乎未移动；稳态左右轮速仅 `0.23/0.57 mm/s`，PWM 均值 `169.9/199.2`，积分里程 `4.76 mm`；试验后已自动恢复 |
| `HOST/logs/straight_20260716_001824` | 无，保存的 GIMBAL=V4，新增 ADC | 两轮几乎不转；稳态轮速 `0/0.51 mm/s`，PWM 均值 `148.3/214.9`，积分里程 `2.2 mm`；用户随后给小车驱动电池充电 |
| `HOST/logs/straight_20260716_014636` | 电池充满；无临时参数；120 mm/s、8 秒 | 行驶 `725.37 mm`，稳态左右轮速 `111.79/111.19 mm/s`，末端计数差 `-2`；相对 yaw 峰值 `6.4°`、末端 `-1.0°`。实车观察为起步先向一侧偏、随后纠正、后段较直 |
| `HOST/logs/straight_20260716_015557` | 未保存 `SYNC=500`；120 mm/s、12 秒 | 行驶 `1187.45 mm`，稳态轮速 `112.17/111.63 mm/s`，末端计数差 `-6`（右轮累计多约 `3.48 mm`）；yaw 峰值由上一轮 `6.4°` 降至 `3.6°`、末端 `+2.9°`。实车不再反向修正，只剩一点持续左偏；试验后已自动恢复 `SYNC=1000` |
| `HOST/logs/straight_20260716_020208` | 未保存 `SYNC=500` 重复；120 mm/s、12 秒 | 行驶 `1189.30 mm`，稳态轮速 `111.65/110.43 mm/s`，末端计数差 `+4`；yaw 峰值 `4.1°`、末端 `+1.8°`。实车前半段左偏、后半段慢慢回正，但末端仍左偏；再次自动恢复 `SYNC=1000` |
| `HOST/logs/straight_20260716_020730` | 未保存 `SYNC=500,BIAS=80`；120 mm/s、12 秒 | 行驶 `1181.40 mm`，稳态轮速 `111.65/110.36 mm/s`，末端计数差 `+11`；相对 yaw 峰值 `3.7°`、末端 `+0.2°`，约 10 秒有一次 `1.9°` IMU 跳变。实车前段左偏后基本回到笔直，整体仍略左；该精度已满足阶段一目标 |

连续两次“高 PWM、编码器近零、实车几乎不动”是继续盲调前馈的停止条件。2026-07-16 的 ADC 运行窗口和随后同长度静止窗口均为 `RAW 961～1354`，所以不能把此前计算的 `24.7%` 单点差直接称为启动掉压。驱动电池充满后同一保存参数已恢复正常行驶，支持此前近零轮速主要与电池状态有关；8 秒试跑的静止/运行 ADC 最小值同为 `1217`，没有观察到可归因于运行的最低值下降。分压比仍未知，不把 ADC 原码换算成电池电压。

8 秒日志进一步定位到起步横摆而不是稳态轮速失配：约 `2.2 s` 时左右累计计数差最低到 `-44`，当前 `SYNC=1000` 会把这一误差转换成很大的左右目标差；起步斜坡内同步增益还会放大为两倍。随后控制器反向追回计数差，于是形成实车看到的“先侧向一方、再纠正”。两轮未保存 `SYNC=500` A/B 都消除了大幅反向过纠；固定 `SYNC=500` 后将 `BIAS` 提到 80，又把末端相对 yaw 降到 `+0.2°`。用户不要求绝对笔直，该结果已接受为阶段一参数并保存到 GIMBAL；后续不再重复人工直线复位。

`square_cli_20260716_022840` 证明第一次方框失败不是“转弯状态机直接转了 180°”：状态机在 IMU `90.0°` 时进入 EXIT，并在 `corner_count=1` 后回到 LINE。危险的第二段旋转来自 120 mm/s 下循迹目标连续打到近似单边慢、单边快，车辆继续旋转后才丢线。因此旧 HEX 不得再次运行；对应目录已补充 `load_metadata.json` 和 `summary.json`，保留用户“约 180° 掉头并提车”的实车观察。

`square_cli_20260716_025901` 证明 `GUARDVER=1` 的停车包络有效，也证明“先让未适配参数跑完资格圈，再开始自动调参”存在循环依赖。结合 `022840` 的完整扫线顺序重新解释后，约 69°～81° 的右外侧并不是新边，而是旧入边离开；新边应在 gap 后从左侧进入。对应目录同样已补充载荷元数据和汇总。后续 `gimbal-auto` 必须从第一圈就执行转角标定，而不是把整圈成功当作所有学习之前的空门槛。

`square_cli_20260716_032458` 进一步排除了“保守 LINEKP/LINEKD 仍不合适”这一表面解释。最新直线辨识能走直，是因为 RUN 分支用 `SYNC/BIAS` 主动生成了不对称目标；方框首边却给两轮相同目标，右轮在灰度仍居中时已经领先。灰度外环发现偏差后虽然修正方向正确，但车体已经向左逃逸。随后测试的 `GUARDVER=3` 把同一编码器同步依据以 `±15 mm/s` trim 接入首边并保持总包络不变，但下一轮证明其绝对累计学习入口仍会受首个脉冲方向影响。

`square_cli_20260716_035838` 又暴露出第二层问题：低速编码器脉冲很稀疏，不能用启动以来的绝对累计差直接学习前馈。本轮第一脉冲来自左轮，算法在真正运动前把方向学反；等右轮明显领先后，补偿还要穿过零点，且启动斜坡结束时增益减半，所以即使后面达到 `±35 mm/s` 包络也只能追赶、不能预防。`GUARDVER=4` 因而把直线辨识得到的约 `+10 mm/s` 做成持久 `GSTART`，80 mm/s 后丢弃早期脉冲并从新原点做增量微调；安全到达首弯才更新学习值。

`square_cli_20260716_042526` 证明 `GSTART=10` 与增量同步方向都正确，但也揭示了第三层结构问题。左轮比右轮晚约 620 ms 越过静摩擦；同步 trim 在最后中心死区帧已增长到约 `31 mm/s`，旧 `±35` 总包络只剩约 `4 mm/s` 给灰度。车体到 `7.1°` 后实际已经开始回正，横向位移仍把黑线推到第 6/7 路；旧主机却在 `error=23` 连续三帧时 STOP。`GUARDVER=5` 不再从同一点反复失败：启动托举在固件 10 ms 环内完成，有效外侧线进入低速 RECOVER，机械/航向/灰度分开限幅，真正丢线、堵转、航向硬越界或恢复超时才中止。

`square_cli_20260716_053734` 又把问题缩小到恢复退出的传感器量化边界，而不是轮速、丢线或 `LINEKP/LINEKD`。共同 40 mm/s 门槛和左右 `80/50` PWM 托举把首动差从约 620 ms 降到 20 ms；RECOVER 内全部样本仍在线，yaw 和计数差都在收敛。旧代码却把 `mask=48/error=6` 排除在 `error<=4` 退出条件之外。按真实 yaw 序列和固件 100 ms heading 更新、每 10 ms `2 mm/s` 斜率限制重放，当前 exit6 逻辑约在 `2010 ms` 首次同时满足四项退出条件，并在 `2100 ms` 累计满 10 帧，明显早于旧 2.5 秒超时；这是离线反事实回放结论，仍需新 HEX 实车确认。当前修改不动 `LINEKP/LINEKD`、10 帧确认、2.5 秒/200 mm 上限或航向硬保护。

`square_cli_20260716_061232` 已实车证明 exit6 修复有效：车辆没有再进入 RECOVER，并以低误差完成整条首边，所以继续调启动 PWM、轮速 PI 或 `LINEKP/LINEKD` 都没有数据依据。真正失败点是旧转角语义。完整历史转弯 `022840` 显示同一几何序列：旧入边从中心扫到右外侧（约 `56°～68° mask64`），随后约 `70°～80° mask0`，新出边才从左侧重新进入（约 `81.5° mask1 → 86.3° mask3 → 88.7° mask2`）。Guard6 因此明确区分 departing/gap/outgoing，目标角到位但仍在 gap 时也只进入受控搜索；它不会再把 `65°/mask64` 算作完成。此判断已加入两份真实日志回归测试。

`square_cli_20260716_065843` 证明 Guard6 的下一处瓶颈不是“PWM 不够”或“已经丢线”。该轮输出没有达到 PWM 上限，262 帧灰度全部有效，停车前已经稳定回到中心掩码；旧 RECOVER 却仍要求同时回到较小的绝对 yaw，并受固定时间/里程上限约束，因此在车辆仍有可用传感器反馈、且 yaw 已开始改善时误停。Guard7 随后尝试用传感器进展驱动的 `SEEK/SETTLE` 代替固定超时，但实车证明“再增加一个恢复控制器”仍不是正确结构。

`square_cli_20260716_073722` 给出了决定性的 Guard7 反例。首边 701 帧灰度全部有效，说明车始终具备直接循迹反馈；然而灰度 PD、IMU 航向保持和 `SEEK/SETTLE` 先后争夺转向权，形成从启动开始的蛇形与多次反向修正。首弯的第二个问题更明确：角度到 `90.2°` 时灰度为 `mask=0/line_valid=0`，这只能说明车仍在两条边之间的 gap，不能称为捕获。旧代码却进入 `CAPTURE_BRAKE/CAPTURE_ALIGN`，并把直线轮间 trim 带入无反馈前进，最终由真实 `LINE LOST` 停车。主机没有提前误停，硬失线保护在这里是正确的。

Guard8 因而采用更小、更清晰的控制所有权：有效线内灰度是直线唯一主控，IMU 只观测与硬保护；转弯角度只决定何时减速搜索，真实灰度出边才决定何时捕获；`CAPTURE_ALIGN` 不继承直线辅助。这样既去掉会打断可用反馈的软状态机，又保留真正失线、堵转、`25°`、`115°/3.2 s` 等必要安全边界。

## 6. 重云台适配：下一步正确顺序

重云台不是简单的“循迹 Kp 太小”。它至少改变四件事：同一 PWM 下的轮速与加速时间、左右轮负载差、横摆响应、急转时的附着和侧翻风险。必须保留轻载档，并新增 `V4_GIMBAL` 档。

### 阶段 A：先记录机械与供电条件

当前已经确认：云台使用独立电池且与小车电气上完全不连接；云台通电和稳定姿态正常；重心定性位于车体中部偏前；左右驱动轮都能自由转动且阻力接近。工程按原理图接线继续，不把“用户接错线”作为默认假设。暂时无法取得的质量、精确重心和高度继续在日志中明确记录为 `null`，不填写猜测值。

能取得时继续补充：

1. 原车总质量、云台质量、安装后总质量；
2. 云台重心离地高度、相对车体中心的前后和左右偏移；
3. 电池静置电压，以及两轮启动时是否明显掉压；
4. 左右轮是否压紧、打滑，灰度板离地高度是否因载荷变化；
5. 云台是否能相对车体摆动。若机械安装有松动，控制器无法消除由载荷摆动产生的蛇形。

立创工程 `F:\画板存放\ProDoc_Board1_2026-07-10(1).epro2` 已核对：TB6612 模块的 `ADC` 网络接到 MSPM0 `PB19 / ADC1_CH6`。当前固件每 10 ms 异步采样一次，并提供 `POWER`、`POWER RESET` 协议。由于顶层原理图没有给出 TB6612 模块内部/板载分压比，日志只把 `RAW` 记为 12 位 ADC 原码、把 `PINMV` 记为 MCU 引脚电压；`BATTERY_SCALE=UNKNOWN` 明确禁止把 `PINMV` 冒充电池电压。静止信号自身已有明显周期范围，因此必须比较独立静止窗口和运行窗口，不能再用一次随机 `RAW` 与窗口最小值之差推断启动掉压。

### 阶段 B：低速重载辨识，不覆盖 V4

- 已建立明确的 `LIGHT` 和 `GIMBAL` 参数档，Flash V3 保存两套完整参数和当前档位；
- 旧 Flash V1/V2 参数只迁移为 `LIGHT`；GIMBAL 低速轮速阶段使用 `SYNC=500、BIAS=80、GSTART=10`，其中 `GSTART` 会在安全完成首边后继续自动学习；启动静摩擦由运行时自适应 PWM 托举承担，不把瞬态 `31 mm/s` 错当永久稳态 GSTART；
- `PROFILE LIGHT/GIMBAL` 只在空闲时切档，`SAVE` 只更新当前档，切档前未保存的试验值不会串到另一档；
- 重载辨识先检查轮速前馈、最小 PWM、左右偏差和加速斜坡；
- 现有“悬空轮自动调参”不能代表重载落地后的滚阻与附着，不能直接拿它覆盖重载参数；
- 先在清空的直线场地依次测试 `120 → 160 → 200 mm/s`，每档通过后再升速；
- 任何失线、轮胎打滑、车体明显侧倾、云台摆动或串口断联都立即停止。

已有低速直线记录工具可作为第一轮数据采集入口：

```powershell
python .\HOST\ground_straight_test.py COM12 `
  --speed 120 --duration-ms 8000 `
  --base-mass-g BASE_G --gimbal-mass-g GIMBAL_G `
  --mounting-mass-g MOUNT_G --total-mass-g TOTAL_G `
  --gimbal-cog-height-mm COG_HEIGHT_MM `
  --gimbal-cog-forward-mm COG_FORWARD_MM `
  --gimbal-cog-left-mm COG_LEFT_MM `
  --battery-idle-v BATTERY_IDLE_V `
  --battery-startup-v BATTERY_STARTUP_V `
  --sensor-height-mm SENSOR_HEIGHT_MM `
  --sensor-height-light-mm LIGHT_SENSOR_HEIGHT_MM `
  --wheel-load balanced `
  --wheel-slip-observed none `
  --confirm-gimbal-mount-rigid `
  --confirm-ground-clear --confirm-observer-ready
```

把全大写占位值替换为实测数值；前/左偏移为正，后/右偏移为负。工具会核对总质量与部件和、记录供电及灰度板高度，强制选择并回读 `GIMBAL + Flash V3`，还会确认 ADC 已连续采样后才允许 `RUN`。工具先记录 2 秒静止 ADC 窗口，再清零并记录运行窗口；若爬坡后连续检测到两轮近零而 PWM 已很高，会自动提前发送 `STOP`。当前固件还会在 RUN 开始时设置相对航向零点，逐样本记录经过回绕、大跳变和短时 I²C 失败过滤的 yaw；汇总包括末端偏航、峰值、稳态斜率和 IMU 成功/错误计数变化。8 秒、120 mm/s 的理想直线距离约 0.87 m。输出目录包含 `load_metadata.json`、`straight_samples.csv`、`summary.json` 和协议 trace。160/200 mm/s 还必须显式带 `--previous-speed-passed`，表示已经审阅前一速度日志。

若首次 120 mm/s 检查暂时无法取得数值，可以明确把缺失项记录为 `null`，不能填写猜测值：

```powershell
python .\HOST\ground_straight_test.py COM12 `
  --speed 120 --duration-ms 8000 `
  --allow-unknown-measurements `
  --wheel-load unknown --wheel-slip-observed none `
  --confirm-gimbal-mount-rigid `
  --confirm-ground-clear --confirm-observer-ready `
  --notes "gimbal separate battery; powered and stabilized; COG center-forward"
```

它只用于安全采集，不等于自动生成重载参数，也不会修改 `LINEKP/LINEKD`。必须先审阅 120 mm/s 的左右稳态速度、纹波、PWM、里程差、偏航和压降，再决定 GIMBAL 槽内的轮速参数修改。

需要对重载左轮做受控 A/B 时，可在同一条 120 mm/s 命令中增加 `--trial-lmin 40` 和/或 `--trial-lff 500`。验证直线同步修正时可增加 `--trial-sync 500`；它限制为 `200..1000`。确认同步动态后，可用 `--trial-bias 80` 在同一轮固定 `SYNC` 并单独辨识累计里程偏置；`BIAS` 限制为固件合法范围 `-80..80`。临时 `SYNC/BIAS` 禁止与临时 `LMIN/LFF` 混测，以免混淆因果。工具只在本次运行前执行未保存的 `SET`；停车后重新装载已保存的 GIMBAL 槽并逐项核对回读，不执行 `SAVE`，同时把试验值、原值和恢复结果写入 `summary.json`。这些选项禁止用于 160/200 mm/s，也不会改动 LIGHT 槽。

阶段一接受值已经用独立事务保存并确认 LIGHT 未变。需要重新提交或回退 GIMBAL 的轮速对时使用：

```powershell
python .\HOST\pid_lab_cli.py save-gimbal-stage1 --port COM12
python .\HOST\pid_lab_cli.py restore-gimbal-v4 --port COM12
```

### 阶段 C：轮速稳定后再改循迹与转弯

- 先让左右实际速度在重载下连续、无停顿、无明显偏航，再调整循迹；
- 循迹需要按误差区间和速度做平滑增益/限幅调度，使小误差稳定、大误差仍有足够修正，不能在阈值处突然翻倍；
- 优先限制修正变化率和目标加速度，避免重心高时左右目标瞬间反向；
- Guard9 在有效线内始终使用同一套灰度循迹，按误差连续提高转向权限并降低公共速度；不再叠加有效线 RECOVER 或 IMU 航向转向。IMU 只参与观测、评分、转弯角度和 `25°` 硬保护；
- 首条边从已保存的 GIMBAL `GSTART` 立即获得轮间前馈，再利用 `SYNC/BIAS` 和重新置零后的增量编码器做有界微调，补上直线辨识与方框循迹之间原先断开的环节；不再通过人工逐轮目测去猜左右目标；
- 转弯参数独立重定：Guard9 保留每弯旧边离开、新边首见、中心稳定 yaw/mask/里程记录；90° 只开始低速搜索，完整 `RIGHT_DEPART→gap→LEFT_ENTRY` 证据连续成立才 CAPTURE。成功中心角更新 `TURNANGLE`，新边首见里程更新 `TURNDIST`；
- 保留 TurnGuard，但 50° 只是一道早捕线保护，不是重载转弯已经完成的证明；
- 自动调参从同一次放车的首条稳定直边就开始，不再要求先用固定参数完整跑一圈。主机对 `LINEKP/LINEKD` 做有界坐标小步试探，只在稳定、居中、有效线窗口切换；候选一旦使误差、摆动、无效线或 yaw 趋势恶化就立即回滚；
- 只有累计至少 4 个真实 `TURN CENTER`，并且最终参数再通过至少 2 条稳定直边验证，主机才执行一次 `SAVE`。任何失败都先 `STOP`，再完整恢复任务前 GIMBAL RAM 参数；LIGHT 永不改动。

以下命令是烧录 Guard9 并确认 `PARAM` 回读后的目标入口；Guard8 失败检查点禁止运行：

```powershell
python .\HOST\pid_lab_cli.py gimbal-auto --port COM12 --speed 120 --track-safe
```

该命令先选择 GIMBAL 并强制确认固件版本；`SYNC/BIAS` 只在不匹配时提交为阶段一接受值 `500/80`，已有合法非零 `GSTART` 原样保留。主机从第一条稳定直边开始积累基线，对 `LINEKP/LINEKD` 做有界坐标小步试探，每次只改一个坐标；参数切换、评分、恶化回滚、转弯搜索、角度/里程学习和后续直边验证都在同一次连续方框任务中进行。累计至少 4 个有效中心弯并再通过至少 2 条直边后才单次保存；失败不会留下候选参数。

第 0 节列出的有符号 CCW yaw、完整扫线相位、角度/里程窗口和捕获连续性修复已经完成，并已升级 `GUARDVER=9`、补回归、无警告构建和不可变 HEX/SHA-256 归档。下一步是在现场可断电观察下仅运行一次上面的 120 mm/s `gimbal-auto`，根据完整日志验收或继续单点修正。

建议验收顺序为：重载直线 120、重载方框 120、160、200，各速度连续多圈无失线后才考虑更高速度。不要一开始追求 380 mm/s。

## 7. 上位机和终端入口

首次安装依赖：

```powershell
python -m pip install -r .\HOST\requirements.txt
```

启动 GUI：

```powershell
python .\HOST\pid_autotune_gui.py
```

也可以双击 `HOST/start_autotune.bat`。

常用只读或安全命令：

```powershell
python .\HOST\pid_lab_cli.py ports
python .\HOST\pid_lab_cli.py diagnose --port COM12
python .\HOST\pid_lab_cli.py params --port COM12
python .\HOST\pid_lab_cli.py stop --port COM12
python .\HOST\pid_lab_cli.py profile --port COM12 --name light
python .\HOST\pid_lab_cli.py profile --port COM12 --name gimbal
python .\HOST\pid_lab_cli.py save-gimbal-stage1 --port COM12
python .\HOST\pid_lab_cli.py restore-gimbal-v4 --port COM12
python .\HOST\pid_lab_cli.py restore-v4 --port COM12
```

`diagnose` 现在会额外输出 `POWER,OK,...,BATTERY_SCALE,UNKNOWN`；若 ADC 尚未产生有效结果会直接报错，不会继续把该固件当作可采集版本。

方框命令会驱动车辆，必须先清空并封闭场地：

```powershell
python .\HOST\pid_lab_cli.py square --port COM12 --speed 120 --laps 1 --turn-distance 98 --track-safe
```

重云台初测不要直接运行 `wheel-tune` 或 `line-tune`。先完成第 6 节的机械、供电和低速数据检查。

## 8. 工程保留结构

- `main.c`、`app_config.h`：主程序和默认参数；
- `LAB/`：命令协议、轮速闭环、V4 循迹和方框状态机；
- `MOTOR/`、`ENCODER/`、`SENSOR/`、`MPU/`、`POWER/`、`SERIAL/`：底层硬件模块，其中 `POWER/` 采集 TB6612 ADC 到 PB19 的原码和窗口极值；
- `ESP8266_BRIDGE/`：PC 与车端的 ESP8266/串口桥；
- `HOST/pid_autotune_gui.py`：GUI 与自动调参核心；
- `HOST/pid_lab_cli.py`：无界面诊断、回读、分档恢复、方框记录、Guard9 `TURN SEARCH/CAPTURE/CENTER` 事件归档，以及同一次放车内从首边开始的 GIMBAL 坐标小步调参、验证、回滚和单次保存；
- `HOST/ground_straight_test.py`：强制载荷元数据、GIMBAL 档回读、ADC 静止/运行窗口比较、连续相对 yaw、失速提前 STOP 和 120→160→200 门控的落地直线采集；
- `HOST/imu_motor_interference_test.py`、`HOST/bridge_stress_test.py`：载荷与供电改变后仍有价值的诊断工具；
- `HOST/test_pid_autotune.py`：上位机回归测试；
- `build_mspm0_hex.ps1`：MSPM0 无警告构建脚本；
- `docs/HARDWARE_REFERENCES.md`：硬件资料索引；
- `Debug/`：保留当前 Guard9 HEX、同哈希 Guard9 候选归档、Guard8/7 实车失败回退件、Guard6 实车误停回退件、历次 Guard5/4 检查点、GSTART/启动/包络/ADC 修改前回退件、V4-TurnGuard 回退件和精确 V4 回退件；其他内容可由构建脚本重新生成。

已清理内容包括：过时调参冠军文件、已否决循迹试验日志、重复试跑、旧诊断 HEX、旧别名 HEX、对象文件、链接中间文件、SysConfig 临时输出和 Python 缓存。不要把这些历史结果重新当作当前参数来源。

## 9. 构建与自检

在工程根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_mspm0_hex.ps1
powershell -ExecutionPolicy Bypass -File .\build_v4_light_hex.ps1
python -m py_compile .\HOST\pid_autotune_gui.py .\HOST\pid_lab_cli.py .\HOST\ground_straight_test.py
python -m unittest discover -s .\HOST -p "test_*.py"
```

正常构建只更新 `Debug/pid_lab_mspm0.hex`，不会覆盖任何显式归档/回退件。重新构建后必须重新记录大小和 SHA-256，再复制成新的候选归档。构建脚本以 `-Wall -Wextra -Werror` 强制 warning 失败；Guard9 的回归重点包含：`065843` 全程有效线不能触发另一套有效线恢复控制器，`073722` 的 `90.2°/mask0` 不得进入 CAPTURE，正确 `mask1→3→2` 序列必须允许捕获，单独 `mask24`、IMU 失效、过角/过里程和刹停/ALIGN 丢线必须 fail-stop，以及失败任务不得 `SAVE`。当前首边限速版 HEX 为 109,103 B、SHA-256 `BECE426327DFE2DABF0FB8FEC0EDD2E4549C8F69A3CDDC99B20C3E2F5B7457CF`；归档名为 `pid_lab_mspm0_guard9_firstedge100_20260717.hex`。上一版直线包络归档为 108,965 B、SHA-256 `D4F0EB1B54DF0F7BD4DAFE638C84BCC54E881AEA4B9B0765886E2AB7A8066313`。上一版 Guard9 转弯门禁归档为 108,994 B、SHA-256 `CD7A871ACB4CA7B851BB9E7B76F17171EA5DD34182DF0B17DEDBBD0E2A236777`。Guard8 失败回退件仍为 108,365 B、SHA-256 `132F616C025C352AF4809C2E3EE649D737AD97669A1BBEE6F1061CEC4913020D`。轻载 V4-TurnGuard/精确 V4 仍分别为 `E8EA86F3A9CD9BA089644B90D8C7E6922221B08E38C6136FD7068A4648A36A93` 和 `39AB0AF11D68C6541B7C159AD4590EF7AFE632C9B79B4F9DA43B3ED5CF7B5AA1`。

## 10. 新对话直接使用的开场提示

```text
请先完整阅读 README.md，再检查当前源码和保留日志。工程唯一基线是 V4：
Debug/pid_lab_mspm0_v4_exact.hex 是截图中的精确 V4；
Debug/pid_lab_mspm0_v4_turnguard.hex 是修改前的 V4 直线循迹原样加最小 50° TurnGuard 回退件；
Debug/pid_lab_mspm0.hex 的 LIGHT 仍保持 V4-TurnGuard；GIMBAL 为 Guard9：直线和 CAPTURE_ALIGN 只由灰度主控，首边恢复120 mm/s、起步斜坡500 ms，丢线早期保留最后一次灰度转向，直线差速由 PID/自动调参决定；IMU 只做观测/评分和宽松航向保护。转弯继续使用有符号逆时针 yaw 与必要的捕线顺序，避免过早把旧边当成新边；刹停/ALIGN 无线不能向前。第8路继续屏蔽。当前 HEX 为109196 B，SHA-256 6BC9485047A9B100AE7E3450632F1C8508180D3CE5906B3C9581E1CBBE51F4D1；同哈希归档为 Debug/pid_lab_mspm0_guard9_recovery_pid_20260717.hex。
轻载 V4 已在 300/340/380 mm/s 完成过方框，但安装很重的云台后已经不能正常工作。
不要恢复已否决的中间循迹算法，不要覆盖轻载 V4 参数，不要先盲调 LINEKP/LINEKD。
已确认云台独立供电且与小车无电气连接、姿态稳定、重心中部偏前、两轮机械阻力接近；不要默认怀疑用户接线。重载阶段一使用 `SYNC=500、BIAS=80、GSTART=10`，LIGHT 仍为 V4 `SYNC=1000、BIAS=50`。07:37 Guard7 实车首边701帧全有效却持续蛇形，5次恢复和多次误差/yaw换向证明灰度、IMU航向、恢复状态机互相冲突；首弯又在90.2°、mask0时错误CAPTURE，随后60帧全invalid并真正LINE LOST，无SAVE。Guard8改为灰度单主控，但08:30首次实车暴露CLI启动竞态，日志只到440ms且无候选、无SAVE；日志中断后用户观察到首边仍蛇形、首弯过转到起步直线并原路返回。CLI竞态已修复。Guard9 已实现有符号CCW yaw、75°～110°窗口、RIGHT_DEPART→连续gap→LEFT_ENTRY、0.75～1.8×TURNDIST和捕获连续性门禁，62项测试与无警告构建通过，新HEX已归档；尚未实车验收。下一步烧录后回读GUARDVER=9，仅跑120 mm/s gimbal-auto并保留完整日志。LIGHT不变。
每次修改都要保留可回退 HEX、明确 SHA-256，并先跑上位机测试和固件无警告构建。
```
