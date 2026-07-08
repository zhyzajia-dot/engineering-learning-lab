"""
pid_autotune_gui.py - MSPM0 PI / 循迹 / 逆时针方框参数自动整定上位机

主要功能：
  1. 通过 USB-TTL（或 ESP-NOW 桥接）连接小车，提供串口收发面板
  2. 对左右轮的 PI / FF / MIN_PWM 参数做开环 + 阶跃自动寻优
  3. 对循迹外环 (LINEKP / LINEKD) 做自动寻优
  4. 启动逆时针方框模式，并实时绘制目标/速度曲线、记录日志
  5. 支持脱机直线测试 (ARM) 和上次日志回放 (DUMP)

使用：
  pip install -r requirements.txt
  python pid_autotune_gui.py
"""

from __future__ import annotations

import csv
import json
import math
import queue
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


# ----------------------------------------------------------------
# 串口 / 调参常量
# ----------------------------------------------------------------

BAUD_RATE = 115200  # 与下位机约定的串口波特率

# 单轮阶跃自动寻优时的目标速度（mm/s）
TARGET_SPEED_MMPS = 300

# 开环前馈辨识：在每个 PWM 上停留 1.25 s，稳态后取均值
OPEN_PWM_POINTS = (70, 100, 130, 160, 190, 220)

# 单轮 PI 自动寻优时扫描的候选值
KP_CANDIDATES = (200, 350, 500, 650, 800, 1000)
KI_CANDIDATES = (0, 50, 100, 200, 350, 500)

# 循迹外环 LINEKP / LINEKD 自动寻优时扫描的候选值
LINE_KP_CANDIDATES = (4500, 6000, 7500, 9000)
LINE_KD_CANDIDATES = (1500, 2500, 3000, 3500)

# 循迹调参时使用的目标速度 (mm/s)
LINE_TUNE_SPEED_MMPS = 280

# 调参过程中注入的“探测差速”幅度和持续时间
LINE_PROBE_KICK_MMPS = 55
LINE_PROBE_KICK_MS = 300

# 直线同步外环机械偏置（与下位机 LAB_DEFAULT_RIGHT_BIAS_MMPS 一致）
STRAIGHT_BIAS_X10000 = 25

# 循迹参数的安全范围（防止越界写入）
LINE_KP_SAFE_RANGE = (2500, 14000)
LINE_KD_SAFE_RANGE = (500, 6000)

# 自动整定评分公式的版本号；变更打分逻辑时同步 +1，
# 旧的 champion.json 在 score_version 不匹配时会被忽略
LINE_SCORE_VERSION = 2


@dataclass
class PidSample:
    """单帧 PID 上传样本：与下位机 send_stream_sample() 的字段一一对应。

    顺序和下位机固件强绑定：每加一列必须同时改下位机 csv 头、这里的字段、
    parse() 里的索引以及 _save_samples() 的表头。"""
    time_ms: int
    mode: str
    left_target: int
    right_target: int
    left_speed: int
    right_speed: int
    left_pwm: int
    right_pwm: int
    left_error: int
    right_error: int
    count_diff: int
    # 下面是循迹 / 方框相关扩展字段，旧固件可能没有，需要做长度兼容
    line_error: int = 0
    line_mask: int = 0
    line_valid: int = 0
    yaw_x10: int = 0
    corner_count: int = 0
    square_state: int = 0
    turn_travel_mm: int = 0

    @classmethod
    def parse(cls, line: str) -> "PidSample | None":
        """解析下位机发来的一行 'PID,...'，列数不匹配或前缀错误都返回 None。

        列数分别对应旧固件（12）、加循迹（15）、加方框（18）、加转弯里程（19）。"""
        parts = line.strip().split(",")
        if len(parts) not in (12, 15, 18, 19) or parts[0] != "PID":
            return None
        try:
            # 注意：parts[2] 是 mode 字符串，所以把第 1 列和第 3 列以后拼起来一起转 int
            values = [int(value) for value in parts[1:2] + parts[3:]]
        except ValueError:
            return None
        return cls(
            time_ms=values[0],
            mode=parts[2],
            left_target=values[1],
            right_target=values[2],
            left_speed=values[3],
            right_speed=values[4],
            left_pwm=values[5],
            right_pwm=values[6],
            left_error=values[7],
            right_error=values[8],
            count_diff=values[9],
            # 越界时取 0：下位机是按可选列追加的
            line_error=values[10] if len(values) > 10 else 0,
            line_mask=values[11] if len(values) > 11 else 0,
            line_valid=values[12] if len(values) > 12 else 0,
            yaw_x10=values[13] if len(values) > 13 else 0,
            corner_count=values[14] if len(values) > 14 else 0,
            square_state=values[15] if len(values) > 15 else 0,
            turn_travel_mm=values[16] if len(values) > 16 else 0,
        )

    def speed(self, wheel: str) -> int:
        """按 'L' / 'R' 取对应轮的实际速度。"""
        return self.left_speed if wheel == "L" else self.right_speed

    def target(self, wheel: str) -> int:
        """按 'L' / 'R' 取对应轮的目标速度。"""
        return self.left_target if wheel == "L" else self.right_target

    def pwm(self, wheel: str) -> int:
        """按 'L' / 'R' 取对应轮的 PWM 占空比。"""
        return self.left_pwm if wheel == "L" else self.right_pwm


class SerialLink:
    """串口收发管理。

    三个关键设计：
      - 独立的读线程：read() 阻塞不会卡住 UI
      - 两个队列：data_queue 只给 AutoTuner 用（程序化解析），
        display_queue 同步给所有原始行（用于界面日志/绘图）
      - 写加锁：避免 UI 和后台线程同时发命令时把帧交错"""

    def __init__(self) -> None:
        self.serial: serial.Serial | None = None
        # AutoTuner 关心的数据行（每行完整）
        self.data_queue: queue.Queue[str] = queue.Queue()
        # UI 日志 / 绘图用的所有原始行（包括 PONG / STATUS / SENSOR 等）
        self.display_queue: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.reader_thread: threading.Thread | None = None
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        """串口是否处于打开状态。"""
        return self.serial is not None and self.serial.is_open

    def connect(self, port: str) -> None:
        """打开串口并启动读线程。

        关闭 DTR / RTS 是为了避免部分 CH340 模块在打开瞬间复位下位机；
        清空两个缓冲区可以丢掉上次会话的残留数据。"""
        self.close(send_stop=False)
        self.serial = serial.Serial(
            port,
            BAUD_RATE,
            timeout=0.1,
            write_timeout=1.5,
        )
        try:
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
        except (OSError, serial.SerialException):
            # 打开后配置失败：保证不留半开状态
            self.serial.close()
            self.serial = None
            raise
        self.stop_event.clear()
        self.reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True
        )
        self.reader_thread.start()

    def close(self, send_stop: bool = True) -> None:
        """停止读线程并关闭串口。

        send_stop=True 时先给下位机发一条 STOP 命令，避免它持续输出。
        任何异常都被吞掉：断连过程不允许再抛错给用户。"""
        self.stop_event.set()
        if send_stop and self.connected:
            try:
                self.send("STOP")
            except (OSError, RuntimeError, serial.SerialException):
                pass
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        self.serial = None

    def send(self, command: str) -> None:
        """发出一条 ASCII 命令（自动加 \\r\\n）。

        写超时允许重试 1 次：第一次失败时清空发送缓冲后等 200 ms 再发。
        仍然失败则抛出异常给调用方，UI 弹窗提示。"""
        if not self.connected:
            raise RuntimeError("Serial port is not connected")
        payload = (command.strip() + "\r\n").encode("ascii")
        with self.write_lock:
            assert self.serial is not None
            last_error: Exception | None = None
            for attempt in range(2):
                try:
                    written = self.serial.write(payload)
                    if written != len(payload):
                        raise serial.SerialTimeoutException(
                            f"short serial write: {written}/{len(payload)}"
                        )
                    return
                except serial.SerialTimeoutException as exc:
                    last_error = exc
                    try:
                        self.serial.reset_output_buffer()
                    except serial.SerialException:
                        break
                    if attempt == 0:
                        time.sleep(0.2)
            raise serial.SerialTimeoutException(
                "COM write timeout after retry; reconnect the CH340"
            ) from last_error

    def drain_data(self) -> None:
        """清空 data_queue 的所有待处理行。

        每次切换新一段测试前调用，避免上一段残留的 PONG/STATUS 干扰本次断言。"""
        while True:
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                return

    def _reader_loop(self) -> None:
        """独立读线程：把串口字节流按 \\n 切行，再分别塞进 data / display 队列。"""
        pending = bytearray()
        while not self.stop_event.is_set():
            try:
                assert self.serial is not None
                raw = self.serial.read(256)
            except (serial.SerialException, OSError) as exc:
                if not self.stop_event.is_set():
                    # 主动关闭时不刷错误信息；被动断开才提示
                    self.display_queue.put(f"SERIAL ERROR: {exc}")
                return
            if not raw:
                continue
            pending.extend(raw)
            # 按行切分：剩下的不完整字节会留在 pending 等待下一帧
            while b"\n" in pending:
                raw_line, _, pending = pending.partition(b"\n")
                line = raw_line.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                self.data_queue.put(line)
                self.display_queue.put(line)


class AutoTuner:
    """自动整定核心。所有实际和小车对话的工作都在这里。

    内部用独立工作线程：
      - start(wheels)        -> 跑单/双轮 PI / FF / MIN_PWM 寻优
      - start_line()         -> 跑方框模式下的循迹 LINEKP / LINEKD 寻优

    每次寻优的中间结果都会写进 logs/<时间戳>/ 目录，方便事后回看。"""

    def __init__(
        self,
        link: SerialLink,
        status_queue: queue.Queue[str],
        finished_callback,
    ) -> None:
        self.link = link
        # 推给 UI 的状态文本（每行一条）
        self.status_queue = status_queue
        # 工作线程结束时由工作线程调用，UI 据此恢复按钮状态
        self.finished_callback = finished_callback
        # 用户中途点 STOP 时设置这个标志
        self.cancel_event = threading.Event()
        self.thread: threading.Thread | None = None
        # 当前寻优会话的输出目录（每次启动时新建一个时间戳子目录）
        self.session_dir: Path | None = None
        # {'L': {...}, 'R': {...}}，保存最终写给小车的参数
        self.results: dict[str, dict[str, int | float]] = {}

    @property
    def running(self) -> bool:
        """是否还有寻优线程在跑（供 UI 决定是否禁用按钮）。"""
        return self.thread is not None and self.thread.is_alive()

    def start(self, wheels: tuple[str, ...]) -> None:
        """启动单/双轮 PI 自动寻优线程。重复调用会被忽略。"""
        if self.running:
            return
        self.cancel_event.clear()
        self.thread = threading.Thread(
            target=self._run, args=(wheels,), daemon=True
        )
        self.thread.start()

    def start_line(self) -> None:
        """启动循迹 LINEKP / LINEKD 自动寻优线程。重复调用会被忽略。"""
        if self.running:
            return
        self.cancel_event.clear()
        self.thread = threading.Thread(
            target=self._run_line, daemon=True
        )
        self.thread.start()

    def cancel(self) -> None:
        """用户点 STOP：设置取消标志并尝试立即让小车停下来。"""
        self.cancel_event.set()
        if self.link.connected:
            try:
                self.link.send("STOP")
            except Exception:
                pass

    def _status(self, text: str) -> None:
        """给 UI 推一条状态文本。"""
        self.status_queue.put(text)

    def _check_cancelled(self) -> None:
        """在长循环里周期性调用：用户取消时直接抛异常跳出。"""
        if self.cancel_event.is_set():
            raise RuntimeError("Autotune cancelled")

    def _run(self, wheels: tuple[str, ...]) -> None:
        """单/双轮 PI 寻优主流程。

        步骤：
          1. 新建 logs/<时间戳> 目录
          2. PING/PONG 验证连接
          3. 对每个轮子跑 _tune_wheel()（辨识前馈 + 搜 Kp/Ki + 验证）
          4. 写 SET BIAS、SAVE，把最终参数交给下位机 Flash
          5. 把 results 写到 tuned_params.json"""
        try:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = Path(__file__).parent / "logs" / stamp
            self.session_dir.mkdir(parents=True, exist_ok=True)

            self._verify_connection()
            for wheel in wheels:
                self._check_cancelled()
                self.results[wheel] = self._tune_wheel(wheel)

            self.link.drain_data()
            self._send_set("BIAS", STRAIGHT_BIAS_X10000)
            self.link.send("SAVE")
            if self._wait_for(lambda item: item == "OK SAVE", 3.0) is None:
                raise RuntimeError("MCU could not save parameters to flash")

            result_path = self.session_dir / "tuned_params.json"
            result_path.write_text(
                json.dumps(self.results, indent=2), encoding="ascii"
            )
            self._status(f"COMPLETE: parameters saved to {result_path}")
        except Exception as exc:
            self._status(f"FAILED: {exc}")
            try:
                self.link.send("STOP")
            except Exception:
                pass
        finally:
            # 任何出口都要通知 UI 恢复按钮
            self.finished_callback()

    def _verify_connection(self) -> None:
        """发 PING 等 PONG，确认下位机协议在线。"""
        self.link.drain_data()
        self.link.send("PING")
        line = self._wait_for(lambda item: item == "PONG", 2.0)
        if line is None:
            raise RuntimeError("No PONG from firmware")
        self._status("Firmware connection verified")

    def _run_line(self) -> None:
        """方框模式下的循迹 LINEKP / LINEKD 自动寻优主流程。

        步骤：
          1. 验证连接 + 灰度 + IMU 都正常
          2. 启动 SQUARE 10 圈作为测试舞台
          3. 粗扫 LINEKP -> 精扫 LINEKP（2 轮局部搜索）
          4. 用当前最佳 Kp 粗扫 LINEKD -> 精扫 LINEKD
          5. 在 (best_kp, best_kd) 周围做 3x3 联合寻优
          6. 和历史 champion 对比，把更优的写为新 champion
          7. 多次验证取中位数作为最终分数
          8. SET LINEKP/LINEKD + SAVE"""
        try:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = (
                Path(__file__).parent / "logs" / f"line_{stamp}"
            )
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self._verify_connection()
            # 灰度检查
            self.link.drain_data()
            self.link.send("SENSOR")
            sensor_line = self._wait_for(
                lambda item: item.startswith("SENSOR,"), 2.0
            )
            if sensor_line is None:
                raise RuntimeError("no response from line sensor check")
            sensor_parts = sensor_line.split(",")
            if (
                len(sensor_parts) < 3 or
                sensor_parts[1] != "OK" or
                int(sensor_parts[2]) == 0
            ):
                raise RuntimeError(
                    f"line sensor is not ready or sees no line: {sensor_line}"
                )
            self._status(f"Line sensor verified: {sensor_line}")

            # IMU 检查
            self.link.drain_data()
            self.link.send("IMU")
            imu_line = self._wait_for(
                lambda item: item.startswith("IMU,"), 2.0
            )
            if imu_line is None or not imu_line.startswith("IMU,OK,"):
                raise RuntimeError(
                    f"IMU is not ready for square autotune: {imu_line}"
                )

            # 让小车在方框模式下持续跑，方便我们在每个 Kp/Kd 组合下打分
            self.link.drain_data()
            self.link.send(f"SQUARE {LINE_TUNE_SPEED_MMPS} 10")
            started = self._wait_for(
                lambda item: item.startswith("TEST START,SQUARE"), 3.0
            )
            if started is None:
                raise RuntimeError("square line-autotune could not start")
            self._status(
                "Square line autotune running; turns are excluded from scoring"
            )

            # 粗扫 + 精扫 LINEKP
            kp_scores = [
                (
                    self._run_square_line_candidate(kp, 0, f"kp_{kp}"),
                    kp,
                )
                for kp in LINE_KP_CANDIDATES
            ]
            kp_scores.sort()
            best_kp = kp_scores[0][1]
            best_kp = self._refine_line_axis(
                "kp", best_kp, 0, kp_scores, 1500
            )

            # 用当前 best_kp 粗扫 + 精扫 LINEKD
            kd_scores = [
                (
                    self._run_square_line_candidate(
                        best_kp, kd, f"kd_{kd}"
                    ),
                    kd,
                )
                for kd in LINE_KD_CANDIDATES
            ]
            kd_scores.sort()
            best_kd = kd_scores[0][1]
            best_kd = self._refine_line_axis(
                "kd", best_kd, best_kp, kd_scores, 750
            )

            # 在 (best_kp, best_kd) 附近做 3x3 联合寻优，
            # 用更小的 sample_target 以便更快出结果
            joint_scores: list[tuple[float, int, int]] = []
            for kp_delta in (-750, 0, 750):
                for kd_delta in (-500, 0, 500):
                    kp = max(
                        LINE_KP_SAFE_RANGE[0],
                        min(LINE_KP_SAFE_RANGE[1],
                            best_kp + kp_delta),
                    )
                    kd = max(
                        LINE_KD_SAFE_RANGE[0],
                        min(LINE_KD_SAFE_RANGE[1],
                            best_kd + kd_delta),
                    )
                    score = self._run_square_line_candidate(
                        kp,
                        kd,
                        f"joint_kp{kp}_kd{kd}",
                        sample_target=80,
                    )
                    joint_scores.append((score, kp, kd))

            # 读取历史冠军。旧冠军不会凭一次短采样就被覆盖，最后会与挑战者
            # 使用相同长度、相同次数的测试做正面对比。
            champion_path = (
                Path(__file__).parent / "logs" / "best_line_params.json"
            )
            champion: dict | None = None
            if champion_path.exists():
                try:
                    champion = json.loads(
                        champion_path.read_text(encoding="ascii")
                    )
                    if champion.get("score_version") != LINE_SCORE_VERSION:
                        champion = None
                    elif not {
                        "line_kp_x1000", "line_kd_x1000"
                    }.issubset(champion):
                        champion = None
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    champion = None
            joint_scores.sort()
            _, challenger_kp, challenger_kd = joint_scores[0]

            def validate_candidate(kp: int, kd: int, label: str) -> list[float]:
                """同一候选至少测两次；波动过大时自动补第三次。"""
                scores = [
                    self._run_square_line_candidate(
                        kp, kd, f"{label}_{index + 1}",
                        sample_target=150,
                    )
                    for index in range(2)
                ]
                if (max(scores) - min(scores) >
                        0.25 * statistics.mean(scores)):
                    scores.append(self._run_square_line_candidate(
                        kp, kd, f"{label}_3", sample_target=150
                    ))
                return scores

            challenger_scores = validate_candidate(
                challenger_kp, challenger_kd, "challenger_validation"
            )
            challenger_score = statistics.median(challenger_scores)
            incumbent_scores: list[float] = []
            selected_reason = "no compatible incumbent"
            champion_replaced = True
            best_kp, best_kd = challenger_kp, challenger_kd
            final_score = challenger_score

            if champion is not None:
                old_kp = int(champion["line_kp_x1000"])
                old_kd = int(champion["line_kd_x1000"])
                if (old_kp, old_kd) != (challenger_kp, challenger_kd):
                    incumbent_scores = validate_candidate(
                        old_kp, old_kd, "incumbent_validation"
                    )
                    incumbent_score = statistics.median(incumbent_scores)
                    # 新参数至少实测提升 3% 才替换冠军，防止环境噪声造成退化。
                    if challenger_score > incumbent_score * 0.97:
                        best_kp, best_kd = old_kp, old_kd
                        final_score = incumbent_score
                        champion_replaced = False
                        selected_reason = (
                            "incumbent kept: challenger improvement < 3%"
                        )
                    else:
                        selected_reason = "challenger improved median score >= 3%"
                else:
                    champion_replaced = False
                    selected_reason = "search rediscovered incumbent"

            validation_scores = (
                challenger_scores if champion_replaced else
                (incumbent_scores or challenger_scores)
            )
            validation_count = 150 * len(validation_scores)

            self.link.send("STOP")
            self._wait_for(lambda item: item == "OK STOP", 2.0)
            self._send_set("LINEKP", best_kp)
            self._send_set("LINEKD", best_kd)
            self.link.drain_data()
            self.link.send("SAVE")
            if self._wait_for(lambda item: item == "OK SAVE", 3.0) is None:
                raise RuntimeError("MCU could not save line parameters")

            result = {
                "line_kp_x1000": best_kp,
                "line_kd_x1000": best_kd,
                "validation_score": round(final_score, 5),
                "validation_samples": validation_count,
                "validation_scores": [
                    round(value, 5) for value in validation_scores
                ],
                "challenger": {
                    "line_kp_x1000": challenger_kp,
                    "line_kd_x1000": challenger_kd,
                    "scores": [round(value, 5)
                               for value in challenger_scores],
                },
                "incumbent_scores": [
                    round(value, 5) for value in incumbent_scores
                ],
                "selection_reason": selected_reason,
                "search": "coarse + 2x local + joint neighborhood",
                "score_version": LINE_SCORE_VERSION,
            }
            result_path = self.session_dir / "line_tuned_params.json"
            result_path.write_text(
                json.dumps(result, indent=2), encoding="ascii"
            )
            # 只有确实胜出或没有旧冠军时才覆盖全局最佳参数文件。
            if champion_replaced:
                champion_path.write_text(
                    json.dumps(result, indent=2), encoding="ascii"
                )
            self._status(
                f"LINE COMPLETE: Kp={best_kp}/1000, "
                f"Kd={best_kd}/1000, saved to {result_path}"
            )
        except Exception as exc:
            self._status(f"LINE FAILED: {exc}")
            try:
                self.link.send("STOP")
            except Exception:
                pass
        finally:
            self.finished_callback()

    def _refine_line_axis(
        self,
        axis: str,
        initial: int,
        fixed: int,
        seed_scores: list[tuple[float, int]],
        initial_step: int,
    ) -> int:
        """对单个轴 (kp 或 kd) 做两轮局部搜索。

        每轮在当前最佳值两侧等间距取 5 个候选，
        步长每轮减半。第二轮起点用第一轮结果。"""
        bounds = (
            LINE_KP_SAFE_RANGE if axis == "kp" else LINE_KD_SAFE_RANGE
        )
        measured = {value: score for score, value in seed_scores}
        best = initial
        step = initial_step

        for round_index in range(2):
            offsets = (-step, -(step // 2), 0, step // 2, step)
            candidates = sorted(
                {
                    max(bounds[0], min(bounds[1], best + offset))
                    for offset in offsets
                }
            )
            for value in candidates:
                if value in measured:
                    # 之前粗扫已经测过了就不再重复
                    continue
                kp = value if axis == "kp" else fixed
                kd = fixed if axis == "kp" else value
                measured[value] = self._run_square_line_candidate(
                    kp,
                    kd,
                    f"refine_{axis}_r{round_index + 1}_{value}",
                    sample_target=80,
                )
            best = min(measured, key=measured.get)
            self._status(
                f"{axis.upper()} refine round {round_index + 1}: "
                f"best={best}/1000 score={measured[best]:.3f}"
            )
            step = max(250 if axis == "kp" else 125, step // 2)
        return best

    def _run_square_line_candidate(
        self,
        kp: int,
        kd: int,
        name: str,
        sample_target: int = 100,
    ) -> float:
        """在线评估一组 (LINEKP, LINEKD) 在方框模式直线段上的表现。

        工作方式：
          - 先 SET 一组 Kp/Kd
          - 等方框模式跑出一段直线，注入两个相反方向的 KICK 来激起响应
          - 等 sample_target 个有效直线样本后调用 _score_line 打分
          - 把原始样本落盘 line_<name>.csv"""
        self._check_cancelled()
        self._send_set("LINEKP", kp)
        self._send_set("LINEKD", kd)
        self._status(
            f"Square live: Kp={kp}/1000, Kd={kd}/1000"
        )

        samples: list[PidSample] = []
        deadline = time.monotonic() + 12.0
        kick_stage = 0
        settle_samples = 20
        while len(samples) < sample_target and time.monotonic() < deadline:
            self._check_cancelled()
            if kick_stage == 0 and len(samples) >= 15:
                # 先向右踢一脚
                self.link.send(
                    f"KICK {LINE_PROBE_KICK_MMPS} {LINE_PROBE_KICK_MS}"
                )
                kick_stage = 1
            elif kick_stage == 1 and len(samples) >= sample_target // 2:
                # 再向左踢一脚，构成一次正反扰动
                self.link.send(
                    f"KICK {-LINE_PROBE_KICK_MMPS} {LINE_PROBE_KICK_MS}"
                )
                kick_stage = 2
            try:
                line = self.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if line.startswith("SQUARE ERROR"):
                raise RuntimeError(line)
            if line.startswith("SQUARE DONE"):
                raise RuntimeError("square ended before autotune completed")
            sample = PidSample.parse(line)
            if (
                sample is not None
                and sample.mode == "SQUARE"
                and sample.square_state == 0  # 只在 LINE 状态打分（排除转弯）
                and sample.time_ms >= 500     # 排除斜坡启动阶段
            ):
                if settle_samples > 0:
                    # 跳过前 20 个样本，让速度进入稳态
                    settle_samples -= 1
                    continue
                samples.append(sample)
                if max(sample.left_speed, sample.right_speed) > 1000:
                    # 异常速度：主动停机并报错
                    self.link.send("STOP")
                    raise RuntimeError("Unsafe encoder speed detected")

        if len(samples) < sample_target:
            raise RuntimeError(
                f"incomplete square line window: {len(samples)}/"
                f"{sample_target}"
            )
        score = self._score_line(samples)
        self._save_samples(f"line_{name}.csv", samples)
        self._status(
            f"Square live: {name} score={score:.3f}"
        )
        return score

    @staticmethod
    def _score_line(samples: list[PidSample]) -> float:
        """给一段循迹 / 方框直线段样本打一个综合分（越小越好）。

        组成项：
          - 居中误差均值      weight = 1.0
          - 误差波动 (pstdev) weight = 0.8
          - 误差一阶导        weight = 0.6
          - 丢线比例          weight = 5.0 （强烈惩罚）
          - 速度偏差          weight = 0.4
          - 收敛时间          weight = 1.5
          - 超调量            weight = 1.0
          - 蛇形次数          weight = 1.2
        最后对整个段做两段切分（前后半），分别统计收敛/超调/蛇形后取均值。"""
        settled = [sample for sample in samples if sample.time_ms >= 500]
        if not settled:
            return math.inf
        valid = [sample for sample in settled if sample.line_valid != 0]
        if len(valid) < len(settled) * 0.8:
            # 丢线超过 20%：直接给一个惩罚
            return math.inf
        errors = [sample.line_error for sample in valid]
        # 把误差除以 36（最大可能幅度）归一化
        mean_error = statistics.mean(abs(error) for error in errors) / 36.0
        ripple = (
            statistics.pstdev(errors) / 36.0 if len(errors) > 1 else 0.0
        )
        derivative = (
            statistics.mean(
                abs(errors[index] - errors[index - 1])
                for index in range(1, len(errors))
            ) / 36.0
            if len(errors) > 1 else 0.0
        )
        lost_ratio = 1.0 - (len(valid) / len(settled))
        speed_error = statistics.mean(
            abs(
                LINE_TUNE_SPEED_MMPS -
                ((sample.left_speed + sample.right_speed) / 2.0)
            )
            for sample in valid
        ) / LINE_TUNE_SPEED_MMPS

        # 把样本拆成前后两段分别统计：避免早期 KICK 残余影响后半段打分
        recovery_values: list[float] = []
        overshoot_values: list[float] = []
        snake_values: list[float] = []
        split = len(errors) // 2
        for start, end in ((0, split), (split, len(errors))):
            segment = errors[start:end]
            if len(segment) < 8:
                continue
            peak_local = max(
                range(len(segment)),
                key=lambda index: abs(segment[index]),
            )
            peak_error = segment[peak_local]
            peak_abs = abs(peak_error)
            if peak_abs < 6:
                # 这段本身很稳：无收敛 / 超调 / 蛇形惩罚
                recovery_values.append(0.0)
                overshoot_values.append(0.0)
                snake_values.append(0.0)
                continue

            # 收敛时间：从峰值往后找连续 5 个样本 |err| <= 4 的位置
            settled_local: int | None = None
            for index in range(peak_local, len(segment) - 4):
                if all(
                    abs(segment[probe]) <= 4
                    for probe in range(index, index + 5)
                ):
                    settled_local = index
                    break
            if settled_local is None:
                recovery_values.append(2.0)
            else:
                recovery_ms = (
                    valid[start + settled_local].time_ms -
                    valid[start + peak_local].time_ms
                )
                recovery_values.append(
                    min(2.0, recovery_ms / 1200.0)
                )

            # 超调：峰值之后方向相反的最大误差占峰值比例
            opposite = [
                abs(error) for error in segment[peak_local + 1:]
                if error * peak_error < 0
            ]
            overshoot_values.append(
                (max(opposite) / peak_abs) if opposite else 0.0
            )

            # 蛇形：峰值之后 |err|>=4 的符号变化次数 / 4
            signs = [
                1 if error > 0 else -1
                for error in segment[peak_local:]
                if abs(error) >= 4
            ]
            sign_changes = sum(
                1 for index in range(1, len(signs))
                if signs[index] != signs[index - 1]
            )
            snake_values.append(min(2.0, sign_changes / 4.0))

        recovery_penalty = (
            statistics.mean(recovery_values) if recovery_values else 0.0
        )
        overshoot_penalty = (
            statistics.mean(overshoot_values) if overshoot_values else 0.0
        )
        snake_penalty = (
            statistics.mean(snake_values) if snake_values else 0.0
        )

        return (
            mean_error + (0.8 * ripple) + (0.6 * derivative) +
            (5.0 * lost_ratio) + (0.4 * speed_error) +
            (1.5 * recovery_penalty) +
            (1.0 * overshoot_penalty) +
            (1.2 * snake_penalty)
        )

    def _wait_for(self, predicate, timeout: float) -> str | None:
        """从 data_queue 读行直到 predicate() 命中或超时，返回命中的行或 None。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_cancelled()
            try:
                line = self.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if predicate(line):
                return line
        return None

    def _collect_for(self, duration: float) -> list[PidSample]:
        """在指定时长内收集尽可能多的 PID 样本。

        主要用于开环前馈辨识阶段：每个 PWM 点停留期间把所有数据存起来。"""
        samples: list[PidSample] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_cancelled()
            try:
                line = self.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            sample = PidSample.parse(line)
            if sample is not None:
                samples.append(sample)
                if max(sample.left_speed, sample.right_speed) > 1000:
                    self.link.send("STOP")
                    raise RuntimeError("Unsafe encoder speed detected")
        return samples

    def _send_set(self, name: str, value: int) -> None:
        """发 SET name value 命令并等 80 ms 让下位机写入并回复。"""
        self.link.send(f"SET {name} {value}")
        time.sleep(0.08)

    def _tune_wheel(self, wheel: str) -> dict[str, int | float]:
        """对单个车轮跑完整寻优：前馈 -> Kp -> Ki -> 验证。

        返回值是 results 字典里对应那一项的内容。"""
        label = "left" if wheel == "L" else "right"
        self._status(f"{label}: automatic feedforward identification")
        min_pwm, ff_x1000 = self._identify_feedforward(wheel)
        self._send_set(f"{wheel}MIN", min_pwm)
        self._send_set(f"{wheel}FF", ff_x1000)

        self._status(
            f"{label}: min_pwm={min_pwm}, feedforward={ff_x1000}/1000"
        )
        best_kp = self._search_kp(wheel)
        best_ki = self._search_ki(wheel, best_kp)

        self._send_set(f"{wheel}KP", best_kp)
        self._send_set(f"{wheel}KI", best_ki)
        validation, score = self._run_step(
            wheel, best_kp, best_ki, 3000, "validation"
        )
        self._status(
            f"{label}: final Kp={best_kp}/1000, "
            f"Ki={best_ki}/1000, score={score:.3f}"
        )
        return {
            "min_pwm": min_pwm,
            "ff_x1000": ff_x1000,
            "kp_x1000": best_kp,
            "ki_x1000": best_ki,
            "validation_score": round(score, 5),
            "validation_samples": len(validation),
        }

    def _identify_feedforward(self, wheel: str) -> tuple[int, int]:
        """通过开环扫 PWM 拟合 pwm = a * speed + b，得出 min_pwm 和 ff_x1000。

        步骤：
          1. 开 STREAM ON
          2. 在每个 PWM 点上停留 1.25 s，丢弃前 0.7 s 启动数据后再取均值
          3. 用所有稳态点做线性回归（最小二乘）
          4. 把截距截到 [20, 160] -> min_pwm，斜率 * 1000 截到 [50, 1000] -> ff_x1000"""
        points: list[tuple[float, int]] = []
        all_rows: list[list[object]] = []

        self.link.drain_data()
        self.link.send("STREAM ON")
        time.sleep(0.15)

        for pwm in OPEN_PWM_POINTS:
            self._check_cancelled()
            self._status(f"{wheel}: PWM sweep {pwm}")
            self.link.send(f"PWM {wheel} {pwm}")
            samples = self._collect_for(1.25)
            settled = [
                sample.speed(wheel)
                for sample in samples
                if sample.time_ms >= 700
            ]
            speed = statistics.mean(settled) if settled else 0.0
            all_rows.append([pwm, round(speed, 3), len(samples)])
            if speed >= 35:
                # 速度太低说明还在死区；不进入拟合
                points.append((speed, pwm))
            self.link.send(f"PWM {wheel} 0")
            time.sleep(0.18)

        self.link.send("STREAM OFF")
        self._save_rows(
            f"{wheel}_feedforward.csv",
            ["pwm", "steady_speed_mmps", "sample_count"],
            all_rows,
        )

        if len(points) < 3:
            raise RuntimeError(
                f"{wheel}: encoder did not provide enough usable speed points"
            )

        # 一元线性回归：speed -> pwm
        mean_speed = statistics.mean(point[0] for point in points)
        mean_pwm = statistics.mean(point[1] for point in points)
        variance = sum((speed - mean_speed) ** 2 for speed, _ in points)
        if variance <= 0:
            raise RuntimeError(f"{wheel}: invalid PWM-speed identification")

        slope = sum(
            (speed - mean_speed) * (pwm - mean_pwm)
            for speed, pwm in points
        ) / variance
        intercept = mean_pwm - slope * mean_speed

        min_pwm = max(20, min(160, round(intercept)))
        ff_x1000 = max(50, min(1000, round(slope * 1000)))
        return min_pwm, ff_x1000

    def _search_kp(self, wheel: str) -> int:
        """在 KP_CANDIDATES 里挑最优 Kp（Ki 临时置 0）。"""
        self._send_set(f"{wheel}KI", 0)
        scored: list[tuple[float, int]] = []
        for kp in KP_CANDIDATES:
            self._check_cancelled()
            self._status(f"{wheel}: testing Kp={kp}/1000")
            _, score = self._run_step(wheel, kp, 0, 1800, f"kp_{kp}")
            scored.append((score, kp))
            time.sleep(0.25)
        scored.sort()
        self._status(f"{wheel}: selected Kp={scored[0][1]}/1000")
        return scored[0][1]

    def _search_ki(self, wheel: str, kp: int) -> int:
        """在 KI_CANDIDATES 里挑最优 Ki（Kp 固定为传入值）。"""
        scored: list[tuple[float, int]] = []
        for ki in KI_CANDIDATES:
            self._check_cancelled()
            self._status(f"{wheel}: testing Ki={ki}/1000")
            _, score = self._run_step(wheel, kp, ki, 2500, f"ki_{ki}")
            scored.append((score, ki))
            time.sleep(0.25)
        scored.sort()
        self._status(f"{wheel}: selected Ki={scored[0][1]}/1000")
        return scored[0][1]

    def _run_step(
        self, wheel: str, kp: int, ki: int, duration_ms: int, name: str
    ) -> tuple[list[PidSample], float]:
        """跑一次单轮 STEP 测试并打 _score_step 分，返回 (样本, 分数)。"""
        self._send_set(f"{wheel}KP", kp)
        self._send_set(f"{wheel}KI", ki)
        self.link.drain_data()
        self.link.send(f"STEP {wheel} {TARGET_SPEED_MMPS} {duration_ms}")

        samples: list[PidSample] = []
        deadline = time.monotonic() + (duration_ms / 1000.0) + 3.0
        completed = False
        while time.monotonic() < deadline:
            self._check_cancelled()
            try:
                line = self.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if line == "TEST DONE":
                completed = True
                break
            sample = PidSample.parse(line)
            if sample is not None:
                samples.append(sample)
                if sample.speed(wheel) > 1000:
                    self.link.send("STOP")
                    raise RuntimeError("Unsafe encoder speed detected")

        if not completed or len(samples) < 20:
            raise RuntimeError(f"{wheel}: incomplete step test")

        score = self._score_step(samples, wheel)
        self._save_samples(f"{wheel}_{name}.csv", samples)
        return samples, score

    @staticmethod
    def _score_step(samples: list[PidSample], wheel: str) -> float:
        """给单轮阶跃响应打一个综合分（越小越好）。

        组成项：
          - 整体平均误差       weight = 1.0
          - 超调              weight = 3.0 （重点惩罚）
          - 末段稳态误差       weight = 2.5
          - 末段波动 (pstdev)  weight = 1.2
          - 上升时间惩罚       weight = 0.35
          - 饱和占比           weight = 2.0 （PWM 长期贴边视为不好）"""
        target = TARGET_SPEED_MMPS
        speeds = [sample.speed(wheel) for sample in samples]
        if not speeds:
            return math.inf

        absolute_error = statistics.mean(
            abs(target - speed) for speed in speeds
        ) / target
        overshoot = max(0, max(speeds) - target) / target

        # 末段 1/4 视为稳态
        tail_count = max(5, len(speeds) // 4)
        tail = speeds[-tail_count:]
        steady_error = abs(statistics.mean(tail) - target) / target
        ripple = statistics.pstdev(tail) / target if len(tail) > 1 else 0

        # 上升时间：第一次达到 90% 目标的时间
        rise_time = samples[-1].time_ms
        for sample in samples:
            if sample.speed(wheel) >= int(target * 0.9):
                rise_time = sample.time_ms
                break
        rise_penalty = min(1.0, rise_time / 1500.0)

        # PWM 饱和占比：贴着 500 满占空比的比例
        saturation = sum(
            1 for sample in samples if sample.pwm(wheel) >= 495
        ) / len(samples)

        return (
            absolute_error
            + (3.0 * overshoot)
            + (2.5 * steady_error)
            + (1.2 * ripple)
            + (0.35 * rise_penalty)
            + (2.0 * saturation)
        )

    def _save_samples(self, name: str, samples: list[PidSample]) -> None:
        """把一组 PidSample 落盘为 CSV，列顺序和下位机 send_stream_sample 一致。"""
        rows = [
            [
                sample.time_ms,
                sample.mode,
                sample.left_target,
                sample.right_target,
                sample.left_speed,
                sample.right_speed,
                sample.left_pwm,
                sample.right_pwm,
                sample.left_error,
                sample.right_error,
                sample.count_diff,
                sample.line_error,
                sample.line_mask,
                sample.line_valid,
                sample.yaw_x10,
                sample.corner_count,
                sample.square_state,
                sample.turn_travel_mm,
            ]
            for sample in samples
        ]
        self._save_rows(
            name,
            [
                "time_ms",
                "mode",
                "left_target",
                "right_target",
                "left_mmps",
                "right_mmps",
                "left_pwm",
                "right_pwm",
                "left_error",
                "right_error",
                "count_diff",
                "line_error",
                "line_mask",
                "line_valid",
                "yaw_x10",
                "corner_count",
                "square_state",
                "turn_travel_mm",
            ],
            rows,
        )

    def _save_rows(
        self, name: str, header: list[str], rows: list[list[object]]
    ) -> None:
        """把行数据写到 session_dir/name.csv。"""
        assert self.session_dir is not None
        with (self.session_dir / name).open(
            "w", newline="", encoding="ascii"
        ) as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)


class App(tk.Tk):
    """tkinter 顶层窗口。负责：

    - 串口选择 / 打开关闭
    - 调参按钮、参数旋钮、方框配置面板
    - 50 ms 一次的 _poll_queues() 轮询：拉取串口行，更新日志、绘图、状态栏
    - 把后台寻优线程的 status_queue / display_queue 渲染到 UI"""
    def __init__(self) -> None:
        super().__init__()
        self.title("MSPM0 小车 PID 自动调试上位机")
        self.geometry("1220x760")
        self.minsize(980, 680)

        self.link = SerialLink()
        # AutoTuner 推给 UI 的状态文本
        self.status_queue: queue.Queue[str] = queue.Queue()
        self.tuner = AutoTuner(
            self.link, self.status_queue, self._autotune_finished
        )
        # 最近 250 帧的 PID 样本，用于实时绘图
        self.plot_samples: list[PidSample] = []
        # 接收 DUMP 时的全部样本（最后落盘到 straight_dump.csv）
        self.dump_samples: list[PidSample] = []
        self.dump_receiving = False
        # 方框跑动时的实时样本（最后落盘到 square_run.csv）
        self.square_samples: list[PidSample] = []
        self.square_running = False
        # 保存下位机最近一次 PARAM 报告，写入每次方框测试快照。
        self.current_parameters: dict[str, int] = {}
        # 用来把绘图刷新降到每 10 帧一次（CPU 优化）
        self.pid_display_divider = 0

        # ---------- 串口 / 状态相关 StringVar ----------
        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接")
        # 安全门：必须显式勾选才能开自动整定 / 方框
        self.raise_confirmed = tk.BooleanVar(value=False)
        self.track_confirmed = tk.BooleanVar(value=False)
        # ---------- 用户可调参数 ----------
        self.straight_speed_var = tk.IntVar(value=200)
        self.straight_seconds_var = tk.IntVar(value=8)
        self.square_speed_var = tk.IntVar(value=340)
        self.square_laps_var = tk.IntVar(value=1)
        self.turn_angle_var = tk.IntVar(value=900)
        self.turn_fast_var = tk.IntVar(value=185)
        self.turn_slow_var = tk.IntVar(value=140)
        self.turn_margin_var = tk.IntVar(value=180)
        self.turn_exit_var = tk.IntVar(value=140)
        self.turn_distance_var = tk.IntVar(value=85)
        self.status_var = tk.StringVar(value="就绪")
        self.advanced_visible = False

        self._build_ui()
        self._refresh_ports()
        # 50 ms 一次的轮询循环（之后由 _poll_queues 内部 self.after 续约）
        self.after(50, self._poll_queues)
        # 用户点窗口 X：先安全关闭串口再退出
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        """创建精简主界面；低频诊断功能放进可折叠高级区。"""
        top = ttk.Frame(self, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=28, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(
            side=tk.LEFT
        )
        self.connect_button = ttk.Button(
            top, text="连接", command=self._toggle_connection
        )
        self.connect_button.pack(side=tk.LEFT, padx=6)
        ttk.Label(top, textvariable=self.connection_var).pack(
            side=tk.LEFT, padx=12
        )
        self.advanced_button = ttk.Button(
            top, text="展开高级设置", command=self._toggle_advanced
        )
        self.advanced_button.pack(side=tk.RIGHT)

        tune = ttk.LabelFrame(self, text="自动调参", padding=(10, 6))
        tune.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Checkbutton(
            tune,
            text="已确认两个驱动轮悬空",
            variable=self.raise_confirmed,
        ).pack(side=tk.LEFT)
        self.auto_both_button = ttk.Button(
            tune,
            text="整车轮速调参",
            command=lambda: self._start_autotune(("L", "R")),
        )
        self.auto_both_button.pack(side=tk.LEFT, padx=6)
        ttk.Separator(tune, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8
        )
        ttk.Checkbutton(
            tune,
            text="小车已放在封闭循迹赛道，场地安全",
            variable=self.track_confirmed,
        ).pack(side=tk.LEFT)
        self.auto_line_button = ttk.Button(
            tune, text="循迹 PID 自动调参", command=self._start_line_autotune
        )
        self.auto_line_button.pack(side=tk.LEFT, padx=6)
        ttk.Button(tune, text="紧急停止", command=self._stop).pack(
            side=tk.RIGHT
        )

        race = ttk.LabelFrame(self, text="方形赛道运行", padding=(10, 6))
        race.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Label(race, text="速度 (mm/s)").pack(side=tk.LEFT)
        ttk.Spinbox(
            race,
            from_=80,
            to=450,
            increment=10,
            width=7,
            textvariable=self.square_speed_var,
        ).pack(side=tk.LEFT, padx=(6, 14))
        ttk.Label(race, text="圈数").pack(side=tk.LEFT)
        ttk.Spinbox(
            race,
            from_=1,
            to=10,
            increment=1,
            width=5,
            textvariable=self.square_laps_var,
        ).pack(side=tk.LEFT, padx=6)
        self.square_button = ttk.Button(
            race,
            text="运行逆时针方形",
            command=self._start_square,
        )
        self.square_button.pack(side=tk.LEFT, padx=6)

        self.advanced_frame = ttk.LabelFrame(
            self, text="高级诊断与底层参数", padding=(10, 6)
        )
        diagnostic = ttk.Frame(self.advanced_frame)
        diagnostic.pack(fill=tk.X)
        self.auto_left_button = ttk.Button(
            diagnostic,
            text="仅调左轮",
            command=lambda: self._start_autotune(("L",)),
        )
        self.auto_left_button.pack(side=tk.LEFT, padx=(0, 6))
        self.auto_right_button = ttk.Button(
            diagnostic,
            text="仅调右轮",
            command=lambda: self._start_autotune(("R",)),
        )
        self.auto_right_button.pack(side=tk.LEFT, padx=6)
        ttk.Button(
            diagnostic, text="应用上次轮速参数",
            command=self._apply_last_result
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            diagnostic, text="准备脱机直线测试",
            command=self._straight_test
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            diagnostic, text="读取上次直线日志",
            command=self._read_last_run
        ).pack(side=tk.LEFT, padx=6)

        straight = ttk.Frame(self.advanced_frame)
        straight.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(straight, text="直线速度").pack(side=tk.LEFT)
        ttk.Spinbox(
            straight, from_=80, to=600, increment=10, width=6,
            textvariable=self.straight_speed_var
        ).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(straight, text="时间(秒)").pack(side=tk.LEFT)
        ttk.Spinbox(
            straight, from_=1, to=12, increment=1, width=5,
            textvariable=self.straight_seconds_var
        ).pack(side=tk.LEFT, padx=(4, 14))

        turn = ttk.Frame(self.advanced_frame)
        turn.pack(fill=tk.X, pady=(8, 0))
        turn_fields = (
            ("转角×0.1°", self.turn_angle_var, 700, 1100, 5),
            ("快速PWM", self.turn_fast_var, 100, 300, 5),
            ("慢速PWM", self.turn_slow_var, 80, 250, 5),
            ("减速余量×0.1°", self.turn_margin_var, 50, 350, 5),
            ("出弯速度", self.turn_exit_var, 80, 250, 10),
            ("转弯里程mm", self.turn_distance_var, 50, 140, 1),
        )
        for label, variable, minimum, maximum, increment in turn_fields:
            ttk.Label(turn, text=label).pack(side=tk.LEFT)
            ttk.Spinbox(
                turn, from_=minimum, to=maximum, increment=increment,
                width=6, textvariable=variable
            ).pack(side=tk.LEFT, padx=(4, 10))

        self.status_label = ttk.Label(
            self, textvariable=self.status_var, padding=(10, 0, 10, 6)
        )
        self.status_label.pack(fill=tk.X)

        # --- 实时绘图 ---
        self.canvas = tk.Canvas(
            self, height=260, background="#ffffff", highlightthickness=1
        )
        self.canvas.pack(fill=tk.X, padx=10, pady=(0, 8))

        # --- 日志框 ---
        log_frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log = tk.Text(log_frame, wrap=tk.NONE, height=16)
        scroll_y = ttk.Scrollbar(
            log_frame, orient=tk.VERTICAL, command=self.log.yview
        )
        self.log.configure(yscrollcommand=scroll_y.set)
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)

    def _toggle_advanced(self) -> None:
        """展开或收起低频使用的单轮、直线和底层转弯设置。"""
        self.advanced_visible = not self.advanced_visible
        if self.advanced_visible:
            self.advanced_frame.pack(
                fill=tk.X, padx=10, pady=(0, 6),
                before=self.status_label
            )
            self.advanced_button.configure(text="收起高级设置")
        else:
            self.advanced_frame.pack_forget()
            self.advanced_button.configure(text="展开高级设置")

    def _refresh_ports(self) -> None:
        """重新扫描系统串口并填充到下拉框；默认选第一个。"""
        ports = [
            f"{port.device} - {port.description}"
            for port in list_ports.comports()
        ]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _selected_port(self) -> str:
        """从下拉框文字里取纯 device 名（去掉 description 部分）。"""
        return self.port_var.get().split(" - ", 1)[0].strip()

    def _toggle_connection(self) -> None:
        """点击 Connect / Disconnect 按钮的入口。

        Connect 时除了 open 还会发 PING / PARAM 验证下位机在跑；
        Disconnect 时调 link.close()，会自动 STOP 小车。"""
        if self.link.connected:
            self.link.close()
            self.connection_var.set("未连接")
            self.connect_button.configure(text="连接")
            return
        port = self._selected_port()
        if not port:
            messagebox.showerror("串口", "请先选择串口")
            return
        try:
            self.link.connect(port)
            self.connection_var.set(f"正在打开：{port}")
            self.update_idletasks()
            time.sleep(1.2)
            self.link.send("PING")
            time.sleep(0.1)
            self.link.send("PARAM")
            self.connection_var.set(f"已连接：{port}")
            self.connect_button.configure(text="断开")
        except Exception as exc:
            self.link.close(send_stop=False)
            self.connection_var.set("未连接")
            self.connect_button.configure(text="连接")
            messagebox.showerror("串口连接失败", str(exc))

    def _start_autotune(self, wheels: tuple[str, ...]) -> None:
        """Auto Tune Left / Right / Both 按钮的统一入口。

        前置条件：已连接串口、已勾 raise 确认、未在寻优中。"""
        if not self.link.connected:
            messagebox.showerror("自动调参", "请先连接小车")
            return
        if not self.raise_confirmed.get():
            messagebox.showerror(
                "安全确认", "自动调整轮速前必须悬空两个驱动轮"
            )
            return
        if self.tuner.running:
            return
        self.plot_samples.clear()
        self.status_var.set("正在自动调整轮速参数")
        self._set_tune_buttons(False)
        self.tuner.start(wheels)

    def _start_line_autotune(self) -> None:
        """Auto Tune Line 按钮入口。

        与 _start_autotune 类似，但要求勾 track 确认（车已经在封闭赛道上）。"""
        if not self.link.connected:
            messagebox.showerror("循迹调参", "请先连接小车")
            return
        if not self.track_confirmed.get():
            messagebox.showerror(
                "安全确认",
                "请把小车放在封闭循迹赛道，并确认场地安全",
            )
            return
        if self.tuner.running:
            return
        self.plot_samples.clear()
        self.status_var.set("正在自动调整循迹 PID")
        self._set_tune_buttons(False)
        self.tuner.start_line()

    def _start_square(self) -> None:
        """Run CCW Square 按钮入口。

        1. 校验所有旋钮数值在合法范围内
        2. 弹确认框（提醒车已在赛道上）
        3. 依次 SET 一组 TURNxxx 参数并 SAVE
        4. 发 SQUARE speed laps 启动方框
        5. 状态栏提示运行中，并禁用按钮直到收到 SQUARE DONE / ERROR"""
        if not self.link.connected:
            messagebox.showerror("方形赛道", "请先连接小车")
            return
        if not self.track_confirmed.get():
            messagebox.showerror(
                "安全确认",
                "请把小车放在逆时针方形赛道，并确认场地安全",
            )
            return
        if self.tuner.running or self.square_running:
            return
        try:
            speed = int(self.square_speed_var.get())
            laps = int(self.square_laps_var.get())
            angle = int(self.turn_angle_var.get())
            fast_pwm = int(self.turn_fast_var.get())
            slow_pwm = int(self.turn_slow_var.get())
            margin = int(self.turn_margin_var.get())
            exit_speed = int(self.turn_exit_var.get())
            turn_distance = int(self.turn_distance_var.get())
        except (TypeError, ValueError):
            messagebox.showerror("方形赛道", "存在无效参数")
            return
        # 参数范围校验：超出就弹错并不发任何命令
        if not 80 <= speed <= 450 or not 1 <= laps <= 10:
            messagebox.showerror(
                "方形赛道", "速度必须为80～450 mm/s，圈数必须为1～10"
            )
            return
        if not 700 <= angle <= 1100:
            messagebox.showerror("方形赛道", "转角必须为700～1100")
            return
        if not 100 <= fast_pwm <= 300:
            messagebox.showerror("方形赛道", "快速PWM必须为100～300")
            return
        if not 80 <= slow_pwm <= min(250, fast_pwm):
            messagebox.showerror(
                "方形赛道", "慢速PWM必须为80～250，且不能大于快速PWM"
            )
            return
        if not 50 <= margin <= 350 or not 80 <= exit_speed <= 250:
            messagebox.showerror(
                "方形赛道",
                "减速余量必须为50～350，出弯速度必须为80～250",
            )
            return
        if not 50 <= turn_distance <= 140:
            messagebox.showerror("方形赛道", "转弯里程必须为50～140 mm")
            return
        if not messagebox.askokcancel(
            "Run counter-clockwise square",
            "Put the car on a counter-clockwise square black-line track.\n"
            "It will follow the line and make four left turns per lap.\n\n"
            f"Run {laps} lap(s) at {speed} mm/s?\n\n"
            "Keep the STOP button reachable.",
        ):
            return
        try:
            commands = (
                ("TURNANGLE", angle),
                ("TURNFAST", fast_pwm),
                ("TURNSLOW", slow_pwm),
                ("TURNMARGIN", margin),
                ("TURNEXIT", exit_speed),
                ("TURNDIST", turn_distance),
            )
            self.link.drain_data()
            self.link.send("SENSOR")
            time.sleep(0.08)
            self.link.send("IMU")
            time.sleep(0.08)
            # 每次 SET 之间稍等一下，让下位机处理完上一条再发下一条
            for name, value in commands:
                self.link.send(f"SET {name} {value}")
                time.sleep(0.06)
            self.link.send("SAVE")
            time.sleep(0.2)
            self.square_samples.clear()
            self.plot_samples.clear()
            self.square_running = True
            self.square_button.configure(state=tk.DISABLED)
            self.link.send(f"SQUARE {speed} {laps}")
            self.status_var.set(
                f"CCW square running: {laps} lap(s), STOP remains available"
            )
        except Exception as exc:
            self.square_running = False
            self.square_button.configure(state=tk.NORMAL)
            messagebox.showerror("方形赛道", str(exc))

    def _stop(self) -> None:
        """STOP 按钮入口：取消寻优、停方框、恢复按钮。"""
        self.tuner.cancel()
        self.square_running = False
        self.square_button.configure(state=tk.NORMAL)
        self.status_var.set("已请求紧急停止")

    def _apply_last_result(self) -> None:
        """Apply Last 按钮入口：把 logs/ 里最新的 tuned_params.json 写回小车。

        找到最近一次结果（按时间戳倒序），把 L/R 两个轮子的 min/ff/kp/ki
        和 BIAS 都写下去再 SAVE。"""
        if not self.link.connected:
            messagebox.showerror("Parameters", "Connect to the car first")
            return
        result_files = sorted(
            (Path(__file__).parent / "logs").glob(
                "*/tuned_params.json"
            ),
            reverse=True,
        )
        if not result_files:
            messagebox.showerror("Parameters", "No saved tuning result")
            return
        try:
            results = json.loads(result_files[0].read_text(encoding="ascii"))
            for wheel in ("L", "R"):
                if wheel not in results:
                    continue
                values = results[wheel]
                commands = (
                    (f"{wheel}MIN", values["min_pwm"]),
                    (f"{wheel}FF", values["ff_x1000"]),
                    (f"{wheel}KP", values["kp_x1000"]),
                    (f"{wheel}KI", values["ki_x1000"]),
                )
                for name, value in commands:
                    self.link.send(f"SET {name} {int(value)}")
                    time.sleep(0.08)
            self.link.send(f"SET BIAS {STRAIGHT_BIAS_X10000}")
            time.sleep(0.08)
            self.link.send("SAVE")
            self.status_var.set(f"Applied {result_files[0]}")
        except Exception as exc:
            messagebox.showerror("Parameters", str(exc))

    def _straight_test(self) -> None:
        """Arm Untethered Test 按钮入口。

        把当前参数 SAVE 一次，然后发 ARM speed ms：
          - 下位机把 speed/duration 存到 Flash
          - 等待 SET 按键被按下
          - 按下后 2 s 启动斜坡跑直线
        让用户可以拔掉 TTL 把车放到空地上。"""
        if not self.link.connected:
            messagebox.showerror("Straight test", "Connect to the car first")
            return
        if self.tuner.running:
            messagebox.showerror("Straight test", "Autotune is still running")
            return
        try:
            speed = int(self.straight_speed_var.get())
            seconds = int(self.straight_seconds_var.get())
        except (TypeError, ValueError):
            messagebox.showerror("Straight test", "Invalid speed or duration")
            return
        if not 80 <= speed <= 600 or not 1 <= seconds <= 12:
            messagebox.showerror(
                "Straight test",
                "Speed must be 80..600 mm/s and duration 1..12 seconds",
            )
            return

        distance_m = speed * seconds / 1000.0
        approved = messagebox.askokcancel(
            "Untethered straight test",
            "The MCU will save parameters and wait for the SET key.\n\n"
            "After arming: disconnect USB-TTL, place the car on a clear "
            "floor, then press SET. It waits 2 seconds and ramps up gently "
            f"for 1.5 seconds.\n\nIt will drive at {speed} mm/s for "
            f"{seconds} seconds (about {distance_m:.2f} m).",
        )
        if not approved:
            return
        try:
            self.plot_samples.clear()
            self.link.send("SAVE")
            time.sleep(0.25)
            self.link.send(f"ARM {speed} {seconds * 1000}")
            self.status_var.set(
                "ARMED: disconnect TTL, place car, press SET"
            )
        except Exception as exc:
            messagebox.showerror("Straight test", str(exc))

    def _read_last_run(self) -> None:
        """Read Last Run 按钮入口：发 DUMP 让下位机把上次脱机跑的日志回传。

        回传结束后由 _poll_queues 看到 DUMP DONE 行触发 _save_dump。"""
        if not self.link.connected:
            messagebox.showerror("Run log", "Connect to the car first")
            return
        if self.tuner.running:
            messagebox.showerror("Run log", "Autotune is still running")
            return
        self.plot_samples.clear()
        self.dump_samples.clear()
        self.dump_receiving = True
        self.link.drain_data()
        self.link.send("DUMP")
        self.status_var.set("Reading last untethered run")

    def _save_dump(self) -> None:
        """把 dump_samples 落盘到 logs/straight_<时间戳>/straight_dump.csv。"""
        if not self.dump_samples:
            self.status_var.set("No stored run data")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = Path(__file__).parent / "logs" / f"straight_{stamp}"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / "straight_dump.csv"
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "time_ms",
                    "mode",
                    "left_target",
                    "right_target",
                    "left_mmps",
                    "right_mmps",
                    "left_pwm",
                    "right_pwm",
                    "left_error",
                    "right_error",
                    "count_diff",
                    "line_error",
                    "line_mask",
                    "line_valid",
                    "yaw_x10",
                    "corner_count",
                    "square_state",
                ]
            )
            for sample in self.dump_samples:
                writer.writerow(
                    [
                        sample.time_ms,
                        sample.mode,
                        sample.left_target,
                        sample.right_target,
                        sample.left_speed,
                        sample.right_speed,
                        sample.left_pwm,
                        sample.right_pwm,
                        sample.left_error,
                        sample.right_error,
                        sample.count_diff,
                        sample.line_error,
                        sample.line_mask,
                        sample.line_valid,
                        sample.yaw_x10,
                        sample.corner_count,
                        sample.square_state,
                    ]
                )
        self.status_var.set(f"Straight run saved: {path}")

    def _save_square_run(self, result: str) -> None:
        """方框跑完后把 square_samples 落盘到 logs/square_<时间戳>/square_run.csv。

        result 是下位机回传的 'SQUARE DONE' 或 'SQUARE ERROR,...'，会拼到状态栏。"""
        if not self.square_samples:
            self.status_var.set(result)
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = Path(__file__).parent / "logs" / f"square_{stamp}"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / "square_run.csv"
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "time_ms", "mode", "left_target", "right_target",
                    "left_mmps", "right_mmps", "left_pwm", "right_pwm",
                    "left_error", "right_error", "count_diff",
                    "line_error", "line_mask", "line_valid", "yaw_x10",
                    "corner_count", "square_state",
                    "turn_travel_mm",
                ]
            )
            for sample in self.square_samples:
                writer.writerow(
                    [
                        sample.time_ms, sample.mode, sample.left_target,
                        sample.right_target, sample.left_speed,
                        sample.right_speed, sample.left_pwm, sample.right_pwm,
                        sample.left_error, sample.right_error,
                        sample.count_diff, sample.line_error, sample.line_mask,
                        sample.line_valid, sample.yaw_x10,
                        sample.corner_count, sample.square_state,
                        sample.turn_travel_mm,
                    ]
                )
        valid = [sample for sample in self.square_samples if sample.line_valid]
        abs_errors = sorted(abs(sample.line_error) for sample in valid)
        p95_index = max(0, int(len(abs_errors) * 0.95) - 1)
        summary = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "result": result,
            "requested": {
                "speed_mmps": self.square_speed_var.get(),
                "laps": self.square_laps_var.get(),
                "turn_angle_x10": self.turn_angle_var.get(),
                "turn_fast_pwm": self.turn_fast_var.get(),
                "turn_slow_pwm": self.turn_slow_var.get(),
                "turn_slow_margin_x10": self.turn_margin_var.get(),
                "turn_exit_mmps": self.turn_exit_var.get(),
                "turn_distance_mm": self.turn_distance_var.get(),
            },
            "parameters": dict(self.current_parameters),
            "statistics": {
                "samples": len(self.square_samples),
                "duration_ms": (
                    self.square_samples[-1].time_ms -
                    self.square_samples[0].time_ms
                ),
                "corners_completed": max(
                    sample.corner_count for sample in self.square_samples
                ),
                "valid_line_samples": len(valid),
                "invalid_line_samples": len(self.square_samples) - len(valid),
                "mean_abs_line_error": (
                    round(statistics.mean(abs_errors), 3)
                    if abs_errors else None
                ),
                "p95_abs_line_error": (
                    abs_errors[p95_index] if abs_errors else None
                ),
                "mean_left_mmps": round(statistics.mean(
                    sample.left_speed for sample in self.square_samples
                ), 2),
                "mean_right_mmps": round(statistics.mean(
                    sample.right_speed for sample in self.square_samples
                ), 2),
            },
        }
        # JSON 与原始 CSV 放在同一目录，之后比较不同测试无需靠记忆参数。
        (folder / "run_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.status_var.set(f"{result}; log saved: {path}")

    def _apply_parameter_report(self, line: str) -> None:
        """解析下位机发来的 'PARAM,KEY1,VAL1,KEY2,VAL2,...' 并更新对应 IntVar。

        主要用于连接后下位机回传当前参数时同步到 UI。"""
        parts = line.split(",")
        values: dict[str, int] = {}
        # 从第 2 列开始按 (key, value) 对取
        for index in range(2, len(parts) - 1, 2):
            try:
                values[parts[index]] = int(parts[index + 1])
            except ValueError:
                continue
        self.current_parameters.update(values)
        variable_map = {
            "TURNANGLE": self.turn_angle_var,
            "TURNFAST": self.turn_fast_var,
            "TURNSLOW": self.turn_slow_var,
            "TURNMARGIN": self.turn_margin_var,
            "TURNEXIT": self.turn_exit_var,
            "TURNDIST": self.turn_distance_var,
        }
        for name, variable in variable_map.items():
            if name in values:
                variable.set(values[name])

    def _autotune_finished(self) -> None:
        """AutoTuner 工作线程结束后会调它：往 status_queue 推 __FINISHED__ 哨兵。"""
        self.status_queue.put("__FINISHED__")

    def _set_tune_buttons(self, enabled: bool) -> None:
        """同时启用/禁用所有自动整定按钮，避免用户重复点击。"""
        state = tk.NORMAL if enabled else tk.DISABLED
        self.auto_left_button.configure(state=state)
        self.auto_right_button.configure(state=state)
        self.auto_both_button.configure(state=state)
        self.auto_line_button.configure(state=state)

    def _poll_queues(self) -> None:
        """50 ms 一次的轮询：

        1. 拉空 display_queue：可解析成 PidSample 的塞进 plot/dump/square 缓冲，
           其它特殊行（DUMP DONE / SQUARE DONE / SQUARE LEARNED / PARAM / 错误）
           各走各的处理。
        2. 拉空 status_queue：哨兵 __FINISHED__ 表示寻优结束，否则就是普通状态。
        3. 重绘 canvas。
        4. self.after(50, ...) 续约。"""
        while True:
            try:
                line = self.link.display_queue.get_nowait()
            except queue.Empty:
                break
            sample = PidSample.parse(line)
            if sample is not None:
                # 是 PID 帧：分类保存
                self.plot_samples.append(sample)
                # 只保留最近 250 帧，避免内存涨
                self.plot_samples = self.plot_samples[-250:]
                if self.dump_receiving:
                    self.dump_samples.append(sample)
                if self.square_running and sample.mode == "SQUARE":
                    self.square_samples.append(sample)
                # 10 帧才走一次重绘
                self.pid_display_divider = (self.pid_display_divider + 1) % 10
                if self.pid_display_divider != 0:
                    continue
            elif line == "DUMP DONE" and self.dump_receiving:
                # 下位机告知 dump 全部回传完毕
                self.dump_receiving = False
                self._save_dump()
            elif self.square_running and (
                line.startswith("SQUARE DONE")
                or line.startswith("SQUARE ERROR")
            ):
                # 方框自然结束 / 异常中止
                self.square_running = False
                self.square_button.configure(state=tk.NORMAL)
                self._save_square_run(line)
            elif line.startswith("SQUARE LEARNED,TURNDIST,"):
                # 下位机把推荐转弯里程自动写回 UI
                try:
                    learned = int(line.rsplit(",", 1)[1])
                    self.turn_distance_var.set(learned)
                except ValueError:
                    pass
            elif line.startswith("PARAM,"):
                # 下位机发当前参数快照：同步到 UI
                self._apply_parameter_report(line)
            elif line.startswith("SERIAL ERROR:"):
                # 串口挂了：自动关闭 UI 连接状态
                self.link.close(send_stop=False)
                self.connection_var.set("未连接")
                self.connect_button.configure(text="连接")
            self._append_log(line)

        while True:
            try:
                status = self.status_queue.get_nowait()
            except queue.Empty:
                break
            if status == "__FINISHED__":
                self._set_tune_buttons(True)
                self.status_var.set("Autotune finished")
            else:
                self.status_var.set(status)
                self._append_log(status)

        self._draw_plot()
        self.after(50, self._poll_queues)

    def _append_log(self, text: str) -> None:
        """往 Text 日志框追加一行并滚到底。"""
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)

    def _draw_plot(self) -> None:
        """用 plot_samples 在 canvas 上重画 4 条折线：左/右 target 和 speed。"""
        canvas = self.canvas
        canvas.delete("all")
        width = max(100, canvas.winfo_width())
        height = max(100, canvas.winfo_height())
        margin = 38

        canvas.create_line(margin, 10, margin, height - margin, fill="#777")
        canvas.create_line(
            margin, height - margin, width - 10, height - margin, fill="#777"
        )
        canvas.create_text(18, 16, text="mm/s", fill="#555")
        canvas.create_text(
            width - 38, height - 16, text="samples", fill="#555"
        )

        if len(self.plot_samples) < 2:
            canvas.create_text(
                width // 2,
                height // 2,
                text="实时目标速度与实际速度",
                fill="#777",
            )
            return

        values = []
        for sample in self.plot_samples:
            values.extend(
                [
                    sample.left_target,
                    sample.right_target,
                    sample.left_speed,
                    sample.right_speed,
                ]
            )
        max_value = max(400, max(values) + 40)

        def point(index: int, value: int) -> tuple[float, float]:
            # 把 (样本下标, 速度) 映射到画布坐标
            x = margin + (
                index * (width - margin - 12) / (len(self.plot_samples) - 1)
            )
            y = (height - margin) - (
                value * (height - margin - 14) / max_value
            )
            return x, y

        series = (
            ("left target", "#999999", [s.left_target for s in self.plot_samples]),
            ("left speed", "#d62728", [s.left_speed for s in self.plot_samples]),
            ("right target", "#bbbbbb", [s.right_target for s in self.plot_samples]),
            ("right speed", "#1f77b4", [s.right_speed for s in self.plot_samples]),
        )
        for name, color, data in series:
            coords = []
            for index, value in enumerate(data):
                coords.extend(point(index, value))
            canvas.create_line(*coords, fill=color, width=2)

        legend_x = margin + 8
        for name, color, _ in series:
            canvas.create_line(
                legend_x, 18, legend_x + 18, 18, fill=color, width=3
            )
            canvas.create_text(
                legend_x + 22, 18, text=name, anchor=tk.W, fill="#333"
            )
            legend_x += 105

    def _on_close(self) -> None:
        """窗口 X 按钮：先取消寻优、再关闭串口、最后退出 tk 主循环。"""
        self.tuner.cancel()
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
