"""
pid_autotune_gui.py - MSPM0 小车 PID 自动调参上位机

主要功能：
  1. 通过 ESP-NOW 串口桥（或外接 USB-TTL）连接小车
  2. 对左右轮的 PI / FF / MIN_PWM 参数做开环 + 阶跃自动寻优
  3. 对循迹外环 (LINEKP / LINEKD) 做自动寻优
  4. 显示实时速度曲线、调参过程与通信诊断日志

使用：
  pip install -r requirements.txt
  python pid_autotune_gui.py
"""

from __future__ import annotations

import csv
import json
import math
import queue
import random
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TextIO
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


# ----------------------------------------------------------------
# 串口 / 调参常量
# ----------------------------------------------------------------

BAUD_RATE = 115200  # 与下位机约定的串口波特率

# 可靠协议版本必须与固件一致。所有会改变状态的调参命令都带随机 token，
# ACK 丢失后可安全重发，不会重新启动测试或重复擦写 Flash。
PROTOCOL_VERSION = 2
COMMAND_ACK_TIMEOUT = 0.65
COMMAND_RETRIES = 5

# 每个开环点 1.5 s，前 0.7 s 由固件丢弃，只统计稳态段。
OPEN_TUNE_DURATION_MS = 1500

# 编码器固件以 2500 mm/s 作为异常钳位；接近该值的前馈点已失去
# 线性信息，不能用于拟合 PWM-速度斜率。
ENCODER_SPEED_CLIP_MMPS = 2450

# 单轮阶跃自动寻优时的目标速度（mm/s）
TARGET_SPEED_MMPS = 300

# 开环前馈辨识必须覆盖闭环目标速度附近。实测 PWM=220 已达到约
# 1200 mm/s，原先从 220 扫到 600 会让一半数据撞上 2500 mm/s 钳位，
# 再把高速直线外推到 300 mm/s，得到的 FF/MIN 没有可信度。
# 从低占空比逐步上探，达到目标速度约 2.2 倍后立即停止扫描；最后两个点只在
# 电机死区或供电较弱时才会执行，避免再次把轮子推到编码器钳位区。
OPEN_PWM_POINTS = (35, 45, 55, 70, 90, 115, 145, 180, 220, 270)

# 单轮 PI 自动寻优时扫描的候选值
# 同时覆盖 7 月 3 日旧量纲下的较高最优值和 7 月 14 日新数据下的低增益
# 区域；不稳定的高增益由 MCU 900 mm/s 硬保护提前淘汰。
KP_CANDIDATES = (50, 100, 200, 350, 500, 800, 1200)
KI_CANDIDATES = (0, 50, 100, 200, 350, 500, 800)

# 粗扫后把前两名各复测一次；最终验证另跑一次，既过滤偶发扰动又避免过去
# 每个轴重复四次复测造成的长时间空转。
WHEEL_FINALIST_COUNT = 2
WHEEL_FINALIST_RETESTS = 1

# 循迹外环 LINEKP / LINEKD 自动寻优时扫描的候选值
LINE_KP_CANDIDATES = (4500, 6000, 7500, 9000)
LINE_KD_CANDIDATES = (1500, 2500, 3000, 3500)

# 循迹调参时使用的目标速度 (mm/s)
LINE_TUNE_SPEED_MMPS = 280

# 调参过程中注入的“探测差速”幅度和持续时间
LINE_PROBE_KICK_MMPS = 55
LINE_PROBE_KICK_MS = 300

# 直线同步外环机械偏置（与下位机 LAB_DEFAULT_RIGHT_BIAS_MMPS 一致）
STRAIGHT_BIAS_MMPS = 25

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
        if len(parts) == 13 and parts[0] == "LT":
            try:
                values = [int(value) for value in parts[1:]]
            except ValueError:
                return None
            return cls(
                time_ms=values[0], mode="SQUARE",
                left_target=values[1], right_target=values[2],
                left_speed=values[3], right_speed=values[4],
                left_pwm=0, right_pwm=0, left_error=0, right_error=0,
                count_diff=0, line_error=values[5], line_mask=values[6],
                line_valid=values[7], yaw_x10=values[8],
                corner_count=values[9], square_state=values[10],
                turn_travel_mm=values[11],
            )
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
        self.data_queue: queue.Queue[str] = queue.Queue(maxsize=4096)
        # UI 日志 / 绘图用的所有原始行（包括 PONG / STATUS / SENSOR 等）
        self.display_queue: queue.Queue[str] = queue.Queue(maxsize=4096)
        self.stop_event = threading.Event()
        self.reader_thread: threading.Thread | None = None
        self.write_lock = threading.Lock()
        self.trace_lock = threading.Lock()
        self.trace_path: Path | None = None
        self.trace_handle: TextIO | None = None

    def set_trace_path(self, path: Path | None) -> None:
        """设置本次任务的完整协议追踪文件；不影响串口实时线程。"""
        with self.trace_lock:
            if self.trace_handle is not None:
                try:
                    self.trace_handle.close()
                except OSError:
                    pass
                self.trace_handle = None
            self.trace_path = path
            if path is not None:
                try:
                    self.trace_handle = path.open(
                        "w", encoding="utf-8", buffering=1
                    )
                except OSError:
                    self.trace_path = None

    def _trace(self, direction: str, text: str) -> None:
        with self.trace_lock:
            if self.trace_handle is None:
                return
            timestamp = datetime.now().isoformat(timespec="milliseconds")
            try:
                self.trace_handle.write(f"{timestamp} {direction} {text}\n")
            except OSError:
                self.trace_path = None
                self.trace_handle = None

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
        self.set_trace_path(None)

    def send(self, command: str) -> None:
        """发出一条 ASCII 命令（自动加 \\r\\n）。

        写超时允许重试 1 次：第一次失败时清空发送缓冲后等 200 ms 再发。
        仍然失败则抛出异常给调用方，UI 弹窗提示。"""
        if not self.connected:
            raise RuntimeError("Serial port is not connected")
        payload = (command.strip() + "\r\n").encode("ascii")
        self._trace("TX", command.strip())
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

    @staticmethod
    def _put_latest(target: queue.Queue[str], line: str) -> None:
        """队列满时丢最旧一行，保证最新 ACK/安全错误仍能进入。"""
        try:
            target.put_nowait(line)
            return
        except queue.Full:
            pass
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        try:
            target.put_nowait(line)
        except queue.Full:
            pass

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
                    self._put_latest(
                        self.display_queue, f"SERIAL ERROR: {exc}"
                    )
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
                self._trace("RX", line)
                self._put_latest(self.data_queue, line)
                self._put_latest(self.display_queue, line)


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
        self.tune_token = random.SystemRandom().randrange(1, 2_147_483_648)
        self.protocol_info: dict[str, int | str] = {}
        self.feedforward_diagnostics: dict[str, dict[str, object]] = {}

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

    def _save_failure(self, text: str) -> None:
        """把失败原因写进本次会话目录，避免只留在 GUI 文本框。"""
        if self.session_dir is not None:
            try:
                (self.session_dir / "failure.txt").write_text(
                    text + "\n", encoding="utf-8"
                )
            except OSError:
                # 日志落盘失败不能阻止后面的 STOP 安全命令。
                pass
        self._save_session_info()

    def _save_session_info(self) -> None:
        if self.session_dir is None:
            return
        try:
            (self.session_dir / "session_info.json").write_text(
                json.dumps({
                    "protocol": self.protocol_info,
                    "feedforward": self.feedforward_diagnostics,
                }, indent=2),
                encoding="ascii",
            )
        except OSError:
            pass

    def _check_cancelled(self) -> None:
        """在长循环里周期性调用：用户取消时直接抛异常跳出。"""
        if self.cancel_event.is_set():
            raise RuntimeError("Autotune cancelled")

    def _run(self, wheels: tuple[str, ...]) -> None:
        """单/双轮 PI 寻优主流程。

        步骤：
          1. 新建 logs/<时间戳> 目录
          2. 直接对每个轮子跑 _tune_wheel()（辨识前馈 + 搜 Kp/Ki + 验证）
          4. 写 SET BIAS、SAVE，把最终参数交给下位机 Flash
          5. 把 results 写到 tuned_params.json"""
        try:
            # 每次会话从空结果开始，避免单轮重跑时把上一次会话的旧结果
            # 一起写进新的 tuned_params.json。
            self.results = {}
            self.feedforward_diagnostics = {}
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = Path(__file__).parent / "logs" / stamp
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.link.set_trace_path(self.session_dir / "protocol_trace.log")

            self._verify_connection()
            self._stop_reliably()
            self._status("Reliable wheel tune started")
            for wheel in wheels:
                self._check_cancelled()
                self.results[wheel] = self._tune_wheel(wheel)

            self.link.drain_data()
            self._send_set("BIAS", STRAIGHT_BIAS_MMPS)
            self._save_parameters()

            result_path = self.session_dir / "tuned_params.json"
            result_path.write_text(
                json.dumps(self.results, indent=2), encoding="ascii"
            )
            self._save_session_info()
            self._status(f"COMPLETE: parameters saved to {result_path}")
        except Exception as exc:
            failure = f"FAILED: {exc}"
            self._save_failure(failure)
            self._status(failure)
            try:
                self.link.send("STOP")
            except Exception:
                pass
        finally:
            # 任何出口都要通知 UI 恢复按钮
            self.finished_callback()

    def _verify_connection(self) -> None:
        """可靠握手并校验协议版本、STBY 与编码器量纲。"""
        self.link.drain_data()
        token = self._next_tune_token()
        expected = f"HELLO,{token},"
        line = self._request_with_retry(
            f"HELLO {token}",
            lambda item: item.startswith(expected),
            "firmware HELLO",
        )
        if line is None:
            raise RuntimeError(
                "No reliable HELLO response; flash the matching MSPM0 and "
                "both ESP8266 bridge firmwares"
            )
        parts = line.split(",")
        try:
            version = int(parts[2])
            fields = {
                parts[index]: parts[index + 1]
                for index in range(3, len(parts) - 1, 2)
            }
            mm_x1000 = int(fields["ENCODER_MM_X1000"])
            sample_ms = int(fields["SAMPLE_MS"])
        except (IndexError, KeyError, ValueError):
            raise RuntimeError(f"Invalid firmware HELLO: {line}")
        if version != PROTOCOL_VERSION:
            raise RuntimeError(
                f"Protocol mismatch: PC={PROTOCOL_VERSION}, firmware={version}"
            )
        if fields.get("STBY") != "HIGH":
            raise RuntimeError("TB6612 STBY is LOW; motor driver is disabled")
        if not 100 <= mm_x1000 <= 2000 or not 5 <= sample_ms <= 50:
            raise RuntimeError(
                f"Unsafe encoder calibration reported by firmware: {line}"
            )
        self.protocol_info = {
            "version": version,
            "encoder_mm_per_count_x1000": mm_x1000,
            "encoder_sample_ms": sample_ms,
            "stby": fields["STBY"],
        }
        self._status(
            f"Firmware v{version} verified; encoder={mm_x1000}/1000 mm/count, "
            f"sample={sample_ms} ms"
        )

    def _request_with_retry(
        self, command: str, predicate, description: str,
        retries: int = COMMAND_RETRIES,
        timeout: float = COMMAND_ACK_TIMEOUT,
    ) -> str | None:
        """重发幂等命令，直到收到与本次 token 匹配的响应。"""
        for _ in range(retries):
            self._check_cancelled()
            self.link.send(command)
            reply = self._wait_for(predicate, timeout)
            if reply is not None:
                return reply
        self._status(f"Communication retry exhausted: {description}")
        return None

    def _stop_reliably(self) -> None:
        """STOP 可幂等重发；开始任务前清掉任何孤立的旧测试。"""
        self.link.drain_data()
        reply = self._request_with_retry(
            "STOP", lambda item: item == "OK STOP", "STOP", retries=4
        )
        if reply != "OK STOP":
            raise RuntimeError("MCU did not confirm the safe STOP state")

    def _verify_motor_driver(self) -> None:
        """Verify that firmware commands the TB6612 STBY output high."""
        self.link.drain_data()
        line = self._request_with_retry(
            "STATUS", lambda item: item.startswith("STATUS,"), "STATUS"
        )
        if line is None:
            raise RuntimeError("No STATUS response from firmware")
        if ",STBY,HIGH" not in line:
            if ",STBY,LOW" in line:
                raise RuntimeError(
                    "Firmware STBY output is LOW; motor driver is disabled"
                )
            raise RuntimeError(
                "Firmware is too old: flash the new HEX with STBY status"
            )
        self._status("TB6612 STBY output is HIGH")

    def _start_square_reliably(self, speed: int, laps: int) -> None:
        """带 token 启动方框；重复请求不会让已经运行的车辆重新起跑。"""
        token = self._next_tune_token()
        expected = f"SQ,{token},OK"
        self.link.drain_data()
        reply = self._request_with_retry(
            f"SQUARE {speed} {laps} {token}",
            lambda item: item.startswith(f"SQ,{token},"),
            "SQUARE start",
            timeout=0.9,
        )
        if reply != expected:
            raise RuntimeError(f"square start rejected: {reply or 'timeout'}")

    def _send_kick(self, amount: int, duration_ms: int) -> bool:
        """可靠注入一次循迹扰动；若恰逢转弯则返回 False，稍后再试。"""
        token = self._next_tune_token()
        expected = f"KA,{token},OK"
        reply = self._request_with_retry(
            f"KICK {amount} {duration_ms} {token}",
            lambda item: item.startswith(f"KA,{token},"),
            "line KICK",
            retries=2,
        )
        return reply == expected

    def _run_line(self) -> None:
        """方框模式下的循迹 LINEKP / LINEKD 自动寻优主流程。

        步骤：
          1. 验证连接 + 灰度 + IMU 都正常
          2. 启动 SQUARE 10 圈作为测试舞台
          3. 粗扫 LINEKP -> 一轮局部搜索
          4. 用当前最佳 Kp 粗扫 LINEKD -> 一轮局部搜索
          5. 在 (best_kp, best_kd) 周围做 3x3 联合寻优
          6. 和历史 champion 对比，把更优的写为新 champion
          7. 多次验证取中位数作为最终分数
          8. SET LINEKP/LINEKD + SAVE"""
        try:
            self.feedforward_diagnostics = {}
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = (
                Path(__file__).parent / "logs" / f"line_{stamp}"
            )
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.link.set_trace_path(self.session_dir / "protocol_trace.log")
            self._verify_connection()
            self._stop_reliably()
            self._verify_motor_driver()
            # 灰度检查
            self.link.drain_data()
            sensor_line = self._request_with_retry(
                "SENSOR", lambda item: item.startswith("SENSOR,"), "SENSOR"
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
            imu_line = self._request_with_retry(
                "IMU", lambda item: item.startswith("IMU,"), "IMU"
            )
            if imu_line is None or not imu_line.startswith("IMU,OK,"):
                raise RuntimeError(
                    f"IMU is not ready for square autotune: {imu_line}"
                )

            # 让小车在方框模式下持续跑，方便我们在每个 Kp/Kd 组合下打分
            self.link.drain_data()
            compact_ready = self._request_with_retry(
                "STREAM TUNE", lambda item: item == "OK STREAM TUNE",
                "compact telemetry",
            )
            if compact_ready is None:
                raise RuntimeError("compact line telemetry was not enabled")
            self._start_square_reliably(LINE_TUNE_SPEED_MMPS, 10)
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
            self._save_parameters()

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
                "search": "coarse + local + joint neighborhood",
                "score_version": LINE_SCORE_VERSION,
            }
            result_path = self.session_dir / "line_tuned_params.json"
            result_path.write_text(
                json.dumps(result, indent=2), encoding="ascii"
            )
            self._save_session_info()
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
            failure = f"LINE FAILED: {exc}"
            self._save_failure(failure)
            self._status(failure)
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
        """对单个轴做一轮三点局部搜索，随后由联合邻域完成精修。"""
        bounds = (
            LINE_KP_SAFE_RANGE if axis == "kp" else LINE_KD_SAFE_RANGE
        )
        measured = {value: score for score, value in seed_scores}
        best = initial
        step = initial_step

        offsets = (-(step // 2), 0, step // 2)
        candidates = sorted({
            max(bounds[0], min(bounds[1], best + offset))
            for offset in offsets
        })
        for value in candidates:
            if value in measured:
                continue
            kp = value if axis == "kp" else fixed
            kd = fixed if axis == "kp" else value
            measured[value] = self._run_square_line_candidate(
                kp, kd, f"refine_{axis}_{value}", sample_target=80
            )
        best = min(measured, key=measured.get)
        self._status(
            f"{axis.upper()} local refine: best={best}/1000 "
            f"score={measured[best]:.3f}"
        )
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
                if self._send_kick(LINE_PROBE_KICK_MMPS, LINE_PROBE_KICK_MS):
                    kick_stage = 1
            elif kick_stage == 1 and len(samples) >= sample_target // 2:
                # 再向左踢一脚，构成一次正反扰动
                if self._send_kick(-LINE_PROBE_KICK_MMPS, LINE_PROBE_KICK_MS):
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
                if max(abs(sample.left_speed), abs(sample.right_speed)) > 1000:
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
            if line.startswith(("SQUARE ERROR", "SQUARE DONE", "SERIAL ERROR:")):
                raise RuntimeError(line)
        return None

    def _run_tune_transaction(
        self,
        command: str,
        token: int,
        wheel: str,
        kind: str,
        result_prefix: str,
        test_duration_ms: int,
    ) -> str | None:
        """可靠启动一次本地测试，再通过只读 TUNEGET 取回缓存结果。"""
        ack = f"TA,{token},{kind},{wheel}"
        result: str | None = None
        self.link.drain_data()

        # TUNEOPEN/TUNESTEP 是幂等命令：相同 token 重发只补 ACK，不重启电机。
        for _ in range(COMMAND_RETRIES):
            self._check_cancelled()
            self.link.send(command)
            reply = self._wait_for(
                lambda item: (
                    item == ack or item.startswith(result_prefix) or
                    item.startswith(f"TE,{token},")
                ),
                COMMAND_ACK_TIMEOUT,
            )
            if reply is None:
                continue
            if reply.startswith(result_prefix):
                return reply
            if reply == ack:
                break
            if reply == f"TE,{token},BUSY":
                time.sleep(0.15)
                continue
            raise RuntimeError(f"MCU rejected tune request: {reply}")
        else:
            return None

        deadline = time.monotonic() + (test_duration_ms / 1000.0) + 2.5
        next_get = time.monotonic() + (test_duration_ms / 1000.0) + 0.05
        while time.monotonic() < deadline:
            self._check_cancelled()
            try:
                line = self.link.data_queue.get(timeout=0.08)
            except queue.Empty:
                line = None
            if line is not None and line.startswith(result_prefix):
                result = line
                break
            if time.monotonic() >= next_get:
                self.link.send(f"TUNEGET {token}")
                next_get = time.monotonic() + 0.22
        return result

    def _next_tune_token(self) -> int:
        self.tune_token = (self.tune_token % 2_147_483_647) + 1
        return self.tune_token

    def _send_set(self, name: str, value: int) -> None:
        """带 token 写参数；响应丢失可重发且不再回传冗长 PARAM 行。"""
        token = self._next_tune_token()
        expected = f"SA,{token},{name},{value}"
        reply = self._request_with_retry(
            f"SET {name} {value} {token}",
            lambda item: item == expected or item.startswith("ERR "),
            f"SET {name}",
        )
        if reply != expected:
            raise RuntimeError(
                f"MCU rejected SET {name} {value}: {reply or 'timeout'}"
            )

    def _save_parameters(self) -> None:
        """带 token 保存；ACK 丢失后不会再次擦写 Flash。"""
        token = self._next_tune_token()
        expected = f"SV,{token},OK"
        reply = self._request_with_retry(
            f"SAVE {token}",
            lambda item: item in (expected, f"SV,{token},ERR"),
            "Flash SAVE",
            timeout=1.0,
        )
        if reply != expected:
            raise RuntimeError(
                f"MCU could not save parameters to flash: "
                f"{reply or 'timeout'}"
            )

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
        validation_samples, score = self._run_step(
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
            "validation_samples": validation_samples,
        }

    def _identify_feedforward(self, wheel: str) -> tuple[int, int]:
        """通过开环扫 PWM 拟合 pwm = a * speed + b，得出 min_pwm 和 ff_x1000。

        每个档位由 MCU 本地完成固定测试窗并只回传汇总；拟合采用 Theil-Sen
        中位数斜率，降低偶发异常帧的影响。"""
        points: list[tuple[float, int]] = []
        all_rows: list[list[object]] = []

        for pwm in OPEN_PWM_POINTS:
            self._check_cancelled()
            self._status(f"{wheel}: local PWM identification {pwm}")
            token = self._next_tune_token()
            command = (
                f"TUNEOPEN {wheel} {pwm} {OPEN_TUNE_DURATION_MS} {token}"
            )
            line = self._run_tune_transaction(
                command, token, wheel, "O", f"TO,{token},{wheel},",
                OPEN_TUNE_DURATION_MS,
            )
            if line is None:
                raise RuntimeError(f"{wheel}: no local PWM result for {pwm}")
            parts = line.split(",")
            try:
                reported_pwm = int(parts[3])
                speed = float(int(parts[4]))
                sample_count = int(parts[5])
                max_speed = int(parts[6])
                flags = int(parts[7])
            except (IndexError, ValueError):
                raise RuntimeError(f"{wheel}: invalid PWM result: {line}")
            if reported_pwm != pwm:
                raise RuntimeError(f"{wheel}: incomplete PWM result: {line}")
            all_rows.append(
                [pwm, round(speed, 3), sample_count, max_speed, flags]
            )
            if flags != 0:
                self._status(
                    f"{wheel}: safety stop at PWM={pwm}, max={max_speed} mm/s"
                )
                break
            if sample_count < 20:
                raise RuntimeError(f"{wheel}: incomplete PWM result: {line}")
            if 35 <= speed < ENCODER_SPEED_CLIP_MMPS:
                points.append((speed, pwm))
            if speed >= TARGET_SPEED_MMPS * 2.2 and len(points) >= 4:
                break
        self._save_rows(
            f"{wheel}_feedforward.csv",
            ["pwm", "steady_speed_mmps", "sample_count", "max_speed", "flags"],
            all_rows,
        )

        if len(points) < 3:
            raise RuntimeError(
                f"{wheel}: encoder did not provide enough usable speed points"
            )

        ordered = sorted(points, key=lambda point: point[1])
        large_drops = sum(
            1 for previous, current in zip(ordered, ordered[1:])
            if current[0] < previous[0] * 0.85
        )
        if large_drops:
            raise RuntimeError(
                f"{wheel}: PWM-speed response is not monotonic; check encoder "
                "wiring, wheel friction, and battery"
            )
        speeds = [speed for speed, _ in points]
        if min(speeds) > TARGET_SPEED_MMPS or max(speeds) < TARGET_SPEED_MMPS:
            raise RuntimeError(
                f"{wheel}: open-loop scan did not bracket {TARGET_SPEED_MMPS} "
                f"mm/s (measured {min(speeds):.0f}..{max(speeds):.0f}); "
                "verify encoder scale and motor supply"
            )

        # 只要目标工作区附近有足够数据，就不用远高于目标的高速点拉偏
        # FF 斜率；若电机死区很大，再自动退回全部可用点。
        operating_points = [
            point for point in points
            if point[0] <= TARGET_SPEED_MMPS * 2.5
        ]
        if len(operating_points) >= 3:
            points = operating_points

        slopes = [
            (pwm_b - pwm_a) / (speed_b - speed_a)
            for index, (speed_a, pwm_a) in enumerate(points)
            for speed_b, pwm_b in points[index + 1:]
            if abs(speed_b - speed_a) >= 5
        ]
        if not slopes:
            raise RuntimeError(f"{wheel}: invalid PWM-speed identification")
        slope = statistics.median(slopes)
        intercept = statistics.median(
            pwm - slope * speed for speed, pwm in points
        )

        min_pwm = max(0, min(450, round(intercept)))
        ff_x1000 = max(50, min(1000, round(slope * 1000)))
        predicted_target_pwm = intercept + slope * TARGET_SPEED_MMPS
        if not min(pwm for _, pwm in points) - 20 <= predicted_target_pwm <= \
                max(pwm for _, pwm in points) + 20:
            raise RuntimeError(f"{wheel}: feedforward fit extrapolates outside scan")
        self.feedforward_diagnostics[wheel] = {
            "slope_pwm_per_mmps": round(slope, 6),
            "intercept_pwm": round(intercept, 3),
            "predicted_pwm_at_target": round(predicted_target_pwm, 3),
            "usable_points": [[round(speed, 3), pwm] for speed, pwm in points],
        }
        return min_pwm, ff_x1000

    def _search_kp(self, wheel: str) -> int:
        """Kp 粗扫后复测前两名，以中位数抵抗电池/编码器偶发扰动。"""
        self._send_set(f"{wheel}KI", 0)
        scored: list[tuple[float, int]] = []
        for kp in KP_CANDIDATES:
            self._check_cancelled()
            self._status(f"{wheel}: testing Kp={kp}/1000")
            _, score = self._run_step(wheel, kp, 0, 1500, f"kp_{kp}")
            scored.append((score, kp))
        best = self._retest_wheel_finalists(wheel, "kp", scored, 0)
        self._status(f"{wheel}: selected Kp={best}/1000")
        return best

    def _search_ki(self, wheel: str, kp: int) -> int:
        """Ki 粗扫后复测前两名，以中位数选出稳定解。"""
        scored: list[tuple[float, int]] = []
        for ki in KI_CANDIDATES:
            self._check_cancelled()
            self._status(f"{wheel}: testing Ki={ki}/1000")
            _, score = self._run_step(wheel, kp, ki, 2000, f"ki_{ki}")
            scored.append((score, ki))
        best = self._retest_wheel_finalists(wheel, "ki", scored, kp)
        self._status(f"{wheel}: selected Ki={best}/1000")
        return best

    def _retest_wheel_finalists(
        self,
        wheel: str,
        axis: str,
        scored: list[tuple[float, int]],
        fixed: int,
    ) -> int:
        """复测两个最佳候选，采用重复测试的中位数决策。"""
        finalists = sorted(scored)[:WHEEL_FINALIST_COUNT]
        ranked: list[tuple[float, int]] = []
        for initial_score, value in finalists:
            scores = [initial_score]
            for index in range(WHEEL_FINALIST_RETESTS):
                self._check_cancelled()
                kp = value if axis == "kp" else fixed
                ki = fixed if axis == "kp" else value
                self._status(
                    f"{wheel}: validating {axis.upper()}={value}/1000 "
                    f"({index + 1}/{WHEEL_FINALIST_RETESTS})"
                )
                _, score = self._run_step(
                    wheel, kp, ki,
                    1800 if axis == "kp" else 2300,
                    f"{axis}_{value}_validation_{index + 1}",
                )
                scores.append(score)
            median_score = statistics.median(scores)
            ranked.append((median_score, value))
            self._status(
                f"{wheel}: {axis.upper()}={value}/1000 "
                f"median score={median_score:.3f}"
            )
        ranked.sort()
        return ranked[0][1]

    def _run_step(
        self, wheel: str, kp: int, ki: int, duration_ms: int, name: str
    ) -> tuple[int, float]:
        """运行静默本地 STEP；无线桥只回传一条汇总指标。"""
        self._send_set(f"{wheel}KP", kp)
        self._send_set(f"{wheel}KI", ki)
        token = self._next_tune_token()
        command = (
            f"TUNESTEP {wheel} {TARGET_SPEED_MMPS} {duration_ms} {token}"
        )
        line = self._run_tune_transaction(
            command, token, wheel, "S", f"TS,{token},{wheel},", duration_ms
        )
        if line is None:
            raise RuntimeError(f"{wheel}: no local STEP result")
        try:
            (_, reported_token, reported_wheel, count, sum_abs, max_speed,
             tail_count, tail_sum, tail_square_sum, rise_ms,
              saturation_count, flags) = line.split(",")
            if (int(reported_token) != token or reported_wheel != wheel):
                raise ValueError
            count = int(count)
            sum_abs = int(sum_abs)
            max_speed = int(max_speed)
            tail_count = int(tail_count)
            tail_sum = int(tail_sum)
            tail_square_sum = int(tail_square_sum)
            rise_ms = int(rise_ms)
            saturation_count = int(saturation_count)
            flags = int(flags)
        except ValueError:
            raise RuntimeError(f"{wheel}: invalid local STEP result: {line}")
        if flags != 0:
            score = 100.0 + (max_speed / max(1, TARGET_SPEED_MMPS))
            self._save_rows(
                f"{wheel}_{name}.csv",
                ["sample_count", "mean_abs_error", "max_speed", "tail_count",
                 "tail_mean", "tail_ripple", "rise_ms", "saturation",
                 "flags", "score"],
                [[count, 0, max_speed, tail_count, 0, 0, rise_ms, 0,
                  flags, round(score, 6)]],
            )
            self._status(
                f"{wheel}: {name} rejected by firmware safety limit "
                f"(max={max_speed} mm/s)"
            )
            return count, score
        if count < 20 or tail_count < 5:
            raise RuntimeError(f"{wheel}: incomplete local STEP result: {line}")

        target = TARGET_SPEED_MMPS
        mean_error = (sum_abs / count) / target
        overshoot = max(0.0, max_speed - target) / target
        tail_mean = tail_sum / tail_count
        variance = max(0.0, (tail_square_sum / tail_count) - tail_mean ** 2)
        ripple = math.sqrt(variance) / target
        steady_error = abs(tail_mean - target) / target
        rise_penalty = min(1.0, rise_ms / 1500.0)
        saturation = saturation_count / count
        score = (
            mean_error + (3.0 * overshoot) + (2.5 * steady_error) +
            (1.2 * ripple) + (0.35 * rise_penalty) +
            (2.0 * saturation)
        )
        self._save_rows(
            f"{wheel}_{name}.csv",
            ["sample_count", "mean_abs_error", "max_speed", "tail_count",
             "tail_mean", "tail_ripple", "rise_ms", "saturation", "flags",
             "score"],
            [[count, round(mean_error, 6), max_speed, tail_count,
              round(tail_mean, 3), round(ripple, 6), rise_ms,
              round(saturation, 6), flags, round(score, 6)]],
        )
        return count, score

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
        self.title("MSPM0 小车 PID 自动调参上位机")
        self.geometry("1100x720")
        self.minsize(920, 640)

        self.link = SerialLink()
        # AutoTuner 推给 UI 的状态文本
        self.status_queue: queue.Queue[str] = queue.Queue()
        self.tuner = AutoTuner(
            self.link, self.status_queue, self._autotune_finished
        )
        # 最近 250 帧的 PID 样本，用于实时绘图
        self.plot_samples: list[PidSample] = []
        # 手动方形赛道 / IMU 转弯验证的原始数据与参数快照。
        self.square_samples: list[PidSample] = []
        self.square_running = False
        self.current_parameters: dict[str, int] = {}
        # 用来把绘图刷新降到每 10 帧一次（CPU 优化）
        self.pid_display_divider = 0

        # ---------- 串口 / 状态相关 StringVar ----------
        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="未连接")
        # 安全门：必须显式勾选才能启动对应的自动整定。
        self.raise_confirmed = tk.BooleanVar(value=False)
        self.track_confirmed = tk.BooleanVar(value=False)
        self.manual_pwm_var = tk.IntVar(value=120)
        # 方形赛道使用 IMU yaw 完成转弯；这些是对应的可调参数。
        self.square_speed_var = tk.IntVar(value=340)
        self.square_laps_var = tk.IntVar(value=1)
        self.turn_angle_var = tk.IntVar(value=900)
        self.turn_fast_var = tk.IntVar(value=185)
        self.turn_slow_var = tk.IntVar(value=140)
        self.turn_margin_var = tk.IntVar(value=180)
        self.turn_exit_var = tk.IntVar(value=140)
        self.turn_distance_var = tk.IntVar(value=85)
        self.status_var = tk.StringVar(value="就绪")

        self._build_ui()
        self._refresh_ports()
        # 50 ms 一次的轮询循环（之后由 _poll_queues 内部 self.after 续约）
        self.after(50, self._poll_queues)
        # 用户点窗口 X：先安全关闭串口再退出
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        """创建调参专用主界面。"""
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

        manual = ttk.LabelFrame(self, text="直接电机 PWM", padding=(10, 6))
        manual.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Label(manual, text="PWM (0-300，悬空短测)").pack(side=tk.LEFT)
        ttk.Spinbox(
            manual, from_=0, to=300, increment=10, width=6,
            textvariable=self.manual_pwm_var,
        ).pack(side=tk.LEFT, padx=(6, 12))
        self.manual_left_button = ttk.Button(
            manual, text="左轮输出", command=lambda: self._send_direct_pwm("L")
        )
        self.manual_left_button.pack(side=tk.LEFT, padx=3)
        self.manual_right_button = ttk.Button(
            manual, text="右轮输出", command=lambda: self._send_direct_pwm("R")
        )
        self.manual_right_button.pack(side=tk.LEFT, padx=3)
        self.manual_both_button = ttk.Button(
            manual, text="两轮输出", command=lambda: self._send_direct_pwm("B")
        )
        self.manual_both_button.pack(side=tk.LEFT, padx=3)
        ttk.Button(manual, text="停止", command=self._stop).pack(
            side=tk.LEFT, padx=(12, 3)
        )

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

        square = ttk.LabelFrame(
            self, text="方形赛道 / IMU 转弯验证", padding=(10, 6)
        )
        square.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Label(square, text="速度 (mm/s)").pack(side=tk.LEFT)
        ttk.Spinbox(
            square, from_=80, to=450, increment=10, width=7,
            textvariable=self.square_speed_var,
        ).pack(side=tk.LEFT, padx=(6, 12))
        ttk.Label(square, text="圈数").pack(side=tk.LEFT)
        ttk.Spinbox(
            square, from_=1, to=10, increment=1, width=5,
            textvariable=self.square_laps_var,
        ).pack(side=tk.LEFT, padx=(6, 12))
        self.square_button = ttk.Button(
            square, text="运行逆时针方形", command=self._start_square
        )
        self.square_button.pack(side=tk.LEFT, padx=6)

        turn = ttk.Frame(square)
        turn.pack(side=tk.RIGHT)
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
                width=5, textvariable=variable,
            ).pack(side=tk.LEFT, padx=(3, 6))

        quick_diagnostic = ttk.LabelFrame(
            self, text="通信诊断", padding=(10, 6)
        )
        quick_diagnostic.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Label(
            quick_diagnostic, text="检查结果显示在下方日志："
        ).pack(side=tk.LEFT)
        ttk.Button(
            quick_diagnostic,
            text="检查通信",
            command=lambda: self._send_diagnostic("PING"),
        ).pack(side=tk.LEFT, padx=(10, 4))
        ttk.Button(
            quick_diagnostic,
            text="检查灰度传感器",
            command=lambda: self._send_diagnostic("SENSOR"),
        ).pack(side=tk.LEFT, padx=(10, 4))
        ttk.Button(
            quick_diagnostic,
            text="检查 IMU",
            command=lambda: self._send_diagnostic("IMU"),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            quick_diagnostic,
            text="检查无线桥",
            command=lambda: self._send_diagnostic("RADIOPING"),
        ).pack(side=tk.LEFT, padx=4)

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
        self.log = tk.Text(log_frame, wrap=tk.NONE, height=16, state=tk.DISABLED)
        scroll_y = ttk.Scrollbar(
            log_frame, orient=tk.VERTICAL, command=self.log.yview
        )
        self.log.configure(yscrollcommand=scroll_y.set)
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)

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
            self.connect_button.configure(state=tk.DISABLED)
            # ESP8266 上电提示可能仍在输出；用 after 延迟探测，不能 sleep 卡住 UI。
            self.after(800, lambda: self._finish_connection_probe(port))
        except Exception as exc:
            self.link.close(send_stop=False)
            self.connection_var.set("未连接")
            self.connect_button.configure(text="连接")
            messagebox.showerror("串口连接失败", str(exc))

    def _finish_connection_probe(self, port: str) -> None:
        """串口打开后的非阻塞探测；自动任务还会执行严格的 V2 握手。"""
        try:
            if not self.link.connected:
                raise RuntimeError("serial port closed during startup")
            self.link.send("PING")
            self.link.send("PARAM")
            self.connection_var.set(f"已连接：{port}")
            self.connect_button.configure(text="断开", state=tk.NORMAL)
        except Exception as exc:
            self.link.close(send_stop=False)
            self.connection_var.set("未连接")
            self.connect_button.configure(text="连接", state=tk.NORMAL)
            messagebox.showerror("串口连接失败", str(exc))

    def _send_diagnostic(self, command: str) -> None:
        """通过快捷按钮发送一条只读诊断命令。"""
        if not self.link.connected:
            messagebox.showerror("通信诊断", "请先连接小车")
            return
        try:
            self.link.send(command)
            self.status_var.set(f"已发送 {command}，等待下方日志返回")
        except Exception as exc:
            messagebox.showerror("通信诊断", str(exc))

    def _send_direct_pwm(self, wheel: str) -> None:
        """不等待 PING/STATUS/ACK，直接让指定轮输出当前 PWM。"""
        if not self.link.connected:
            messagebox.showerror("直接电机 PWM", "请先连接小车")
            return
        if self.tuner.running or self.square_running:
            messagebox.showerror("直接电机 PWM", "请先停止当前自动任务")
            return
        try:
            pwm = int(self.manual_pwm_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("直接电机 PWM", "PWM 必须是 0 到 300 的整数")
            return
        if not 0 <= pwm <= 300:
            messagebox.showerror("直接电机 PWM", "PWM 必须在 0 到 300 之间")
            return
        try:
            self.link.send(f"PWM {wheel} {pwm}")
            target = {"L": "左轮", "R": "右轮", "B": "两轮"}[wheel]
            self.status_var.set(f"已直接发送：{target} PWM={pwm}")
        except Exception as exc:
            messagebox.showerror("直接电机 PWM", str(exc))

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
        if self.tuner.running or self.square_running:
            return
        self.link.send("STOP")
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
        if self.tuner.running or self.square_running:
            return
        self.link.send("STOP")
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
        self.link.send("STOP")
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
        self.square_samples.clear()
        self.plot_samples.clear()
        self.square_running = True
        self._set_tune_buttons(False)
        self.status_var.set("正在检查传感器、写入参数并启动方形赛道")
        threading.Thread(
            target=self._configure_and_start_square,
            args=(
                speed, laps, angle, fast_pwm, slow_pwm,
                margin, exit_speed, turn_distance,
            ),
            daemon=True,
        ).start()

    def _wait_host_reply(self, predicate, timeout: float) -> str | None:
        """供非调参工作线程等待固件回复；UI 日志仍走 display_queue。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.square_running:
                return None
            try:
                line = self.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if predicate(line):
                return line
        return None

    def _configure_and_start_square(
        self,
        speed: int,
        laps: int,
        angle: int,
        fast_pwm: int,
        slow_pwm: int,
        margin: int,
        exit_speed: int,
        turn_distance: int,
    ) -> None:
        """后台完成诊断、参数确认、Flash 保存和 SQUARE 启动握手。"""
        try:
            self.tuner.cancel_event.clear()
            self.link.drain_data()
            self.tuner._verify_connection()
            self.tuner._stop_reliably()

            sensor = self.tuner._request_with_retry(
                "SENSOR", lambda item: item.startswith("SENSOR,"), "SENSOR"
            )
            if sensor is None or not sensor.startswith("SENSOR,OK,"):
                raise RuntimeError(
                    f"line sensor is not ready: {sensor or 'timeout'}"
                )

            imu = self.tuner._request_with_retry(
                "IMU", lambda item: item.startswith("IMU,"), "IMU"
            )
            if imu is None or not imu.startswith("IMU,OK,"):
                raise RuntimeError(f"IMU is not ready: {imu or 'timeout'}")

            # 先把慢速 PWM 临时降到合法下限，再设置快速和最终慢速值。
            # 这样无论 Flash 中旧的 fast/slow 大小关系如何，都不会因为
            # 固件的交叉范围检查而悄悄拒绝新参数。
            commands = (
                ("TURNSLOW", 80),
                ("TURNFAST", fast_pwm),
                ("TURNSLOW", slow_pwm),
                ("TURNANGLE", angle),
                ("TURNMARGIN", margin),
                ("TURNEXIT", exit_speed),
                ("TURNDIST", turn_distance),
            )
            for name, value in commands:
                self.tuner._send_set(name, value)

            self.tuner._save_parameters()
            self.tuner._start_square_reliably(speed, laps)
            self.status_queue.put(
                f"CCW square running: {laps} lap(s), STOP remains available"
            )
        except Exception as exc:
            try:
                if self.link.connected:
                    self.link.send("STOP")
            except Exception:
                pass
            self.link.display_queue.put(f"HOST SQUARE ERROR,{exc}")

    def _stop(self) -> None:
        """紧急停止：取消自动整定或方形验证，并立即给小车发送 STOP。"""
        self.tuner.cancel()
        self.square_running = False
        self._set_tune_buttons(True)
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
            self.tuner.cancel_event.clear()
            self.tuner._verify_connection()
            self.tuner._stop_reliably()
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
                    self.tuner._send_set(name, int(value))
            self.tuner._send_set("BIAS", STRAIGHT_BIAS_MMPS)
            self.tuner._save_parameters()
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
        self.link.send("STOP")
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
            self.tuner.cancel_event.clear()
            self.tuner._verify_connection()
            self.tuner._stop_reliably()
            self.tuner._save_parameters()
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
        """同时启用/禁用调参按钮，避免用户重复点击。"""
        state = tk.NORMAL if enabled else tk.DISABLED
        self.auto_both_button.configure(state=state)
        self.auto_line_button.configure(state=state)
        self.square_button.configure(state=state)

    def _poll_queues(self) -> None:
        """50 ms 一次的轮询：

        1. 拉空 display_queue：可解析成 PidSample 的塞进实时绘图缓冲，
           方形验证运行时同时保存其日志；其它文本直接写入日志。
        2. 拉空 status_queue：哨兵 __FINISHED__ 表示寻优结束，否则就是普通状态。
        3. 重绘 canvas。
        4. self.after(50, ...) 续约。"""
        plot_needs_redraw = False
        while True:
            try:
                line = self.link.display_queue.get_nowait()
            except queue.Empty:
                break
            sample = PidSample.parse(line)
            if sample is not None:
                # 是 PID 帧：保存最近 250 帧用于实时绘图。
                self.plot_samples.append(sample)
                # 只保留最近 250 帧，避免内存涨
                self.plot_samples = self.plot_samples[-250:]
                if self.square_running and sample.mode == "SQUARE":
                    self.square_samples.append(sample)
                # 10 帧才走一次重绘
                self.pid_display_divider = (self.pid_display_divider + 1) % 10
                if self.pid_display_divider != 0:
                    continue
                plot_needs_redraw = True
            elif self.square_running and (
                line.startswith("SQUARE DONE")
                or line.startswith("SQUARE ERROR")
                or line.startswith("ERR SQUARE")
                or line.startswith("HOST SQUARE ERROR")
            ):
                self.square_running = False
                self._set_tune_buttons(True)
                if self.square_samples:
                    self._save_square_run(line)
                else:
                    self.status_var.set(line)
            elif line.startswith("SQUARE LEARNED,TURNDIST,"):
                try:
                    self.turn_distance_var.set(int(line.rsplit(",", 1)[1]))
                except ValueError:
                    pass
            elif line.startswith("PARAM,"):
                self._apply_parameter_report(line)
            elif line.startswith("SERIAL ERROR:"):
                # 串口挂了：自动关闭 UI 连接状态
                self.link.close(send_stop=False)
                self.connection_var.set("未连接")
                self.connect_button.configure(text="连接")
            if self.tuner.running and line.startswith((
                "TO,", "TS,", "TA,", "TP,", "TE,", "SA,", "SV,", "SQ,",
                "KA,", "LT,", "TUNE PENDING", "TEST START,STEP", "OK PWM,",
            )):
                # 自动整定只显示上位机阶段状态；协议帧仍留在 data_queue
                # 给算法使用，不把结果行和 ACK 刷满日志框。
                continue
            self._append_log(line)

        while True:
            try:
                status = self.status_queue.get_nowait()
            except queue.Empty:
                break
            if status == "__FINISHED__":
                self._set_tune_buttons(True)
            else:
                self.status_var.set(status)
                self._append_log(status)

        if plot_needs_redraw:
            self._draw_plot()
        self.after(50, self._poll_queues)

    def _append_log(self, text: str) -> None:
        """往 Text 日志框追加一行并滚到底。"""
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

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
