import csv
import json
import math
import queue
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports
import tkinter as tk
from tkinter import messagebox, ttk


BAUD_RATE = 115200
BRIDGE_BOOT_SECONDS = 3.0
STARTUP_STATUS_RETRIES = 4
AUTOTUNE_HANDSHAKE_RETRIES = 12
AUTOTUNE_HANDSHAKE_INTERVAL_MS = 700
PLOT_HISTORY = 360
POLL_MS = 50
IMAGE_WIDTH = 480
IMAGE_HEIGHT = 320
TELEMETRY_MIN_VALID_AREA = 1000
TELEMETRY_AREA_RATIO_MIN = 0.20
TELEMETRY_AREA_RATIO_MAX = 5.00
CONTROL_PROFILE = "GIMBAL_PID_V10"
PID_SCORE_PROFILE = "PID_FAST_FOLLOW_STABLE_V10"
DEFAULT_TRACK_KI = 0.25
DEFAULT_TRACK_VELOCITY_FF_GAIN = 0.032
DEFAULT_MAX_TRACK_VELOCITY_FF_RPM = 4.0
DEFAULT_MAX_TRACK_INTEGRAL_RPM = 12.0
DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE = 0.15
COMPATIBLE_CHAMPION_PROFILES = (
    CONTROL_PROFILE,
    "GIMBAL_PID_V9D",
    "GIMBAL_PID_V9C",
    "GIMBAL_PID_V9",
    "GIMBAL_PID_V8",
    "GIMBAL_PID_V6",
    "GIMBAL_PD_V5",
    "GIMBAL_PD_V4",
)
MIN_COMPARABLE_AREA_RATIO = 0.65
MAX_COMPARABLE_AREA_RATIO = 1.55


class SerialLink:
    def __init__(self, receive_queue):
        self.receive_queue = receive_queue
        self.serial = None
        self.reader = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self):
        return self.serial is not None and self.serial.is_open

    def open(self, port):
        self.close()
        self.serial = serial.Serial(
            port,
            BAUD_RATE,
            timeout=0.1,
            write_timeout=1.0,
        )
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.reset_input_buffer()
        self.stop_event.clear()
        self.reader = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.reader.start()

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                raw = self.serial.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.receive_queue.put(line)
            except (OSError, serial.SerialException) as error:
                self.receive_queue.put("SERIAL_ERROR,%s" % error)
                if not self.serial or not self.serial.is_open:
                    return
                time.sleep(0.2)

    def send(self, command):
        if not self.connected:
            raise RuntimeError("串口未连接")
        payload = (command.strip() + "\r\n").encode("utf-8")
        with self.write_lock:
            self.serial.write(payload)
            self.serial.flush()

    def close(self):
        self.stop_event.set()
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        self.serial = None


class GimbalApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("K230 二维云台视觉 PID 自动调参上位机")
        self.geometry("1180x780")
        self.minsize(1000, 680)

        self.receive_queue = queue.Queue()
        self.link = SerialLink(self.receive_queue)
        self.connected_at = 0.0
        self.autotune_pending = False
        self.autotune_phase = None
        self.autotune_attempts = 0
        self.autotune_generation = 0
        self.champion_load_pending = False
        self.champion_load_attempts = 0
        self.startup_status_pending = False
        self.startup_status_attempts = 0
        self.plot_samples = deque(maxlen=PLOT_HISTORY)
        self.rect_counter = 0
        self.session_areas = []
        self.last_rect_time_ms = None
        self.last_rect_area = 0
        self.dropped_rect_samples = 0
        self.tune_result_counts = {
            "PID": 0,
            "FOLLOW": 0,
            "VERIFY": 0,
        }

        self.host_folder = Path(__file__).resolve().parent
        self.champion_path = self.host_folder / "champion_params.json"
        self.champion = self._read_champion()
        self.session_folder = None
        self.csv_file = None
        self.csv_writer = None
        self.runtime_file = None

        self.connection_var = tk.StringVar(value="未连接")
        self.state_var = tk.StringVar(value="等待连接")
        self.target_var = tk.StringVar(value="目标：--")
        self.error_var = tk.StringVar(value="误差：X -- / Y --")
        self.command_var = tk.StringVar(value="命令：X -- / Y -- RPM")
        self.params_var = tk.StringVar(
            value="X: Kp/Ki/Kd --/--/--    Y: Kp/Ki/Kd --/--/--"
        )
        self.champion_var = tk.StringVar(
            value=self._champion_summary()
        )
        self.confirm_var = tk.BooleanVar(value=False)

        self._build_ui()
        self._refresh_ports()
        self.after(POLL_MS, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _read_champion(self):
        if not self.champion_path.exists():
            return None
        try:
            data = json.loads(
                self.champion_path.read_text(encoding="utf-8")
            )
            required_axis_fields = {
                "kp",
                "ki",
                "kd",
                "polarity",
                "score",
            }
            for axis in ("x", "y"):
                axis_data = data.get(axis)
                if isinstance(axis_data, dict) and "ki" not in axis_data:
                    axis_data["ki"] = DEFAULT_TRACK_KI
            follow_data = data.get("follow")
            if not isinstance(follow_data, dict):
                follow_data = {}
            follow_data.setdefault(
                "ff_gain", DEFAULT_TRACK_VELOCITY_FF_GAIN
            )
            follow_data.setdefault(
                "ff_max", DEFAULT_MAX_TRACK_VELOCITY_FF_RPM
            )
            follow_data.setdefault(
                "integral_limit", DEFAULT_MAX_TRACK_INTEGRAL_RPM
            )
            follow_data.setdefault(
                "large_i_scale",
                DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE,
            )
            data["follow"] = follow_data
            if (
                data.get("profile") not in COMPATIBLE_CHAMPION_PROFILES
                or "total_score" not in data
                or "median_area" not in data
                or not isinstance(data.get("x"), dict)
                or not isinstance(data.get("y"), dict)
                or not required_axis_fields.issubset(data["x"])
                or not required_axis_fields.issubset(data["y"])
            ):
                return None
            numeric_values = [
                data["total_score"],
                data["median_area"],
                data["x"]["kp"],
                data["x"]["ki"],
                data["x"]["kd"],
                data["x"]["score"],
                data["y"]["kp"],
                data["y"]["ki"],
                data["y"]["kd"],
                data["y"]["score"],
                data["follow"]["ff_gain"],
                data["follow"]["ff_max"],
                data["follow"]["integral_limit"],
                data["follow"]["large_i_scale"],
            ]
            if not all(
                math.isfinite(float(value))
                for value in numeric_values
            ):
                return None
            if (
                float(data["total_score"]) <= 0.0
                or float(data["median_area"]) <= 0.0
                or float(data["x"]["score"]) <= 0.0
                or float(data["y"]["score"]) <= 0.0
                or int(data["x"]["polarity"]) not in (-1, 1)
                or int(data["y"]["polarity"]) not in (-1, 1)
                or not (
                    0.015
                    <= float(data["follow"]["ff_gain"])
                    <= 0.070
                )
                or not (
                    2.0
                    <= float(data["follow"]["ff_max"])
                    <= 8.0
                )
                or not (
                    6.0
                    <= float(data["follow"]["integral_limit"])
                    <= 18.0
                )
                or not (
                    0.08
                    <= float(data["follow"]["large_i_scale"])
                    <= 0.35
                )
            ):
                return None
            # Old gains remain loadable, but their scores are marked as a
            # different test profile and cannot defend against a V10 PID run.
            data["profile"] = CONTROL_PROFILE
            return data
        except (OSError, ValueError, TypeError, KeyError):
            return None

    def _champion_summary(self):
        if not self.champion:
            return "冠军参数：尚未建立"
        return (
            "冠军参数：X %.3f/%.3f/%.3f  Y %.3f/%.3f/%.3f  总分 %.2f%s"
            % (
                self.champion["x"]["kp"],
                self.champion["x"]["ki"],
                self.champion["x"]["kd"],
                self.champion["y"]["kp"],
                self.champion["y"]["ki"],
                self.champion["y"]["kd"],
                self.champion["total_score"],
                (
                    ""
                    if self.champion.get("score_profile")
                    == PID_SCORE_PROFILE
                    else "（旧评分，待重测）"
                ),
            )
        )

    def _champion_command(self, champion):
        follow = champion.get("follow", {})
        return (
            "PARAMS,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%d,%.6f,%.3f,%.3f,%.4f"
            % (
                champion["x"]["kp"],
                champion["x"]["ki"],
                champion["x"]["kd"],
                int(champion["x"]["polarity"]),
                champion["y"]["kp"],
                champion["y"]["ki"],
                champion["y"]["kd"],
                int(champion["y"]["polarity"]),
                float(
                    follow.get(
                        "ff_gain", DEFAULT_TRACK_VELOCITY_FF_GAIN
                    )
                ),
                float(
                    follow.get(
                        "ff_max", DEFAULT_MAX_TRACK_VELOCITY_FF_RPM
                    )
                ),
                float(
                    follow.get(
                        "integral_limit",
                        DEFAULT_MAX_TRACK_INTEGRAL_RPM,
                    )
                ),
                float(
                    follow.get(
                        "large_i_scale",
                        DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE,
                    )
                ),
            )
        )

    def _load_champion_to_k230(self, quiet=False):
        if not self.champion:
            if not quiet:
                messagebox.showinfo(
                    "没有冠军参数",
                    "请先完成一次自动调参，建立首个冠军参数。",
                )
            return
        if not self.link.connected:
            if not quiet:
                messagebox.showerror("加载失败", "请先连接COM端口")
            return
        self.champion_load_pending = True
        self.champion_load_attempts = 0
        self._send_champion_attempt(quiet)
        self.params_var.set(
            "X: %.3f/%.3f/%.3f    Y: %.3f/%.3f/%.3f"
            % (
                self.champion["x"]["kp"],
                self.champion["x"]["ki"],
                self.champion["x"]["kd"],
                self.champion["y"]["kp"],
                self.champion["y"]["ki"],
                self.champion["y"]["kd"],
            )
        )
        self.state_var.set("正在向K230加载冠军参数")

    def _send_champion_attempt(self, quiet=True):
        if (
            not self.champion_load_pending
            or not self.link.connected
            or not self.champion
        ):
            return
        if self.champion_load_attempts >= 4:
            self.champion_load_pending = False
            self.state_var.set("冠军参数加载无回复，请检查无线链路后重试")
            self._append_log("冠军参数连续4次加载无回复")
            return
        self.champion_load_attempts += 1
        self._send(self._champion_command(self.champion), quiet=quiet)
        self.after(700, self._send_champion_attempt)

    def _begin_startup_status_probe(self):
        if not self.link.connected or self.autotune_pending:
            return
        self.startup_status_pending = True
        self.startup_status_attempts = 0
        self._send_startup_status_probe()

    def _send_startup_status_probe(self):
        if not self.startup_status_pending or not self.link.connected:
            return
        if self.startup_status_attempts >= STARTUP_STATUS_RETRIES:
            self.startup_status_pending = False
            self.state_var.set("K230状态无回复，请检查无线链路")
            self._append_log("连续%d次读取K230状态无回复" % STARTUP_STATUS_RETRIES)
            return
        self.startup_status_attempts += 1
        self._send("STATUS", quiet=True)
        self.after(700, self._send_startup_status_probe)

    def _write_champion(self, champion):
        temporary_path = self.champion_path.with_suffix(".json.tmp")
        try:
            temporary_path.write_text(
                json.dumps(
                    champion,
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            temporary_path.replace(self.champion_path)
        except OSError as error:
            self._append_log("冠军参数保存失败：%s" % error)
            return False
        self.champion = champion
        self.champion_var.set(self._champion_summary())
        return True

    def _session_median_area(self):
        if not self.session_areas:
            return 0.0
        return float(statistics.median(self.session_areas))

    def _build_candidate(self, values):
        required = (
            "XKP",
            "XKI",
            "XKD",
            "XPOL",
            "XSCORE",
            "YKP",
            "YKI",
            "YKD",
            "YPOL",
            "YSCORE",
        )
        if any(name not in values for name in required):
            self._append_log("本轮结果字段不完整，不参与冠军比较")
            return None
        if not all(math.isfinite(values[name]) for name in required):
            self._append_log("本轮结果包含无效数值，不参与冠军比较")
            return None

        x_score = float(values["XSCORE"])
        y_score = float(values["YSCORE"])
        if x_score < 0.0 or y_score < 0.0:
            self._append_log("本轮评分无效，不参与冠军比较")
            return None
        follow = {
            "ff_gain": float(
                values.get("FFG", DEFAULT_TRACK_VELOCITY_FF_GAIN)
            ),
            "ff_max": float(
                values.get("FFMAX", DEFAULT_MAX_TRACK_VELOCITY_FF_RPM)
            ),
            "integral_limit": float(
                values.get("ILIM", DEFAULT_MAX_TRACK_INTEGRAL_RPM)
            ),
            "large_i_scale": float(
                values.get(
                    "LISCALE",
                    DEFAULT_TRACK_LARGE_ERROR_INTEGRAL_SCALE,
                )
            ),
            "score": float(values.get("FSCORE", 0.0)),
        }
        if not (
            math.isfinite(follow["ff_gain"])
            and math.isfinite(follow["ff_max"])
            and math.isfinite(follow["integral_limit"])
            and math.isfinite(follow["large_i_scale"])
            and 0.015 <= follow["ff_gain"] <= 0.070
            and 2.0 <= follow["ff_max"] <= 8.0
            and 6.0 <= follow["integral_limit"] <= 18.0
            and 0.08 <= follow["large_i_scale"] <= 0.35
        ):
            self._append_log("FOLLOW params invalid")
            return None
        return {
            "profile": CONTROL_PROFILE,
            "score_profile": PID_SCORE_PROFILE,
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "total_score": x_score + y_score,
            "median_area": self._session_median_area(),
            "follow": follow,
            "x": {
                "kp": float(values["XKP"]),
                "ki": float(values["XKI"]),
                "kd": float(values["XKD"]),
                "polarity": int(values["XPOL"]),
                "score": x_score,
            },
            "y": {
                "kp": float(values["YKP"]),
                "ki": float(values["YKI"]),
                "kd": float(values["YKD"]),
                "polarity": int(values["YPOL"]),
                "score": y_score,
            },
        }

    def _consider_champion(self, values):
        candidate = self._build_candidate(values)
        if candidate is None:
            return

        if self.champion is None:
            if self._write_champion(candidate):
                self._append_log(
                    "已建立首个冠军参数，总分 %.3f"
                    % candidate["total_score"]
                )
                self.state_var.set("首个冠军参数已保存，正在跟踪")
            return

        if self.champion.get("score_profile") != PID_SCORE_PROFILE:
            if self._write_champion(candidate):
                self._append_log(
                    "旧冠军使用旧评分体系；本轮已建立首个完整PID冠军"
                )
                self.state_var.set("完整PID冠军已保存，正在跟踪")
            return

        old_area = float(self.champion.get("median_area", 0.0))
        new_area = float(candidate.get("median_area", 0.0))
        comparable = old_area > 0.0 and new_area > 0.0
        if comparable:
            area_ratio = new_area / old_area
            comparable = (
                MIN_COMPARABLE_AREA_RATIO
                <= area_ratio
                <= MAX_COMPARABLE_AREA_RATIO
            )

        if not comparable:
            self._append_log(
                "本轮目标大小与冠军记录不可比（中位面积 %.0f / %.0f），"
                "冠军不变；本轮参数继续使用"
                % (new_area, old_area)
            )
            self.state_var.set("调参完成；环境不可比，冠军保持不变")
            return

        # V10 retests the loaded incumbent on the same target before it can
        # advance, then runs coarse/local/fine PID finals and a follow
        # profile final.
        # Therefore the K230 result is already a head-to-head winner under
        # current conditions. Do not compare it with a stale score recorded
        # under a previous session's noise and lighting.
        candidate["median_area"] = old_area
        if self._write_champion(candidate):
            self._append_log(
                "旧冠军已参加本轮同场复测；综合冠军赛结果已保存，总分 %.3f"
                % candidate["total_score"]
            )
            self.state_var.set("综合冠军已保存，正在跟踪")

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="串口").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(
            top, width=12, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(
            top, text="刷新", command=self._refresh_ports
        ).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(
            top, text="连接", command=self._toggle_connection
        )
        self.connect_button.pack(side=tk.LEFT, padx=6)
        ttk.Label(
            top, textvariable=self.connection_var
        ).pack(side=tk.LEFT, padx=8)
        ttk.Label(
            top, textvariable=self.state_var
        ).pack(side=tk.RIGHT)

        safety = ttk.LabelFrame(
            self, text="自动调参安全确认", padding=8
        )
        safety.pack(fill=tk.X, padx=8, pady=(0, 6))
        ttk.Checkbutton(
            safety,
            variable=self.confirm_var,
            text=(
                "A4黑框完整可见；云台位于安全初始角度；"
                "线缆有余量；电机总电源可随时切断"
            ),
        ).pack(side=tk.LEFT)

        controls = ttk.Frame(self, padding=(8, 2))
        controls.pack(fill=tk.X)
        self.tune_button = ttk.Button(
            controls,
            text="一键自动调参",
            command=self._start_autotune,
        )
        self.tune_button.pack(side=tk.LEFT)
        ttk.Button(
            controls,
            text="加载冠军参数",
            command=self._load_champion_to_k230,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            controls, text="急停", command=self._stop
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            controls,
            text="开始跟踪",
            command=lambda: self._send("TRACK ON"),
        ).pack(side=tk.LEFT)
        ttk.Button(
            controls,
            text="停止跟踪",
            command=lambda: self._send("TRACK OFF"),
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            controls,
            text="松轴",
            command=lambda: self._send("RELEASE"),
        ).pack(side=tk.LEFT)
        ttk.Button(
            controls,
            text="锁轴",
            command=lambda: self._send("LOCK"),
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            controls,
            text="读取状态",
            command=lambda: self._send("STATUS"),
        ).pack(side=tk.LEFT)
        ttk.Button(
            controls,
            text="PING",
            command=lambda: self._send("PING"),
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(
            controls,
            text="RadioPING",
            command=lambda: self._send("RADIOPING"),
        ).pack(side=tk.LEFT)

        info = ttk.Frame(self, padding=8)
        info.pack(fill=tk.X)
        ttk.Label(
            info, textvariable=self.target_var, width=22
        ).pack(side=tk.LEFT)
        ttk.Label(
            info, textvariable=self.error_var, width=28
        ).pack(side=tk.LEFT)
        ttk.Label(
            info, textvariable=self.command_var, width=28
        ).pack(side=tk.LEFT)
        ttk.Label(
            info, textvariable=self.params_var
        ).pack(side=tk.LEFT)
        ttk.Label(
            self,
            textvariable=self.champion_var,
            padding=(8, 0, 8, 4),
        ).pack(fill=tk.X)

        middle = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        plot_frame = ttk.LabelFrame(middle, text="视觉误差曲线")
        table_frame = ttk.LabelFrame(middle, text="候选参数评分")
        middle.add(plot_frame, weight=3)
        middle.add(table_frame, weight=2)

        self.canvas = tk.Canvas(
            plot_frame,
            background="#101418",
            highlightthickness=0,
            height=330,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        columns = ("axis", "mode", "kp", "ki", "kd", "score")
        self.result_table = ttk.Treeview(
            table_frame,
            columns=columns,
            show="headings",
            height=13,
        )
        headings = {
            "axis": "轴",
            "mode": "测试",
            "kp": "Kp",
            "ki": "Ki",
            "kd": "Kd",
            "score": "评分（越小越好）",
        }
        widths = {
            "axis": 38,
            "mode": 48,
            "kp": 58,
            "ki": 58,
            "kd": 58,
            "score": 110,
        }
        for column in columns:
            self.result_table.heading(
                column, text=headings[column]
            )
            self.result_table.column(
                column, width=widths[column], anchor=tk.CENTER
            )
        self.result_table.pack(fill=tk.BOTH, expand=True)

        log_frame = ttk.LabelFrame(self, text="运行日志")
        log_frame.pack(fill=tk.BOTH, padx=8, pady=(2, 8))
        self.log_text = tk.Text(
            log_frame,
            height=11,
            wrap=tk.NONE,
            state=tk.DISABLED,
            font=("Consolas", 10),
        )
        log_scroll = ttk.Scrollbar(
            log_frame,
            orient=tk.VERTICAL,
            command=self.log_text.yview,
        )
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    def _refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if "COM9" in ports:
            self.port_combo.set("COM9")
        elif ports and self.port_combo.get() not in ports:
            self.port_combo.set(ports[0])

    def _toggle_connection(self):
        if self.link.connected:
            self._disconnect()
            return

        port = self.port_combo.get()
        if not port:
            messagebox.showerror("连接失败", "没有选择串口")
            return
        try:
            self.link.open(port)
        except (OSError, serial.SerialException) as error:
            messagebox.showerror("连接失败", str(error))
            return

        self.connected_at = time.monotonic()
        self.connection_var.set("%s @ %d" % (port, BAUD_RATE))
        self.state_var.set("ESP启动中，约2秒")
        self.connect_button.configure(text="断开")
        self._append_log("已连接%s，等待PC端ESP启动" % port)
        self.after(
            int(BRIDGE_BOOT_SECONDS * 1000) + 200,
            self._begin_startup_status_probe,
        )

    def _disconnect(self):
        self.autotune_pending = False
        self.autotune_phase = None
        self.champion_load_pending = False
        self.startup_status_pending = False
        self._close_session()
        self.link.close()
        self.connection_var.set("未连接")
        self.state_var.set("连接已关闭")
        self.connect_button.configure(text="连接")

    def _send(self, command, quiet=False):
        try:
            self.link.send(command)
            if not quiet:
                self._append_log("TX > " + command)
        except (RuntimeError, OSError, serial.SerialException) as error:
            if not quiet:
                messagebox.showerror("发送失败", str(error))

    def _start_autotune(self):
        if not self.link.connected:
            messagebox.showerror("无法启动", "请先连接COM端口")
            return
        if not self.confirm_var.get():
            messagebox.showwarning(
                "需要安全确认",
                "请先勾选A4居中、机械中位和线缆安全确认。",
            )
            return

        remaining = (
            BRIDGE_BOOT_SECONDS
            - (time.monotonic() - self.connected_at)
        )
        self.plot_samples.clear()
        self.tune_result_counts = {
            "PID": 0,
            "FOLLOW": 0,
            "VERIFY": 0,
        }
        self.champion_load_pending = False
        self.startup_status_pending = False
        for item in self.result_table.get_children():
            self.result_table.delete(item)
        self._start_session()
        self.state_var.set("自动调参准备中")
        self.autotune_pending = True

        if remaining > 0:
            self.after(
                int(remaining * 1000) + 100,
                self._begin_autotune_handshake,
            )
        else:
            self._begin_autotune_handshake()

    def _begin_autotune_handshake(self):
        if not self.autotune_pending or not self.link.connected:
            return
        # Phase 1: test ESP-NOW link with RADIOPING (bypasses K230)
        self.autotune_phase = "RADIOPING"
        self.autotune_attempts = 0
        self.autotune_generation += 1
        self._service_autotune_handshake(
            self.autotune_generation
        )

    def _advance_handshake_to_ping(self):
        """ESP-NOW link confirmed; now test K230 UART with PING."""
        self.autotune_phase = "PING"
        self.autotune_attempts = 0
        self.autotune_generation += 1
        self._service_autotune_handshake(
            self.autotune_generation
        )

    def _service_autotune_handshake(self, generation=None):
        if (
            generation is not None
            and generation != self.autotune_generation
        ):
            return
        if (
            not self.autotune_pending
            or not self.link.connected
            or self.autotune_phase not in (
                "RADIOPING", "PING", "AUTOTUNE",
            )
        ):
            return

        radio_retries = 3  # fewer retries for ESP-NOW test
        max_attempts = (
            radio_retries
            if self.autotune_phase == "RADIOPING"
            else AUTOTUNE_HANDSHAKE_RETRIES
        )
        if self.autotune_attempts >= max_attempts:
            phase = self.autotune_phase
            self.autotune_pending = False
            self.autotune_phase = None
            if phase == "RADIOPING":
                self.state_var.set(
                    "ESP-NOW无线链路不通：请检查两块ESP8266供电和MAC配对"
                )
            else:
                self.state_var.set(
                    "K230无回复：请查看本次runtime.log和CanMV终端"
                )
            self._append_log(
                "HANDSHAKE_TIMEOUT,phase=%s,attempts=%d"
                % (phase, self.autotune_attempts)
            )
            self._close_session()
            return

        command = self.autotune_phase
        self.autotune_attempts += 1
        self._send(command, quiet=True)
        self._append_log(
            "TX > %s,attempt=%d"
            % (command, self.autotune_attempts)
        )
        if command == "RADIOPING":
            self.state_var.set("正在检测ESP-NOW无线链路…")
        elif command == "PING":
            self.state_var.set("正在确认K230在线…")
        else:
            self.state_var.set("K230在线，正在请求开始自动调参…")
        self.after(
            AUTOTUNE_HANDSHAKE_INTERVAL_MS,
            lambda current_generation=self.autotune_generation:
            self._service_autotune_handshake(current_generation),
        )

    def _stop(self):
        self.autotune_pending = False
        self.autotune_phase = None
        self._send("STOP")
        self.state_var.set("已发送急停")

    def _start_session(self):
        self._close_session()
        self.session_areas = []
        self.rect_counter = 0
        self.last_rect_time_ms = None
        self.last_rect_area = 0
        self.dropped_rect_samples = 0
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_folder = (
            Path(__file__).resolve().parent / "logs" / stamp
        )
        self.session_folder.mkdir(parents=True, exist_ok=True)
        self.csv_file = (
            self.session_folder / "telemetry.csv"
        ).open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "time_ms",
                "valid",
                "cx",
                "cy",
                "error_x",
                "error_y",
                "area",
                "command_x_rpm",
                "command_y_rpm",
                "frame_ms",
            ]
        )
        self.runtime_file = (
            self.session_folder / "runtime.log"
        ).open("w", encoding="utf-8")
        self._append_log("日志目录：" + str(self.session_folder))

    def _close_session(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        if self.runtime_file is not None:
            self.runtime_file.flush()
            self.runtime_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.runtime_file = None

    def _save_result(self, values):
        if self.session_folder is None:
            self._start_session()
        path = self.session_folder / "best_params.json"
        result = dict(values)
        result["PROFILE"] = CONTROL_PROFILE
        result["SCORE_PROFILE"] = PID_SCORE_PROFILE
        result["MEDIAN_AREA"] = self._session_median_area()
        with path.open("w", encoding="utf-8") as file:
            json.dump(result, file, ensure_ascii=False, indent=2)
        self._append_log("最佳参数已保存：" + str(path))

    def _accept_rect_sample(
        self,
        time_ms,
        valid,
        cx,
        cy,
        area,
        frame_ms,
    ):
        if valid not in (0, 1, 2):
            return False
        if (
            self.last_rect_time_ms is not None
            and time_ms + 5000 < self.last_rect_time_ms
        ):
            # K230 ticks restarted; do not compare area with the old run.
            self.last_rect_area = 0
        if valid in (1, 2):
            if (
                area < TELEMETRY_MIN_VALID_AREA
                or cx < 0
                or cx >= IMAGE_WIDTH
                or cy < 0
                or cy >= IMAGE_HEIGHT
            ):
                self.dropped_rect_samples += 1
                return False
            if self.last_rect_area > 0:
                area_ratio = area / float(self.last_rect_area)
                if (
                    area_ratio < TELEMETRY_AREA_RATIO_MIN
                    or area_ratio > TELEMETRY_AREA_RATIO_MAX
                ):
                    self.dropped_rect_samples += 1
                    return False
            self.last_rect_area = area
        self.last_rect_time_ms = time_ms
        return True

    def _parse_rect(self, line):
        parts = line.split(",")
        if len(parts) < 11:
            return
        try:
            time_ms = int(parts[1])
            valid = int(parts[2])
            cx = int(parts[3])
            cy = int(parts[4])
            error_x = int(parts[5])
            error_y = int(parts[6])
            area = int(parts[7]) if len(parts) > 7 else 0
            command_x = float(parts[8]) if len(parts) > 8 else 0.0
            command_y = float(parts[9]) if len(parts) > 9 else 0.0
            frame_ms = int(parts[10]) if len(parts) > 10 else 0
        except ValueError:
            return

        if not self._accept_rect_sample(
            time_ms,
            valid,
            cx,
            cy,
            area,
            frame_ms,
        ):
            if (
                self.dropped_rect_samples > 0
                and self.dropped_rect_samples % 20 == 0
            ):
                self._append_log(
                    "Telemetry filtered: %d bad RECT samples"
                    % self.dropped_rect_samples
                )
            return

        if valid == 1:
            self.target_var.set("目标：已锁定")
        elif valid == 2:
            self.target_var.set("目标：短时保持")
        else:
            self.target_var.set("目标：未检测到")
        self.error_var.set(
            "误差：X %d px / Y %d px" % (error_x, error_y)
        )
        self.command_var.set(
            "命令：X %.1f / Y %.1f RPM" % (command_x, command_y)
        )
        self.plot_samples.append((time_ms, error_x, error_y, valid))
        if valid == 1 and area > 0:
            self.session_areas.append(area)
            if len(self.session_areas) > 5000:
                del self.session_areas[:1000]
        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [
                    time_ms,
                    valid,
                    cx,
                    cy,
                    error_x,
                    error_y,
                    area,
                    command_x,
                    command_y,
                    frame_ms,
                ]
            )
        self.rect_counter += 1
        if self.csv_file is not None and self.rect_counter % 10 == 0:
            self.csv_file.flush()

    def _parse_tune_result(self, line):
        parts = line.split(",")
        if len(parts) < 13 or parts[2] != "RESULT":
            return
        try:
            axis = parts[1]
            values = {}
            for index in range(3, len(parts) - 1, 2):
                values[parts[index]] = parts[index + 1]
            mode = values.get("MODE", "--")
            kp = float(values["KP"])
            ki = float(values["KI"])
            kd = float(values["KD"])
            score = float(values["SCORE"])
        except (ValueError, IndexError, KeyError):
            return
        if mode in self.tune_result_counts:
            self.tune_result_counts[mode] += 1
            self.state_var.set(
                "%s轴%s候选已完成 %d 组"
                % (
                    axis,
                    mode,
                    self.tune_result_counts[mode],
                )
            )
        self.result_table.insert(
            "",
            tk.END,
            values=(
                axis,
                mode,
                "%.3f" % kp,
                "%.3f" % ki,
                "%.3f" % kd,
                "%.3f" % score,
            ),
        )

    def _parse_complete(self, line):
        parts = line.split(",")
        values = {}
        try:
            for index in range(1, len(parts) - 1, 2):
                values[parts[index]] = float(parts[index + 1])
        except (ValueError, IndexError):
            return
        self.params_var.set(
            "X: %.3f/%.3f/%.3f    Y: %.3f/%.3f/%.3f"
            % (
                values.get("XKP", 0.0),
                values.get("XKI", 0.0),
                values.get("XKD", 0.0),
                values.get("YKP", 0.0),
                values.get("YKI", 0.0),
                values.get("YKD", 0.0),
            )
        )
        self.state_var.set("调参完成，正在跟踪")
        self._save_result(values)
        self._sort_result_table()
        self._consider_champion(values)

    def _sort_result_table(self):
        rows = list(self.result_table.get_children())
        rows.sort(
            key=lambda item: (
                0
                if self.result_table.set(item, "mode") == "PID"
                else 1,
                float(self.result_table.set(item, "score")),
            )
        )
        for index, item in enumerate(rows):
            self.result_table.move(item, "", index)

    def _handle_line(self, line):
        if line.startswith("RECT,"):
            self._parse_rect(line)
            return

        self._append_log(line)
        if line.startswith("TUNE,"):
            self.autotune_pending = False
            self.autotune_phase = None
            self.state_var.set(line)
            self._parse_tune_result(line)
        elif line.startswith("AUTOTUNE_START"):
            self.autotune_pending = False
            self.autotune_phase = None
            self.state_var.set("自动调参进行中")
        elif line.startswith("OK,AUTOTUNE_WAIT_TARGET"):
            self.autotune_pending = False
            self.autotune_phase = None
            self.state_var.set("K230已确认，等待稳定识别矩形")
        elif line.startswith("AUTOTUNE_WAIT_TARGET"):
            self.autotune_pending = False
            self.autotune_phase = None
            self.state_var.set("等待稳定识别A4")
        elif line.startswith("ERR,AUTOTUNE_ALREADY_RUNNING"):
            self.autotune_pending = False
            self.autotune_phase = None
            self.state_var.set("K230已经在自动调参")
        elif line.startswith("OK,TRACK_ON"):
            self.state_var.set("两轴电机已使能，正在视觉跟踪")
        elif line.startswith("OK,TRACK_OFF"):
            self.state_var.set("视觉跟踪已停止")
        elif line.startswith("ERR,TRACK_NEEDS_AUTOTUNE"):
            if self.champion:
                self.state_var.set("K230参数尚未加载，正在加载冠军参数")
                self._load_champion_to_k230(quiet=True)
            else:
                self.state_var.set("K230中没有有效参数，请先完成自动调参")
        elif line.startswith("OK,PARAMS_LOADED"):
            self.champion_load_pending = False
            self.state_var.set("冠军参数已加载，可直接开始跟踪")
        elif line.startswith("ERR,INVALID_PARAMS"):
            self.champion_load_pending = False
            self.state_var.set("冠军参数被K230拒绝，请重新自动调参")
        elif line.startswith("ERR,PARAMS_BUSY"):
            self.champion_load_pending = False
            self.state_var.set("K230正在调参，已忽略冠军参数加载")
        elif line.startswith("AUTOTUNE_COMPLETE"):
            self.autotune_pending = False
            self.autotune_phase = None
            self._parse_complete(line)
        elif (
            line.startswith("AUTOTUNE_ABORT")
            or line.startswith("AUTOTUNE_ERROR")
            or line.startswith("TRACK_ABORT")
            or line.startswith("FATAL")
        ):
            self.autotune_pending = False
            self.autotune_phase = None
            if "user STOP" in line:
                reason = "收到手动急停"
            elif "limit" in line.lower():
                reason = "超过临时行程限制"
            elif (
                line.startswith("AUTOTUNE_ERROR")
                or line.startswith("FATAL")
            ):
                reason = "K230程序异常"
            else:
                reason = "跟踪控制中止"
            self.state_var.set(reason + "：" + line)
            self._close_session()
        elif line.startswith("STATUS,"):
            was_startup_probe = self.startup_status_pending
            self.startup_status_pending = False
            parts = line.split(",")
            k230_profile = parts[2] if len(parts) > 2 else ""
            if k230_profile != CONTROL_PROFILE:
                self.state_var.set(
                    "控制版本不匹配：K230=%s，上位机=%s"
                    % (k230_profile or "未知", CONTROL_PROFILE)
                )
                self._append_log("拒绝自动加载旧版本冠军参数")
            elif was_startup_probe and self.champion:
                self._load_champion_to_k230(quiet=True)
            else:
                self.state_var.set(line)
        elif line.startswith("VISION_ERROR,"):
            self.state_var.set("K230视觉检测异常，详情已写入runtime.log")
        elif line == "PONG":
            if (
                self.autotune_pending
                and self.autotune_phase == "PING"
            ):
                self.autotune_phase = "AUTOTUNE"
                self.autotune_attempts = 0
                self.autotune_generation += 1
                self._service_autotune_handshake(
                    self.autotune_generation
                )
            else:
                self.state_var.set("链路正常")
        elif line.startswith("BRIDGE_RADIO_TX_FAILED"):
            if (
                self.autotune_pending
                and self.autotune_phase == "RADIOPING"
            ):
                self.state_var.set(
                    "ESP-NOW发送失败（第%d次），正在重试…"
                    % self.autotune_attempts
                )
            elif self.autotune_pending:
                self.state_var.set("无线单包发送失败，正在自动重试…")
            else:
                self.state_var.set(
                    "ESP-NOW发送失败：请检查K230端ESP8266"
                )
        elif line.startswith("BRIDGE_RADIO_TX_OK"):
            if self.autotune_pending:
                self.state_var.set(
                    "PC端ESP已发送%s，等待K230回复…"
                    % (self.autotune_phase or "请求")
                )
        elif line.startswith("BRIDGE_RADIO_PONG"):
            if (
                self.autotune_pending
                and self.autotune_phase == "RADIOPING"
            ):
                self.state_var.set(
                    "ESP-NOW无线链路正常，正在确认K230在线…"
                )
                self._advance_handshake_to_ping()
            else:
                self.state_var.set("两块ESP8266无线链路正常")
        elif line.startswith("SERIAL_ERROR,"):
            self.state_var.set("串口错误")

    def _append_log(self, text):
        timestamp = datetime.now().strftime("%H:%M:%S")
        if self.runtime_file is not None:
            try:
                self.runtime_file.write(
                    "[%s] %s\n" % (timestamp, text)
                )
                self.runtime_file.flush()
            except OSError:
                pass
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, "[%s] %s\n" % (timestamp, text))
        self.log_text.see(tk.END)
        # 防止长时间运行后Text控件无限增长。
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.log_text.delete("1.0", "300.0")
        self.log_text.configure(state=tk.DISABLED)

    def _draw_plot(self):
        canvas = self.canvas
        canvas.delete("all")
        width = max(100, canvas.winfo_width())
        height = max(100, canvas.winfo_height())
        margin = 34

        canvas.create_line(
            margin,
            height / 2,
            width - 8,
            height / 2,
            fill="#58616a",
        )
        canvas.create_text(
            8, 10, anchor=tk.NW, fill="#ff6767", text="X误差"
        )
        canvas.create_text(
            70, 10, anchor=tk.NW, fill="#55aaff", text="Y误差"
        )

        if len(self.plot_samples) < 2:
            canvas.create_text(
                width / 2,
                height / 2,
                fill="#9aa4ad",
                text="等待RECT遥测数据",
            )
            return

        max_error = max(
            20,
            max(
                max(abs(sample[1]), abs(sample[2]))
                for sample in self.plot_samples
            ),
        )
        samples = list(self.plot_samples)

        def points_for(index):
            points = []
            for position, sample in enumerate(samples):
                x = margin + position * (
                    width - margin - 10
                ) / (len(samples) - 1)
                value = sample[index]
                y = height / 2 - value * (
                    height / 2 - 28
                ) / max_error
                points.extend((x, y))
            return points

        canvas.create_line(
            *points_for(1), fill="#ff6767", width=2, smooth=False
        )
        canvas.create_line(
            *points_for(2), fill="#55aaff", width=2, smooth=False
        )
        canvas.create_text(
            4,
            26,
            anchor=tk.NW,
            fill="#9aa4ad",
            text="+%d" % max_error,
        )
        canvas.create_text(
            4,
            height - 18,
            anchor=tk.NW,
            fill="#9aa4ad",
            text="-%d" % max_error,
        )

    def _poll(self):
        processed = 0
        while processed < 300:
            try:
                line = self.receive_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)
            processed += 1
        self._draw_plot()
        self.after(POLL_MS, self._poll)

    def _on_close(self):
        if self.link.connected:
            try:
                self.link.send("STOP")
                time.sleep(0.1)
            except (RuntimeError, OSError, serial.SerialException):
                pass
        self._close_session()
        self.link.close()
        self.destroy()


if __name__ == "__main__":
    GimbalApp().mainloop()
