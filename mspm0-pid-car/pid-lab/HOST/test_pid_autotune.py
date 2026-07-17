"""不接实车即可运行的上位机协议与算法回归测试。"""

from __future__ import annotations

import queue
import io
import tempfile
import threading
import unittest
from pathlib import Path
from contextlib import redirect_stderr
from unittest import mock

import pid_autotune_gui as gui
import pid_lab_cli as cli
import ground_straight_test as ground


class FakeLink:
    def __init__(self, responder=None) -> None:
        self.data_queue: queue.Queue[str] = queue.Queue()
        self.display_queue: queue.Queue[str] = queue.Queue()
        self.commands: list[str] = []
        self.responder = responder
        self.connected = True

    def send(self, command: str) -> None:
        self.commands.append(command)
        if self.responder is not None:
            for line in self.responder(command, self.commands.count(command)):
                self.data_queue.put(line)

    def drain_data(self) -> None:
        while True:
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                return

    def set_trace_path(self, _path: Path | None) -> None:
        return


def make_tuner(link: FakeLink) -> gui.AutoTuner:
    return gui.AutoTuner(link, queue.Queue(), lambda: None)  # type: ignore[arg-type]


class ProtocolTests(unittest.TestCase):
    def test_parameter_report_parser_and_stable_display(self) -> None:
        values = gui.parse_parameter_report(
            "PARAM,x1000,RKP,500,LKP,350,LINEKP,8250,BAD,nope"
        )
        self.assertEqual(
            values, {"RKP": 500, "LKP": 350, "LINEKP": 8250}
        )
        self.assertEqual(
            gui.format_parameters(values),
            "LKP=350  RKP=500  LINEKP=8250",
        )

    def test_guard8_turn_event_parser_and_center_validity(self) -> None:
        self.assertEqual(
            gui.parse_turn_event("TURN CAPTURE,1,815,112,1,-36"),
            {
                "event": "capture",
                "corner": 1,
                "yaw_x10": 815,
                "travel_mm": 112,
                "mask": 1,
                "error": -36,
            },
        )
        self.assertEqual(
            gui.parse_turn_event("TURN CENTER,1,902,136,24,0"),
            {
                "event": "center",
                "corner": 1,
                "yaw_x10": 902,
                "travel_mm": 136,
                "mask": 24,
                "error": 0,
            },
        )
        self.assertEqual(
            gui.parse_turn_event("TURN LEARN,1,901,102"),
            {
                "event": "learn",
                "corner": 1,
                "turn_angle_x10": 901,
                "turn_distance_mm": 102,
            },
        )
        self.assertEqual(
            gui.parse_turn_event("TURN SEARCH,1,902,142,0,0"),
            {
                "event": "search",
                "corner": 1,
                "yaw_x10": 902,
                "travel_mm": 142,
                "mask": 0,
                "error": 0,
            },
        )
        self.assertFalse(gui.is_successful_gimbal_center_event(
            gui.parse_turn_event("TURN CAPTURE,1,902,136,24,0")
        ))
        self.assertFalse(gui.is_successful_gimbal_center_event(
            gui.parse_turn_event("TURN CENTER,1,902,136,0,0")
        ))
        self.assertFalse(gui.is_successful_gimbal_center_event(
            gui.parse_turn_event("TURN CENTER,1,902,136,24,7")
        ))
        self.assertTrue(gui.is_successful_gimbal_center_event(
            gui.parse_turn_event("TURN CENTER,1,902,136,24,0")
        ))
        # Firmware completion accepts a center probe at |error| <= 6.
        # Parameter switching remains stricter at |error| <= 4.
        self.assertTrue(gui.is_successful_gimbal_center_event(
            gui.parse_turn_event("TURN CENTER,1,902,136,48,6")
        ))
        self.assertIsNone(gui.parse_turn_event("TURN CAPTURE,broken"))

    def test_set_retries_after_lost_ack(self) -> None:
        def respond(command: str, attempt: int) -> list[str]:
            if command.startswith("SET ") and attempt == 2:
                _, name, value, token = command.split()
                return [f"SA,{token},{name},{value}"]
            return []

        link = FakeLink(respond)
        tuner = make_tuner(link)
        with mock.patch.object(gui, "COMMAND_ACK_TIMEOUT", 0.01):
            tuner._send_set("LKP", 150)
        self.assertEqual(len(link.commands), 2)
        self.assertEqual(link.commands[0], link.commands[1])

    def test_profile_selection_requires_flash_v3_readback(self) -> None:
        def respond(command: str, _attempt: int) -> list[str]:
            if command == "PROFILE GIMBAL":
                return ["OK PROFILE GIMBAL FLASH 3"]
            if command == "PARAM":
                return [
                    "PARAM,x1000,PROFILE,1,FLASHVER,3,LKP,350,LINEKP,6750"
                ]
            return []

        tuner = make_tuner(FakeLink(respond))
        values = tuner._select_profile("gimbal")
        self.assertEqual(values["PROFILE"], 1)
        self.assertEqual(values["FLASHVER"], 3)

    def test_tune_result_is_recovered_with_tuneget(self) -> None:
        state: dict[str, str] = {}

        def respond(command: str, attempt: int) -> list[str]:
            if command.startswith("TUNEOPEN "):
                _, wheel, pwm, _, token = command.split()
                state.update(wheel=wheel, pwm=pwm, token=token)
                # 第一个启动 ACK 丢失；相同 token 重发后固件只补 ACK。
                return [f"TA,{token},O,{wheel}"] if attempt == 2 else []
            if command.startswith("TUNEGET "):
                return [
                    f"TO,{state['token']},{state['wheel']},{state['pwm']},"
                    "310,40,340,0"
                ]
            return []

        link = FakeLink(respond)
        tuner = make_tuner(link)
        with mock.patch.object(gui, "COMMAND_ACK_TIMEOUT", 0.01):
            result = tuner._run_tune_transaction(
                "TUNEOPEN L 100 1 42", 42, "L", "O", "TO,42,L,", 1
            )
        self.assertEqual(result, "TO,42,L,100,310,40,340,0")
        self.assertEqual(link.commands.count("TUNEOPEN L 100 1 42"), 2)
        self.assertIn("TUNEGET 42", link.commands)

    def test_protocol_handshake_checks_hardware_scale(self) -> None:
        def respond(command: str, _attempt: int) -> list[str]:
            if command.startswith("HELLO "):
                token = command.split()[1]
                return [
                    f"HELLO,{token},2,STBY,HIGH,ENCODER_MM_X1000,580,"
                    "SAMPLE_MS,10"
                ]
            return []

        tuner = make_tuner(FakeLink(respond))
        tuner._verify_connection()
        self.assertEqual(tuner.protocol_info["version"], 2)
        self.assertEqual(tuner.protocol_info["encoder_sample_ms"], 10)

    def test_runtime_snapshot_and_cancelled_rollback(self) -> None:
        def respond(command: str, _attempt: int) -> list[str]:
            if command == "PARAM":
                return [
                    "PARAM,x1000,LKP,350,LKI,200,RKP,500,RKI,100,"
                    "LFF,461,RFF,434,LMIN,31,RMIN,27,SYNC,1000,"
                    "BIAS,50,GSTART,10,LINEKP,6000,LINEKD,2500,"
                    "TURNDIST,95"
                ]
            if command == "STOP":
                return ["OK STOP"]
            if command.startswith("SET "):
                _, name, value, token = command.split()
                return [f"SA,{token},{name},{value}"]
            return []

        link = FakeLink(respond)
        tuner = make_tuner(link)
        snapshot = tuner._read_parameters()
        self.assertEqual(snapshot["LINEKP"], 6000)
        self.assertEqual(snapshot["LINEKD"], 2500)

        tuner.cancel_event.set()
        tuner._restore_runtime_parameters(
            snapshot, gui.LINE_RUNTIME_PARAMETERS
        )
        self.assertTrue(tuner.cancel_event.is_set())
        self.assertTrue(any(
            command.startswith("SET LINEKP 6000 ")
            for command in link.commands
        ))
        self.assertTrue(any(
            command.startswith("SET LINEKD 2500 ")
            for command in link.commands
        ))
        self.assertTrue(any(
            command.startswith("SET TURNDIST 95 ")
            for command in link.commands
        ))
        self.assertFalse(any(command.startswith("SAVE")
                             for command in link.commands))

    def test_square_rollback_restores_ram_without_save(self) -> None:
        class SquareTuner:
            def __init__(self) -> None:
                self.restored = None

            def _restore_runtime_parameters(self, snapshot, names) -> None:
                self.restored = (snapshot, names)

        app = gui.App.__new__(gui.App)
        app.square_transaction_lock = threading.RLock()
        app.square_parameter_snapshot = {"TURNFAST": 185}
        app.tuner = SquareTuner()
        message = app._rollback_square_parameters()
        self.assertIn("host SAVE not confirmed", message)
        self.assertEqual(
            app.tuner.restored,
            ({"TURNFAST": 185}, gui.TURN_RUNTIME_PARAMETERS),
        )
        self.assertIsNone(app.square_parameter_snapshot)

    def test_square_commits_only_in_success_finalizer(self) -> None:
        link = FakeLink()
        link.data_queue.put("SQUARE DONE,4")

        class SquareTuner:
            def __init__(self) -> None:
                self.cancel_event = threading.Event()
                self.saved = False

            def _save_parameters(self) -> None:
                self.assert_queue_empty()
                self.saved = True

            def assert_queue_empty(self) -> None:
                if not link.data_queue.empty():
                    raise AssertionError("stale SQUARE DONE was not drained")

        app = gui.App.__new__(gui.App)
        app.square_transaction_lock = threading.RLock()
        app.square_parameter_snapshot = {"TURNFAST": 185}
        app.tuner = SquareTuner()
        app.link = link
        app._finalize_square_success("SQUARE DONE,4")
        self.assertTrue(app.tuner.saved)
        self.assertIsNone(app.square_parameter_snapshot)
        self.assertEqual(
            app.link.display_queue.get_nowait(),
            "HOST SQUARE COMMITTED,SQUARE DONE,4",
        )


class AlgorithmTests(unittest.TestCase):
    def test_firmware_guard6_separates_old_edge_from_outgoing_capture(
        self,
    ) -> None:
        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "#define LAB_TURN_LINE_CAPTURE_MIN_YAW_X10 500", source
        )
        self.assertIn(
            "#define LAB_GIMBAL_TURN_OUTGOING_MIN_YAW_X10  750", source
        )
        self.assertIn("g_gimbalTurnDepartingOuterSeen = 1U", source)
        self.assertIn("g_gimbalTurnGapSeen = 1U", source)
        self.assertIn("SQUARE_STATE_CAPTURE_BRAKE", source)
        self.assertIn("SQUARE_STATE_CAPTURE_ALIGN", source)
        self.assertIn('SERIAL_SendString("TURN CAPTURE,")', source)
        self.assertIn('SERIAL_SendString("TURN CENTER,")', source)
        self.assertIn("ENCODER_ResetSpeedFeedback()", source)
        self.assertIn("g_turnCapturedByLine = 1U", source)
        self.assertIn("g_turnCapturedByLine = lineCaptureReady", source)
        self.assertIn("(g_activeProfile != LAB_PROFILE_GIMBAL)", source)

    def test_guard9_turn_gate_requires_signed_ordered_capture(self) -> None:
        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        for required in (
            "LAB_GIMBAL_TURN_OUTGOING_MIN_YAW_X10  750",
            "LAB_GIMBAL_TURN_OUTGOING_MAX_YAW_X10 1100",
            "LAB_GIMBAL_TURN_RIGHT_DEPART_MASK     0x60U",
            "LAB_GIMBAL_TURN_LEFT_ENTRY_MASK       0x07U",
            "LAB_GIMBAL_TURN_LEFT_ENTRY_MAX_ERROR   -6",
            "LAB_GIMBAL_TURN_GAP_CONFIRM              2U",
            "LAB_GIMBAL_TURN_TRAVEL_MIN_NUM            3",
            "LAB_GIMBAL_TURN_TRAVEL_MAX_NUM            9",
            'square_abort("GIMBAL IMU LOST")',
            'square_abort("GIMBAL IMU UNRELIABLE")',
            'square_abort("GIMBAL CAPTURE TRAVEL LIMIT")',
            'square_abort("GIMBAL CAPTURE LINE LOST")',
            'square_abort("GIMBAL CAPTURE ALIGN LINE LOST")',
        ):
            self.assertIn(required, source)
        yaw_start = source.index("yaw = IMU_GetRelativeYawX10();")
        yaw_read = source[
            yaw_start:
            source.index("elapsed = nowMs - g_squareStateStartMs;", yaw_start)
        ]
        self.assertNotIn("if (yaw < 0)", yaw_read)

        def capture_result(
            samples: list[tuple[bool, int, int, int]],
            turn_distance: int = 98,
        ) -> str:
            departing = False
            gap_count = 0
            gap_seen = False
            outgoing_count = 0
            for imu_ready, yaw_x10, travel_mm, mask in samples:
                if not imu_ready:
                    return "ABORT"
                if yaw_x10 > 1150 or travel_mm * 5 > turn_distance * 9:
                    return "ABORT"
                error, valid = gui.line_error_from_mask(mask)
                if (
                    valid and mask & 0x60 and error >= 10 and
                    yaw_x10 >= 500
                ):
                    departing = True
                if departing and not valid and yaw_x10 >= 650:
                    gap_count += 1
                    gap_seen = gap_count >= 2
                elif not gap_seen:
                    gap_count = 0
                candidate = (
                    departing and gap_seen and valid and mask & 0x07 and
                    error <= -6 and 750 <= yaw_x10 <= 1100 and
                    travel_mm * 4 >= turn_distance * 3 and
                    travel_mm * 5 <= turn_distance * 9
                )
                outgoing_count = outgoing_count + 1 if candidate else 0
                if outgoing_count >= 3:
                    return "BRAKE"
            return "SEARCH"

        valid_history = [
            (True, 560, 70, 64),
            (True, 700, 90, 0),
            (True, 720, 92, 0),
            (True, 815, 108, 1),
            (True, 863, 110, 3),
            (True, 887, 112, 2),
        ]
        self.assertEqual(capture_result(valid_history), "BRAKE")
        self.assertEqual(capture_result([
            (True, 560, 70, 64),
            (True, 700, 90, 0),
            (True, 720, 92, 0),
            (True, 902, 142, 0),
        ]), "SEARCH")
        self.assertEqual(capture_result([
            (True, 560, 70, 64),
            (True, 700, 90, 0),
            (True, 720, 92, 0),
            (True, 850, 110, 24),
            (True, 870, 112, 24),
            (True, 890, 114, 24),
        ]), "SEARCH")
        self.assertEqual(capture_result([
            (True, 560, 70, 64),
            (True, 700, 90, 0),
            (True, 720, 92, 0),
            (True, 1600, 170, 64),
        ]), "ABORT")
        self.assertEqual(capture_result([
            (True, 560, 70, 64),
            (False, 600, 80, 0),
        ]), "ABORT")

    def test_gimbal_line_search_starts_from_conservative_bootstrap(self) -> None:
        self.assertEqual(gui.GIMBAL_BOOTSTRAP_LINE_PAIR, (3000, 800))
        self.assertEqual(gui.GIMBAL_LINE_KP_SAFE_RANGE, (2000, 5000))
        self.assertEqual(gui.GIMBAL_LINE_KD_SAFE_RANGE, (500, 2000))

    def test_line_mask_error_matches_firmware_and_masks_sensor_8(self) -> None:
        self.assertEqual(gui.line_error_from_mask(24), (0, True))
        self.assertEqual(gui.line_error_from_mask(48), (6, True))
        self.assertEqual(gui.line_error_from_mask(24 | 0x80), (0, True))
        self.assertEqual(gui.line_error_from_mask(0), (0, False))

    def test_line_search_is_local_and_never_requires_zero_kd(self) -> None:
        self.assertEqual(
            gui.local_line_candidates(6750, 750, gui.LINE_KP_SAFE_RANGE),
            (6000, 6750, 7500),
        )
        kd_values = gui.local_line_candidates(
            2000, 500, gui.LINE_KD_SAFE_RANGE
        )
        self.assertEqual(kd_values, (1500, 2000, 2500))
        self.assertNotIn(0, kd_values)

    def test_line_preflight_rejects_off_center_mask_before_motion(self) -> None:
        def respond(command: str, _attempt: int) -> list[str]:
            if command == "SENSOR":
                return ["SENSOR,OK,48,0,93,4,48"]
            return []

        link = FakeLink(respond)
        tuner = make_tuner(link)
        with self.assertRaisesRegex(RuntimeError, "centered on the black line"):
            tuner._verify_centered_line_sensor()
        self.assertFalse(any(command.startswith("SQUARE")
                             for command in link.commands))

    def test_passive_candidate_sends_no_kick(self) -> None:
        def centered_sample(time_ms: int, corner: int = 0,
                            state: int = 0) -> str:
            return (
                f"LT,{time_ms},340,340,340,340,0,24,1,0,"
                f"{corner},{state},0"
            )

        def respond(command: str, _attempt: int) -> list[str]:
            if command.startswith("SET "):
                _, name, value, token = command.split()
                reply = [f"SA,{token},{name},{value}"]
                if name == "LINEKD":
                    time_ms = 800
                    for corner in range(4):
                        count = 12 if corner == 0 else 2
                        for _ in range(count):
                            reply.append(centered_sample(time_ms, corner))
                            time_ms += 20
                    reply.append(centered_sample(time_ms, 4, state=1))
                return reply
            return []

        link = FakeLink(respond)
        for index in range(gui.LINE_SWITCH_CENTER_SAMPLES):
            link.data_queue.put(centered_sample(600 + index * 20))
        tuner = make_tuner(link)
        with tempfile.TemporaryDirectory() as directory:
            tuner.session_dir = Path(directory)
            score, sample_count = tuner._run_square_line_candidate(
                6000, 2000, "passive", minimum_edge_samples=2
            )
        self.assertLess(score, 0.2)
        self.assertEqual(sample_count, 8)
        self.assertFalse(any(command.startswith("KICK")
                             for command in link.commands))

    def test_low_speed_line_candidate_timeout_allows_full_square(self) -> None:
        self.assertGreaterEqual(gui.line_candidate_timeout_seconds(120), 60.0)
        self.assertGreaterEqual(gui.line_candidate_timeout_seconds(200), 60.0)
        self.assertEqual(gui.line_candidate_timeout_seconds(340), 25.0)

    def test_adaptive_feedforward_fit_brackets_target(self) -> None:
        tuner = make_tuner(FakeLink())

        def fake_transaction(
            _command: str, token: int, wheel: str, _kind: str,
            _prefix: str, _duration: int,
        ) -> str:
            pwm = int(_command.split()[2])
            speed = pwm * 3
            return f"TO,{token},{wheel},{pwm},{speed},40,{speed + 20},0"

        tuner._run_tune_transaction = fake_transaction  # type: ignore[method-assign]
        with tempfile.TemporaryDirectory() as directory:
            tuner.session_dir = Path(directory)
            min_pwm, ff = tuner._identify_feedforward("L")
        self.assertLessEqual(min_pwm, 2)
        self.assertTrue(325 <= ff <= 342)
        self.assertIn("L", tuner.feedforward_diagnostics)

    def test_compact_line_sample_parser(self) -> None:
        sample = gui.PidSample.parse("LT,100,280,280,275,282,-2,24,1,31,0,0,0")
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.mode, "SQUARE")
        self.assertEqual(sample.right_speed, 282)
        self.assertEqual(sample.line_mask, 24)

    def test_line_score_penalizes_snake_and_control_jumps(self) -> None:
        def samples(snake: bool) -> list[gui.PidSample]:
            result: list[gui.PidSample] = []
            for index in range(120):
                if snake:
                    error = 12 if (index // 3) % 2 == 0 else -12
                    turn = 100 if error > 0 else -100
                else:
                    error = (0, 1, 0, -1)[index % 4]
                    turn = error * 2
                result.append(gui.PidSample(
                    time_ms=600 + index * 20,
                    mode="SQUARE",
                    left_target=360 + turn,
                    right_target=360 - turn,
                    left_speed=360 + turn,
                    right_speed=360 - turn,
                    left_pwm=0,
                    right_pwm=0,
                    left_error=0,
                    right_error=0,
                    count_diff=0,
                    line_error=error,
                    line_mask=24,
                    line_valid=1,
                ))
            return result

        stable_score = gui.AutoTuner._score_line(samples(False))
        snake_score = gui.AutoTuner._score_line(samples(True))
        self.assertLess(stable_score, 0.2)
        self.assertGreater(snake_score, stable_score + 2.0)

    def test_line_score_uses_requested_gimbal_speed(self) -> None:
        samples = [
            gui.PidSample(
                time_ms=600 + index * 20,
                mode="SQUARE",
                left_target=120,
                right_target=120,
                left_speed=120,
                right_speed=120,
                left_pwm=0,
                right_pwm=0,
                left_error=0,
                right_error=0,
                count_diff=0,
                line_error=0,
                line_mask=24,
                line_valid=1,
            )
            for index in range(100)
        ]
        low_speed_score = gui.AutoTuner._score_line(samples, 120)
        light_speed_score = gui.AutoTuner._score_line(samples, 340)
        self.assertLess(low_speed_score, light_speed_score)

    def test_guard8_scoring_skips_startup_ramp_with_one_shared_gate(
        self,
    ) -> None:
        def sample(
            time_ms: int,
            target: int = 120,
            line_valid: int = 1,
            square_state: int = 0,
        ) -> gui.PidSample:
            return gui.PidSample(
                time_ms=time_ms,
                mode="SQUARE",
                left_target=target,
                right_target=target,
                left_speed=target,
                right_speed=target,
                left_pwm=0,
                right_pwm=0,
                left_error=0,
                right_error=0,
                count_diff=0,
                line_error=0,
                line_mask=24,
                line_valid=line_valid,
                square_state=square_state,
            )

        self.assertGreaterEqual(gui.GIMBAL_SCORE_MIN_TIME_MS, 1000)
        self.assertFalse(gui.is_gimbal_score_window_eligible(
            sample(999), 120
        ))
        self.assertFalse(gui.is_gimbal_score_window_eligible(
            sample(gui.GIMBAL_SCORE_MIN_TIME_MS - 1), 120
        ))
        self.assertTrue(gui.is_gimbal_score_window_eligible(
            sample(gui.GIMBAL_SCORE_MIN_TIME_MS), 120
        ))
        self.assertTrue(gui.is_gimbal_score_window_eligible(
            sample(gui.GIMBAL_SCORE_MIN_TIME_MS, target=100), 120
        ))
        self.assertTrue(gui.is_gimbal_score_window_eligible(
            sample(gui.GIMBAL_SCORE_MIN_TIME_MS, line_valid=0), 120
        ))
        self.assertFalse(gui.is_gimbal_score_window_eligible(
            sample(gui.GIMBAL_SCORE_MIN_TIME_MS, square_state=2), 120
        ))
        self.assertEqual(
            gui.is_gimbal_scoreable_straight(
                sample(gui.GIMBAL_SCORE_MIN_TIME_MS, line_valid=0), 120
            ),
            gui.is_gimbal_score_window_eligible(
                sample(gui.GIMBAL_SCORE_MIN_TIME_MS, line_valid=0), 120
            ),
        )

        host_source = (
            Path(__file__).resolve().parent / "pid_autotune_gui.py"
        ).read_text(encoding="utf-8")
        worker_start = host_source.index(
            "    def _run_gimbal_continuous_auto("
        )
        worker_end = host_source.index(
            "    def _run_line(", worker_start
        )
        worker = host_source[worker_start:worker_end]
        score_gate = worker.index(
            "scoreable = is_gimbal_score_window_eligible("
        )
        skip_gate = worker.index("if not scoreable:", score_gate)
        self.assertLess(skip_gate, worker.index(
            "baseline_samples.append(sample)"
        ))
        self.assertLess(skip_gate, worker.index(
            "candidate_samples.append(sample)"
        ))
        self.assertLess(skip_gate, worker.index(
            "validation_samples.append(sample)"
        ))

    def test_gimbal_yaw_observer_penalizes_snake_without_changing_gray_score(
        self,
    ) -> None:
        def samples(yaw_at) -> list[gui.PidSample]:
            return [
                gui.PidSample(
                    time_ms=gui.GIMBAL_SCORE_MIN_TIME_MS + index * 20,
                    mode="SQUARE",
                    left_target=120,
                    right_target=120,
                    left_speed=120,
                    right_speed=120,
                    left_pwm=0,
                    right_pwm=0,
                    left_error=0,
                    right_error=0,
                    count_diff=0,
                    line_error=0,
                    line_mask=24,
                    line_valid=1,
                    yaw_x10=yaw_at(index),
                    corner_count=0,
                    square_state=0,
                    turn_travel_mm=0,
                )
                for index in range(40)
            ]

        stable = samples(lambda _index: 0)
        snaking = samples(lambda index: 50 if index % 2 == 0 else -50)
        tuner = make_tuner(FakeLink())

        self.assertEqual(
            tuner._score_line(stable, 120),
            tuner._score_line(snaking, 120),
        )
        self.assertLess(
            tuner._score_gimbal_line(stable, 120),
            tuner._score_gimbal_line(snaking, 120),
        )

    def test_v5_score_rejects_far_tail_even_when_it_recovers(self) -> None:
        def full_lap(far: bool) -> list[gui.PidSample]:
            result: list[gui.PidSample] = []
            for corner in range(4):
                for index in range(50):
                    error = 18 if far and index < 10 else (
                        6 if not far and index < 2 else 0
                    )
                    turn = min(100, abs(error) * 4)
                    if error < 0:
                        turn = -turn
                    result.append(gui.PidSample(
                        time_ms=600 + (corner * 50 + index) * 20,
                        mode="SQUARE",
                        left_target=340 + turn,
                        right_target=340 - turn,
                        left_speed=340 + turn,
                        right_speed=340 - turn,
                        left_pwm=0,
                        right_pwm=0,
                        left_error=0,
                        right_error=0,
                        count_diff=0,
                        line_error=error,
                        line_mask=24,
                        line_valid=1,
                        corner_count=corner,
                    ))
            return result

        near_score = gui.AutoTuner._score_line(full_lap(False))
        far_score = gui.AutoTuner._score_line(full_lap(True))
        self.assertGreater(far_score, near_score + 0.5)


class CliTests(unittest.TestCase):
    def test_gimbal_square_target_guard_stops_repeated_post_corner_split(
        self,
    ) -> None:
        sample = gui.PidSample(
            time_ms=11730,
            mode="SQUARE",
            left_target=20,
            right_target=220,
            left_speed=0,
            right_speed=0,
            left_pwm=0,
            right_pwm=0,
            left_error=0,
            right_error=0,
            count_diff=0,
            corner_count=1,
            square_state=0,
        )
        count = 0
        for _ in range(cli.GIMBAL_SQUARE_TARGET_DELTA_CONFIRM):
            count = cli.update_gimbal_square_target_guard(sample, True, count)
        self.assertEqual(count, cli.GIMBAL_SQUARE_TARGET_DELTA_CONFIRM)
        sample.left_target = 80
        sample.right_target = 160
        self.assertEqual(
            cli.update_gimbal_square_target_guard(sample, True, count), 0
        )
        sample.left_target = 20
        sample.right_target = 220
        self.assertEqual(
            cli.update_gimbal_square_target_guard(sample, False, count), 0
        )

    def test_guard9_valid_line_has_single_gray_controller_owner(self) -> None:
        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        self.assertIn("LAB_LINE_SENSOR_VALID_MASK   0x7FU", source)
        self.assertIn("LAB_GIMBAL_GUARD_VERSION              10", source)
        self.assertIn("uint8_t gimbalLineControl", source)
        self.assertIn(
            "if ((gimbalEnvelopeActive != 0U) && (valid != 0U))", source
        )
        self.assertIn("outputTurn = g_lineTurnMmps;", source)
        self.assertIn("LAB_GIMBAL_LINE_SPEED_FLOOR_MMPS   100", source)
        self.assertIn("LAB_GIMBAL_LEFT_BREAKAWAY_PWM          80", source)
        self.assertIn("LAB_GIMBAL_RIGHT_BREAKAWAY_PWM         50", source)
        self.assertIn("g_gimbalStartupSeedMmps", source)
        self.assertIn("gimbal_service_breakaway", source)
        self.assertIn("gimbal_apply_breakaway_common_gate", source)
        for retired in (
            "g_gimbalRecovery",
            "g_gimbalHeadingTurn",
            "g_gimbalHeadingTarget",
            "g_gimbalStartupSync",
            "RECOVERY SEEK",
            "RECOVERY SETTLE",
            "#if 0",
        ):
            self.assertNotIn(retired, source)

    def test_guard9_capture_align_does_not_stack_startup_or_imu_turn(
        self,
    ) -> None:
        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        selector_start = source.index("uint8_t gimbalEnvelopeActive")
        selector_end = source.index(
            "int16_t gimbalLineLimit", selector_start
        )
        selector = source[selector_start:selector_end]
        self.assertIn("gimbalEnvelopeActive", selector)
        self.assertIn("SQUARE_STATE_CAPTURE_ALIGN", selector)

        output_start = source.index("outputTurn = g_lineTurnMmps;")
        envelope_start = source.index(
            "if (gimbalEnvelopeActive != 0U)", output_start
        )
        envelope_end = source.index("#endif", envelope_start)
        envelope = source[envelope_start:envelope_end]
        self.assertIn(
            "outputTurn = clamp_i16(\n"
            "                outputTurn,",
            envelope,
        )
        self.assertNotIn("auxiliaryTurn", envelope)
        self.assertNotIn("+ g_gimbal", envelope)
        self.assertNotIn("Startup", envelope)
        self.assertNotIn("Heading", envelope)
        self.assertIn(
            "g_leftTarget = clamp_i16((int32_t)baseTarget + outputTurn",
            source,
        )
        self.assertIn(
            "g_rightTarget = clamp_i16((int32_t)baseTarget - outputTurn",
            source,
        )

    def test_gimbal_square_requires_guard_firmware_before_motion(self) -> None:
        self.assertEqual(gui.GIMBAL_GUARD_VERSION, 10)
        with self.assertRaisesRegex(RuntimeError, "guard firmware"):
            cli.require_gimbal_square_guard(
                {"PROFILE": gui.PROFILE_IDS["GIMBAL"], "FLASHVER": 3},
                120,
            )
        with self.assertRaisesRegex(RuntimeError, "guard firmware"):
            cli.require_gimbal_square_guard(
                {
                    "PROFILE": gui.PROFILE_IDS["GIMBAL"],
                    "FLASHVER": 3,
                    "GUARDVER": 5,
                },
                120,
            )
        with self.assertRaisesRegex(RuntimeError, "guard firmware"):
            cli.require_gimbal_square_guard(
                {
                    "PROFILE": gui.PROFILE_IDS["GIMBAL"],
                    "FLASHVER": 3,
                    "GUARDVER": 7,
                },
                120,
            )
        with self.assertRaisesRegex(RuntimeError, "guard firmware"):
            cli.require_gimbal_square_guard(
                {
                    "PROFILE": gui.PROFILE_IDS["GIMBAL"],
                    "FLASHVER": 3,
                    "GUARDVER": 7,
                },
                340,
            )
        self.assertTrue(cli.require_gimbal_square_guard(
            {
                "PROFILE": gui.PROFILE_IDS["GIMBAL"],
                "FLASHVER": 3,
                "GUARDVER": gui.GIMBAL_GUARD_VERSION,
            },
            120,
        ))
        self.assertFalse(cli.require_gimbal_square_guard(
            {"PROFILE": gui.PROFILE_IDS["LIGHT"], "FLASHVER": 3},
            120,
        ))

    def test_gimbal_host_guard_uses_only_sustained_hard_yaw(self) -> None:
        sample = gui.PidSample(
            time_ms=1800,
            mode="SQUARE",
            left_target=140,
            right_target=100,
            left_speed=100,
            right_speed=110,
            left_pwm=100,
            right_pwm=90,
            left_error=40,
            right_error=-10,
            count_diff=-20,
            line_error=23,
            line_mask=96,
            line_valid=1,
            yaw_x10=0,
            corner_count=0,
            square_state=0,
        )
        # Guard9 never turns a valid gray-line error into a host STOP.
        self.assertEqual(
            cli.update_gimbal_hard_yaw_guard(sample, True, 0), 0
        )
        sample.line_error = 36
        sample.line_mask = 64
        self.assertEqual(
            cli.update_gimbal_hard_yaw_guard(sample, True, 0), 0
        )
        sample.yaw_x10 = gui.GIMBAL_HARD_YAW_GUARD_X10
        count = 0
        for _ in range(cli.GIMBAL_HARD_YAW_GUARD_CONFIRM):
            count = cli.update_gimbal_hard_yaw_guard(
                sample, True, count
            )
        self.assertEqual(count, cli.GIMBAL_HARD_YAW_GUARD_CONFIRM)
        sample.square_state = 2
        self.assertEqual(
            cli.update_gimbal_hard_yaw_guard(sample, True, count), 0
        )

    def test_guard4_real_outer_line_was_valid_and_below_hard_yaw(self) -> None:
        # 04:25 GUARD4 实车：yaw 已由 7.1°回落，mask96/error23 仍有效。
        for time_ms, yaw_x10 in (
            (2280, 62), (2300, 61), (2320, 59), (2340, 58), (2400, 54)
        ):
            sample = gui.PidSample(
                time_ms=time_ms,
                mode="SQUARE",
                left_target=155,
                right_target=85,
                left_speed=105,
                right_speed=69,
                left_pwm=135,
                right_pwm=88,
                left_error=50,
                right_error=16,
                count_diff=-43,
                line_error=23,
                line_mask=96,
                line_valid=1,
                yaw_x10=yaw_x10,
                corner_count=0,
                square_state=0,
            )
            self.assertEqual(
                gui.line_error_from_mask(sample.line_mask), (23, True)
            )
            self.assertEqual(
                gui.update_gimbal_hard_yaw_guard(sample, True, 0), 0
            )

    def test_guard5_real_recovery_reaches_exit6_line_yaw_window(self) -> None:
        trace_path = (
            Path(__file__).resolve().parent
            / "logs"
            / "square_cli_20260716_053734"
            / "protocol_trace.log"
        )
        samples: list[gui.PidSample] = []
        for raw_line in trace_path.read_text(encoding="utf-8").splitlines():
            if " RX PID," not in raw_line:
                continue
            sample = gui.PidSample.parse(raw_line.split(" RX ", 1)[1])
            if sample is not None:
                samples.append(sample)

        self.assertEqual(len(samples), 178)
        self.assertTrue(all(sample.line_valid == 1 for sample in samples))
        first_left_motion = next(
            sample.time_ms for sample in samples if sample.left_speed != 0
        )
        first_right_motion = next(
            sample.time_ms for sample in samples if sample.right_speed != 0
        )
        self.assertLessEqual(
            abs(first_left_motion - first_right_motion), 40
        )
        self.assertEqual(max(abs(sample.line_error) for sample in samples), 11)
        self.assertEqual(max(sample.yaw_x10 for sample in samples), 61)
        early_exit_window = [
            sample for sample in samples
            if 1940 <= sample.time_ms <= 2200
        ]
        self.assertGreaterEqual(len(early_exit_window), 10)
        self.assertTrue(all(
            abs(sample.line_error) <= 6
            and abs(sample.yaw_x10) <= 40
            for sample in early_exit_window
        ))
        for sample in samples[-3:]:
            self.assertEqual(sample.line_mask, 48)
            self.assertEqual(sample.line_error, 6)
            self.assertLessEqual(abs(sample.yaw_x10), 40)
        self.assertEqual(samples[-1].yaw_x10, -40)

    def test_guard6_real_turn_sequence_rejects_departing_old_edge(
        self,
    ) -> None:
        trace_path = (
            Path(__file__).resolve().parent
            / "logs"
            / "square_cli_20260716_022840"
            / "protocol_trace.log"
        )
        samples: list[gui.PidSample] = []
        for raw_line in trace_path.read_text(encoding="utf-8").splitlines():
            if " RX PID," not in raw_line:
                continue
            sample = gui.PidSample.parse(raw_line.split(" RX ", 1)[1])
            if sample is not None and sample.square_state in (2, 3):
                samples.append(sample)

        departing_seen = False
        gap_seen = False
        outgoing_count = 0
        capture_yaw: int | None = None
        for sample in samples:
            error, valid = gui.line_error_from_mask(sample.line_mask)
            if valid and error >= 10 and sample.yaw_x10 >= 500:
                departing_seen = True
            if departing_seen and not valid and sample.yaw_x10 >= 650:
                gap_seen = True
            candidate = (
                valid and error <= 6 and sample.yaw_x10 >= 750 and
                (gap_seen or sample.yaw_x10 >= 800)
            )
            outgoing_count = outgoing_count + 1 if candidate else 0
            if outgoing_count >= 3:
                capture_yaw = sample.yaw_x10
                break

        self.assertTrue(departing_seen)
        self.assertTrue(gap_seen)
        self.assertIsNotNone(capture_yaw)
        assert capture_yaw is not None
        self.assertGreaterEqual(capture_yaw, 800)
        self.assertTrue(any(
            559 <= sample.yaw_x10 <= 677 and sample.line_mask == 64
            for sample in samples
        ))
        self.assertTrue(any(
            702 <= sample.yaw_x10 <= 803 and sample.line_mask == 0
            for sample in samples
        ))

    def test_guard6_latest_mask64_at_651_is_not_center_completion(
        self,
    ) -> None:
        trace_path = (
            Path(__file__).resolve().parent
            / "logs"
            / "square_cli_20260716_061232"
            / "protocol_trace.log"
        )
        samples: list[gui.PidSample] = []
        for raw_line in trace_path.read_text(encoding="utf-8").splitlines():
            if " RX PID," not in raw_line:
                continue
            sample = gui.PidSample.parse(raw_line.split(" RX ", 1)[1])
            if sample is not None:
                samples.append(sample)

        old_exit = [
            sample for sample in samples if sample.square_state == 4
        ]
        self.assertEqual(len(old_exit), 40)
        self.assertEqual(old_exit[0].yaw_x10, 651)
        self.assertEqual(old_exit[0].line_mask, 64)
        self.assertEqual(old_exit[0].line_error, 0)
        self.assertTrue(all(
            sample.line_mask == 64 and
            (sample.line_mask & ((1 << 3) | (1 << 4))) == 0
            for sample in old_exit
        ))

    def test_guard8_073722_mask0_at_90deg_cannot_complete_turn(
        self,
    ) -> None:
        trace_path = (
            Path(__file__).resolve().parent
            / "logs"
            / "square_cli_20260716_073722"
            / "protocol_trace.log"
        )
        trace = trace_path.read_text(encoding="utf-8").splitlines()
        rx = [
            line.split(" RX ", 1)[1]
            for line in trace
            if " RX " in line
        ]
        tx = [
            line.split(" TX ", 1)[1]
            for line in trace
            if " TX " in line
        ]
        # Guard7's historical bug stopped at the nominal angle with mask=0.
        self.assertIn("TURN CAPTURE,1,902,142,0,0", rx)
        self.assertFalse(any(line.startswith((
            "TURN CENTER", "TURN LEARN", "TURN DONE", "SQUARE DONE"
        )) for line in rx))
        self.assertIn("SQUARE ERROR,LINE LOST", rx)
        self.assertFalse(any(line.startswith("SAVE") for line in tx))

        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        search_start = source.index("turnSearchTargetReached =")
        search_end = source.index(
            "if (g_squareState == SQUARE_STATE_CAPTURE_BRAKE)",
            search_start,
        )
        search_block = source[search_start:search_end]
        self.assertIn("(turnSearchTargetReached != 0U)", search_block)
        self.assertIn("(lineCaptureReady == 0U)", search_block)
        self.assertIn('SERIAL_SendString("TURN SEARCH,")', search_block)

        capture_send = source.index('SERIAL_SendString("TURN CAPTURE,")')
        capture_gate = source[
            source.rfind("if (", 0, capture_send):capture_send
        ]
        self.assertIn("(lineCaptureReady != 0U)", capture_gate)
        self.assertIn("(turnLineValid != 0U)", capture_gate)
        self.assertIn("(outgoingLineCandidate != 0U)", capture_gate)

    def test_guard7_real_valid_line_was_not_physical_line_loss(
        self,
    ) -> None:
        trace_path = (
            Path(__file__).resolve().parent
            / "logs"
            / "square_cli_20260716_065843"
            / "protocol_trace.log"
        )
        samples: list[gui.PidSample] = []
        for raw_line in trace_path.read_text(encoding="utf-8").splitlines():
            if " RX PID," not in raw_line:
                continue
            sample = gui.PidSample.parse(raw_line.split(" RX ", 1)[1])
            if sample is not None:
                samples.append(sample)

        self.assertGreater(len(samples), 200)
        self.assertTrue(all(sample.line_valid for sample in samples))
        self.assertTrue(any(sample.line_error == 11 for sample in samples))
        self.assertTrue(any(sample.line_error == 6 for sample in samples))
        self.assertTrue(any(sample.line_error == 2 for sample in samples))
        centered_tail = [
            sample for sample in samples[-40:]
            if sample.line_mask == 24 and sample.line_error == 0
        ]
        self.assertGreaterEqual(len(centered_tail), 25)

    def test_compact_line_autotune_keeps_hard_yaw_backstop(self) -> None:
        link = FakeLink()
        tuner = make_tuner(link)
        tuner._configure_gimbal_square_guard(
            {
                "PROFILE": gui.PROFILE_IDS["GIMBAL"],
                "GUARDVER": gui.GIMBAL_GUARD_VERSION,
            },
            120,
        )
        for index in range(gui.GIMBAL_HARD_YAW_GUARD_CONFIRM):
            link.data_queue.put(
                f"LT,{1800 + index * 20},120,120,100,110,"
                f"0,24,1,{gui.GIMBAL_HARD_YAW_GUARD_X10},0,0,0"
            )
        with self.assertRaisesRegex(RuntimeError, "HARD YAW GUARD"):
            tuner._wait_for_centered_line_window(required=5, timeout=0.5)
        self.assertIn("STOP", link.commands)

    def test_restore_v4_stops_saves_and_verifies_turn_baseline(self) -> None:
        tuner = mock.Mock()
        tuner._read_parameters.return_value = {
            "LINEKP": 6750,
            "LINEKD": 2000,
            "TURNDIST": 98,
            "FLASHVER": 3,
            "PROFILE": 1,
        }

        cli.restore_v4_baseline(tuner)

        tuner._verify_connection.assert_called_once_with()
        tuner._stop_reliably.assert_called_once_with()
        tuner._select_profile.assert_called_once_with("LIGHT")
        self.assertEqual(
            tuner._send_set.call_args_list,
            [
                mock.call("LINEKP", 6750),
                mock.call("LINEKD", 2000),
                mock.call("TURNDIST", 98),
            ],
        )
        tuner._save_parameters.assert_called_once_with()
        self.assertEqual(tuner._read_parameters.call_count, 2)

    def test_restore_line_stops_saves_and_verifies_stable_pair(self) -> None:
        tuner = mock.Mock()
        tuner._read_parameters.return_value = {
            "LINEKP": 6750,
            "LINEKD": 2000,
            "FLASHVER": 3,
            "PROFILE": 1,
        }

        cli.restore_stable_line(tuner)

        tuner._verify_connection.assert_called_once_with()
        tuner._stop_reliably.assert_called_once_with()
        tuner._select_profile.assert_called_once_with("LIGHT")
        self.assertEqual(
            tuner._send_set.call_args_list,
            [mock.call("LINEKP", 6750), mock.call("LINEKD", 2000)],
        )
        tuner._save_parameters.assert_called_once_with()
        self.assertEqual(tuner._read_parameters.call_count, 2)

    def test_save_gimbal_stage1_commits_only_gimbal_and_verifies_light(self) -> None:
        light = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "PROFILE": 0,
            "FLASHVER": 3,
        }
        gimbal_before = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "PROFILE": 1,
            "FLASHVER": 3,
        }
        gimbal_after = {
            **gimbal_before,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 10,
        }
        tuner = mock.Mock()
        tuner._select_profile.side_effect = [
            light, gimbal_before, light, gimbal_after,
        ]
        tuner._read_parameters.return_value = gimbal_after

        actual = cli.save_gimbal_stage1(tuner)

        self.assertEqual(actual["SYNC"], 500)
        self.assertEqual(actual["BIAS"], 80)
        self.assertEqual(actual["GSTART"], 10)
        self.assertEqual(
            tuner._select_profile.call_args_list,
            [
                mock.call("LIGHT"), mock.call("GIMBAL"),
                mock.call("LIGHT"), mock.call("GIMBAL"),
            ],
        )
        self.assertEqual(
            tuner._send_set.call_args_list,
            [
                mock.call("SYNC", 500),
                mock.call("BIAS", 80),
                mock.call("GSTART", 10),
            ],
        )
        tuner._save_parameters.assert_called_once_with()

    def test_prepare_gimbal_auto_preserves_learned_gstart(self) -> None:
        current = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 1000,
            "BIAS": 50,
            "GSTART": -17,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        prepared = {**current, "SYNC": 500, "BIAS": 80}
        tuner = mock.Mock()
        tuner._select_profile.return_value = current
        with mock.patch.object(
            cli, "commit_gimbal_wheel_pair", return_value=prepared
        ) as commit:
            actual = cli.prepare_gimbal_auto_profile(tuner)
        self.assertEqual(actual["GSTART"], -17)
        commit.assert_called_once_with(
            tuner,
            {"SYNC": 500, "BIAS": 80, "GSTART": -17},
            "auto preparation",
        )

    def test_prepare_gimbal_auto_skips_matching_flash_write(self) -> None:
        current = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 17,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        tuner = mock.Mock()
        tuner._select_profile.return_value = current
        with mock.patch.object(
            cli, "commit_gimbal_wheel_pair"
        ) as commit:
            actual = cli.prepare_gimbal_auto_profile(tuner)
        self.assertEqual(actual, current)
        commit.assert_not_called()
        tuner._send_set.assert_not_called()
        tuner._save_parameters.assert_not_called()

    def test_prepare_gimbal_auto_initializes_zero_gstart(self) -> None:
        current = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 0,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        tuner = mock.Mock()
        tuner._select_profile.return_value = current
        with mock.patch.object(
            cli, "commit_gimbal_wheel_pair", return_value={
                **current, "GSTART": 10,
            }
        ) as commit:
            actual = cli.prepare_gimbal_auto_profile(tuner)
        self.assertEqual(actual["GSTART"], 10)
        commit.assert_called_once_with(
            tuner,
            {"SYNC": 500, "BIAS": 80, "GSTART": 10},
            "auto preparation",
        )

    def test_prepare_gimbal_auto_rejects_old_guard_before_write(self) -> None:
        current = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "GSTART": 10,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION - 1,
        }
        tuner = mock.Mock()
        tuner._select_profile.return_value = current
        with (
            mock.patch.object(cli, "commit_gimbal_wheel_pair") as commit,
            self.assertRaisesRegex(RuntimeError, "guard firmware"),
        ):
            cli.prepare_gimbal_auto_profile(tuner)
        commit.assert_not_called()
        tuner._send_set.assert_not_called()
        tuner._save_parameters.assert_not_called()

    def test_restore_gimbal_v4_uses_isolated_default_wheel_pair(self) -> None:
        tuner = mock.Mock()
        with mock.patch.object(
            cli, "commit_gimbal_wheel_pair", return_value={"PROFILE": 1}
        ) as commit:
            actual = cli.restore_gimbal_v4(tuner)
        self.assertEqual(actual["PROFILE"], 1)
        commit.assert_called_once_with(
            tuner,
            {"SYNC": 1000, "BIAS": 50, "GSTART": 0},
            "V4 wheel rollback",
        )

    def test_gimbal_auto_uses_one_continuous_worker_without_qualification(
        self,
    ) -> None:
        original = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 10,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        bootstrap = {
            **original,
            **cli.GIMBAL_BOOTSTRAP_PARAMETERS,
        }
        tuner = mock.Mock()
        tuner._select_profile.return_value = original
        tuner._read_parameters.return_value = bootstrap
        tuner.running = False
        statuses: queue.Queue[str] = queue.Queue()
        statuses.put("GIMBAL COMPLETE: simulated Guard9 result")
        args = mock.Mock(speed=120)

        with (
            mock.patch.object(cli, "prepare_gimbal_auto_profile") as prepare,
            mock.patch.object(cli, "run_square") as run_square,
            mock.patch.object(cli, "run_tune") as run_tune,
        ):
            self.assertTrue(cli.run_gimbal_auto(tuner, statuses, args))

        self.assertEqual(
            tuner._send_set.call_args_list,
            [
                mock.call("TURNSLOW", 80),
                mock.call("LINEKP", 3000),
                mock.call("LINEKD", 800),
                mock.call("TURNFAST", 150),
                mock.call("TURNMARGIN", 300),
                mock.call("TURNEXIT", 100),
                mock.call("TURNSLOW", 100),
            ],
        )
        tuner.start_gimbal_auto.assert_called_once_with(
            120, original
        )
        prepare.assert_not_called()
        run_square.assert_not_called()
        run_tune.assert_not_called()
        tuner._save_parameters.assert_not_called()

    def test_gimbal_auto_first_edge_trial_skips_ramp_then_rolls_back(
        self,
    ) -> None:
        runtime = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 10,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        link = FakeLink()
        tuner = make_tuner(link)
        tuner._verify_connection = mock.Mock()
        tuner._stop_reliably = mock.Mock()
        tuner._read_parameters = mock.Mock(return_value=runtime)
        tuner._verify_motor_driver = mock.Mock()
        tuner._verify_centered_line_sensor = mock.Mock()
        tuner._configure_gimbal_square_guard = mock.Mock()
        tuner._request_with_retry = mock.Mock(side_effect=[
            "IMU,OK,0,0,0,VER,1,1,0,0",
            "OK STREAM TUNE",
        ])
        tuner._send_set = mock.Mock()
        tuner._score_line = mock.Mock(return_value=0.1)
        tuner._save_samples = mock.Mock()
        tuner._save_parameters = mock.Mock()
        tuner._restore_gimbal_runtime_parameters = mock.Mock()
        tuner._save_failure = mock.Mock()

        def enqueue_first_edge(_speed: int, _laps: int) -> None:
            for time_ms in (200, 800, 1000, 1200):
                link.data_queue.put(
                    f"LT,{time_ms},120,120,120,120,"
                    "0,24,1,0,0,0,0"
                )
            for index in range(4):
                link.data_queue.put(
                    f"LT,{gui.GIMBAL_SCORE_MIN_TIME_MS + index * 20},"
                    "120,120,120,120,0,24,1,0,0,0,0"
                )
            link.data_queue.put("SQUARE ERROR,TEST END")

        tuner._start_square_reliably = mock.Mock(
            side_effect=enqueue_first_edge
        )
        with (
            mock.patch.object(Path, "mkdir"),
            mock.patch.object(Path, "write_text"),
            mock.patch.object(gui, "GIMBAL_BASELINE_SAMPLES", 3),
            mock.patch.object(gui, "LINE_SWITCH_CENTER_SAMPLES", 2),
            mock.patch.object(
                tuner,
                "_gimbal_coordinate_trials",
                return_value=[("LINEKP", 3500)],
            ),
        ):
            tuner._run_gimbal_continuous_auto(120, runtime)

        baseline = tuner._score_line.call_args_list[0].args[0]
        self.assertEqual(
            [sample.time_ms for sample in baseline],
            [
                gui.GIMBAL_SCORE_MIN_TIME_MS,
                gui.GIMBAL_SCORE_MIN_TIME_MS + 20,
                gui.GIMBAL_SCORE_MIN_TIME_MS + 40,
            ],
        )
        tuner._send_set.assert_called_once_with("LINEKP", 3500)
        tuner._restore_gimbal_runtime_parameters.assert_called_once_with(
            runtime
        )
        tuner._save_parameters.assert_not_called()
        self.assertIn("STOP", link.commands)

    def test_gimbal_failure_restores_every_runtime_field_without_save(
        self,
    ) -> None:
        original = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "GSTART": 17,
            "PROFILE": gui.PROFILE_IDS["GIMBAL"],
        }
        expected_names = (
            tuple(gui.FIRMWARE_DEFAULT_PARAMETERS) + ("GSTART",)
        )
        self.assertEqual(gui.GIMBAL_RUNTIME_PARAMETERS, expected_names)
        self.assertEqual(
            cli.GIMBAL_AUTO_ROLLBACK_PARAMETERS, expected_names
        )

        tuner = make_tuner(FakeLink())
        tuner._stop_reliably = mock.Mock()
        tuner._select_profile = mock.Mock()
        tuner._send_set = mock.Mock()
        tuner._save_parameters = mock.Mock()
        tuner._restore_gimbal_runtime_parameters(original)

        expected_calls = [mock.call("TURNSLOW", 80)]
        expected_calls.extend(
            mock.call(name, original[name])
            for name in expected_names
            if name != "TURNSLOW"
        )
        expected_calls.append(
            mock.call("TURNSLOW", original["TURNSLOW"])
        )
        self.assertEqual(tuner._send_set.call_args_list, expected_calls)
        tuner._stop_reliably.assert_called_once_with()
        tuner._select_profile.assert_called_once_with("GIMBAL")
        tuner._save_parameters.assert_not_called()

    def test_gimbal_save_waits_for_four_unique_valid_centers(
        self,
    ) -> None:
        runtime = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 10,
            "LINEKP": 3000,
            "LINEKD": 800,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        link = FakeLink()
        tuner = make_tuner(link)
        tuner._verify_connection = mock.Mock()
        tuner._stop_reliably = mock.Mock()
        tuner._read_parameters = mock.Mock(
            side_effect=[runtime, runtime, runtime]
        )
        tuner._verify_motor_driver = mock.Mock()
        tuner._verify_centered_line_sensor = mock.Mock()
        tuner._configure_gimbal_square_guard = mock.Mock()
        tuner._request_with_retry = mock.Mock(side_effect=[
            "IMU,OK,0,0,0,VER,1,1,0,0",
            "OK STREAM TUNE",
        ])
        tuner._send_set = mock.Mock()
        tuner._score_line = mock.Mock(return_value=0.1)
        tuner._save_samples = mock.Mock()
        tuner._restore_gimbal_runtime_parameters = mock.Mock()
        tuner._save_failure = mock.Mock()

        def centered(time_ms: int, corner: int) -> str:
            return (
                f"LT,{time_ms},120,120,120,120,"
                f"0,24,1,0,{corner},0,0"
            )

        def enqueue_session(_speed: int, _laps: int) -> None:
            start = gui.GIMBAL_SCORE_MIN_TIME_MS
            link.data_queue.put(centered(start, 0))
            link.data_queue.put(centered(start + 20, 0))
            link.data_queue.put(centered(start + 40, 0))
            link.data_queue.put(centered(start + 60, 1))
            link.data_queue.put("TURN CENTER,1,900,120,24,0")
            link.data_queue.put("TURN CENTER,1,900,121,24,0")
            link.data_queue.put("TURN CENTER,4,900,122,0,0")
            link.data_queue.put("TURN CENTER,4,900,123,24,7")
            link.data_queue.put("TURN CENTER,2,900,124,48,6")
            link.data_queue.put("TURN CENTER,3,900,125,24,0")
            link.data_queue.put(centered(start + 80, 1))
            link.data_queue.put("TURN CENTER,4,900,126,48,6")
            link.data_queue.put(centered(start + 100, 2))

        consumed: list[str] = []
        original_get = link.data_queue.get

        def recording_get(*args, **kwargs):
            line = original_get(*args, **kwargs)
            consumed.append(line)
            return line

        link.data_queue.get = recording_get  # type: ignore[method-assign]
        tuner._start_square_reliably = mock.Mock(
            side_effect=enqueue_session
        )
        accepted_at_save: list[set[int]] = []

        def record_save() -> None:
            accepted_at_save.append({
                int(event["corner"])
                for line in consumed
                if (event := gui.parse_turn_event(line)) is not None
                and gui.is_successful_gimbal_center_event(event)
            })

        tuner._save_parameters = mock.Mock(side_effect=record_save)
        with (
            mock.patch.object(Path, "mkdir"),
            mock.patch.object(Path, "write_text"),
            mock.patch.object(gui, "GIMBAL_BASELINE_SAMPLES", 2),
            mock.patch.object(gui, "GIMBAL_VALIDATION_SAMPLES", 2),
            mock.patch.object(gui, "GIMBAL_VALIDATION_MIN_EDGES", 2),
            mock.patch.object(gui, "LINE_SWITCH_CENTER_SAMPLES", 1),
            mock.patch.object(
                tuner, "_gimbal_coordinate_trials", return_value=[]
            ),
        ):
            tuner._run_gimbal_continuous_auto(120, runtime)

        self.assertEqual(accepted_at_save, [{1, 2, 3, 4}])
        tuner._save_parameters.assert_called_once_with()
        tuner._restore_gimbal_runtime_parameters.assert_not_called()
        tuner._save_failure.assert_not_called()

    def test_gimbal_auto_bootstrap_failure_restores_before_motion(self) -> None:
        original = {
            **gui.FIRMWARE_DEFAULT_PARAMETERS,
            "SYNC": 500,
            "BIAS": 80,
            "GSTART": 17,
            "PROFILE": 1,
            "FLASHVER": 3,
            "GUARDVER": gui.GIMBAL_GUARD_VERSION,
        }
        tuner = mock.Mock()
        tuner._select_profile.return_value = original
        tuner._send_set.side_effect = [
            None, RuntimeError("bootstrap link lost"),
        ]
        with self.assertRaisesRegex(RuntimeError, "bootstrap link lost"):
            cli.run_gimbal_auto(
                tuner, queue.Queue(), mock.Mock(speed=120)
            )
        tuner.start_gimbal_auto.assert_not_called()
        tuner._restore_gimbal_runtime_parameters.assert_called_once_with(
            original
        )
        tuner._save_parameters.assert_not_called()

    def test_archived_v4_without_profiles_remains_restorable(self) -> None:
        tuner = mock.Mock()
        tuner._read_parameters.return_value = {
            "LINEKP": 6750, "LINEKD": 2000, "TURNDIST": 98,
        }
        cli.restore_v4_baseline(tuner)
        tuner._select_profile.assert_not_called()
        tuner._save_parameters.assert_called_once_with()

    def test_motion_commands_require_safety_acknowledgement(self) -> None:
        with mock.patch.object(cli, "connect") as connect_mock:
            self.assertEqual(
                cli.main(["wheel-tune", "--port", "COM99"]), 2
            )
            self.assertEqual(
                cli.main(["line-tune", "--port", "COM99"]), 2
            )
            self.assertEqual(
                cli.main(["square", "--port", "COM99"]), 2
            )
            self.assertEqual(
                cli.main(["gimbal-auto", "--port", "COM99"]), 2
            )
        connect_mock.assert_not_called()

    def test_square_argument_validation(self) -> None:
        parser = cli.build_parser()
        args = parser.parse_args([
            "square", "--port", "COM1", "--track-safe",
            "--turn-fast", "120", "--turn-slow", "140",
        ])
        with self.assertRaisesRegex(ValueError, "turn-slow"):
            cli.validate_square(args)

    def test_line_result_summary_shows_champion_and_challenger(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result_dir = Path(directory) / "line_20260715_120000"
            result_dir.mkdir()
            (result_dir / "line_tuned_params.json").write_text(
                '{"line_kp_x1000":8250,"line_kd_x1000":2250,'
                '"challenger":{"line_kp_x1000":6750,'
                '"line_kd_x1000":3500},'
                '"selection_reason":"incumbent kept"}',
                encoding="utf-8",
            )
            summary = gui.load_line_result_summary(Path(directory))
        self.assertIn("冠军 LINEKP=8250, LINEKD=2250", summary)
        self.assertIn("挑战者 LINEKP=6750, LINEKD=3500", summary)
        self.assertIn("incumbent kept", summary)


class GroundStraightMetadataTests(unittest.TestCase):
    @staticmethod
    def valid_args() -> list[str]:
        return [
            "COM99",
            "--base-mass-g", "500",
            "--gimbal-mass-g", "300",
            "--mounting-mass-g", "20",
            "--total-mass-g", "820",
            "--gimbal-cog-height-mm", "145",
            "--gimbal-cog-forward-mm", "12",
            "--gimbal-cog-left-mm", "-3",
            "--battery-idle-v", "8.1",
            "--battery-startup-v", "7.5",
            "--sensor-height-mm", "9.5",
            "--sensor-height-light-mm", "11",
            "--wheel-load", "balanced",
            "--wheel-slip-observed", "none",
            "--confirm-gimbal-mount-rigid",
            "--confirm-ground-clear",
            "--confirm-observer-ready",
        ]

    def test_120_mmps_metadata_is_complete_and_balanced(self) -> None:
        args = ground.parse_args(self.valid_args())
        metadata = ground.build_load_metadata(args)
        self.assertEqual(args.speed, 120)
        self.assertEqual(metadata["control_profile"], "V4_GIMBAL")
        self.assertEqual(metadata["mass_g"]["balance_error"], 0)
        self.assertAlmostEqual(metadata["power_v"]["startup_drop"], 0.6)

    def test_higher_speed_requires_preceding_log_review(self) -> None:
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                ground.parse_args(self.valid_args() + ["--speed", "160"])

    def test_unknown_measurements_are_explicitly_recorded(self) -> None:
        args = ground.parse_args([
            "COM99",
            "--allow-unknown-measurements",
            "--wheel-load", "unknown",
            "--wheel-slip-observed", "none",
            "--confirm-gimbal-mount-rigid",
            "--confirm-ground-clear",
            "--confirm-observer-ready",
            "--notes", "separate battery; stabilized; center-forward COG",
        ])
        metadata = ground.build_load_metadata(args)
        self.assertEqual(metadata["measurement_status"],
                         "partial_unknowns_allowed")
        self.assertIsNone(metadata["mass_g"]["total_measured"])
        self.assertIsNone(metadata["power_v"]["startup_drop"])
        self.assertEqual(metadata["wheel_load_observation"], "unknown")

    def test_trial_lmin_is_temporary_120_mmps_only(self) -> None:
        args = ground.parse_args(
            self.valid_args() + [
                "--trial-lmin", "40", "--trial-lff", "500",
            ]
        )
        self.assertEqual(args.trial_lmin, 40)
        self.assertEqual(args.trial_lff, 500)

        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                ground.parse_args(
                    self.valid_args() + ["--trial-lmin", "101"]
                )
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                ground.parse_args(
                    self.valid_args() + ["--trial-lff", "601"]
                )
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                ground.parse_args(
                    self.valid_args() + [
                        "--speed", "160", "--previous-speed-passed",
                        "--trial-lff", "500",
                    ]
                )

    def test_trial_sync_is_conservative_and_120_mmps_only(self) -> None:
        args = ground.parse_args(
            self.valid_args() + ["--trial-sync", "500"]
        )
        self.assertEqual(args.trial_sync, 500)

        invalid_cases = [
            ["--trial-sync", "199"],
            ["--trial-sync", "1001"],
            ["--trial-sync", "500", "--trial-lmin", "40"],
            [
                "--speed", "160", "--previous-speed-passed",
                "--trial-sync", "500",
            ],
        ]
        for extra_args in invalid_cases:
            with self.subTest(extra_args=extra_args):
                with redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        ground.parse_args(self.valid_args() + extra_args)

    def test_trial_bias_can_pair_only_with_sync_control_trial(self) -> None:
        args = ground.parse_args(
            self.valid_args() + [
                "--trial-sync", "500", "--trial-bias", "80",
            ]
        )
        self.assertEqual(args.trial_sync, 500)
        self.assertEqual(args.trial_bias, 80)

        invalid_cases = [
            ["--trial-bias", "-81"],
            ["--trial-bias", "81"],
            ["--trial-bias", "80", "--trial-lff", "500"],
            [
                "--speed", "160", "--previous-speed-passed",
                "--trial-bias", "80",
            ],
        ]
        for extra_args in invalid_cases:
            with self.subTest(extra_args=extra_args):
                with redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        ground.parse_args(self.valid_args() + extra_args)

    def test_power_report_keeps_adc_pin_voltage_distinct_from_battery(self) -> None:
        values = ground.parse_power_report([
            "POWER,OK,RAW,2048,PINMV,1650,MINRAW,1900,MINPINMV,1531,"
            "MAXRAW,2100,MAXPINMV,1692,SAMPLES,350,BATTERY_SCALE,UNKNOWN"
        ])
        self.assertEqual(values["RAW"], 2048)
        self.assertEqual(values["MINPINMV"], 1531)
        self.assertEqual(values["BATTERY_SCALE"], "UNKNOWN")

        comparison = ground.build_power_adc_summary(
            values,
            {
                "MINRAW": 961, "MAXRAW": 1354, "SAMPLES": 220,
                "BATTERY_SCALE": "UNKNOWN",
            },
            {
                "MINRAW": 961, "MAXRAW": 1354, "MINPINMV": 774,
                "MAXPINMV": 1091, "SAMPLES": 620,
                "BATTERY_SCALE": "UNKNOWN",
            },
        )
        self.assertEqual(comparison["idle_window_span_raw"], 393)
        self.assertEqual(comparison["run_min_minus_idle_min_raw"], 0)
        self.assertEqual(comparison["run_min_vs_idle_min_percent"], 0.0)

    def test_relative_yaw_unwrap_and_slope_handle_180_degree_wrap(self) -> None:
        unwrapped = ground.unwrap_yaw_values([1780, 1790, -1790, -1770])
        self.assertEqual(unwrapped, [1780, 1790, 1810, 1830])
        slope = ground.linear_slope_x10_per_s(
            [0, 1000, 2000], [0, 10, 20]
        )
        self.assertAlmostEqual(slope, 10.0)

    def test_run_telemetry_uses_imu_relative_yaw_without_changing_v4_turn(self) -> None:
        source = (
            Path(__file__).resolve().parent.parent / "LAB" / "lab_ctrl.c"
        ).read_text(encoding="utf-8")
        self.assertIn("IMU_BeginRelativeYaw();", source)
        self.assertIn("return IMU_GetRelativeYawX10();", source)
        self.assertIn("return g_turnYawX10;", source)
        self.assertIn("sample->yawX10 = telemetry_yaw_x10();", source)


if __name__ == "__main__":
    unittest.main()
