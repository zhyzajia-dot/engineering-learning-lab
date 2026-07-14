"""不接实车即可运行的上位机协议与算法回归测试。"""

from __future__ import annotations

import queue
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import pid_autotune_gui as gui


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


class AlgorithmTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
