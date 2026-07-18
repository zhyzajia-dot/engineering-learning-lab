"""Command-line acceptance runner for the MSPM0 PID car.

This module deliberately reuses SerialLink and AutoTuner from the GUI so the
visible and headless workflows exercise the same protocol and algorithms.
Motion commands require an explicit physical-safety acknowledgement.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from serial.tools import list_ports

from pid_autotune_gui import (
    FIRMWARE_DEFAULT_PARAMETERS,
    GIMBAL_BOOTSTRAP_LINE_PAIR,
    GIMBAL_GUARD_VERSION,
    GIMBAL_HARD_YAW_GUARD_CONFIRM,
    GIMBAL_RUNTIME_PARAMETERS,
    GIMBAL_SQUARE_TARGET_DELTA_CONFIRM,
    LINE_RUNTIME_PARAMETERS,
    PROFILE_IDS,
    TURN_RUNTIME_PARAMETERS,
    AutoTuner,
    PidSample,
    SerialLink,
    format_parameters,
    parse_turn_event,
    update_gimbal_square_target_guard,
    update_gimbal_hard_yaw_guard,
)

GIMBAL_STAGE1_PARAMETERS = {"SYNC": 500, "BIAS": 80, "GSTART": 10}
GIMBAL_LOAD_WHEEL_PARAMETERS = {
    # Keep the proven V4 wheel PI gains, but raise the direct torque terms for
    # the heavier chassis.  At the first 40 mm/s target these values produce
    # about 125 PWM on both wheels through the normal PI+FF equation.
    "LMIN": 90,
    "RMIN": 85,
    "LFF": 540,
    "RFF": 520,
}
GIMBAL_GSTART_LIMIT_MMPS = 35
GIMBAL_BOOTSTRAP_PARAMETERS = {
    "LINEKP": GIMBAL_BOOTSTRAP_LINE_PAIR[0],
    "LINEKD": GIMBAL_BOOTSTRAP_LINE_PAIR[1],
    "TURNFAST": 165,
    "TURNSLOW": 110,
    "TURNMARGIN": 180,
    "TURNEXIT": 140,
}
GIMBAL_AUTO_ROLLBACK_PARAMETERS = GIMBAL_RUNTIME_PARAMETERS


def require_gimbal_square_guard(
    parameters: dict[str, int], speed_mmps: int
) -> bool:
    """Require the current guarded firmware for every GIMBAL run."""
    is_gimbal = parameters.get("PROFILE") == PROFILE_IDS["GIMBAL"]
    if is_gimbal and parameters.get("GUARDVER") != GIMBAL_GUARD_VERSION:
        raise RuntimeError(
            "GIMBAL guard firmware is not active; flash the current "
            "Debug/pid_lab_mspm0.hex before motion"
        )
    return is_gimbal and speed_mmps <= 200


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MSPM0 PID car GUI-core acceptance CLI"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("ports", help="list available serial ports")

    for name, help_text in (
        ("diagnose", "run HELLO/STATUS/SENSOR/IMU/RADIOPING/PARAM checks"),
        ("params", "read current runtime parameters"),
        ("restore-line", "restore and save the stable LINEKP/LINEKD defaults"),
        ("restore-v4", "restore and save the proven V4 line/turn baseline"),
        ("save-gimbal-stage1", "save the accepted low-speed GIMBAL wheel pair"),
        ("restore-gimbal-v4", "restore the GIMBAL wheel pair to V4 values"),
        ("stop", "send an immediate STOP and wait for confirmation"),
    ):
        command = subparsers.add_parser(name, help=help_text)
        command.add_argument("--port", required=True)

    profile = subparsers.add_parser(
        "profile", help="select and verify the isolated LIGHT/GIMBAL RAM slot"
    )
    profile.add_argument("--port", required=True)
    profile.add_argument(
        "--name", required=True, choices=("light", "gimbal")
    )

    wheel = subparsers.add_parser(
        "wheel-tune", help="run wheel MIN/FF/PI autotuning"
    )
    wheel.add_argument("--port", required=True)
    wheel.add_argument(
        "--wheels", choices=("both", "left", "right"), default="both"
    )
    wheel.add_argument(
        "--wheels-raised", action="store_true",
        help="required acknowledgement that both drive wheels are suspended",
    )

    line = subparsers.add_parser(
        "line-tune", help="run closed-track LINEKP/LINEKD autotuning"
    )
    line.add_argument("--port", required=True)
    line.add_argument(
        "--track-safe", action="store_true",
        help="required acknowledgement that the car is on a closed safe track",
    )

    gimbal_auto = subparsers.add_parser(
        "gimbal-auto",
        help="continuously autotune the GIMBAL profile at 120 mm/s",
    )
    gimbal_auto.add_argument("--port", required=True)
    gimbal_auto.add_argument("--speed", type=int, choices=(120,), default=120)
    gimbal_auto.add_argument(
        "--track-safe", action="store_true",
        help=(
            "required acknowledgement that the car is centered on a closed "
            "counter-clockwise square track"
        ),
    )

    square = subparsers.add_parser(
        "square", help="run and log a counter-clockwise square"
    )
    square.add_argument("--port", required=True)
    square.add_argument("--speed", type=int, default=340)
    square.add_argument("--laps", type=int, default=1)
    square.add_argument("--turn-angle", type=int, default=900)
    square.add_argument("--turn-fast", type=int, default=185)
    square.add_argument("--turn-slow", type=int, default=140)
    square.add_argument("--turn-margin", type=int, default=180)
    square.add_argument("--turn-exit", type=int, default=140)
    square.add_argument("--turn-distance", type=int, default=98)
    square.add_argument("--timeout", type=float, default=180.0)
    square.add_argument(
        "--track-safe", action="store_true",
        help="required acknowledgement that the car is on a closed safe track",
    )
    return parser


def make_tuner(link: SerialLink) -> tuple[AutoTuner, queue.Queue[str]]:
    statuses: queue.Queue[str] = queue.Queue()
    return AutoTuner(link, statuses, lambda: None), statuses


def connect(port: str) -> SerialLink:
    link = SerialLink()
    link.connect(port)
    # Allow the USB bridge startup banner and any reset noise to finish.
    time.sleep(0.8)
    return link


def print_statuses(statuses: queue.Queue[str]) -> list[str]:
    lines: list[str] = []
    while True:
        try:
            line = statuses.get_nowait()
        except queue.Empty:
            return lines
        lines.append(line)
        print(line)


def wait_for_tuner_worker(
    tuner: AutoTuner,
    statuses: queue.Queue[str],
    worker: object,
) -> list[str]:
    """Wait for the exact worker launched by this CLI invocation.

    Reading ``tuner.running`` immediately after ``Thread.start()`` created a
    startup race: the CLI could observe False, close the serial link, and leave
    the firmware running autonomously.  A concrete Thread handle is joined
    directly, including the case where it finishes before the first poll.
    """
    lines: list[str] = []
    if isinstance(worker, threading.Thread):
        while worker.is_alive():
            lines.extend(print_statuses(statuses))
            worker.join(timeout=0.1)
        # Join once more without a timeout to establish that all worker cleanup
        # and final status queue writes happened before the serial link closes.
        worker.join()
        lines.extend(print_statuses(statuses))
        return lines

    # Keep compatibility with lightweight CLI adapters/test doubles that do
    # not expose the AutoTuner Thread, but require an actual boolean lifecycle
    # flag so a mock or unknown object cannot create another silent exit.
    running = getattr(tuner, "running", None)
    if not isinstance(running, bool):
        raise RuntimeError(
            "GIMBAL worker did not expose a joinable thread or boolean "
            "running state; STOP the car and reconnect the serial link"
        )
    while running:
        lines.extend(print_statuses(statuses))
        time.sleep(0.1)
        running = getattr(tuner, "running", None)
        if not isinstance(running, bool):
            raise RuntimeError(
                "GIMBAL worker lifecycle became invalid while running"
            )
    lines.extend(print_statuses(statuses))
    return lines


def request(tuner: AutoTuner, command: str, prefix: str) -> str:
    tuner.link.drain_data()
    reply = tuner._request_with_retry(
        command, lambda item: item.startswith(prefix), command
    )
    if reply is None:
        raise RuntimeError(f"No {prefix} reply for {command}")
    return reply


def run_diagnostics(tuner: AutoTuner) -> None:
    tuner._verify_connection()
    tuner._stop_reliably()
    print(request(tuner, "STATUS", "STATUS,"))
    sensor = request(tuner, "SENSOR", "SENSOR,")
    imu = request(tuner, "IMU", "IMU,")
    power = request(tuner, "POWER", "POWER,")
    tuner.link.drain_data()
    tuner.link.send("RADIOPING")
    radio_replies: set[str] = set()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and len(radio_replies) < 2:
        try:
            line = tuner.link.data_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        if line in ("BRIDGE_RADIO_PONG", "BRIDGE_RADIO_TX_OK"):
            radio_replies.add(line)
        elif line == "BRIDGE_RADIO_TX_FAILED":
            raise RuntimeError("ESP-NOW bridge reported RADIO TX failure")
    if radio_replies != {"BRIDGE_RADIO_PONG", "BRIDGE_RADIO_TX_OK"}:
        raise RuntimeError(
            "Incomplete RADIOPING result: " +
            (", ".join(sorted(radio_replies)) or "timeout")
        )
    parameters = tuner._read_parameters()
    print(sensor)
    print(imu)
    print(power)
    print("RADIOPING " + ", ".join(sorted(radio_replies)))
    print("PARAM " + format_parameters(parameters))
    if not sensor.startswith("SENSOR,OK,"):
        raise RuntimeError(f"Line sensor check failed: {sensor}")
    if not imu.startswith("IMU,OK,"):
        raise RuntimeError(f"IMU check failed: {imu}")
    if not power.startswith("POWER,OK,"):
        raise RuntimeError(f"TB6612 ADC check failed: {power}")


def run_tune(
    tuner: AutoTuner,
    statuses: queue.Queue[str],
    mode: str,
    wheels: tuple[str, ...] = (),
    line_speed_mmps: int = 340,
    profile_name: str | None = None,
) -> bool:
    if mode == "wheel":
        tuner.start(wheels)
    else:
        tuner.start_line(line_speed_mmps, profile_name)
    final_lines: list[str] = []
    while tuner.running:
        final_lines.extend(print_statuses(statuses))
        time.sleep(0.1)
    final_lines.extend(print_statuses(statuses))
    return any(
        line.startswith(("COMPLETE:", "LINE COMPLETE:"))
        for line in final_lines
    )


def select_light_profile_if_supported(tuner: AutoTuner) -> None:
    """Select LIGHT on Flash V3 while remaining compatible with archived V4."""
    parameters = tuner._read_parameters()
    flash_version = parameters.get("FLASHVER")
    profile = parameters.get("PROFILE")
    if flash_version == 3:
        tuner._select_profile("LIGHT")
    elif flash_version is not None or profile is not None:
        raise RuntimeError(
            f"Unsupported profile metadata: FLASHVER={flash_version}, "
            f"PROFILE={profile}"
        )


def restore_stable_line(tuner: AutoTuner) -> None:
    """Restore the known-stable line pair without starting either motor."""
    expected = {
        "LINEKP": int(FIRMWARE_DEFAULT_PARAMETERS["LINEKP"]),
        "LINEKD": int(FIRMWARE_DEFAULT_PARAMETERS["LINEKD"]),
    }
    tuner._verify_connection()
    tuner._stop_reliably()
    select_light_profile_if_supported(tuner)
    for name, value in expected.items():
        tuner._send_set(name, value)
    tuner._save_parameters()
    actual = tuner._read_parameters()
    mismatch = {
        name: actual.get(name)
        for name, value in expected.items()
        if actual.get(name) != value
    }
    if mismatch:
        raise RuntimeError(
            f"LINE parameter readback mismatch: expected={expected}, actual={mismatch}"
        )
    print(
        f"OK restored and saved LINEKP={expected['LINEKP']}, "
        f"LINEKD={expected['LINEKD']}"
    )


def restore_v4_baseline(tuner: AutoTuner) -> None:
    """Restore the proven V4 line gains and turn-distance baseline safely."""
    expected = {
        "LINEKP": int(FIRMWARE_DEFAULT_PARAMETERS["LINEKP"]),
        "LINEKD": int(FIRMWARE_DEFAULT_PARAMETERS["LINEKD"]),
        "TURNDIST": int(FIRMWARE_DEFAULT_PARAMETERS["TURNDIST"]),
    }
    tuner._verify_connection()
    tuner._stop_reliably()
    select_light_profile_if_supported(tuner)
    for name, value in expected.items():
        tuner._send_set(name, value)
    tuner._save_parameters()
    actual = tuner._read_parameters()
    mismatch = {
        name: actual.get(name)
        for name, value in expected.items()
        if actual.get(name) != value
    }
    if mismatch:
        raise RuntimeError(
            f"V4 baseline readback mismatch: expected={expected}, "
            f"actual={mismatch}"
        )
    print(
        "OK restored and saved V4 baseline: "
        f"LINEKP={expected['LINEKP']}, LINEKD={expected['LINEKD']}, "
        f"TURNDIST={expected['TURNDIST']}"
    )


def commit_gimbal_wheel_pair(
    tuner: AutoTuner,
    expected: dict[str, int],
    label: str,
) -> dict[str, int]:
    """Commit one wheel pair only to GIMBAL and verify LIGHT is unchanged."""
    tuner._verify_connection()
    tuner._stop_reliably()
    light_before = tuner._select_profile("LIGHT")
    gimbal_before = tuner._select_profile("GIMBAL")
    try:
        for name, value in expected.items():
            tuner._send_set(name, value)
        tuner._save_parameters()
        gimbal_after = tuner._read_parameters()
        mismatch = {
            name: gimbal_after.get(name)
            for name, value in expected.items()
            if gimbal_after.get(name) != value
        }
        if (
            gimbal_after.get("PROFILE") != PROFILE_IDS["GIMBAL"] or
            gimbal_after.get("FLASHVER") != 3 or mismatch
        ):
            raise RuntimeError(
                "GIMBAL stage-1 readback mismatch: "
                f"expected={expected}, actual={gimbal_after}"
            )
    except Exception:
        # Restore the previously saved GIMBAL pair if committing or verifying
        # the candidate fails. LIGHT is never written by this transaction.
        tuner._stop_reliably()
        tuner._select_profile("GIMBAL")
        for name in expected:
            if name in gimbal_before:
                tuner._send_set(name, int(gimbal_before[name]))
        tuner._save_parameters()
        raise

    light_after = tuner._select_profile("LIGHT")
    light_mismatch = {
        name: (light_before.get(name), light_after.get(name))
        for name in FIRMWARE_DEFAULT_PARAMETERS
        if light_before.get(name) != light_after.get(name)
    }
    if light_mismatch:
        raise RuntimeError(f"LIGHT profile changed unexpectedly: {light_mismatch}")
    final = tuner._select_profile("GIMBAL")
    if any(
        final.get(name) != value
        for name, value in expected.items()
    ):
        raise RuntimeError(f"final GIMBAL reload mismatch: {final}")
    print(
        f"OK saved GIMBAL {label}; LIGHT unchanged; " +
        format_parameters(final)
    )
    return final


def save_gimbal_stage1(tuner: AutoTuner) -> dict[str, int]:
    """Commit the accepted low-speed pair only to GIMBAL."""
    return commit_gimbal_wheel_pair(
        tuner, GIMBAL_STAGE1_PARAMETERS, "stage 1"
    )


def prepare_gimbal_auto_profile(tuner: AutoTuner) -> dict[str, int]:
    """Prepare GIMBAL once without erasing a previously learned GSTART."""
    tuner._verify_connection()
    tuner._stop_reliably()
    current = tuner._select_profile("GIMBAL")

    # The version gate must precede every parameter write. An older HEX does
    # not implement the startup/recovery behavior assumed by gimbal-auto.
    if current.get("GUARDVER") != GIMBAL_GUARD_VERSION:
        raise RuntimeError(
            "GIMBAL guard firmware is not active; flash the current "
            "Debug/pid_lab_mspm0.hex before changing GIMBAL parameters"
        )

    learned_gstart = current.get("GSTART", 0)
    if (
        learned_gstart != 0 and
        not -GIMBAL_GSTART_LIMIT_MMPS <= learned_gstart <=
        GIMBAL_GSTART_LIMIT_MMPS
    ):
        raise RuntimeError(f"Invalid learned GSTART readback: {learned_gstart}")
    expected = {
        **GIMBAL_LOAD_WHEEL_PARAMETERS,
        "SYNC": GIMBAL_STAGE1_PARAMETERS["SYNC"],
        "BIAS": GIMBAL_STAGE1_PARAMETERS["BIAS"],
        "GSTART": (
            learned_gstart
            if learned_gstart != 0
            else GIMBAL_STAGE1_PARAMETERS["GSTART"]
        ),
    }
    if all(current.get(name) == value for name, value in expected.items()):
        print(
            "OK GIMBAL auto preparation already matches; preserving "
            f"GSTART={expected['GSTART']} without another Flash SAVE"
        )
        return current
    return commit_gimbal_wheel_pair(
        tuner, expected, "auto preparation"
    )


def restore_gimbal_v4(tuner: AutoTuner) -> dict[str, int]:
    """Rollback the GIMBAL wheel pair to its untouched V4 starting values."""
    expected = {
        "SYNC": int(FIRMWARE_DEFAULT_PARAMETERS["SYNC"]),
        "BIAS": int(FIRMWARE_DEFAULT_PARAMETERS["BIAS"]),
        "GSTART": 0,
    }
    return commit_gimbal_wheel_pair(tuner, expected, "V4 wheel rollback")


def run_gimbal_auto(
    tuner: AutoTuner,
    statuses: queue.Queue[str],
    args: argparse.Namespace,
) -> bool:
    """Run Guard25 line/turn learning in one continuous square session."""
    original: dict[str, int] | None = None
    worker_started = False
    try:
        tuner._verify_connection()
        tuner._stop_reliably()
        original = tuner._select_profile("GIMBAL")
        if original.get("GUARDVER") != GIMBAL_GUARD_VERSION:
            raise RuntimeError(
                "GIMBAL guard firmware is not active; flash the current "
                "Debug/pid_lab_mspm0.hex before changing GIMBAL parameters"
            )
        learned_gstart = original.get("GSTART", 0)
        if (
            learned_gstart != 0 and
            not -GIMBAL_GSTART_LIMIT_MMPS <= learned_gstart <=
            GIMBAL_GSTART_LIMIT_MMPS
        ):
            raise RuntimeError(
                f"Invalid learned GSTART readback: {learned_gstart}"
            )

        # All preparation is RAM-only. Guard25 performs the sole Flash SAVE
        # after two valid centered corners and a line-safe handoff.
        stage_parameters = {
            **GIMBAL_LOAD_WHEEL_PARAMETERS,
            "SYNC": GIMBAL_STAGE1_PARAMETERS["SYNC"],
            "BIAS": GIMBAL_STAGE1_PARAMETERS["BIAS"],
            "GSTART": (
                learned_gstart
                if learned_gstart != 0
                else GIMBAL_STAGE1_PARAMETERS["GSTART"]
            ),
        }
        for name, value in stage_parameters.items():
            if original.get(name) != value:
                tuner._send_set(name, value)

        # Drop the slow turn first so TURNFAST can always be lowered without
        # temporarily violating TURNFAST >= TURNSLOW in firmware validation.
        tuner._send_set("TURNSLOW", 80)
        for name in (
            "LINEKP", "LINEKD", "TURNFAST", "TURNMARGIN", "TURNEXIT"
        ):
            value = GIMBAL_BOOTSTRAP_PARAMETERS[name]
            tuner._send_set(name, value)
        tuner._send_set(
            "TURNSLOW", GIMBAL_BOOTSTRAP_PARAMETERS["TURNSLOW"]
        )
        bootstrap = tuner._read_parameters()
        expected_runtime = {
            **stage_parameters,
            **GIMBAL_BOOTSTRAP_PARAMETERS,
        }
        mismatch = {
            name: bootstrap.get(name)
            for name, value in expected_runtime.items()
            if bootstrap.get(name) != value
        }
        if mismatch:
            raise RuntimeError(
                "GIMBAL bootstrap readback mismatch: "
                f"expected={expected_runtime}, actual={mismatch}"
            )
        if not require_gimbal_square_guard(bootstrap, args.speed):
            raise RuntimeError(
                f"GIMBAL profile readback was lost during bootstrap: {bootstrap}"
            )
        print(
            "ACTIVE CALIBRATION: "
            f"LMIN={bootstrap['LMIN']} RMIN={bootstrap['RMIN']} "
            f"LFF={bootstrap['LFF']} RFF={bootstrap['RFF']} "
            f"LINEKP={bootstrap['LINEKP']} LINEKD={bootstrap['LINEKD']} "
            f"TURNFAST={bootstrap['TURNFAST']} "
            f"TURNSLOW={bootstrap['TURNSLOW']} "
            f"GUARDVER={bootstrap['GUARDVER']}"
        )
        print(
            "GIMBAL AUTO Guard25: no fixed qualification lap. The first "
            "straight edge is scored immediately; bounded LINEKP/LINEKD "
            "coordinate trials switch one parameter at a time only on stable "
            "centered windows. Clearly worse trials roll back in RAM. "
            "TURN CAPTURE/CENTER/LEARN (and optional SEARCH) are tracked; "
            "Two valid centered corners are enough to learn turn geometry; "
            "the car continues until firmware emits SQUARE DONE. Flash SAVE "
            "then commits the incumbent, and a late score fluctuation cannot "
            "stop the car while a sensor still sees the line."
        )
        previous_worker = tuner.thread
        tuner.start_gimbal_auto(args.speed, original)
        worker = tuner.thread
        if isinstance(tuner, AutoTuner):
            if worker is None or worker is previous_worker:
                raise RuntimeError(
                    "GIMBAL worker was not created; parameters were restored "
                    "and no motion session was accepted"
                )
            if not isinstance(worker, threading.Thread):
                raise RuntimeError(
                    "GIMBAL worker handle is not joinable; parameters were "
                    "restored and no motion session was accepted"
                )
        worker_started = True
        final_lines = wait_for_tuner_worker(tuner, statuses, worker)
        if any(
            line.startswith("GIMBAL COMPLETE:")
            for line in final_lines
        ):
            return True
        if any(
            line.startswith("GIMBAL FAILED:")
            for line in final_lines
        ):
            return False
        raise RuntimeError(
            "GIMBAL worker exited without a COMPLETE/FAILED status; the "
            "worker has terminated, but its session result is unknown"
        )
    except Exception:
        if original is not None and not worker_started:
            try:
                tuner._restore_gimbal_runtime_parameters(original)
            except Exception as rollback_exc:
                print(
                    "WARNING: full GIMBAL RAM rollback failed: "
                    f"{rollback_exc}",
                    file=sys.stderr,
                )
        raise


def validate_square(args: argparse.Namespace) -> None:
    if not 80 <= args.speed <= 450 or not 1 <= args.laps <= 10:
        raise ValueError("speed must be 80..450 and laps must be 1..10")
    if not 700 <= args.turn_angle <= 1100:
        raise ValueError("turn-angle must be 700..1100")
    if not 100 <= args.turn_fast <= 300:
        raise ValueError("turn-fast must be 100..300")
    if not 80 <= args.turn_slow <= min(250, args.turn_fast):
        raise ValueError("turn-slow must be 80..250 and <= turn-fast")
    if not 50 <= args.turn_margin <= 350:
        raise ValueError("turn-margin must be 50..350")
    if not 80 <= args.turn_exit <= 250:
        raise ValueError("turn-exit must be 80..250")
    if not 50 <= args.turn_distance <= 140:
        raise ValueError("turn-distance must be 50..140")
    if args.timeout <= 0:
        raise ValueError("timeout must be positive")


def run_square(tuner: AutoTuner, args: argparse.Namespace) -> None:
    validate_square(args)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = Path(__file__).parent / "logs" / f"square_cli_{stamp}"
    session.mkdir(parents=True, exist_ok=True)
    tuner.link.set_trace_path(session / "protocol_trace.log")
    snapshot: dict[str, int] | None = None
    committed = False
    target_guard_count = 0
    hard_yaw_guard_count = 0
    target_guard_enabled = False
    turn_events: list[dict[str, int | str]] = []
    try:
        tuner._verify_connection()
        tuner._stop_reliably()
        snapshot = tuner._read_parameters()
        target_guard_enabled = require_gimbal_square_guard(
            snapshot, args.speed
        )
        sensor = request(tuner, "SENSOR", "SENSOR,")
        imu = request(tuner, "IMU", "IMU,")
        if not sensor.startswith("SENSOR,OK,"):
            raise RuntimeError(f"Line sensor is not ready: {sensor}")
        if not imu.startswith("IMU,OK,"):
            raise RuntimeError(f"IMU is not ready: {imu}")

        # Lower TURNSLOW first so cross-field validation cannot reject a
        # legitimate new TURNFAST/TURNSLOW pair because of the old pair.
        for name, value in (
            ("TURNSLOW", 80), ("TURNFAST", args.turn_fast),
            ("TURNSLOW", args.turn_slow), ("TURNANGLE", args.turn_angle),
            ("TURNMARGIN", args.turn_margin), ("TURNEXIT", args.turn_exit),
            ("TURNDIST", args.turn_distance),
        ):
            tuner._send_set(name, value)

        tuner._start_square_reliably(args.speed, args.laps)
        print(f"SQUARE running at {args.speed} mm/s for {args.laps} lap(s)")
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            try:
                line = tuner.link.data_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            sample = PidSample.parse(line)
            target_guard_count = update_gimbal_square_target_guard(
                sample, target_guard_enabled, target_guard_count
            )
            hard_yaw_guard_count = update_gimbal_hard_yaw_guard(
                sample, target_guard_enabled, hard_yaw_guard_count
            )
            if hard_yaw_guard_count >= GIMBAL_HARD_YAW_GUARD_CONFIRM:
                tuner.link.send("STOP")
                raise RuntimeError(
                    "HOST SQUARE ERROR,GIMBAL HARD YAW GUARD"
                )
            if target_guard_count >= GIMBAL_SQUARE_TARGET_DELTA_CONFIRM:
                tuner.link.send("STOP")
                raise RuntimeError(
                    "HOST SQUARE ERROR,GIMBAL TARGET DIFFERENTIAL GUARD"
                )
            turn_event = parse_turn_event(line)
            if turn_event is not None:
                turn_events.append(turn_event)
                print(line)
            if line.startswith("SQUARE LEARNED,"):
                print(line)
            if line.startswith("SQUARE DONE"):
                tuner._save_parameters()
                committed = True
                print(f"{line}; parameters saved; trace={session}")
                return
            if line.startswith(("SQUARE ERROR", "ERR SQUARE")):
                raise RuntimeError(line)
        raise TimeoutError(f"SQUARE did not finish within {args.timeout:g}s")
    finally:
        if turn_events:
            try:
                (session / "turn_events.json").write_text(
                    json.dumps(turn_events, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            except OSError:
                pass
        if not committed:
            try:
                tuner.link.send("STOP")
            except Exception:
                pass
            if snapshot is not None:
                try:
                    rollback_names = TURN_RUNTIME_PARAMETERS
                    if snapshot.get("PROFILE") == PROFILE_IDS["GIMBAL"]:
                        rollback_names = rollback_names + ("GSTART",)
                    tuner._restore_runtime_parameters(
                        snapshot, rollback_names
                    )
                    print(
                        "Square parameters restored in RAM; host SAVE was not "
                        "confirmed (firmware may autosave after SQUARE DONE)"
                    )
                except Exception as exc:
                    print(f"WARNING: square rollback failed: {exc}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "ports":
        for port in list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return 0
    if args.command == "wheel-tune" and not args.wheels_raised:
        print("Refusing motion: pass --wheels-raised after suspending both wheels",
              file=sys.stderr)
        return 2
    if args.command in ("line-tune", "square", "gimbal-auto") and not args.track_safe:
        print("Refusing motion: pass --track-safe on a closed safe track",
              file=sys.stderr)
        return 2

    link: SerialLink | None = None
    tuner: AutoTuner | None = None
    try:
        link = connect(args.port)
        tuner, statuses = make_tuner(link)
        if args.command == "diagnose":
            run_diagnostics(tuner)
        elif args.command == "params":
            tuner._verify_connection()
            print(format_parameters(tuner._read_parameters()))
        elif args.command == "restore-line":
            restore_stable_line(tuner)
        elif args.command == "restore-v4":
            restore_v4_baseline(tuner)
        elif args.command == "save-gimbal-stage1":
            save_gimbal_stage1(tuner)
        elif args.command == "restore-gimbal-v4":
            restore_gimbal_v4(tuner)
        elif args.command == "stop":
            tuner._stop_reliably()
            print("OK STOP")
        elif args.command == "profile":
            tuner._verify_connection()
            tuner._stop_reliably()
            values = tuner._select_profile(args.name)
            selected = args.name.upper()
            if values.get("PROFILE") != PROFILE_IDS[selected]:
                raise RuntimeError(f"Profile verification failed: {values}")
            print(f"OK PROFILE {selected}; " + format_parameters(values))
        elif args.command == "wheel-tune":
            wheel_map = {
                "both": ("L", "R"), "left": ("L",), "right": ("R",),
            }
            return 0 if run_tune(
                tuner, statuses, "wheel", wheel_map[args.wheels]
            ) else 1
        elif args.command == "line-tune":
            return 0 if run_tune(tuner, statuses, "line") else 1
        elif args.command == "square":
            run_square(tuner, args)
        elif args.command == "gimbal-auto":
            return 0 if run_gimbal_auto(tuner, statuses, args) else 1
        return 0
    except KeyboardInterrupt:
        if tuner is not None:
            tuner.cancel()
        print("Interrupted; STOP sent", file=sys.stderr)
        return 130
    except Exception as exc:
        if tuner is not None:
            tuner.cancel()
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    raise SystemExit(main())
