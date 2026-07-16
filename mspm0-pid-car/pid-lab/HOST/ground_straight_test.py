"""Safe, logged ground straight-line validation for the MSPM0 car."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial


RAMP_MS = 1500
MM_PER_COUNT = 0.580
GIMBAL_PROFILE_ID = 1
FLASH_PROFILE_VERSION = 3
TRIAL_LMIN_MAX = 100
TRIAL_LFF_MIN = 50
TRIAL_LFF_MAX = 600
TRIAL_SYNC_MIN = 200
TRIAL_SYNC_MAX = 1000
TRIAL_BIAS_MIN = -80
TRIAL_BIAS_MAX = 80
POWER_IDLE_WINDOW_S = 2.0
STALL_MONITOR_START_MS = RAMP_MS + 500
STALL_MAX_WHEEL_SPEED_MMPS = 5
STALL_MIN_PWM = 140
STALL_CONFIRM_SAMPLES = 20


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Logged V4_GIMBAL ground straight identification. Mechanical, "
            "power, and sensor-height metadata are mandatory."
        )
    )
    parser.add_argument("port", help="PC-side ESP8266/CH340 serial port")
    parser.add_argument("--speed", type=int, choices=(120, 160, 200), default=120)
    parser.add_argument("--duration-ms", type=int, default=3500)
    parser.add_argument(
        "--trial-lmin", type=int,
        help=(
            "temporary GIMBAL left minimum PWM for this 120 mm/s run; "
            "the saved profile is restored after STOP and Flash is not written"
        ),
    )
    parser.add_argument(
        "--trial-lff", type=int,
        help=(
            "temporary GIMBAL left feedforward for this 120 mm/s run; "
            "the saved profile is restored after STOP and Flash is not written"
        ),
    )
    parser.add_argument(
        "--trial-sync", type=int,
        help=(
            "temporary GIMBAL straight synchronization gain for this "
            "120 mm/s run; restricted to a lower-or-equal A/B candidate, "
            "then the saved profile is restored without writing Flash"
        ),
    )
    parser.add_argument(
        "--trial-bias", type=int,
        help=(
            "temporary GIMBAL cumulative wheel-distance bias for this "
            "120 mm/s run; it may accompany --trial-sync, then the saved "
            "profile is restored without writing Flash"
        ),
    )
    parser.add_argument("--confirm-ground-clear", action="store_true")
    parser.add_argument("--confirm-observer-ready", action="store_true")
    parser.add_argument("--previous-speed-passed", action="store_true")
    parser.add_argument(
        "--allow-unknown-measurements", action="store_true",
        help="record unavailable numeric load measurements as null",
    )
    parser.add_argument("--base-mass-g", type=float)
    parser.add_argument("--gimbal-mass-g", type=float)
    parser.add_argument("--mounting-mass-g", type=float)
    parser.add_argument("--total-mass-g", type=float)
    parser.add_argument("--gimbal-cog-height-mm", type=float)
    parser.add_argument("--gimbal-cog-forward-mm", type=float)
    parser.add_argument("--gimbal-cog-left-mm", type=float)
    parser.add_argument("--battery-idle-v", type=float)
    parser.add_argument("--battery-startup-v", type=float)
    parser.add_argument("--sensor-height-mm", type=float)
    parser.add_argument("--sensor-height-light-mm", type=float)
    parser.add_argument(
        "--wheel-load",
        choices=(
            "unknown", "balanced", "left-compressed", "right-compressed",
            "both-compressed",
        ),
        required=True,
    )
    parser.add_argument(
        "--wheel-slip-observed", choices=("none", "left", "right", "both"),
        required=True,
    )
    parser.add_argument("--confirm-gimbal-mount-rigid", action="store_true")
    parser.add_argument("--notes", default="")
    args = parser.parse_args(argv)
    if not args.confirm_ground_clear:
        parser.error("add --confirm-ground-clear after physically clearing the track")
    if not args.confirm_observer_ready:
        parser.error(
            "add --confirm-observer-ready when someone can stop the car on "
            "slip, tilt, gimbal motion, line loss, or link loss"
        )
    if not args.confirm_gimbal_mount_rigid:
        parser.error("tighten the gimbal mount, then add --confirm-gimbal-mount-rigid")
    if args.wheel_slip_observed != "none":
        parser.error("resolve observed wheel slip before running identification")
    if args.speed > 120 and not args.previous_speed_passed:
        parser.error(
            "160/200 mm/s requires --previous-speed-passed after reviewing "
            "the preceding speed log"
        )
    if any(value is not None for value in (
        args.trial_lmin, args.trial_lff, args.trial_sync, args.trial_bias,
    )):
        if args.speed != 120:
            parser.error("trial parameters are restricted to 120 mm/s identification")
    if (args.trial_sync is not None or args.trial_bias is not None) and (
        args.trial_lmin is not None or args.trial_lff is not None
    ):
        parser.error(
            "trial SYNC/BIAS control parameters cannot be mixed with "
            "temporary LMIN/LFF wheel parameters"
        )
    if args.trial_lmin is not None:
        if not 0 <= args.trial_lmin <= TRIAL_LMIN_MAX:
            parser.error(
                f"--trial-lmin must be 0..{TRIAL_LMIN_MAX} for a ground trial"
            )
    if args.trial_lff is not None:
        if not TRIAL_LFF_MIN <= args.trial_lff <= TRIAL_LFF_MAX:
            parser.error(
                f"--trial-lff must be {TRIAL_LFF_MIN}..{TRIAL_LFF_MAX} "
                "for a ground trial"
            )
    if args.trial_sync is not None:
        if not TRIAL_SYNC_MIN <= args.trial_sync <= TRIAL_SYNC_MAX:
            parser.error(
                f"--trial-sync must be {TRIAL_SYNC_MIN}..{TRIAL_SYNC_MAX} "
                "for a conservative ground A/B trial"
            )
    if args.trial_bias is not None:
        if not TRIAL_BIAS_MIN <= args.trial_bias <= TRIAL_BIAS_MAX:
            parser.error(
                f"--trial-bias must be {TRIAL_BIAS_MIN}..{TRIAL_BIAS_MAX} "
                "for a ground A/B trial"
            )
    if not RAMP_MS + 500 <= args.duration_ms <= 15000:
        parser.error(f"duration must be {RAMP_MS + 500}..15000 ms")
    required_measurements = {
        "base-mass-g": args.base_mass_g,
        "gimbal-mass-g": args.gimbal_mass_g,
        "total-mass-g": args.total_mass_g,
        "gimbal-cog-height-mm": args.gimbal_cog_height_mm,
        "gimbal-cog-forward-mm": args.gimbal_cog_forward_mm,
        "gimbal-cog-left-mm": args.gimbal_cog_left_mm,
        "battery-idle-v": args.battery_idle_v,
        "battery-startup-v": args.battery_startup_v,
        "sensor-height-mm": args.sensor_height_mm,
    }
    missing = [
        name for name, value in required_measurements.items() if value is None
    ]
    if missing and not args.allow_unknown_measurements:
        parser.error(
            "missing load measurements: " + ", ".join(missing) +
            "; measure them or add --allow-unknown-measurements"
        )
    if not args.allow_unknown_measurements and args.mounting_mass_g is None:
        args.mounting_mass_g = 0.0
    positive = {
        "base-mass-g": args.base_mass_g,
        "gimbal-mass-g": args.gimbal_mass_g,
        "total-mass-g": args.total_mass_g,
        "gimbal-cog-height-mm": args.gimbal_cog_height_mm,
        "battery-idle-v": args.battery_idle_v,
        "battery-startup-v": args.battery_startup_v,
        "sensor-height-mm": args.sensor_height_mm,
    }
    if args.sensor_height_light_mm is not None:
        positive["sensor-height-light-mm"] = args.sensor_height_light_mm
    for name, value in positive.items():
        if value is not None and value <= 0:
            parser.error(f"{name} must be positive")
    if args.mounting_mass_g is not None and args.mounting_mass_g < 0:
        parser.error("mounting-mass-g must be non-negative")
    mass_values = (
        args.base_mass_g, args.gimbal_mass_g, args.mounting_mass_g,
        args.total_mass_g,
    )
    if all(value is not None for value in mass_values):
        expected_mass = (
            args.base_mass_g + args.gimbal_mass_g + args.mounting_mass_g
        )
        mass_error = args.total_mass_g - expected_mass
        mass_tolerance = max(5.0, expected_mass * 0.02)
        if abs(mass_error) > mass_tolerance:
            parser.error(
                "mass check failed: total differs from base + gimbal + mounting "
                f"by {mass_error:.1f} g (allowed ±{mass_tolerance:.1f} g)"
            )
    if (args.battery_idle_v is not None and
            not 1.0 <= args.battery_idle_v <= 30.0):
        parser.error("battery-idle-v must be 1..30 V")
    if (args.battery_startup_v is not None and
            not 1.0 <= args.battery_startup_v <= 30.0):
        parser.error("battery-startup-v must be 1..30 V")
    if (args.battery_idle_v is not None and
            args.battery_startup_v is not None and
            args.battery_startup_v > args.battery_idle_v + 0.1):
        parser.error("battery-startup-v cannot exceed idle voltage by more than 0.1 V")
    return args


def build_load_metadata(args: argparse.Namespace) -> dict[str, object]:
    mass_components = (
        args.base_mass_g, args.gimbal_mass_g, args.mounting_mass_g,
    )
    expected_mass = (
        sum(mass_components) if all(value is not None for value in mass_components)
        else None
    )
    mass_error = (
        args.total_mass_g - expected_mass
        if args.total_mass_g is not None and expected_mass is not None else None
    )
    return {
        "control_profile": "V4_GIMBAL",
        "profile_id": GIMBAL_PROFILE_ID,
        "measurement_status": (
            "partial_unknowns_allowed" if args.allow_unknown_measurements
            else "complete"
        ),
        "mass_g": {
            "base": args.base_mass_g,
            "gimbal": args.gimbal_mass_g,
            "mounting_hardware": args.mounting_mass_g,
            "total_measured": args.total_mass_g,
            "component_sum": expected_mass,
            "balance_error": mass_error,
        },
        "gimbal_cog_mm": {
            "height_above_ground": args.gimbal_cog_height_mm,
            "forward_from_vehicle_center": args.gimbal_cog_forward_mm,
            "left_from_vehicle_center": args.gimbal_cog_left_mm,
        },
        "power_v": {
            "battery_idle": args.battery_idle_v,
            "battery_two_wheel_startup": args.battery_startup_v,
            "startup_drop": (
                args.battery_idle_v - args.battery_startup_v
                if args.battery_idle_v is not None and
                args.battery_startup_v is not None else None
            ),
        },
        "grayscale_board_height_mm": {
            "gimbal_loaded": args.sensor_height_mm,
            "light_reference": args.sensor_height_light_mm,
            "loaded_minus_light": (
                args.sensor_height_mm - args.sensor_height_light_mm
                if args.sensor_height_mm is not None and
                args.sensor_height_light_mm is not None else None
            ),
        },
        "wheel_load_observation": args.wheel_load,
        "wheel_slip_observed_before_run": args.wheel_slip_observed,
        "gimbal_mount_rigid_confirmed": args.confirm_gimbal_mount_rigid,
        "notes": args.notes,
    }


def parse_parameter_report(lines: list[str]) -> dict[str, int]:
    for line in reversed(lines):
        if not line.startswith("PARAM,"):
            continue
        parts = line.split(",")
        values: dict[str, int] = {}
        for index in range(2, len(parts) - 1, 2):
            try:
                values[parts[index]] = int(parts[index + 1])
            except ValueError:
                continue
        return values
    return {}


def parse_power_report(lines: list[str]) -> dict[str, int | str]:
    for line in reversed(lines):
        if not line.startswith("POWER,OK,"):
            continue
        parts = line.split(",")
        values: dict[str, int | str] = {}
        for index in range(2, len(parts) - 1, 2):
            key = parts[index]
            value = parts[index + 1]
            try:
                values[key] = int(value)
            except ValueError:
                values[key] = value
        return values
    return {}


def parse_imu_report(lines: list[str]) -> dict[str, int | str]:
    for line in reversed(lines):
        if not line.startswith("IMU,"):
            continue
        parts = line.split(",")
        if len(parts) < 5:
            continue
        try:
            return {
                "status": parts[1],
                "yaw_x10": int(parts[2]),
                "ok_count": int(parts[3]),
                "error_count": int(parts[4]),
            }
        except ValueError:
            continue
    return {}


def wrapped_yaw_delta(start: int, end: int) -> int:
    delta = end - start
    while delta > 1800:
        delta -= 3600
    while delta < -1800:
        delta += 3600
    return delta


def unwrap_yaw_values(values: list[int]) -> list[int]:
    if not values:
        return []
    unwrapped = [values[0]]
    for previous, current in zip(values, values[1:]):
        unwrapped.append(unwrapped[-1] + wrapped_yaw_delta(previous, current))
    return unwrapped


def linear_slope_x10_per_s(times_ms: list[int], values: list[int]) -> float | None:
    if len(times_ms) != len(values) or len(values) < 2:
        return None
    mean_time = statistics.mean(times_ms)
    mean_value = statistics.mean(values)
    denominator = sum((time_ms - mean_time) ** 2 for time_ms in times_ms)
    if denominator == 0:
        return None
    numerator = sum(
        (time_ms - mean_time) * (value - mean_value)
        for time_ms, value in zip(times_ms, values)
    )
    return (numerator / denominator) * 1000.0


def build_power_adc_summary(
    preflight: dict[str, int | str],
    idle_window: dict[str, int | str],
    run_window: dict[str, int | str],
) -> dict[str, object]:
    def number(source: dict[str, int | str], key: str) -> int | None:
        value = source.get(key)
        return int(value) if isinstance(value, int) else None

    idle_min = number(idle_window, "MINRAW")
    idle_max = number(idle_window, "MAXRAW")
    run_min = number(run_window, "MINRAW")
    run_max = number(run_window, "MAXRAW")
    return {
        "signal": "TB6612_ADC_to_PB19_ADC1_CH6",
        "battery_scale": "unknown",
        "preflight_latest_raw": number(preflight, "RAW"),
        "preflight_latest_pin_mv": number(preflight, "PINMV"),
        "idle_window_min_raw": idle_min,
        "idle_window_max_raw": idle_max,
        "idle_window_span_raw": (
            idle_max - idle_min
            if idle_min is not None and idle_max is not None else None
        ),
        "idle_window_samples": number(idle_window, "SAMPLES"),
        "run_window_min_raw": run_min,
        "run_window_max_raw": run_max,
        "run_window_span_raw": (
            run_max - run_min
            if run_min is not None and run_max is not None else None
        ),
        "run_window_min_pin_mv": number(run_window, "MINPINMV"),
        "run_window_max_pin_mv": number(run_window, "MAXPINMV"),
        "run_window_samples": number(run_window, "SAMPLES"),
        "run_min_minus_idle_min_raw": (
            run_min - idle_min
            if run_min is not None and idle_min is not None else None
        ),
        "run_max_minus_idle_max_raw": (
            run_max - idle_max
            if run_max is not None and idle_max is not None else None
        ),
        "run_min_vs_idle_min_percent": (
            round(100.0 * (run_min - idle_min) / idle_min, 3)
            if run_min is not None and idle_min not in (None, 0) else None
        ),
    }


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(__file__).resolve().parent / "logs" / f"straight_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    trace_path = output_dir / "protocol_trace.log"
    sample_path = output_dir / "straight_samples.csv"
    summary_path = output_dir / "summary.json"
    metadata_path = output_dir / "load_metadata.json"
    load_metadata = build_load_metadata(args)
    metadata_path.write_text(
        json.dumps(load_metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    trial_requests = {
        name: value for name, value in (
            ("LMIN", args.trial_lmin),
            ("LFF", args.trial_lff),
            ("SYNC", args.trial_sync),
            ("BIAS", args.trial_bias),
        )
        if value is not None
    }

    link: serial.Serial | None = None
    rx_buffer = b""
    trace_rows: list[str] = []
    samples: list[dict[str, int | str]] = []
    final_stop_confirmed = False
    baseline_parameters: dict[str, int] = {}
    trial_parameters: dict[str, int] | None = None
    trial_applied = False
    trial_restored = not trial_requests
    safety_stop_reason: str | None = None
    stall_sample_count = 0

    def trace(direction: str, text: str) -> None:
        row = f"{datetime.now().isoformat(timespec='milliseconds')} {direction} {text}"
        trace_rows.append(row)
        print(row)

    def send(command: str) -> None:
        assert link is not None
        trace("TX", command)
        link.write((command + "\r\n").encode("ascii"))
        link.flush()

    def receive(seconds: float, monitor_stall: bool = False) -> list[str]:
        nonlocal rx_buffer, safety_stop_reason, stall_sample_count
        assert link is not None
        deadline = time.monotonic() + seconds
        result: list[str] = []
        while time.monotonic() < deadline:
            data = link.read(512)
            if not data:
                time.sleep(0.004)
                continue
            rx_buffer += data
            while b"\n" in rx_buffer:
                raw, rx_buffer = rx_buffer.split(b"\n", 1)
                text = raw.rstrip(b"\r").decode("ascii", "replace").strip()
                if text:
                    result.append(text)
                    trace("RX", text)
                    if (
                        monitor_stall and safety_stop_reason is None and
                        text.startswith("PID,")
                    ):
                        fields = text.split(",")
                        try:
                            stalled = (
                                len(fields) == 19 and fields[2] == "RUN" and
                                int(fields[1]) >= STALL_MONITOR_START_MS and
                                abs(int(fields[5])) <=
                                STALL_MAX_WHEEL_SPEED_MMPS and
                                abs(int(fields[6])) <=
                                STALL_MAX_WHEEL_SPEED_MMPS and
                                max(abs(int(fields[7])), abs(int(fields[8]))) >=
                                STALL_MIN_PWM
                            )
                        except ValueError:
                            stalled = False
                        stall_sample_count = (
                            stall_sample_count + 1 if stalled else 0
                        )
                        if stall_sample_count >= STALL_CONFIRM_SAMPLES:
                            safety_stop_reason = (
                                "both_wheels_near_zero_with_high_pwm"
                            )
                            send("STOP")
                            deadline = min(deadline, time.monotonic() + 1.0)
        return result

    def command(text: str, wait: float = 0.8) -> list[str]:
        send(text)
        return receive(wait)

    def restore_trial_parameters() -> bool:
        """Discard the unsaved trial by reloading the saved GIMBAL RAM slot."""
        nonlocal trial_restored
        if not trial_applied or trial_restored:
            return True
        restore_lines = command("PROFILE GIMBAL")
        readback_lines = command("PARAM")
        restored = parse_parameter_report(restore_lines + readback_lines)
        expected_lmin = baseline_parameters.get("LMIN")
        trial_restored = (
            "OK PROFILE GIMBAL FLASH 3" in restore_lines and
            restored.get("PROFILE") == GIMBAL_PROFILE_ID and
            restored.get("FLASHVER") == FLASH_PROFILE_VERSION and
            restored.get("LMIN") == expected_lmin and
            all(
                restored.get(name) == baseline_parameters.get(name)
                for name in trial_requests
            )
        )
        return trial_restored

    try:
        link = serial.Serial(args.port, 115200, timeout=0.03, write_timeout=1)
        time.sleep(0.3)
        link.reset_input_buffer()

        stop_lines = command("STOP")
        profile_lines = command("PROFILE GIMBAL")
        parameter_lines = command("PARAM")
        parameters = parse_parameter_report(profile_lines + parameter_lines)
        baseline_parameters = dict(parameters)
        status_lines = command("STATUS")
        sensor_lines = command("SENSOR")
        imu_before_lines = command("IMU")
        imu_before = parse_imu_report(imu_before_lines)
        yaw_before = (
            int(imu_before["yaw_x10"])
            if imu_before.get("status") == "OK" else None
        )
        power_before_lines = command("POWER")
        power_before = parse_power_report(power_before_lines)
        gates = {
            "stop": "OK STOP" in stop_lines,
            "profile": "OK PROFILE GIMBAL FLASH 3" in profile_lines,
            "profile_readback": (
                parameters.get("PROFILE") == GIMBAL_PROFILE_ID and
                parameters.get("FLASHVER") == FLASH_PROFILE_VERSION
            ),
            "idle": any(",IDLE," in line and ",STBY,HIGH" in line
                        for line in status_lines if line.startswith("STATUS,")),
            "sensor": any(line.startswith("SENSOR,OK,") for line in sensor_lines),
            "imu": yaw_before is not None,
            "power_adc": (
                power_before.get("BATTERY_SCALE") == "UNKNOWN" and
                int(power_before.get("SAMPLES", 0)) > 0
            ),
        }
        if not all(gates.values()):
            raise RuntimeError(f"preflight failed; RUN not sent: {gates}")

        if trial_requests:
            for name, value in trial_requests.items():
                token = int(time.time_ns() % 2_147_483_647) or 1
                expected_ack = f"SA,{token},{name},{value}"
                # Mark the trial as applied before sending. Even if its ACK is
                # lost, finally reloads the saved GIMBAL slot and discards SET.
                trial_applied = True
                set_lines = command(f"SET {name} {value} {token}")
                if expected_ack not in set_lines:
                    raise RuntimeError(
                        f"temporary {name} SET was not confirmed; RUN not sent"
                    )
            trial_lines = command("PARAM")
            trial_parameters = parse_parameter_report(trial_lines)
            if (
                trial_parameters.get("PROFILE") != GIMBAL_PROFILE_ID or
                trial_parameters.get("FLASHVER") != FLASH_PROFILE_VERSION or
                any(
                    trial_parameters.get(name) != value
                    for name, value in trial_requests.items()
                )
            ):
                raise RuntimeError(
                    "temporary parameter readback mismatch; RUN not sent: "
                    f"{trial_parameters}"
                )
            parameters = trial_parameters

        idle_power_reset_lines = command("POWER RESET", wait=0.5)
        if "OK POWER RESET" not in idle_power_reset_lines:
            raise RuntimeError(
                "idle POWER RESET was not confirmed; RUN not sent"
            )
        time.sleep(max(0.0, POWER_IDLE_WINDOW_S - 0.5))
        power_idle_lines = command("POWER")
        power_idle_window = parse_power_report(power_idle_lines)
        if (
            power_idle_window.get("BATTERY_SCALE") != "UNKNOWN" or
            int(power_idle_window.get("SAMPLES", 0)) < 100 or
            "MINRAW" not in power_idle_window or
            "MAXRAW" not in power_idle_window
        ):
            raise RuntimeError(
                "idle TB6612 ADC comparison window is incomplete; RUN not sent: "
                f"{power_idle_window}"
            )

        power_reset_lines = command("POWER RESET", wait=0.5)
        if "OK POWER RESET" not in power_reset_lines:
            raise RuntimeError("run POWER RESET was not confirmed; RUN not sent")

        send(f"RUN {args.speed} {args.duration_ms}")
        run_lines = receive(
            (args.duration_ms / 1000.0) + 1.1, monitor_stall=True
        )
        for line in run_lines:
            if not line.startswith("PID,"):
                continue
            fields = line.split(",")
            if len(fields) != 19 or fields[2] != "RUN":
                continue
            try:
                samples.append({
                    "time_ms": int(fields[1]),
                    "mode": fields[2],
                    "left_target": int(fields[3]),
                    "right_target": int(fields[4]),
                    "left_speed": int(fields[5]),
                    "right_speed": int(fields[6]),
                    "left_pwm": int(fields[7]),
                    "right_pwm": int(fields[8]),
                    "count_diff": int(fields[11]),
                    "yaw_x10": int(fields[15]),
                })
            except ValueError:
                continue

        stop_after = command("STOP")
        final_stop_confirmed = "OK STOP" in stop_after
        power_after_lines = command("POWER")
        power_after = parse_power_report(power_after_lines)
        imu_after_lines = command("IMU")
        imu_after = parse_imu_report(imu_after_lines)
        yaw_after = (
            int(imu_after["yaw_x10"])
            if imu_after.get("status") == "OK" else None
        )
        if not final_stop_confirmed:
            raise RuntimeError("final STOP was not confirmed")
        if (
            power_after.get("BATTERY_SCALE") != "UNKNOWN" or
            int(power_after.get("SAMPLES", 0)) < 20 or
            "MINRAW" not in power_after or
            "MAXRAW" not in power_after
        ):
            raise RuntimeError(
                "test stopped, but the TB6612 ADC run window is incomplete: "
                f"{power_after}"
            )
        if not restore_trial_parameters():
            raise RuntimeError(
                "test stopped, but the saved GIMBAL parameters were not restored"
            )
        if len(samples) < 20:
            raise RuntimeError(f"only {len(samples)} valid RUN samples received")

        ordered = sorted(samples, key=lambda row: int(row["time_ms"]))
        actual_sample_end_ms = int(ordered[-1]["time_ms"])
        steady_end_ms = min(args.duration_ms - 50, actual_sample_end_ms)
        steady = [
            row for row in samples
            if RAMP_MS + 200 <= int(row["time_ms"]) <= steady_end_ms
        ]
        if len(steady) < 20:
            raise RuntimeError(f"only {len(steady)} steady samples received")

        def values(key: str) -> list[int]:
            return [int(row[key]) for row in steady]

        yaw_unwrapped = unwrap_yaw_values(
            [int(row["yaw_x10"]) for row in ordered]
        )
        steady_times = [int(row["time_ms"]) for row in steady]
        yaw_by_time = {
            int(row["time_ms"]): yaw
            for row, yaw in zip(ordered, yaw_unwrapped)
        }
        steady_yaw = [yaw_by_time[time_ms] for time_ms in steady_times]
        distance = 0.0
        for previous, current in zip(ordered, ordered[1:]):
            dt = max(0, int(current["time_ms"]) - int(previous["time_ms"])) / 1000.0
            previous_speed = (int(previous["left_speed"]) + int(previous["right_speed"])) / 2.0
            current_speed = (int(current["left_speed"]) + int(current["right_speed"])) / 2.0
            distance += (previous_speed + current_speed) * 0.5 * dt

        last = ordered[-1]
        summary = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "load_metadata": load_metadata,
            "runtime_parameters": parameters,
            "saved_parameters_before_trial": (
                baseline_parameters if trial_applied else None
            ),
            "trial_overrides": (
                {
                    name: {
                        "saved": baseline_parameters.get(name),
                        "trial": value,
                    }
                    for name, value in trial_requests.items()
                }
                if trial_applied else {}
            ),
            "trial_flash_written": False,
            "trial_runtime_restored": trial_restored,
            "power_adc": build_power_adc_summary(
                power_before, power_idle_window, power_after
            ),
            "port": args.port,
            "speed_command_mmps": args.speed,
            "duration_ms": args.duration_ms,
            "requested_duration_ms": args.duration_ms,
            "actual_sample_end_ms": actual_sample_end_ms,
            "host_safety_stop_reason": safety_stop_reason,
            "sample_count": len(samples),
            "steady_sample_count": len(steady),
            "left_target_mean_mmps": statistics.mean(values("left_target")),
            "right_target_mean_mmps": statistics.mean(values("right_target")),
            "left_speed_mean_mmps": statistics.mean(values("left_speed")),
            "right_speed_mean_mmps": statistics.mean(values("right_speed")),
            "left_speed_ripple_sd_mmps": statistics.pstdev(values("left_speed")),
            "right_speed_ripple_sd_mmps": statistics.pstdev(values("right_speed")),
            "left_pwm_mean": statistics.mean(values("left_pwm")),
            "right_pwm_mean": statistics.mean(values("right_pwm")),
            "final_count_diff": int(last["count_diff"]),
            "final_distance_diff_mm": int(last["count_diff"]) * MM_PER_COUNT,
            "integrated_average_distance_mm": distance,
            "yaw_before_x10": yaw_before,
            "yaw_after_x10": yaw_after,
            "yaw_delta_x10": (wrapped_yaw_delta(yaw_before, yaw_after)
                               if yaw_before is not None and yaw_after is not None else None),
            "imu_ok_count_delta": (
                int(imu_after["ok_count"]) - int(imu_before["ok_count"])
                if "ok_count" in imu_before and "ok_count" in imu_after
                else None
            ),
            "imu_error_count_delta": (
                int(imu_after["error_count"]) - int(imu_before["error_count"])
                if "error_count" in imu_before and "error_count" in imu_after
                else None
            ),
            "run_relative_yaw": {
                "start_x10": yaw_unwrapped[0],
                "end_x10": yaw_unwrapped[-1],
                "delta_x10": yaw_unwrapped[-1] - yaw_unwrapped[0],
                "steady_mean_x10": statistics.mean(steady_yaw),
                "steady_peak_abs_from_start_x10": max(
                    abs(value - yaw_unwrapped[0]) for value in steady_yaw
                ),
                "steady_slope_x10_per_s": linear_slope_x10_per_s(
                    steady_times, steady_yaw
                ),
            },
            "test_done_seen": "TEST DONE" in run_lines,
            "final_stop_confirmed": final_stop_confirmed,
        }
        with sample_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print("\nSUMMARY")
        print(json.dumps(summary, indent=2))
        print(f"Saved to {output_dir}")
        return 0
    finally:
        if link is not None and link.is_open:
            try:
                if not final_stop_confirmed:
                    send("STOP")
                    final_stop_confirmed = "OK STOP" in receive(0.8)
                if trial_applied and not trial_restored:
                    restore_trial_parameters()
            finally:
                link.close()
        trace_path.write_text("\n".join(trace_rows) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
