"""Measure IMU yaw while a physically fixed car runs suspended motors."""

from __future__ import annotations

import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import serial


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--pwm", type=int, default=120)
    parser.add_argument("--confirm-fixed-suspended", action="store_true")
    args = parser.parse_args()
    if not args.confirm_fixed_suspended:
        parser.error("add --confirm-fixed-suspended after fixing the chassis")
    if not 40 <= args.pwm <= 300:
        parser.error("pwm must be 40..300")
    return args


def shortest_delta(start: int, end: int) -> int:
    delta = end - start
    while delta > 1800:
        delta -= 3600
    while delta < -1800:
        delta += 3600
    return delta


def unwrap(values: list[int]) -> list[int]:
    if not values:
        return []
    result = [values[0]]
    for previous, current in zip(values, values[1:]):
        result.append(result[-1] + shortest_delta(previous, current))
    return result


def main() -> int:
    args = arguments()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path(__file__).resolve().parent / "logs" / f"imu_motor_{stamp}"
    output.mkdir(parents=True, exist_ok=False)
    samples: list[dict[str, int | float | str]] = []
    trace: list[str] = []
    link: serial.Serial | None = None
    buffer = b""
    stop_confirmed = False

    def log(direction: str, line: str) -> None:
        row = f"{datetime.now().isoformat(timespec='milliseconds')} {direction} {line}"
        trace.append(row)
        print(row)

    def send(command: str) -> None:
        assert link is not None
        log("TX", command)
        link.write((command + "\r\n").encode("ascii"))
        link.flush()

    def receive(seconds: float) -> list[str]:
        nonlocal buffer
        assert link is not None
        deadline = time.monotonic() + seconds
        result: list[str] = []
        while time.monotonic() < deadline:
            data = link.read(256)
            if not data:
                time.sleep(0.003)
                continue
            buffer += data
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.rstrip(b"\r").decode("ascii", "replace").strip()
                if line:
                    result.append(line)
                    log("RX", line)
        return result

    def sample_phase(name: str, duration: float) -> None:
        started = time.monotonic()
        deadline = started + duration
        while time.monotonic() < deadline:
            sent_at = time.monotonic()
            send("IMU")
            lines = receive(0.09)
            for line in lines:
                if not line.startswith("IMU,OK,"):
                    continue
                fields = line.split(",")
                samples.append({
                    "phase": name,
                    "elapsed_ms": round((time.monotonic() - started) * 1000, 1),
                    "yaw_x10": int(fields[2]),
                    "ok_count": int(fields[3]),
                    "error_count": int(fields[4]),
                })
            remaining = 0.12 - (time.monotonic() - sent_at)
            if remaining > 0:
                time.sleep(remaining)

    def phase_metrics(name: str) -> dict[str, float | int]:
        phase_rows = [row for row in samples if row["phase"] == name]
        raw = [int(row["yaw_x10"]) for row in phase_rows]
        values = unwrap(raw)
        if not values:
            return {"samples": 0}
        error_start = int(phase_rows[0]["error_count"])
        error_end = int(phase_rows[-1]["error_count"])
        return {
            "samples": len(values),
            "start_deg": values[0] / 10.0,
            "end_deg": values[-1] / 10.0,
            "drift_deg": (values[-1] - values[0]) / 10.0,
            "span_deg": (max(values) - min(values)) / 10.0,
            "max_step_deg": max(
                [abs(b - a) / 10.0 for a, b in zip(values, values[1:])] or [0.0]
            ),
            "rejected_frames": max(0, error_end - error_start),
        }

    try:
        link = serial.Serial(args.port, 115200, timeout=0.02, write_timeout=1)
        time.sleep(0.3)
        link.reset_input_buffer()
        send("STOP")
        if "OK STOP" not in receive(0.8):
            raise RuntimeError("initial STOP was not confirmed")
        sample_phase("baseline", 2.0)
        send(f"PWM B {args.pwm}")
        receive(0.2)
        sample_phase("motors", 2.15)
        send("STOP")
        stop_confirmed = "OK STOP" in receive(0.8)
        if not stop_confirmed:
            raise RuntimeError("final STOP was not confirmed")
        sample_phase("after", 2.0)

        summary = {
            "port": args.port,
            "pwm": args.pwm,
            "baseline": phase_metrics("baseline"),
            "motors": phase_metrics("motors"),
            "after": phase_metrics("after"),
            "pwm_timeout_seen": any("SAFE STOP: PWM TIMEOUT" in row for row in trace),
            "final_stop_confirmed": stop_confirmed,
        }
        with (output / "imu_samples.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print("\nSUMMARY")
        print(json.dumps(summary, indent=2))
        print(f"Saved to {output}")
        return 0
    finally:
        if link is not None and link.is_open:
            try:
                if not stop_confirmed:
                    send("STOP")
                    stop_confirmed = "OK STOP" in receive(0.8)
            finally:
                link.close()
        (output / "protocol_trace.log").write_text("\n".join(trace) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
