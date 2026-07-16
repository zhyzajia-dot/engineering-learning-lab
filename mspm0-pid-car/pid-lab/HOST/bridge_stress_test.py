"""End-to-end reliability test for the two-ESP8266 serial bridge.

The test is read-only apart from sending STOP before diagnostics.  It never
starts a motor.  RADIOPING is consumed by the car-side bridge; HELLO travels
through both bridges to the MSPM0 and back with a unique token.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass, field

import serial


BAUD_RATE = 115200
REPLY_TIMEOUT_S = 2.0
QUIET_WINDOW_S = 0.06
FINAL_DRAIN_S = 1.0


@dataclass
class Results:
    requested: int = 0
    received: int = 0
    lost: int = 0
    duplicates: int = 0
    wrong_or_late: int = 0
    errors: int = 0
    latencies_ms: list[float] = field(default_factory=list)

    def report(self, name: str) -> None:
        print(f"\n{name}")
        print(
            f"  requested={self.requested} received={self.received} "
            f"lost={self.lost} duplicates={self.duplicates} "
            f"wrong_or_late={self.wrong_or_late} errors={self.errors}"
        )
        if self.latencies_ms:
            ordered = sorted(self.latencies_ms)
            p90_index = max(0, round(0.90 * (len(ordered) - 1)))
            print(
                f"  latency_ms min={ordered[0]:.1f} "
                f"mean={statistics.mean(ordered):.1f} "
                f"p90={ordered[p90_index]:.1f} max={ordered[-1]:.1f}"
            )

    def passed(self) -> bool:
        if (
            self.received != self.requested
            or self.lost
            or self.duplicates
            or self.wrong_or_late
            or self.errors
            or not self.latencies_ms
        ):
            return False
        ordered = sorted(self.latencies_ms)
        p90 = ordered[max(0, round(0.90 * (len(ordered) - 1)))]
        return p90 <= 100.0 and ordered[-1] < 1000.0


class LinePort:
    def __init__(self, port: str) -> None:
        self.serial = serial.Serial(
            port,
            BAUD_RATE,
            timeout=0.02,
            write_timeout=1.0,
        )
        self.buffer = bytearray()

    def close(self) -> None:
        self.serial.close()

    def send(self, line: str) -> None:
        self.serial.write((line + "\r\n").encode("ascii"))
        self.serial.flush()

    def read_line(self, deadline: float) -> str | None:
        while time.monotonic() < deadline:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self.buffer[: newline + 1])
                del self.buffer[: newline + 1]
                return raw.decode("ascii", errors="replace").strip()
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if chunk:
                self.buffer.extend(chunk)
        return None

    def drain(self, seconds: float) -> list[str]:
        lines: list[str] = []
        deadline = time.monotonic() + seconds
        while True:
            line = self.read_line(deadline)
            if line is None:
                break
            if line:
                lines.append(line)
        return lines


def safe_stop(port: LinePort) -> bool:
    port.drain(0.25)
    port.send("STOP")
    deadline = time.monotonic() + REPLY_TIMEOUT_S
    while True:
        line = port.read_line(deadline)
        if line is None:
            print("STOP failed: no OK STOP response")
            return False
        if line == "OK STOP":
            print("Safety STOP confirmed")
            return True
        print(f"  preflight RX {line}")


def run_radioping(port: LinePort, count: int) -> Results:
    result = Results(requested=count)
    for index in range(count):
        start = time.monotonic()
        port.send("RADIOPING")
        got_pong = False
        got_tx_ok = False
        deadline = start + REPLY_TIMEOUT_S
        while time.monotonic() < deadline and not (got_pong and got_tx_ok):
            line = port.read_line(deadline)
            if line is None:
                break
            if line == "BRIDGE_RADIO_PONG":
                if got_pong:
                    result.duplicates += 1
                else:
                    got_pong = True
                    result.latencies_ms.append(
                        (time.monotonic() - start) * 1000.0
                    )
            elif line == "BRIDGE_RADIO_TX_OK":
                if got_tx_ok:
                    result.duplicates += 1
                got_tx_ok = True
            elif line == "BRIDGE_RADIO_TX_FAILED":
                result.errors += 1
            elif line.startswith("ERR COMMAND"):
                result.errors += 1
                print(f"  RADIOPING {index + 1}: leaked to MSPM0: {line}")
            elif line:
                result.wrong_or_late += 1
                print(f"  RADIOPING {index + 1}: unexpected: {line}")

        if got_pong and got_tx_ok:
            result.received += 1
        else:
            result.lost += 1
            print(
                f"  RADIOPING {index + 1}: timeout "
                f"(tx_ok={got_tx_ok}, pong={got_pong})"
            )

        for line in port.drain(QUIET_WINDOW_S):
            if line in ("BRIDGE_RADIO_PONG", "BRIDGE_RADIO_TX_OK"):
                result.duplicates += 1
            elif line.startswith("ERR COMMAND"):
                result.errors += 1
            else:
                result.wrong_or_late += 1

        if (index + 1) % 10 == 0 or index + 1 == count:
            print(f"RADIOPING progress {index + 1}/{count}")
    return result


def run_hello(port: LinePort, count: int) -> Results:
    result = Results(requested=count)
    base_token = int(time.time() * 1000) % 1_000_000_000
    seen: set[int] = set()

    for index in range(count):
        token = base_token + index
        expected = f"HELLO,{token},"
        start = time.monotonic()
        port.send(f"HELLO {token}")
        got_current = False
        deadline = start + REPLY_TIMEOUT_S
        while time.monotonic() < deadline and not got_current:
            line = port.read_line(deadline)
            if line is None:
                break
            if line.startswith("HELLO,"):
                try:
                    reply_token = int(line.split(",", 2)[1])
                except (IndexError, ValueError):
                    result.errors += 1
                    continue
                if reply_token == token:
                    if token in seen:
                        result.duplicates += 1
                    else:
                        seen.add(token)
                        got_current = True
                        result.latencies_ms.append(
                            (time.monotonic() - start) * 1000.0
                        )
                elif reply_token in seen:
                    result.duplicates += 1
                else:
                    result.wrong_or_late += 1
                    print(
                        f"  HELLO {index + 1}: expected {token}, "
                        f"received {reply_token}"
                    )
            elif line.startswith(("ERR ", "SERIAL ERROR", "BRIDGE_ERROR")):
                result.errors += 1
                print(f"  HELLO {index + 1}: error: {line}")
            elif line:
                result.wrong_or_late += 1
                print(f"  HELLO {index + 1}: unexpected: {line}")

        if got_current:
            result.received += 1
        else:
            result.lost += 1
            print(f"  HELLO {index + 1}: timeout token={token}")

        for line in port.drain(QUIET_WINDOW_S):
            if line.startswith(expected):
                result.duplicates += 1
            elif line.startswith("HELLO,"):
                result.wrong_or_late += 1
            elif line.startswith(("ERR ", "SERIAL ERROR", "BRIDGE_ERROR")):
                result.errors += 1
            elif line:
                result.wrong_or_late += 1

        if (index + 1) % 10 == 0 or index + 1 == count:
            print(f"HELLO progress {index + 1}/{count}")

    for line in port.drain(FINAL_DRAIN_S):
        if line.startswith("HELLO,"):
            result.duplicates += 1
        elif line:
            result.wrong_or_late += 1
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stress-test ESP8266 bridge without starting motors"
    )
    parser.add_argument("port", help="PC-side bridge COM port, e.g. COM9")
    parser.add_argument(
        "--count", type=int, default=100,
        help="requests per RADIOPING/HELLO phase (default: 100)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.count < 1:
        print("--count must be positive", file=sys.stderr)
        return 2

    print(f"Opening {args.port} at {BAUD_RATE} baud")
    try:
        port = LinePort(args.port)
    except (OSError, serial.SerialException) as exc:
        print(f"Cannot open {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        if not safe_stop(port):
            return 1
        print(f"\nRunning {args.count} RADIOPING requests...")
        radio = run_radioping(port, args.count)
        print(f"\nRunning {args.count} unique HELLO requests...")
        hello = run_hello(port, args.count)
        port.send("STOP")
        port.drain(0.25)
    finally:
        port.close()

    radio.report("RADIOPING bridge-local result")
    hello.report("HELLO end-to-end result")
    passed = radio.passed() and hello.passed()
    print("\nOVERALL:", "PASS" if passed else "FAIL")
    if not passed:
        print(
            "Required: zero loss/duplicates/order errors, p90 <= 100 ms, "
            "and max < 1000 ms."
        )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
