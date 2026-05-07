"""WebSocket client for Bala2 Fire — low-latency host-side PID transport.

Connects to ws://<host>:8080/ws and exchanges fixed-size binary frames:

  Server -> client (28 B, sent at ~100 Hz):
    magic(u32 le)  loops(u32)  angle_deg(f32)  gyro_dps(f32)
    accel_mag_g(f32)  loop_dt_ms(f32)  state(u8)  reserved(3B)

  Client -> server (8 B, sent at whatever rate caller wants):
    magic(u32 le)  left(i16)  right(i16)
    -> handed straight to balancer_set_pwm() with 200 ms watchdog.

Usage (PID balancer):
    pip install websocket-client
    python tools/host_pid_balance_ws.py --kp 25 --kd 0.6

This is the right transport when round-trip latency matters — single
TCP connection, ~30-byte framing, no HTTP parse per sample.
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import websocket  # pip install websocket-client
except ImportError as e:
    raise SystemExit(
        "Missing dependency. Install with:  pip install websocket-client"
    ) from e


IMU_MAGIC = 0x554D4942  # 'BIMU' little-endian
PWM_MAGIC = 0x4D575042  # 'BPWM' little-endian

_IMU_FMT = "<IIffffB3x"   # 28 bytes (3 padding bytes)
_IMU_SIZE = struct.calcsize(_IMU_FMT)
assert _IMU_SIZE == 28, _IMU_SIZE

_PWM_FMT = "<Ihh"         # 8 bytes
_PWM_SIZE = struct.calcsize(_PWM_FMT)
assert _PWM_SIZE == 8, _PWM_SIZE


# Mirrors balancer_state_t in components/balancer/include/balancer.h.
STATE_NAMES = {
    0: "boot", 1: "disarmed", 2: "arming", 3: "armed",
    4: "crashed", 5: "fault", 6: "external",
}


@dataclass
class ImuSample:
    loops: int
    angle_deg: float
    gyro_dps: float
    accel_mag_g: float
    loop_dt_ms: float
    state: int
    rx_t: float        # local monotonic time when this sample was received

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, "?")


class Bala2WS:
    """Thread-safe WebSocket client. RX runs on a background thread; the
    most recent IMU sample is exposed via .latest()."""

    def __init__(self, host: str, port: int = 8080, path: str = "/ws"):
        self.url = f"ws://{host}:{port}{path}"
        self._ws: Optional[websocket.WebSocket] = None
        self._lock = threading.Lock()
        self._latest: Optional[ImuSample] = None
        self._rx_count = 0
        self._rx_errs = 0
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

    # ---- lifecycle ----

    def connect(self, timeout: float = 2.0) -> None:
        self._ws = websocket.create_connection(self.url, timeout=timeout)
        # After connect, switch to a short timeout so recv() unblocks for stop checks.
        self._ws.settimeout(0.5)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    def __enter__(self) -> "Bala2WS":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- IO ----

    def latest(self) -> Optional[ImuSample]:
        with self._lock:
            return self._latest

    def wait_for_sample(self, timeout: float = 2.0) -> Optional[ImuSample]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.latest()
            if s is not None:
                return s
            time.sleep(0.01)
        return None

    def send_pwm(self, left: int, right: int) -> None:
        if self._ws is None:
            raise RuntimeError("not connected")
        # Clamp to int16 range so struct.pack can't blow up.
        left  = max(-32768, min(32767, int(left)))
        right = max(-32768, min(32767, int(right)))
        frame = struct.pack(_PWM_FMT, PWM_MAGIC, left, right)
        self._ws.send_binary(frame)

    @property
    def stats(self) -> dict:
        return {"rx_count": self._rx_count, "rx_errs": self._rx_errs}

    # ---- internal ----

    def _rx_loop(self) -> None:
        ws = self._ws
        assert ws is not None
        while not self._stop.is_set():
            try:
                opcode, data = ws.recv_data()
            except websocket.WebSocketTimeoutException:
                continue
            except Exception:
                self._rx_errs += 1
                if self._rx_errs > 5:
                    return
                continue
            if opcode != websocket.ABNF.OPCODE_BINARY or len(data) != _IMU_SIZE:
                continue
            magic, loops, angle, gyro, amag, dt_ms, state = struct.unpack(_IMU_FMT, data)
            if magic != IMU_MAGIC:
                self._rx_errs += 1
                continue
            sample = ImuSample(loops, angle, gyro, amag, dt_ms, state, time.monotonic())
            with self._lock:
                self._latest = sample
            self._rx_count += 1


if __name__ == "__main__":
    # Demo: arm the bot, drive forward for 10 s, then backward for 10 s,
    # disarm. Telemetry is streamed via WebSocket; motion commands go
    # through MCP `drive` (the on-device PID is what physically holds
    # balance — `drive` biases its setpoint so the bot leans + rolls).
    #
    # Drive command has a 500 ms watchdog on the device, so we resend
    # at ~10 Hz. Stand the bot upright when prompted.
    import argparse
    import sys
    from bala2_client import Bala2, Bala2Error

    p = argparse.ArgumentParser(description="Drive forward then back demo.")
    p.add_argument("host", nargs="?", default="192.168.8.138")
    p.add_argument("--linear", type=float, default=0.3,
                   help="forward bias magnitude (-1..1); used as +/- per phase")
    p.add_argument("--duration", type=float, default=10.0,
                   help="seconds per phase (forward, then back)")
    p.add_argument("--drive-hz", type=float, default=10.0,
                   help="rate to resend drive() (must be >2 to keep watchdog fed)")
    p.add_argument("--arm-timeout", type=float, default=15.0,
                   help="how long to wait for the bot to come upright + still")
    args = p.parse_args()

    rest = Bala2(args.host)
    try:
        rest.initialize()
    except Bala2Error as e:
        print(f"connect failed: {e}", file=sys.stderr)
        sys.exit(2)

    def wait_armed(deadline: float) -> bool:
        last = None
        next_arm = 0.0
        while time.monotonic() < deadline:
            try:
                s = rest.get_status()
            except Bala2Error:
                time.sleep(0.2); continue
            if s["state"] != last:
                last = s["state"]
                print(f"  arm: state={s['state']:<8} ang={s['angle_deg']:+5.1f}")
            if s["state"] == "armed":
                return True
            if s["state"] in ("disarmed", "crashed", "fault") and time.monotonic() >= next_arm:
                try: rest.arm()
                except Bala2Error: pass
                next_arm = time.monotonic() + 0.5
            time.sleep(0.1)
        return False

    print("disarming first (clean slate) ...")
    try:
        rest.disarm(); time.sleep(0.2)
    except Bala2Error:
        pass

    print(f"stand the bot up. arming (timeout {args.arm_timeout:.0f}s) ...")
    if not wait_armed(time.monotonic() + args.arm_timeout):
        print("arm timed out", file=sys.stderr)
        sys.exit(1)
    print("ARMED. starting drive sequence.")

    period = 1.0 / args.drive_hz

    def drive_phase(label: str, linear: float, angular: float, duration: float,
                    ws: "Bala2WS") -> bool:
        """Returns False if the bot crashed mid-phase."""
        print(f"-- {label}: linear={linear:+.2f} angular={angular:+.2f} for {duration:.1f}s")
        t0 = time.monotonic()
        next_tick = t0
        next_print = t0
        while time.monotonic() - t0 < duration:
            try:
                rest.drive(linear, angular)
            except Bala2Error as e:
                print(f"  drive err: {e}", file=sys.stderr)

            now = time.monotonic()
            if now >= next_print:
                next_print = now + 0.5
                s = ws.latest()
                if s is not None:
                    print(f"  t={now-t0:5.1f}s  state={s.state_name:<8} "
                          f"ang={s.angle_deg:+6.1f}  rate={s.gyro_dps:+6.0f}")
                    if s.state_name not in ("armed",):
                        print(f"  !! lost balance (state={s.state_name}); aborting phase",
                              file=sys.stderr)
                        return False

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
        return True

    crashed = False
    try:
        with Bala2WS(args.host) as ws:
            ws.wait_for_sample(timeout=2.0)   # let RX prime
            ok = drive_phase("FORWARD", +abs(args.linear), 0.0, args.duration, ws)
            if ok:
                # Brief coast to bleed off speed before reversing.
                drive_phase("STOP   ", 0.0, 0.0, 0.5, ws)
                ok = drive_phase("BACK   ", -abs(args.linear), 0.0, args.duration, ws)
            crashed = not ok
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        try:
            rest.drive(0.0, 0.0)
            rest.disarm()
            print("disarmed")
        except Bala2Error as e:
            print(f"cleanup err: {e}", file=sys.stderr)

    sys.exit(3 if crashed else 0)

