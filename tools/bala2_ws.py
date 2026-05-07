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
    # Smoke test: connect, print 20 samples, exit.
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.8.138"
    with Bala2WS(host) as bot:
        s = bot.wait_for_sample()
        if s is None:
            print("no samples received in 2s", file=sys.stderr)
            sys.exit(1)
        t0 = time.monotonic()
        seen_loops = -1
        n = 0
        while n < 20:
            s = bot.latest()
            if s and s.loops != seen_loops:
                print(f"t={time.monotonic()-t0:5.2f}  state={s.state_name:<8} "
                      f"ang={s.angle_deg:+6.1f}  rate={s.gyro_dps:+6.0f}  "
                      f"loops={s.loops}  dt={s.loop_dt_ms:.2f}ms")
                seen_loops = s.loops
                n += 1
            time.sleep(0.005)
        elapsed = time.monotonic() - t0
        st = bot.stats
        print(f"\n{st['rx_count']} samples in {elapsed:.2f}s "
              f"({st['rx_count']/elapsed:.1f} Hz),  rx_errs={st['rx_errs']}")
