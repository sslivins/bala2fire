"""Host-side PID balancer using the WebSocket transport.

This is the low-latency cousin of host_pid_balance.py. Instead of two
HTTP round trips per loop iteration (~50 ms total), it:
  * receives binary IMU frames pushed by the device at ~100 Hz
  * sends a binary PWM frame back per cycle (single TCP write)

Round-trip is roughly one Wi-Fi hop (~3-10 ms on a quiet AP), so the
loop can run at the same rate the device produces samples.

Setup:
    pip install websocket-client

Usage:
    python tools/host_pid_balance_ws.py
    python tools/host_pid_balance_ws.py --kp 25 --kd 0.7 --setpoint 0.5
"""

from __future__ import annotations

import argparse
import sys
import time

from bala2_client import Bala2, Bala2Error
from bala2_ws import Bala2WS


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="192.168.8.138")
    p.add_argument("--kp", type=float, default=25.0)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.6)
    p.add_argument("--setpoint", type=float, default=0.0)
    p.add_argument("--pwm-limit", type=int, default=600)
    p.add_argument("--cutoff-deg", type=float, default=25.0)
    p.add_argument("--print-every", type=float, default=0.5)
    p.add_argument("--max-iter", type=int, default=0)
    args = p.parse_args()

    # Use MCP/HTTP for the one-shot setup; WebSocket for the hot loop.
    rest = Bala2(args.host)
    try:
        rest.initialize()
    except Bala2Error as e:
        print(f"connect failed: {e}", file=sys.stderr)
        return 2

    print("disarming + entering EXTERNAL ...")
    try:
        rest.disarm()
        time.sleep(0.2)
    except Bala2Error:
        pass
    r = rest.set_external(True)
    if not r.get("ok"):
        print(f"set_external failed: {r}", file=sys.stderr)
        return 1

    integral = 0.0
    sent = 0
    aborted = False
    last_loops = -1
    last_print = time.monotonic()
    last_t = None
    t0 = time.monotonic()

    try:
        with Bala2WS(args.host) as ws:
            first = ws.wait_for_sample(timeout=2.0)
            if first is None:
                print("no IMU frames received over WebSocket", file=sys.stderr)
                return 1
            print(f"WS up. first sample: state={first.state_name} ang={first.angle_deg:+.1f}")

            while True:
                s = ws.latest()
                if s is None or s.loops == last_loops:
                    # No new sample yet; tiny sleep keeps the CPU sane.
                    time.sleep(0.001)
                    continue
                last_loops = s.loops

                if abs(s.angle_deg) > args.cutoff_deg:
                    print(f"!! host cutoff: angle={s.angle_deg:+.1f}", file=sys.stderr)
                    ws.send_pwm(0, 0)
                    aborted = True
                    break

                now = time.monotonic()
                dt = (now - last_t) if last_t is not None else (s.loop_dt_ms / 1000.0)
                last_t = now
                dt = max(1e-3, dt)

                error = args.setpoint - s.angle_deg
                integral += args.ki * error * dt
                integral = max(-args.pwm_limit, min(args.pwm_limit, integral))
                pwm = args.kp * error + integral - args.kd * s.gyro_dps
                pwm = max(-args.pwm_limit, min(args.pwm_limit, pwm))
                ipwm = int(pwm)

                ws.send_pwm(ipwm, ipwm)
                sent += 1

                if now - last_print >= args.print_every:
                    last_print = now
                    elapsed = now - t0
                    age_ms = (now - s.rx_t) * 1000
                    print(
                        f"t={elapsed:6.1f}s  ang={s.angle_deg:+6.1f}  "
                        f"rate={s.gyro_dps:+6.0f}  pwm={ipwm:+5d}  "
                        f"state={s.state_name:<8}  loop_hz={sent/max(1e-3,elapsed):5.1f}  "
                        f"sample_age={age_ms:4.1f}ms"
                    )

                if args.max_iter and sent >= args.max_iter:
                    break
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        # Best-effort: stop motors and leave EXTERNAL via REST.
        try:
            rest.set_external(False)
        except Bala2Error as e:
            print(f"set_external(false) err: {e}", file=sys.stderr)
        elapsed = time.monotonic() - t0
        print(f"summary: {sent} cycles in {elapsed:.1f}s "
              f"({sent/max(1e-3,elapsed):.1f} Hz)"
              + ("  [aborted by host cutoff]" if aborted else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
