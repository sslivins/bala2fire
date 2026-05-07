"""Trace one arm + brief drive cycle, dumping WS telemetry to stdout.

Use this to figure out *why* the bot crashes: watch angle, gyro, PWM
between the moment of arming and the fall.

Usage:
    python tools/trace_crash.py 192.168.8.138 --linear 0.0 --duration 4
"""
from __future__ import annotations
import argparse, sys, time
from bala2_client import Bala2, Bala2Error
from bala2_ws import Bala2WS

p = argparse.ArgumentParser()
p.add_argument("host", nargs="?", default="192.168.8.138")
p.add_argument("--linear", type=float, default=0.0)
p.add_argument("--duration", type=float, default=4.0)
p.add_argument("--arm-timeout", type=float, default=15.0)
args = p.parse_args()

rest = Bala2(args.host)
rest.initialize()

try: rest.disarm(); time.sleep(0.2)
except Bala2Error: pass

print("stand bot up; arming...")
deadline = time.monotonic() + args.arm_timeout
last = None
while time.monotonic() < deadline:
    s = rest.get_status()
    if s["state"] != last:
        last = s["state"]
        print(f"  state={s['state']:<8} ang={s['angle_deg']:+5.1f}")
    if s["state"] == "armed":
        break
    if s["state"] in ("disarmed", "crashed", "fault"):
        try: rest.arm()
        except Bala2Error: pass
    time.sleep(0.1)
else:
    print("arm timeout"); sys.exit(1)

print(f"ARMED. tracing {args.duration:.1f}s with linear={args.linear:+.2f}")
print(f"{'t_ms':>6}  {'state':<8}  {'ang':>7}  {'gyro':>7}  {'pwm':>5}  {'loops':>6}")

with Bala2WS(args.host) as ws:
    ws.wait_for_sample(2.0)
    t0 = time.monotonic()
    last_loops = -1
    crashed_at = None
    next_drive = t0
    while time.monotonic() - t0 < args.duration:
        # Keep drive watchdog fed.
        now = time.monotonic()
        if now >= next_drive:
            next_drive = now + 0.1
            try: rest.drive(args.linear, 0.0)
            except Bala2Error: pass

        s = ws.latest()
        if s and s.loops != last_loops:
            last_loops = s.loops
            # Approximate PWM via status (not in WS frame).
            t_ms = (now - t0) * 1000
            # Print every sample to capture the crash transient.
            print(f"{t_ms:6.0f}  {s.state_name:<8}  "
                  f"{s.angle_deg:+7.2f}  {s.gyro_dps:+7.1f}  "
                  f"  -    {s.loops:6d}")
            if s.state_name in ("crashed", "fault") and crashed_at is None:
                crashed_at = now - t0
                print(f"!! crashed at t={crashed_at*1000:.0f}ms")
                # Keep capturing for 200 ms after to see the wind-down.
                stop_at = now + 0.2
                while time.monotonic() < stop_at:
                    s = ws.latest()
                    if s and s.loops != last_loops:
                        last_loops = s.loops
                        t_ms = (time.monotonic() - t0) * 1000
                        print(f"{t_ms:6.0f}  {s.state_name:<8}  "
                              f"{s.angle_deg:+7.2f}  {s.gyro_dps:+7.1f}  "
                              f"  -    {s.loops:6d}")
                    time.sleep(0.005)
                break
        time.sleep(0.002)

# Final status (so we can see overrun / i2c err counters).
fs = rest.get_status()
print(f"\nfinal status: state={fs['state']} loops={fs['loops']} "
      f"overruns={fs['overrun_count']} i2c_errs={fs['i2c_err_count']} "
      f"loop_dt_ms={fs['loop_dt_ms']:.2f}")

try: rest.drive(0,0); rest.disarm()
except Bala2Error: pass
