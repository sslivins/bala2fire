"""Sample /telemetry at high rate during a balance attempt.

Usage:
    .\.venv\Scripts\python.exe sample_balance.py [duration_seconds]

Polls /telemetry every 50 ms (20 Hz) for `duration` seconds and prints
a compact one-line summary per sample. After the run, prints a stats
block: peak |angle|, RMS angle error, max gyro rate, PWM saturation
fraction, mean loop_dt_ms, total overruns/i2c errors. Helpful for
diagnosing whether a fall is "wheels couldn't catch up" (PWM saturating
at +/- limit), "oscillation" (angle crosses zero rapidly), or "drift"
(angle wanders monotonically with sub-saturation PWM).
"""
from __future__ import annotations
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bala2_mcp as m  # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
PERIOD_S = 0.05  # 20 Hz


def main() -> int:
    print(f"sampling /telemetry from {m.BASE_URL} for {DURATION:.1f}s...")
    samples = []
    t0 = time.time()
    next_t = t0
    while time.time() - t0 < DURATION:
        try:
            s = m._get("/telemetry", timeout=1.0)
        except Exception as e:
            print(f"  error at t={time.time()-t0:.2f}: {e}", file=sys.stderr)
            time.sleep(PERIOD_S)
            continue
        if not isinstance(s, dict):
            continue
        s["_t"] = time.time() - t0
        samples.append(s)
        print(f"  t={s['_t']:5.2f}  state={s['state']:<8}  "
              f"ang={s['angle_deg']:+6.1f}  gyro={s['gyro_dps']:+6.0f}  "
              f"pwm={s['pwm_left']:+5d}/{s['pwm_right']:+5d}")
        next_t += PERIOD_S
        slack = next_t - time.time()
        if slack > 0:
            time.sleep(slack)

    if not samples:
        print("no samples collected")
        return 1

    angles = [s["angle_deg"] for s in samples]
    gyros = [s["gyro_dps"] for s in samples]
    pwms = [(s["pwm_left"] + s["pwm_right"]) / 2 for s in samples]
    setpoints = [s["setpoint_deg"] for s in samples]
    pwm_limit = 800  # current sdkconfig

    rms_err = math.sqrt(sum((a - sp) ** 2 for a, sp in zip(angles, setpoints))
                        / len(angles))
    peak_abs = max(abs(a) for a in angles)
    max_gyro = max(abs(g) for g in gyros)
    sat_frac = sum(1 for p in pwms if abs(p) >= pwm_limit * 0.95) / len(pwms)
    armed_frac = sum(1 for s in samples
                     if s["state"] == "ARMED") / len(samples)

    last = samples[-1]
    print()
    print(f"=== {len(samples)} samples over {samples[-1]['_t']:.2f}s "
          f"(eff {len(samples)/samples[-1]['_t']:.1f} Hz) ===")
    print(f"final state    : {last['state']}")
    print(f"armed fraction : {armed_frac*100:5.1f}%")
    print(f"peak |angle|   : {peak_abs:6.1f} deg")
    print(f"RMS angle err  : {rms_err:6.2f} deg")
    print(f"max |gyro|     : {max_gyro:6.0f} dps")
    print(f"PWM saturation : {sat_frac*100:5.1f}% (limit ±{pwm_limit})")
    print(f"loop_dt mean   : "
          f"{sum(s['loop_dt_ms'] for s in samples)/len(samples):.2f} ms")
    print(f"overruns total : {last['overruns'] - samples[0]['overruns']}")
    print(f"i2c errs total : {last['i2c_errs'] - samples[0]['i2c_errs']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
