"""Host-side PID balancer for the Bala2 Fire (EXTERNAL mode).

WARNING: Wi-Fi RTT (typically 5-50 ms) limits how fast this can close
the loop. Expect the bot to be twitchier than with the on-device PID.
This is intended for experimentation / education, not production use.

Workflow:
    1. disarm (in case bot is armed)
    2. set_external(true)
    3. tight loop:
         imu = get_imu()
         pwm = pid(imu.angle_deg, imu.gyro_dps)
         set_pwm(pwm, pwm)
       Both calls go in one HTTP turn each, so loop rate ~ 1 / (2 * RTT).
    4. on Ctrl+C / fall: set_external(false)

Usage:
    python tools/host_pid_balance.py                       # defaults
    python tools/host_pid_balance.py --kp 25 --kd 0.7
    python tools/host_pid_balance.py --rate-target 30      # cap loop Hz
"""

from __future__ import annotations

import argparse
import sys
import time

from bala2_client import Bala2, Bala2Error


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="192.168.8.138")
    p.add_argument("--port", type=int, default=8080)
    # PID gains. Defaults are conservative; tune for your bot.
    p.add_argument("--kp", type=float, default=25.0,
                   help="proportional gain (PWM per deg)")
    p.add_argument("--ki", type=float, default=0.0,
                   help="integral gain (PWM per deg-second)")
    p.add_argument("--kd", type=float, default=0.6,
                   help="derivative gain (PWM per dps)")
    p.add_argument("--setpoint", type=float, default=0.0,
                   help="target tilt angle (deg). Trim if bot drifts.")
    p.add_argument("--pwm-limit", type=int, default=600,
                   help="max abs PWM (on-device hard cap is in Kconfig)")
    p.add_argument("--rate-target", type=float, default=0.0,
                   help="target loop Hz cap (0 = run as fast as possible)")
    p.add_argument("--cutoff-deg", type=float, default=25.0,
                   help="abort if |angle| exceeds this (host-side belt-and-braces)")
    p.add_argument("--print-every", type=float, default=0.5,
                   help="seconds between status prints")
    p.add_argument("--max-iter", type=int, default=0,
                   help="stop after N iterations (0 = forever)")
    args = p.parse_args()

    bot = Bala2(args.host, port=args.port, timeout=0.5)
    try:
        bot.initialize()
    except Bala2Error as e:
        print(f"connect failed: {e}", file=sys.stderr)
        return 2

    # Make sure we start clean.
    print("disarming first (if armed) ...")
    try:
        bot.disarm()
        time.sleep(0.2)
    except Bala2Error:
        pass

    print("requesting EXTERNAL mode ...")
    r = bot.set_external(True)
    if not r.get("ok"):
        print(f"set_external failed: {r}", file=sys.stderr)
        return 1
    print("EXTERNAL on. Stand the bot up. Loop starts now.")

    # PID state
    integral = 0.0
    sent = 0
    aborted = False
    t0 = time.monotonic()
    last_print = t0
    last_t = None
    period = (1.0 / args.rate_target) if args.rate_target > 0 else 0.0
    next_tick = t0

    try:
        while True:
            now = time.monotonic()

            try:
                imu = bot.get_imu()
            except Bala2Error as e:
                print(f"get_imu err: {e}", file=sys.stderr)
                continue

            angle = float(imu["angle_deg"])
            gyro  = float(imu["gyro_dps"])

            # Host-side safety cutoff (the on-device one also fires, this
            # just lets us bail without waiting for the next round-trip).
            if abs(angle) > args.cutoff_deg:
                print(f"!! host cutoff: angle={angle:+.1f} deg", file=sys.stderr)
                bot.set_pwm(0, 0)
                aborted = True
                break

            # dt
            if last_t is None:
                dt = 0.02
            else:
                dt = max(1e-3, now - last_t)
            last_t = now

            error = args.setpoint - angle
            # gyro is the derivative of angle (dps), use directly to avoid noise
            integral += args.ki * error * dt
            integral = max(-args.pwm_limit, min(args.pwm_limit, integral))
            pwm = args.kp * error + integral - args.kd * gyro
            pwm = max(-args.pwm_limit, min(args.pwm_limit, pwm))
            ipwm = int(pwm)

            try:
                rs = bot.set_pwm(ipwm, ipwm)
                if not rs.get("ok"):
                    # Most likely we got bumped out of EXTERNAL by a crash.
                    print(f"set_pwm rejected: {rs}", file=sys.stderr)
                    break
                sent += 1
            except Bala2Error as e:
                print(f"set_pwm err: {e}", file=sys.stderr)

            if now - last_print >= args.print_every:
                last_print = now
                hz = sent / max(1e-3, now - t0)
                print(
                    f"t={now-t0:6.1f}s  ang={angle:+6.1f}  rate={gyro:+6.0f}  "
                    f"pwm={ipwm:+5d}  state={imu.get('state'):<8}  "
                    f"loops_dev={imu.get('loops')}  loop_hz_host={hz:5.1f}"
                )

            if args.max_iter and sent >= args.max_iter:
                break

            if period > 0:
                next_tick += period
                sleep_for = next_tick - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        try:
            bot.set_pwm(0, 0)
        except Bala2Error:
            pass
        try:
            r = bot.set_external(False)
            print(f"set_external(false): {r}")
        except Bala2Error as e:
            print(f"set_external err: {e}", file=sys.stderr)
        elapsed = time.monotonic() - t0
        print(f"summary: {sent} cycles in {elapsed:.1f}s "
              f"({sent/max(1e-3,elapsed):.1f} Hz)"
              + ("  [aborted by host cutoff]" if aborted else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
