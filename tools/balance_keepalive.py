"""Keep the Bala2 Fire balanced from a host-side tight loop.

The on-device balancer is what actually runs the PID at ~100-200 Hz.
This script is a *supervisor* that, over the MCP/HTTP API:

  1. arms the controller (waiting until the robot is upright + still),
  2. sends drive(0, 0) at a high rate so the watchdog stays fed,
  3. polls telemetry; if the bot crashes / faults, it auto re-arms
     once the operator has stood it up again,
  4. on Ctrl+C, disarms cleanly.

Optional small linear/angular bias can be added via CLI flags --linear
and --angular if you want to nudge it while balancing.

Usage:
    python tools/balance_keepalive.py
    python tools/balance_keepalive.py --host 192.168.8.138 --rate 20
    python tools/balance_keepalive.py --linear 0.1 --angular 0.0 --no-reauto-arm
"""

from __future__ import annotations

import argparse
import sys
import time

from bala2_client import Bala2, Bala2Error


# Balancer states (mirrors components/balancer/include/balancer.h).
S_BOOT = "boot"
S_DISARMED = "disarmed"
S_ARMING = "arming"
S_ARMED = "armed"
S_CRASHED = "crashed"
S_FAULT = "fault"


def try_arm(bot: Bala2, deadline: float, verbose: bool = True) -> bool:
    """Send arm requests until state==armed or deadline elapses."""
    last_state = None
    next_arm = 0.0
    while time.monotonic() < deadline:
        try:
            s = bot.get_status()
        except Bala2Error as e:
            if verbose:
                print(f"  status err while arming: {e}", file=sys.stderr)
            time.sleep(0.2)
            continue

        if s["state"] != last_state:
            last_state = s["state"]
            if verbose:
                print(f"  arm: state={s['state']:<8} ang={s['angle_deg']:+5.1f}")

        if s["state"] == S_ARMED:
            return True

        if s["state"] in (S_DISARMED, S_CRASHED) and time.monotonic() >= next_arm:
            try:
                bot.arm()
            except Bala2Error as e:
                if verbose:
                    print(f"  arm rpc err: {e}", file=sys.stderr)
            next_arm = time.monotonic() + 0.5  # don't spam

        time.sleep(0.1)
    return False


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="192.168.8.138")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--rate", type=float, default=20.0,
                   help="drive command Hz (must be > 2 to keep watchdog fed)")
    p.add_argument("--linear", type=float, default=0.0)
    p.add_argument("--angular", type=float, default=0.0)
    p.add_argument("--status-every", type=float, default=0.5,
                   help="seconds between telemetry prints (0 = silent)")
    p.add_argument("--arm-timeout", type=float, default=15.0,
                   help="seconds to wait for the bot to stand up before giving up")
    p.add_argument("--no-reauto-arm", action="store_true",
                   help="exit on crash instead of re-arming")
    args = p.parse_args()

    if args.rate <= 2.0:
        print("warning: --rate <= 2 Hz; on-device watchdog will time out",
              file=sys.stderr)

    bot = Bala2(args.host, port=args.port, timeout=1.0)
    try:
        info = bot.initialize()
        print(f"connected: {info['serverInfo']['name']} v{info['serverInfo']['version']}")
    except Bala2Error as e:
        print(f"connect failed: {e}", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    sent = 0
    drive_errs = 0
    crashes = 0
    t0 = time.monotonic()

    try:
        # Initial arm.
        print(f"arming (stand the bot up; timeout {args.arm_timeout:.0f}s)...")
        if not try_arm(bot, t0 + args.arm_timeout):
            print("arm timed out", file=sys.stderr)
            return 1
        print("ARMED. holding balance. Ctrl+C to stop.")

        next_tick = time.monotonic()
        next_status = time.monotonic()
        in_armed = True

        while True:
            now = time.monotonic()

            # Hot path: feed the drive watchdog.
            try:
                bot.drive(args.linear, args.angular)
                sent += 1
            except Bala2Error as e:
                drive_errs += 1
                if drive_errs % 10 == 1:
                    print(f"  drive err ({drive_errs}): {e}", file=sys.stderr)

            # Cooler path: telemetry + crash recovery.
            if args.status_every > 0 and now >= next_status:
                next_status = now + args.status_every
                try:
                    s = bot.get_status()
                    print(
                        f"t={now-t0:6.1f}s  state={s['state']:<8} "
                        f"ang={s['angle_deg']:+6.1f}  rate={s['gyro_dps']:+6.0f}  "
                        f"pwm={s['pwm_left']:+5d}/{s['pwm_right']:+5d}  "
                        f"sent={sent}"
                    )
                    if s["state"] != S_ARMED and in_armed:
                        in_armed = False
                        crashes += 1
                        print(f"!! lost balance (state={s['state']}). "
                              f"crashes so far: {crashes}", file=sys.stderr)
                        if args.no_reauto_arm:
                            return 3
                        print("  attempting to re-arm...")
                        if try_arm(bot, time.monotonic() + args.arm_timeout):
                            print("  re-armed")
                            in_armed = True
                            next_tick = time.monotonic()
                        else:
                            print("re-arm timed out", file=sys.stderr)
                            return 1
                    elif s["state"] == S_ARMED:
                        in_armed = True
                except Bala2Error as e:
                    print(f"  status err: {e}", file=sys.stderr)

            # Pace the loop.
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nstopping...", file=sys.stderr)
    finally:
        try:
            print("disarm:", bot.disarm())
        except Bala2Error as e:
            print(f"disarm err: {e}", file=sys.stderr)
        elapsed = time.monotonic() - t0
        print(f"summary: {sent} drives in {elapsed:.1f}s "
              f"({sent/elapsed:.1f} Hz), {drive_errs} errs, {crashes} crashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
