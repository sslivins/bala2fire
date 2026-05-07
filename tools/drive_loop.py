"""Tight-loop demo controller for the Bala2 Fire.

Sends a `drive` command at a fixed rate (default 20 Hz) and prints
telemetry less often. The robot's drive command auto-expires after
500 ms, so any rate >2 Hz keeps it "fed".

Examples:
    python tools/drive_loop.py --linear 0.0 --angular 0.0
    python tools/drive_loop.py --host 192.168.8.138 --linear 0.2 --angular 0.0 --duration 3
    python tools/drive_loop.py --arm --linear 0.0 --angular 0.5 --duration 2 --disarm
"""

from __future__ import annotations

import argparse
import sys
import time

from bala2_client import Bala2, Bala2Error


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="192.168.8.138")
    p.add_argument("--rate", type=float, default=20.0, help="Hz")
    p.add_argument("--linear", type=float, default=0.0)
    p.add_argument("--angular", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=2.0, help="seconds")
    p.add_argument("--arm", action="store_true", help="arm before loop")
    p.add_argument("--disarm", action="store_true", help="disarm after loop")
    p.add_argument("--status-every", type=float, default=0.5,
                   help="print telemetry every N seconds (0 = never)")
    args = p.parse_args()

    bot = Bala2(args.host)
    try:
        bot.initialize()
    except Bala2Error as e:
        print(f"connect failed: {e}", file=sys.stderr)
        return 2

    if args.arm:
        print("arm:", bot.arm())
        time.sleep(0.5)

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tick = t0
    next_status = t0
    n = 0
    try:
        while True:
            now = time.monotonic()
            if now - t0 >= args.duration:
                break

            try:
                bot.drive(args.linear, args.angular)
                n += 1
            except Bala2Error as e:
                print(f"drive err: {e}", file=sys.stderr)

            if args.status_every > 0 and now >= next_status:
                next_status = now + args.status_every
                try:
                    s = bot.get_status()
                    print(
                        f"t={now-t0:5.2f}s  state={s['state']:<8} "
                        f"ang={s['angle_deg']:+6.1f}  "
                        f"pwm={s['pwm_left']:+5d}/{s['pwm_right']:+5d}  "
                        f"loops={s['loops']}"
                    )
                except Bala2Error as e:
                    print(f"status err: {e}", file=sys.stderr)

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()  # we fell behind, resync
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        if args.disarm:
            try:
                print("disarm:", bot.disarm())
            except Bala2Error as e:
                print(f"disarm err: {e}", file=sys.stderr)

    elapsed = time.monotonic() - t0
    print(f"sent {n} drive cmds in {elapsed:.2f}s ({n/elapsed:.1f} Hz)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
