# Bala2 Fire — Hackathon Starter

ESP-IDF starter project for the **M5Stack Bala2 Fire** self-balancing robot.
Builds a minimal app that draws on the TFT, handles the three front buttons,
and reads the MPU6886 accelerometer via the [M5Unified][m5u] library.

This is a **starting point** — the balancing loop is *not* implemented.
That's your job. See [Next steps](#next-steps).

[m5u]: https://github.com/m5stack/M5Unified

## Hardware

- **Core:** M5Stack Fire — ESP32-D0WDQ6, 16 MB flash, 4/8 MB PSRAM, 2.0"
  320×240 ILI9342C TFT, 3 front buttons (A/B/C), MPU6886 + BMM150 IMU,
  speaker, microSD.
- **Base:** Bala2 Fire wheel base with an STM32 that handles low-level
  motor PWM + wheel encoders and exposes them to the Fire over I²C.
  The Fire runs the balancing loop.
- **USB:** CH9102F USB-UART bridge (newer units) or CP2104 (older units).

## Prerequisites

- **ESP-IDF v5.5.x** (this project was bootstrapped on v5.5.2)
- **Python 3.11–3.14**
- **Git**
- **USB driver** for CH9102F (newer Fires) or CP2104 (older). Windows
  typically picks one up automatically; if not, install WCH's CH343SER
  or Silicon Labs' CP210x VCP driver.

See [SETUP.md](SETUP.md) for OS-specific install steps.

## Build & flash

From a PowerShell / bash shell, once per shell:

```powershell
# Windows
. $env:USERPROFILE\esp\v5.5.2\esp-idf\export.ps1
```
```bash
# macOS / Linux
. ~/esp/esp-idf/export.sh
```

Then:

```bash
cd bala2
idf.py set-target esp32      # first time only
idf.py build
idf.py -p <PORT> flash monitor   # Ctrl-] to exit monitor
```

`<PORT>` is e.g. `COM4` on Windows, `/dev/ttyUSB0` on Linux,
`/dev/cu.wchusbserial*` on macOS.

The first build downloads `M5Unified` + `M5GFX` into `managed_components/`
(a few minutes). Subsequent builds are fast.

## Configuring Wi-Fi

The starter can optionally join a Wi-Fi network on boot and display
the IP address on the TFT. Out of the box SSID is blank, so you'll
see `IP: not configured`.

To set credentials:

```bash
idf.py menuconfig
#   Bala2 Fire configuration  --->
#     (your-ssid)     Wi-Fi SSID
#     (your-password) Wi-Fi password
#     (10000)         Wi-Fi connect timeout (ms)
```

Save + exit, then `idf.py flash monitor`. The display will show
`IP: connecting...` → `IP: 192.168.x.y` once it joins, or
`IP: connect failed` after the timeout.

**Credentials live in the gitignored `sdkconfig`**, so they stay on
your machine. If you want per-developer overrides, create
`sdkconfig.local` (also gitignored) — ESP-IDF auto-merges it on top
of `sdkconfig.defaults`.

## Project layout

```
bala2/
├── CMakeLists.txt              # top-level IDF project file
├── sdkconfig.defaults          # PSRAM, 16 MB flash, 240 MHz CPU
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       # `idf.py menuconfig` -> Bala2 Fire configuration
│   ├── idf_component.yml       # pulls in m5stack/M5Unified
│   ├── bala2_wifi.h/.c         # non-blocking Wi-Fi STA connect
│   └── bala2_main.cpp          # app_main() — edit this
├── dependencies.lock           # committed; pins component versions
└── .github/
    └── copilot-instructions.md # guidance for AI coding agents
```

## What the starter app does

- Draws `Bala2 Fire / Hello, world!` on the TFT
- Shows the Wi-Fi STA MAC address
- Joins Wi-Fi if configured via `idf.py menuconfig` and shows the IP
  (or `IP: not configured` / `IP: connect failed`)
- **Button A** → increments an on-screen counter
- **Button B** → plays a 1 kHz beep on the speaker
- **Button C** → reads MPU6886 accel and prints `ax`/`ay`

## Next steps

The STM32 on the base handles only PWM + encoders. **Balancing runs on
the Fire.** Suggested build order:

1. **`bala2_base` component** — I²C driver: `set_motor_speed(l, r)`,
   `read_encoders()`, `set_leds(...)`. Scan I²C (GPIO21/22) first to
   find the base's address; reference the official
   [m5stack/Bala2Fire][bala-ref] repo for the register map.
2. **Attitude filter** — complementary or Mahony filter on the
   MPU6886 gyro + accel → tilt angle (pitch).
3. **`balancer` component** — cascade PID:
   `tilt → wheel-speed setpoint → wheel PWM`. Run at 100–200 Hz in a
   FreeRTOS task pinned to core 1.
4. **Remote control** — Wi-Fi or BLE joystick / web UI.
5. **Telemetry & tuning** — stream `(t, angle, setpoint, pwm)` over
   serial or Wi-Fi for PID tuning.

[bala-ref]: https://github.com/m5stack/Bala2Fire

## License

MIT — do whatever you want, but balance responsibly.
