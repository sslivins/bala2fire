# Copilot / AI-agent instructions for the Bala2 Fire starter

You are helping a hackathon team build firmware for an **M5Stack Bala2
Fire** balancing robot. Read this file first on every new workspace.

## Quickstart: environment bring-up

Before you can build, `idf.py` must be on PATH. Do not skip this.

**Check whether ESP-IDF is already installed:**

```powershell
# Windows
Test-Path "$env:USERPROFILE\esp\v5.5.2\esp-idf\export.ps1"
```
```bash
# macOS / Linux
test -f ~/esp/esp-idf/export.sh && echo yes
```

**If it's already installed, just activate it (every new shell):**

```powershell
. $env:USERPROFILE\esp\v5.5.2\esp-idf\export.ps1
```
```bash
. ~/esp/esp-idf/export.sh
```

**If it's NOT installed:** follow [`SETUP.md`](../SETUP.md) §1. On
Windows strongly prefer the official Online Installer — manual installs
commonly break when the system Python is upgraded. On Unix:

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
. ./export.sh
```

**Common gotcha — Python version mismatch.** If activation fails with
`ESP-IDF Python virtual environment "...idf5.5_pyX.YY_env..." not found`,
the system Python was upgraded after `install.sh` was last run.
Re-run `install.ps1 esp32` / `./install.sh esp32` from the IDF dir.

## Build / flash / monitor loop

```bash
cd <repo root>
idf.py set-target esp32      # only needed once, or after cleaning
idf.py build
idf.py -p <PORT> flash monitor
```

On Windows, find the port with:
```powershell
Get-PnpDevice -Class Ports -PresentOnly | Select Name,InstanceId
# Fire = VID_1A86&PID_55D4 (CH9102F) or VID_10C4 (CP2104)
```

**Do NOT** leave `idf.py monitor` running in a sync shell — it's
interactive and will never exit. If you need serial output, run it in
an async/background shell and kill it after you've read enough, or
have the user run it themselves.

## Hardware cheat sheet — don't get this wrong

The Bala2 Fire is **M5Stack Fire core + balancing base**, not Core2 and
not ATOM. Pins are different.

**M5Stack Fire peripherals:**

| Peripheral     | Pins / Bus                                        |
|----------------|---------------------------------------------------|
| TFT (ILI9342C) | SPI — MOSI=23, SCK=18, DC=27, CS=14, RST=33, BL=32|
| Buttons A/B/C  | GPIO 39 / 38 / 37 (active low, no external pull)  |
| MPU6886 + BMM150 | Internal I²C — SDA=21, SCL=22                   |
| Speaker        | GPIO 25 (DAC)                                     |
| microSD        | Shared SPI with TFT, CS=4                         |
| NeoPixel strip | GPIO 15 (optional M-Bus accessory)                |
| External I²C   | Grove port / M-Bus — typically GPIO 21/22 or 32/33 depending on kit rev |

**Bala2 base architecture:**

- The base has an **STM32 slave** that does motor PWM + reads wheel
  encoders. It exposes them on I²C.
- The **balancing loop runs on the Fire**, not the base. You read the
  MPU6886 on the Fire, run your filter + PID, and push wheel speeds
  down to the base over I²C.
- Reference implementation (with the exact I²C register map):
  <https://github.com/m5stack/Bala2Fire>. Consult it before writing a
  base driver from scratch.

## Code organisation guidance

- **C++ is fine and preferred** — M5Unified is C++. `app_main` is
  wrapped `extern "C"`.
- When the project grows, split into components under `components/`:
  - `bala2_base` — I²C driver for the base (motors, encoders, LEDs)
  - `imu` — attitude filter (complementary / Mahony / Madgwick)
  - `balancer` — PID cascade + safety cutoffs
  - `remote` — Wi-Fi / BLE control surface
- Run the balance loop in a FreeRTOS task **pinned to core 1** at
  100–200 Hz. Keep it deterministic: no `malloc`, no logging inside
  the loop, prefer `esp_timer_get_time()` for timing.
- Keep `sdkconfig.defaults` as the source of truth for board config
  (PSRAM, 16 MB flash, 240 MHz CPU). Changes in `menuconfig` that
  matter should be migrated into `sdkconfig.defaults` and committed.

## Dependencies

Managed by the ESP Component Manager via `main/idf_component.yml`.
`dependencies.lock` is committed — do not delete it casually; it's
what keeps everyone's builds reproducible. `managed_components/` is
gitignored and regenerated on first build.

## Things to NOT do

- Don't commit `build/`, `sdkconfig`, or `managed_components/`.
- Don't add a `platformio.ini` or Arduino-core files alongside the
  ESP-IDF project — pick one toolchain per branch.
- Don't enable `CONFIG_COMPILER_CXX_EXCEPTIONS` unless you actually
  need exceptions. It inflates binary size.
- Don't "trim" the build with `MINIMAL_BUILD ON` — M5Unified pulls in
  many IDF components and the trim breaks the dependency graph.
- Don't put the balancing PID in `app_main` or on core 0 alongside
  Wi-Fi — you'll get jitter and fall over.

## When you're unsure

- Check [`SETUP.md`](../SETUP.md) and [`README.md`](../README.md).
- Probe the IDF version with `idf.py --version`. This starter was
  validated on **v5.5.2** with **Python 3.14** on Windows.
- The reference M5Stack Bala2Fire repo (Arduino-core) is the
  authoritative source for the base's I²C register map.
