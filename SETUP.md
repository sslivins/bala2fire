# Development environment setup

Step-by-step prerequisites install. If you're using an AI coding agent
(Copilot, Claude, Cursor), also see
[`.github/copilot-instructions.md`](.github/copilot-instructions.md).

## 1. Install ESP-IDF v5.5

**Windows** — use the official installer, which bundles Python, Git,
and all the toolchains:

1. Download the **ESP-IDF Windows Installer** (Online) from
   <https://dl.espressif.com/dl/esp-idf/>.
2. Choose ESP-IDF version **v5.5.x**.
3. Default install path is `C:\Users\<you>\esp\v5.5.x\esp-idf`.
4. Tick "Register ESP-IDF PowerShell + CMD shortcuts" — these open a
   pre-activated shell via the Start Menu.

**macOS / Linux** — from a terminal:

```bash
# Prereqs (Ubuntu/Debian): build tools, Python, libs
sudo apt-get install -y git wget flex bison gperf python3 python3-venv \
    python3-pip cmake ninja-build ccache libffi-dev libssl-dev \
    dfu-util libusb-1.0-0

# Prereqs (macOS): Xcode command-line tools + Homebrew packages
xcode-select --install
brew install cmake ninja dfu-util python

# Clone and install the IDF
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

## 2. Activate the ESP-IDF environment

Once per shell session (adds `idf.py`, the toolchain, etc. to PATH):

**Windows PowerShell**
```powershell
. $env:USERPROFILE\esp\v5.5.2\esp-idf\export.ps1
```

**Windows cmd.exe**
```cmd
%USERPROFILE%\esp\v5.5.2\esp-idf\export.bat
```

**macOS / Linux**
```bash
. ~/esp/esp-idf/export.sh
```

Verify:
```
idf.py --version
# ESP-IDF v5.5.2
```

## 3. USB driver for the Fire

The Fire uses either a **CH9102F** (newer units, VID 1A86 / PID 55D4)
or a **CP2104** (older units, VID 10C4) USB-UART bridge.

- **Windows:** usually picks up the right driver automatically. If
  `idf.py flash` fails with a port error, install:
  - CH9102F: <https://www.wch.cn/downloads/CH343SER_ZIP.html>
  - CP2104: <https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers>
- **macOS:** recent versions include both drivers; no action needed.
- **Linux:** kernel includes both (`ch341`, `cp210x`). You may need to
  add your user to the `dialout` (Debian/Ubuntu) or `uucp` (Arch) group
  to access `/dev/ttyUSB0`.

## 4. Identify the COM / tty port

- **Windows:** `Get-PnpDevice -Class Ports -PresentOnly` in PowerShell,
  or Device Manager → Ports (COM & LPT). Look for `VID_1A86&PID_55D4`
  (CH9102) or `VID_10C4` (CP210x).
- **macOS:** `ls /dev/cu.*` — usually `/dev/cu.wchusbserial*` or
  `/dev/cu.SLAB_USBtoUART`.
- **Linux:** `dmesg | tail` after plugging in — usually `/dev/ttyUSB0`.

## 5. First build & flash

```bash
cd bala2
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

The first build fetches `M5Unified` + `M5GFX` from the ESP component
registry (takes a few minutes). Subsequent builds are fast.

`Ctrl-]` exits the monitor.

## Troubleshooting

- **"ESP-IDF Python virtual environment … not found"** — your system
  Python was upgraded. Re-run `install.ps1 esp32` (Windows) or
  `./install.sh esp32` (Unix) in the ESP-IDF folder.
- **"Failed to connect to ESP32: Timed out waiting for packet header"** —
  hold the Fire's side button while `idf.py flash` starts, or try a
  different USB cable (many USB-C cables are charge-only).
- **Build complains about PSRAM** — make sure `sdkconfig.defaults` was
  loaded. Delete `sdkconfig` and re-run `idf.py set-target esp32`.
- **Wrong board autodetected by M5GFX** — explicitly set it in
  `main/bala2_main.cpp` before `M5.begin()`:
  `cfg.external_display.module_display = false;` and/or
  `M5.setDisplay(0);`.
