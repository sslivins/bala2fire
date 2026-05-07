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

## On-device APIs

Once Wi-Fi is up, the firmware exposes two HTTP servers on the Fire:

| Server | Port | Purpose |
|---|---|---|
| OTA + status | **80** | `GET /` (build info HTML), `GET /info` (JSON), `POST /update` (OTA) |
| MCP + WebSocket | **8080** | `POST /mcp` (JSON-RPC 2.0), `GET /ws` (binary WS for low-latency host PID) |

OTA upload from a host:

```powershell
# uses tools/upload.ps1; default IP is 10.25.2.55
./tools/upload.ps1 -Ip 192.168.8.138
```

### MCP (Model Context Protocol) server

The robot speaks a minimal subset of [MCP][mcp] over JSON-RPC 2.0 at
`POST http://<bot-ip>:8080/mcp`. Any MCP client (Claude Desktop, MCP
Inspector, the VS Code Copilot Chat agent, FastMCP-as-client …) can
discover and call its tools. Exposed tools:

| Tool | Args | Effect |
|---|---|---|
| `arm` | – | Request ARM (succeeds when upright + still) |
| `disarm` | – | Force DISARMED, motors off |
| `is_armed` | – | `{"armed": bool}` |
| `get_status` | – | Full balancer telemetry: state, angle, gyro, PWM, encoders, dt, error counts |
| `drive` | `linear` (-1..1), `angular` (-1..1) | Velocity bias while ARMED. Watchdog: ~500 ms |
| `get_imu` | – | Rich IMU snapshot for host-side PID (raw + fused) |
| `set_external` | `enabled` (bool) | Enter/leave EXTERNAL mode (on-device PID off, raw PWM control) |
| `set_pwm` | `left`, `right` (-1023..1023) | Raw motor PWM in EXTERNAL. Watchdog: ~200 ms |

[mcp]: https://modelcontextprotocol.io

**Register the server in VS Code** by adding `.vscode/mcp.json` (already
in this repo):

```json
{
  "servers": {
    "bala2-fire": { "type": "http", "url": "http://192.168.8.138:8080/mcp" }
  }
}
```

Then in VS Code: *Command Palette → MCP: List Servers → Start
`bala2-fire`*. The five tools become callable from Copilot Chat as
`mcp_bala2-fire_arm`, `mcp_bala2-fire_drive`, etc.

**Quick curl smoke test:**

```bash
curl -sS -X POST http://192.168.8.138:8080/mcp \
  -H 'content-type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}'
```

### WebSocket transport (low-latency host PID)

For host-side closed-loop control, JSON-RPC over HTTP is too slow
(~10–40 ms per round trip). The same server exposes a binary WebSocket
at `ws://<bot-ip>:8080/ws`:

- **Server → client (28 B every 10 ms):** `magic / loops / angle_deg /
  gyro_dps / accel_mag_g / loop_dt_ms / state` (little-endian, packed).
- **Client → server (8 B per call):** `magic / left / right` →
  forwarded to `balancer_set_pwm()` with the same 200 ms watchdog.

Single client only; a second connection evicts the first. The device
must be in EXTERNAL state for PWM frames to take effect — set that via
the MCP `set_external` tool first.

### Python client + tools

Stdlib-only client + a few demo scripts under [`tools/`](tools/):

| File | Purpose | Deps |
|---|---|---|
| [`tools/bala2_client.py`](tools/bala2_client.py) | Sync MCP client. `Bala2(host).arm() / .disarm() / .get_status() / .drive(l,a) / .get_imu() / .set_external(bool) / .set_pwm(l,r)` | stdlib |
| [`tools/bala2_ws.py`](tools/bala2_ws.py) | Threaded WebSocket client. RX thread keeps `latest()` fresh; `send_pwm(l, r)` writes a binary frame | `pip install websocket-client` |
| [`tools/drive_loop.py`](tools/drive_loop.py) | Send `drive(linear, angular)` at 20 Hz for N seconds | stdlib |
| [`tools/balance_keepalive.py`](tools/balance_keepalive.py) | Arm + feed the drive watchdog; auto re-arm after a crash | stdlib |
| [`tools/host_pid_balance.py`](tools/host_pid_balance.py) | Host PID over HTTP (~50 Hz, marginal) | stdlib |
| [`tools/host_pid_balance_ws.py`](tools/host_pid_balance_ws.py) | Host PID over WebSocket (~100 Hz, viable) | `websocket-client` |
| [`tools/upload.ps1`](tools/upload.ps1) | OTA flash via `POST /update` | – |

Minimal example:

```python
from bala2_client import Bala2
bot = Bala2("192.168.8.138")
bot.initialize()
print(bot.get_status())   # {'state': 'disarmed', 'angle_deg': ..., ...}
bot.arm()
bot.drive(0.0, 0.3)       # nudge yaw right; resend at >2 Hz to keep moving
bot.disarm()
```

Tight-loop balancer over WebSocket:

```bash
pip install websocket-client
python tools/host_pid_balance_ws.py --host 192.168.8.138 --kp 25 --kd 0.6
```

The on-device 200 Hz PID is still the right answer for actually keeping
the bot up; the WS path exists so you can experiment with custom
controllers (LQR, neural, RL …) without reflashing every iteration.

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
