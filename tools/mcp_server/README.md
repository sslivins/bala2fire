# Bala2 MCP server

Stdio MCP server that exposes the Bala2 Fire's HTTP control surface to
an LLM (Claude Desktop, Copilot CLI, Cursor, etc).

```
LLM client ── stdio (JSON-RPC) ──► bala2_mcp.py ── HTTP ──► robot
```

The robot's HTTP server is implemented in `components/bala2_ota/`.
The endpoints this MCP wraps are: `GET /info`, `GET /telemetry`,
`POST /arm`, `POST /disarm`, `POST /setpoint`.

## Tools exposed

| Tool | Description |
|---|---|
| `get_status()`            | Full telemetry snapshot (state, angle, gyro, encoders, PWM, setpoint, loop dt, error counters). |
| `arm()`                   | Request DISARMED → ARMING. Bot must be upright + still ~300 ms to progress to ARMED. |
| `disarm()`                | E-stop. Always succeeds. |
| `set_setpoint(deg)`       | Bias the angle setpoint. ±2° is the typical "drive forward/back" range; firmware clamps to ±30°. |
| `firmware_info()`         | Version, IDF, OTA slot, uptime, armed flag. |

## Setup

The server requires Python 3.10+. The repo's ESP-IDF install ships
Python 3.11.2 — easiest is to use it directly.

```powershell
# From the repo root, in a shell with ESP-IDF already activated.
cd tools\mcp_server
& 'C:\Espressif\tools\idf-python\3.11.2\python.exe' -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Verify the server starts and lists tools (no robot needed):

```powershell
.\.venv\Scripts\python.exe smoke_test.py
# expect 5 tools listed
```

## Configuration

Environment variables (all optional):

| Var | Default | Purpose |
|---|---|---|
| `BALA2_IP`        | `192.168.8.138`     | Robot IP or hostname. |
| `BALA2_TIMEOUT`   | `5`                 | Default request timeout (seconds). `arm()` always uses 15 s. |
| `BALA2_LOG_LEVEL` | `INFO`              | One of DEBUG/INFO/WARNING/ERROR. Logs go to stderr. |

## Client configuration

### Claude Desktop

Edit `%APPDATA%\Claude\claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "bala2": {
      "command": "C:\\Repos\\bala2\\tools\\mcp_server\\.venv\\Scripts\\python.exe",
      "args": ["C:\\Repos\\bala2\\tools\\mcp_server\\bala2_mcp.py"],
      "env": { "BALA2_IP": "192.168.8.138" }
    }
  }
}
```

Restart Claude Desktop. The five tools should appear in the tool
inspector.

### GitHub Copilot CLI

Add to `~/.copilot/mcp_config.json` (or per-workspace `.copilot/`):

```json
{
  "mcpServers": {
    "bala2": {
      "command": "C:\\Repos\\bala2\\tools\\mcp_server\\.venv\\Scripts\\python.exe",
      "args": ["C:\\Repos\\bala2\\tools\\mcp_server\\bala2_mcp.py"],
      "env": { "BALA2_IP": "192.168.8.138" }
    }
  }
}
```

### Cursor

`.cursor/mcp.json` in the repo root, same shape as the Claude Desktop
config above.

## Verifying against a live robot

With the robot powered on and joined to your Wi-Fi:

```powershell
$env:BALA2_IP = '192.168.8.138'
.\.venv\Scripts\python.exe -c @"
import asyncio, json, os, sys
sys.path.insert(0, '.')
import bala2_mcp as m
print(m._get('/info'))
print(m._get('/telemetry'))
"@
```

Or just curl the endpoints directly:

```powershell
Invoke-RestMethod http://192.168.8.138/info
Invoke-RestMethod http://192.168.8.138/telemetry
Invoke-RestMethod http://192.168.8.138/setpoint -Method Post `
    -ContentType 'application/json' -Body '{"deg":1.0}'
```

If those work, the MCP server will work — it's a thin wrapper.

## Safety notes

- **No auth.** Anyone on the same Wi-Fi LAN can arm the robot. Fine
  for a hackathon / private network; do not expose port 80 publicly.
- **`arm()` rate-limit** is enforced by the firmware state machine
  (transitions take ~300 ms), but the MCP server itself does not
  rate-limit. An LLM stuck in a loop can spam `disarm()`/`arm()`.
- **`disarm()` is the only fire-and-forget tool.** All others wait for
  a JSON reply from the bot — if Wi-Fi is flaky, calls will time out.

## Troubleshooting

- **`could not connect to http://...`** — robot is off, on a different
  network, or has a different DHCP lease. Check the TFT for the
  current IP, or sniff DHCP / arp.
- **`HTTP 409: balancer is ARMED`** — only happens on `/update`.
  Disarm first.
- **Tools list is empty in Claude Desktop** — the server is crashing
  before tool registration. Run `smoke_test.py` from PowerShell; if
  that prints 5 tools, the issue is in the client config (path
  escaping is the usual culprit).
- **JSON-RPC parse errors in the client log** — something is printing
  to stdout. The server uses stderr for logging; if you've added a
  module that prints on import, capture or redirect it before
  `mcp.run()`.
