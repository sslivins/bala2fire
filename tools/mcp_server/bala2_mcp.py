"""MCP server for the Bala2 Fire balancing robot.

This server speaks JSON-RPC over stdio (the standard MCP transport for
locally-spawned servers like Claude Desktop / Copilot CLI / Cursor) and
exposes a small set of tools that wrap the HTTP control surface running
on the robot.

Endpoints expected on the robot (see components/bala2_ota/bala2_ota.c):

    GET  /info        -> {version, idf, slot, uptime_ms, armed}
    GET  /telemetry   -> {state, angle_deg, gyro_dps, accel_g, pwm_left,
                          pwm_right, enc_left, enc_right, loop_dt_ms,
                          setpoint_deg, loops, overruns, i2c_errs}
    POST /arm         -> telemetry snapshot after request
    POST /disarm      -> telemetry snapshot after request
    POST /setpoint    -> {"deg": <number>}; replies {"setpoint_deg",
                          "requested"}

Configuration:
    BALA2_IP          robot IP or hostname; default 192.168.8.138
    BALA2_TIMEOUT     default request timeout in seconds; default 5

Logs go to stderr; stdout is reserved for MCP JSON-RPC frames.
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Any

import httpx
from mcp.server.fastmcp import FastMCP


# ---- Logging ---------------------------------------------------------------
# stdout is reserved for the MCP protocol. All diagnostics go to stderr.
logging.basicConfig(
    stream=sys.stderr,
    level=os.environ.get("BALA2_LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s bala2_mcp: %(message)s",
)
log = logging.getLogger("bala2_mcp")


# ---- Configuration ---------------------------------------------------------
ROBOT_IP = os.environ.get("BALA2_IP", "192.168.8.138")
DEFAULT_TIMEOUT = float(os.environ.get("BALA2_TIMEOUT", "5"))
ARM_TIMEOUT = 15.0  # arming may wait for upright+still; allow more time

BASE_URL = f"http://{ROBOT_IP}"

log.info("Bala2 MCP server starting; robot=%s timeout=%.1fs",
         BASE_URL, DEFAULT_TIMEOUT)


# ---- HTTP helpers ----------------------------------------------------------

def _get(path: str, timeout: float = DEFAULT_TIMEOUT) -> Any:
    url = f"{BASE_URL}{path}"
    log.debug("GET %s", url)
    r = httpx.get(url, timeout=timeout)
    r.raise_for_status()
    ct = r.headers.get("content-type", "")
    return r.json() if "application/json" in ct else r.text


def _post(path: str, json: dict | None = None,
          timeout: float = DEFAULT_TIMEOUT) -> Any:
    url = f"{BASE_URL}{path}"
    log.debug("POST %s json=%s", url, json)
    r = httpx.post(url, json=json, timeout=timeout)
    r.raise_for_status()
    ct = r.headers.get("content-type", "")
    return r.json() if "application/json" in ct else r.text


def _format_error(action: str, exc: Exception) -> str:
    """Tools that hit the network return strings on failure rather than
    raising, so the LLM gets a readable error and can decide what to do
    next instead of crashing the tool call."""
    if isinstance(exc, httpx.TimeoutException):
        return (f"{action} timed out after {DEFAULT_TIMEOUT}s. The robot at "
                f"{ROBOT_IP} may be off, rebooting, or not on the network.")
    if isinstance(exc, httpx.ConnectError):
        return (f"could not connect to {BASE_URL}: {exc}. "
                "Check that the robot is powered on and joined to Wi-Fi.")
    if isinstance(exc, httpx.HTTPStatusError):
        return (f"{action} got HTTP {exc.response.status_code}: "
                f"{exc.response.text.strip()}")
    return f"{action} failed: {type(exc).__name__}: {exc}"


# ---- MCP server ------------------------------------------------------------

mcp = FastMCP("bala2")


@mcp.tool()
def get_status() -> dict | str:
    """Return a full telemetry snapshot from the robot.

    Fields include:
      state          one of BOOT/DISARMED/ARMING/ARMED/CRASHED/FAULT
      angle_deg      fused tilt angle (0 = upright)
      gyro_dps       tilt-axis gyro rate, deg/s
      accel_g        |accelerometer| in g (sanity check; ~1.0 = still)
      setpoint_deg   current angle setpoint (target tilt)
      pwm_left,
      pwm_right      most recent commanded PWM (-1023..+1023)
      enc_left,
      enc_right      raw 32-bit encoder counts (wrap mod 2^32)
      loop_dt_ms     measured period of the 200 Hz balance loop
      loops          monotonic loop counter
      overruns       count of loops that ran late
      i2c_errs       count of I2C faults observed

    Use this to inspect what the robot is doing right now; safe to call
    at any time, including DISARMED.
    """
    try:
        return _get("/telemetry")
    except Exception as e:
        return _format_error("get_status", e)


@mcp.tool()
def arm() -> dict | str:
    """Request the balancer to arm (engage motors).

    Transitions DISARMED -> ARMING. The bot must be roughly upright and
    still for ~300 ms before it will progress to ARMED. The reply is
    telemetry sampled immediately after the request, which often still
    shows state=ARMING; poll get_status() for a moment to see the final
    state.

    No-op if the bot is already ARMED. Refused (no-op) if state is BOOT.

    SAFETY: only arm with the bot physically prepared to move (on the
    floor, not on a desk edge). Use disarm() to stop at any time.
    """
    try:
        return _post("/arm", timeout=ARM_TIMEOUT)
    except Exception as e:
        return _format_error("arm", e)


@mcp.tool()
def disarm() -> dict | str:
    """Force the balancer to DISARMED state and zero the motors.

    Always succeeds (no preconditions). This is the e-stop.
    """
    try:
        return _post("/disarm")
    except Exception as e:
        return _format_error("disarm", e)


@mcp.tool()
def set_setpoint(deg: float) -> dict | str:
    """Set the angle setpoint (target tilt) in degrees.

    Slightly biasing the setpoint forward/backward causes the bot to
    drive that direction while staying balanced (the angle PID
    accelerates the wheels under the leaning chassis to keep it from
    falling). Typical useful range: -2.0 .. +2.0 degrees. The firmware
    clamps to [-30, +30] as a safety net.

    Args:
        deg: setpoint in degrees. 0 means perfectly upright.

    Returns:
        {"setpoint_deg": <applied>, "requested": <as-sent>}
    """
    try:
        return _post("/setpoint", json={"deg": float(deg)})
    except Exception as e:
        return _format_error("set_setpoint", e)


@mcp.tool()
def firmware_info() -> dict | str:
    """Return firmware identity: version (git short SHA), IDF version,
    running OTA slot (ota_0 or ota_1), uptime in milliseconds, and
    whether the balancer is armed.

    Useful for confirming an OTA upload took effect (slot or version
    should change) and for sanity-checking what build is running.
    """
    try:
        return _get("/info")
    except Exception as e:
        return _format_error("firmware_info", e)


# ---- Entrypoint ------------------------------------------------------------

def main() -> None:
    """Run the MCP server on stdio. Blocks until the client closes stdin."""
    log.info("listening on stdio (tools: get_status, arm, disarm, "
             "set_setpoint, firmware_info)")
    mcp.run()


if __name__ == "__main__":
    main()
