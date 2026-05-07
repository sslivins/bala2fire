"""Tiny synchronous client for the Bala2 Fire on-device MCP server.

The robot exposes JSON-RPC 2.0 over HTTP at  POST http://<ip>:8080/mcp .
This module wraps each tool as a plain Python method, suitable for use
from a tight control loop. No external deps -- stdlib only.

Example::

    from bala2_client import Bala2
    bot = Bala2("192.168.8.138")
    print(bot.get_status())
    bot.arm()
    while True:
        bot.drive(linear=0.2, angular=0.0)   # resend at >2 Hz
        time.sleep(0.1)
"""

from __future__ import annotations

import itertools
import json
import urllib.request
import urllib.error
from typing import Any, Dict, Optional


class Bala2Error(RuntimeError):
    pass


class Bala2:
    def __init__(self, host: str, port: int = 8080, timeout: float = 1.0):
        self.url = f"http://{host}:{port}/mcp"
        self.timeout = timeout
        self._ids = itertools.count(1)

    # ---- low-level JSON-RPC ----

    def _rpc(self, method: str, params: Optional[Dict[str, Any]] = None) -> Any:
        payload: Dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": next(self._ids),
            "method": method,
        }
        if params is not None:
            payload["params"] = params
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read()
        except urllib.error.URLError as e:
            raise Bala2Error(f"transport: {e}") from e
        try:
            msg = json.loads(raw)
        except ValueError as e:
            raise Bala2Error(f"bad JSON: {raw!r}") from e
        if "error" in msg:
            err = msg["error"]
            raise Bala2Error(f"rpc {err.get('code')}: {err.get('message')}")
        return msg.get("result")

    def _call_tool(self, name: str, args: Optional[Dict[str, Any]] = None) -> Any:
        result = self._rpc("tools/call", {"name": name, "arguments": args or {}})
        # Tools return {"content":[{"type":"text","text":"..."}], "isError":false}.
        # Try to parse the text as JSON; fall back to the raw string.
        try:
            text = result["content"][0]["text"]
        except (KeyError, IndexError, TypeError):
            return result
        try:
            return json.loads(text)
        except ValueError:
            return text

    # ---- handshake (optional but cheap) ----

    def initialize(self) -> Dict[str, Any]:
        return self._rpc(
            "initialize",
            {
                "protocolVersion": "2025-03-26",
                "capabilities": {},
                "clientInfo": {"name": "bala2_client.py", "version": "0.1.0"},
            },
        )

    def list_tools(self) -> Any:
        return self._rpc("tools/list")

    # ---- one method per exposed tool ----

    def arm(self) -> str:
        return self._call_tool("arm")

    def disarm(self) -> str:
        return self._call_tool("disarm")

    def is_armed(self) -> bool:
        r = self._call_tool("is_armed")
        if isinstance(r, dict) and "armed" in r:
            return bool(r["armed"])
        return "true" in str(r).lower()

    def get_status(self) -> Dict[str, Any]:
        r = self._call_tool("get_status")
        if not isinstance(r, dict):
            raise Bala2Error(f"unexpected get_status payload: {r!r}")
        return r

    def drive(self, linear: float, angular: float) -> str:
        return self._call_tool(
            "drive", {"linear": float(linear), "angular": float(angular)}
        )

    # ---- host-side PID support ----

    def get_imu(self) -> Dict[str, Any]:
        """Rich IMU snapshot: angle_deg, gyro_dps, accel_xyz, gyro_xyz,
        gyro_bias_dps, loop_dt_ms, loops, state."""
        r = self._call_tool("get_imu")
        if not isinstance(r, dict):
            raise Bala2Error(f"unexpected get_imu payload: {r!r}")
        return r

    def set_external(self, enabled: bool) -> Dict[str, Any]:
        """Enter/leave EXTERNAL mode. Enabling requires DISARMED state.
        Returns dict with 'ok' and possibly 'error'."""
        r = self._call_tool("set_external", {"enabled": bool(enabled)})
        if isinstance(r, dict):
            return r
        return {"ok": False, "raw": r}

    def set_pwm(self, left: int, right: int) -> Dict[str, Any]:
        """Raw PWM in EXTERNAL mode. Watchdog ~200 ms; resend at >5 Hz."""
        r = self._call_tool("set_pwm", {"left": int(left), "right": int(right)})
        if isinstance(r, dict):
            return r
        return {"ok": False, "raw": r}


if __name__ == "__main__":
    # Smoke test:  python bala2_client.py [host]
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.8.138"
    bot = Bala2(host)
    print("initialize:", bot.initialize())
    print("tools:", [t["name"] for t in bot.list_tools()["tools"]])
    print("status:", bot.get_status())
