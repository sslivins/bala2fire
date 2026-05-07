"""Live test: drive the MCP server's tools against the actual bot.
Reads from BALA2_IP env var (default 192.168.8.138)."""
from __future__ import annotations
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bala2_mcp as m  # noqa

print(f"robot: {m.BASE_URL}")
print("\n[firmware_info]")
print(m.firmware_info())
print("\n[get_status]")
print(m.get_status())
print("\n[set_setpoint(0.7)]")
print(m.set_setpoint(0.7))
print("\n[get_status after setpoint]")
s = m.get_status()
print(f"  setpoint_deg={s.get('setpoint_deg')} state={s.get('state')}")
print("\n[set_setpoint(0)]")
print(m.set_setpoint(0))
print("\n[disarm]")
print(m.disarm())
print("\nALL TOOLS OK")
