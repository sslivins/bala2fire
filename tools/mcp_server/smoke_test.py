"""Smoke test: spawn bala2_mcp, send `initialize` + `tools/list` over
stdio, print the tool surface. Doesn't require a live robot."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
SERVER = ROOT / "bala2_mcp.py"


def frame(obj: dict) -> bytes:
    return (json.dumps(obj) + "\n").encode("utf-8")


def main() -> int:
    env = os.environ.copy()
    env["BALA2_LOG_LEVEL"] = "WARNING"  # quiet for the test

    proc = subprocess.Popen(
        [str(PY), str(SERVER)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
        bufsize=0,
    )

    init = {
        "jsonrpc": "2.0", "id": 1, "method": "initialize",
        "params": {
            "protocolVersion": "2024-11-05",
            "capabilities": {},
            "clientInfo": {"name": "smoke-test", "version": "0.1"},
        },
    }
    initd = {"jsonrpc": "2.0", "method": "notifications/initialized"}
    listt = {"jsonrpc": "2.0", "id": 2, "method": "tools/list"}

    proc.stdin.write(frame(init))
    proc.stdin.write(frame(initd))
    proc.stdin.write(frame(listt))
    proc.stdin.flush()

    seen = []
    try:
        while len(seen) < 2:
            line = proc.stdout.readline()
            if not line:
                break
            try:
                msg = json.loads(line.decode("utf-8"))
            except Exception:
                continue
            if "id" in msg:
                seen.append(msg)
    finally:
        proc.stdin.close()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    if len(seen) < 2:
        print("FAIL: only got", len(seen), "responses", file=sys.stderr)
        print("stderr:", proc.stderr.read().decode("utf-8", "replace"),
              file=sys.stderr)
        return 1

    init_resp, list_resp = seen
    server_info = init_resp.get("result", {}).get("serverInfo", {})
    print(f"server: {server_info.get('name')} v{server_info.get('version')}")

    tools = list_resp.get("result", {}).get("tools", [])
    print(f"tools ({len(tools)}):")
    for t in tools:
        desc = (t.get("description") or "").split("\n", 1)[0]
        print(f"  - {t['name']:<16} {desc}")

    return 0 if tools else 2


if __name__ == "__main__":
    sys.exit(main())
