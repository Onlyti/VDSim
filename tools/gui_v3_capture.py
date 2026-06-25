#!/usr/bin/env python3
"""Headless GUI v3 screenshot capture (Playwright + ephemeral gui/server.py)."""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
V3 = REPO / "gui" / "v3"


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def wait_http(port: int, timeout: float = 45.0) -> None:
    url = f"http://127.0.0.1:{port}/api/state"
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            with urllib.request.urlopen(url, timeout=2) as resp:
                if resp.status == 200:
                    return
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(0.25)
    raise RuntimeError(f"GUI server did not start on port {port}")


def py_env() -> dict:
    env = os.environ.copy()
    parts = [
        str(REPO / "build" / "python"),
        str(REPO / "python"),
        str(REPO / "cosim"),
        str(REPO / "gui"),
    ]
    env["PYTHONPATH"] = ":".join(parts + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
    return env


def main() -> int:
    dist = V3 / "dist" / "index.html"
    if not dist.is_file():
        subprocess.check_call(["npm", "run", "build"], cwd=V3)

    port = free_port()
    env = py_env()
    server = subprocess.Popen(
        [sys.executable, str(REPO / "gui" / "server.py"), "--port", str(port), "--host", "127.0.0.1"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        wait_http(port)
        env["VDSIM_GUI_PORT"] = str(port)
        r = subprocess.run(
            ["npx", "playwright", "test", "e2e/capture-review.spec.ts", "--config=playwright.config.ts"],
            cwd=V3,
            env=env,
        )
        out = REPO / "docs" / "evidence" / "review" / "gui-captures"
        if out.is_dir():
            print("captures:", ", ".join(p.name for p in sorted(out.glob("*.png"))))
        return r.returncode
    finally:
        server.terminate()
        try:
            server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            server.kill()


if __name__ == "__main__":
    sys.exit(main())
