#!/usr/bin/env python3
"""Headless Playwright E2E for GUI v3 — starts gui/server.py on a free port."""
from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
V3 = ROOT / "gui" / "v3"
DIST = V3 / "dist" / "index.html"
PW_CLI = V3 / "node_modules" / "@playwright" / "test" / "cli.js"


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def wait_http(port: int, path: str = "/api/state", timeout: float = 45.0) -> bool:
    url = f"http://127.0.0.1:{port}{path}"
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            with urllib.request.urlopen(url, timeout=2) as resp:
                if resp.status == 200:
                    return True
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(0.25)
    return False


def ensure_dist() -> None:
    if DIST.is_file():
        return
    if not shutil.which("npm"):
        raise RuntimeError("npm missing — cannot build gui/v3/dist")
    subprocess.check_call(["npm", "run", "build"], cwd=V3)


def ensure_playwright_browser() -> None:
    if not PW_CLI.is_file():
        raise RuntimeError("run: cd gui/v3 && npm install")
    browsers = V3 / "node_modules" / "playwright-core" / ".local-browsers"
    if browsers.is_dir() and any(browsers.iterdir()):
        return
    subprocess.check_call(["npx", "playwright", "install", "chromium"], cwd=V3)


def py_path_env() -> dict:
    env = os.environ.copy()
    parts = [
        str(ROOT / "build" / "python"),
        str(ROOT / "python"),
        str(ROOT / "cosim"),
        str(ROOT / "gui"),
    ]
    env["PYTHONPATH"] = ":".join(parts + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
    return env


def main() -> int:
    ensure_dist()
    ensure_playwright_browser()

    port = free_port()
    env = py_path_env()
    server = subprocess.Popen(
        [sys.executable, str(ROOT / "gui" / "server.py"), "--port", str(port), "--host", "127.0.0.1"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        if not wait_http(port):
            err = server.stderr.read() if server.stderr else ""
            raise RuntimeError(f"GUI server failed on :{port}\n{err}")

        env["VDSIM_GUI_PORT"] = str(port)
        r = subprocess.run(
            ["npx", "playwright", "test", "--config=playwright.config.ts"],
            cwd=V3,
            env=env,
        )
        if r.returncode != 0:
            return r.returncode
        print("ok gui_v3_e2e")
        return 0
    finally:
        server.terminate()
        try:
            server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            server.kill()


if __name__ == "__main__":
    sys.exit(main())
