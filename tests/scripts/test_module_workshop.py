#!/usr/bin/env python3
"""MW6 — module workshop: build + contract-check + register a C++ subsystem module."""
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "python"))

from module_workshop import build_and_check, register  # noqa: E402
from catalog.part_store import delete_user_part         # noqa: E402


def main():
    # 1. The shipped brake template builds and conforms.
    ok = build_and_check("brake", str(REPO / "templates" / "modules"), "brake_module")
    assert ok["status"] == "pass", f"brake template should pass: {ok}"
    assert ok["kind"] == "brake"
    assert Path(ok["so"]).is_file()

    # 2. Wrong category is caught by the contract check (kind mismatch).
    mism = build_and_check("brake", str(REPO / "templates" / "modules"), "steering_module")
    assert mism["status"] == "fail"
    assert "kind mismatch" in mism.get("cause", ""), mism

    # 3. A non-compiling module fails with the compiler error as the cause.
    with tempfile.TemporaryDirectory() as td:
        bad = Path(td) / "bad.cpp"
        bad.write_text(
            '#include "vdsim/module_plugin.hpp"\n'
            "this is not valid c++\n"
            "VDSIM_REGISTER_BRAKE_MODULE(Nope, \"nope\")\n",
            encoding="utf-8",
        )
        res = build_and_check("brake", td, "bad")
        assert res["status"] == "fail"
        assert "build failed" in res.get("cause", ""), res

    # 4. Register the passed module as a catalog part, then clean up.
    info = register("brake", "ws_test_brake", ok["so"])
    try:
        assert info["status"] == "pass", info
        assert info["part_id"] == "module.ws_test_brake"
        assert Path(info["path"]).is_file()
    finally:
        delete_user_part(str(REPO), "module.ws_test_brake")

    print("test_module_workshop: ok")


if __name__ == "__main__":
    main()
