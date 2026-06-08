#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def main():
    ref = REPO / "configs/parts/susp_kinematics/kin/dw_front_sports.yaml"
    csv = REPO / "tools/kinematics/examples/adams_dw_sample.csv"
    script = REPO / "tools/kinematics/adams_xcheck.py"
    r = subprocess.run(
        [sys.executable, str(script), "--csv", str(csv),
         "--reference", str(ref), "--wheel-radius", "0.33"],
        cwd=str(REPO), capture_output=True, text=True,
    )
    print(r.stdout, end="")
    if r.returncode != 0:
        print(r.stderr, file=sys.stderr)
        sys.exit(r.returncode)
    assert "PASS" in r.stdout


if __name__ == "__main__":
    main()
