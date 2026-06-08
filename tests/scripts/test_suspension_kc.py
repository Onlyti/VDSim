#!/usr/bin/env python3
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))

import vdsim  # noqa: E402


def main():
    path = REPO / "configs/parts/susp_kinematics/kin/mp_front_sedan.yaml"
    r = vdsim.run_kc_sweep(str(path))
    assert len(r.travel) == 41
    assert len(r.steer) == 17
    assert len(r.compliance_fy) == 9
    assert abs(r.compliance_fy[-1].compliance_toe_deg) > 0.01
    print("ok", path.stem, len(r.travel), "travel samples")


if __name__ == "__main__":
    main()
