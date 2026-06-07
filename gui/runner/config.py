import os
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
GUI_RUN_DIR = REPO / "runs" / "live"


def gui_run_dir():
    raw = os.environ.get("VDSIM_RUN_DIR")
    d = Path(raw).expanduser() if raw else GUI_RUN_DIR
    if not d.is_absolute():
        d = REPO / d
    d.mkdir(parents=True, exist_ok=True)
    return d.resolve()


def prepare_gui_run_dir(d=None):
    d = d or gui_run_dir()
    for pat in ("vehicle_*.yaml", "tire_*.yaml", "world.yaml", "run_config.yaml",
                "sensors.yaml", "terrain.bin"):
        for f in d.glob(pat):
            f.unlink(missing_ok=True)
    return d
