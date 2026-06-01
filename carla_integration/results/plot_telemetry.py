"""Plot CARLA + VDSim telemetry from telemetry.csv."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    cols = {}
    with open(path) as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            for k, v in row.items():
                cols.setdefault(k, []).append(float(v))
    return cols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    d = load(args.csv)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax1, ax2, ax3, ax4 = axes.flatten()

    # XY trajectory (CARLA world)
    ax1.plot(d["x_world"], d["y_world"], lw=1.2)
    ax1.scatter(d["x_world"][0], d["y_world"][0], c='g', s=60, label='start', zorder=3)
    ax1.scatter(d["x_world"][-1], d["y_world"][-1], c='r', s=60, label='end', zorder=3)
    ax1.set_xlabel("x_world [m]"); ax1.set_ylabel("y_world [m]")
    ax1.set_title("Ego XY trajectory in CARLA world")
    ax1.axis('equal'); ax1.legend(); ax1.grid(True, alpha=0.3)

    # vx and steer
    ax2b = ax2.twinx()
    ax2.plot(d["t"], d["vx"], 'b-', label='vx [m/s]')
    ax2b.plot(d["t"], d["steer"], 'r-', alpha=0.7, label='steer [rad]')
    ax2.set_xlabel("t [s]"); ax2.set_ylabel("vx [m/s]", color='b')
    ax2b.set_ylabel("steer [rad]", color='r')
    ax2.set_title("Speed (PI to 10 m/s) + Pure-Pursuit steer")
    ax2.grid(True, alpha=0.3)

    # yaw rate + lateral accel
    ax3b = ax3.twinx()
    ax3.plot(d["t"], d["yaw_rate"], 'g-', label='yaw rate [rad/s]')
    ax3b.plot(d["t"], d["ay"], 'm-', alpha=0.7, label='ay [m/s²]')
    ax3.set_xlabel("t [s]"); ax3.set_ylabel("yaw rate [rad/s]", color='g')
    ax3b.set_ylabel("ay [m/s²]", color='m')
    ax3.set_title("Yaw rate + lateral accel (figure-8)")
    ax3.grid(True, alpha=0.3)

    # Tire Fz (lateral load transfer)
    ax4.plot(d["t"], d["Fz_FL"], label='FL')
    ax4.plot(d["t"], d["Fz_FR"], label='FR')
    ax4.plot(d["t"], d["Fz_RL"], label='RL')
    ax4.plot(d["t"], d["Fz_RR"], label='RR')
    ax4.set_xlabel("t [s]"); ax4.set_ylabel("Fz [N]")
    ax4.set_title("Per-tire Fz — lateral transfer in turns")
    ax4.legend(loc='upper right'); ax4.grid(True, alpha=0.3)

    fig.suptitle("VDSim Ld2-SevenDOF driving CARLA ego (Town10HD, figure-8)",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"saved -> {args.out}")


if __name__ == "__main__":
    main()
