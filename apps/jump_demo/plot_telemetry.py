"""Plot L5 spatial-strut telemetry (C++ core) — suspension travel, tire Fz,
accelerometer. Labels in English (Korean font breaks in mpl). Wheel order
FL=0, FR=1, RL=2, RR=3.

    python3 plot_telemetry.py --mode loop --csv /tmp/strut_loop.csv --out /tmp/loop_tel.png
    python3 plot_telemetry.py --mode jump --csv /tmp/strut_jump.csv --out /tmp/jump_tel.png
"""
import argparse
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.80665
M = 1500.0          # default total vehicle mass [kg]
WG = M * G          # vehicle weight [N]
WHEELS = ["FL", "FR", "RL", "RR"]
WC = ["#005195", "#01A0E9", "#DC291E", "#f0a000"]


def load(path):
    cols = None
    data = {}
    with open(path) as f:
        r = csv.reader(f)
        cols = next(r)
        for c in cols:
            data[c] = []
        for line in r:
            for c, v in zip(cols, line):
                data[c].append(float(v))
    return data


def loop_angle_deg(d):
    """Unwrapped loop angle [deg]: 0 at bottom, 90 at 3 o'clock, 180 at top."""
    th = [math.atan2(x - 50.0, -(z - 15.0)) for x, z in zip(d["x"], d["z"])]
    out = []
    acc = 0.0
    prev = None
    for v in th:
        if prev is not None:
            dlt = v - prev
            if dlt > math.pi:
                dlt -= 2 * math.pi
            if dlt < -math.pi:
                dlt += 2 * math.pi
            acc += dlt
        else:
            acc = v
        out.append(math.degrees(acc))
        prev = v
    return out


def plot_loop(d, out):
    th = loop_angle_deg(d)
    # first lap only (0 -> 360 deg)
    hi = len(th)
    for i, v in enumerate(th):
        if v > 360.0:
            hi = i
            break
    sl = slice(0, hi)
    x = th[sl]

    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    fig.suptitle("L5 spatial-strut — vertical loop (1.15 v_crit, R=10 m), first lap",
                 fontsize=12)

    for i, w in enumerate(WHEELS):
        ax[0].plot(x, [1000.0 * c for c in d[f"comp_{w.lower()}"][sl]],
                   color=WC[i], label=w, lw=1.4)
    ax[0].axhline(100.0, color="#888", ls="--", lw=0.8, label="bump engage")
    ax[0].set_ylabel("susp. compression [mm]")
    ax[0].legend(ncol=5, fontsize=8, loc="lower center")
    ax[0].set_title("suspension works into the bump stop under loop load "
                    "(force element, not a wall)", fontsize=9)

    for i, w in enumerate(WHEELS):
        ax[1].plot(x, d[f"fz_{w.lower()}"][sl], color=WC[i], label=w, lw=1.4)
    ax[1].axhline(WG / 4.0, color="#888", ls=":", lw=0.8, label="static (W/4)")
    ax[1].set_ylabel("tire Fz [N]")
    ax[1].legend(ncol=5, fontsize=8)

    normal_g = [f / WG for f in d["fz_sum"][sl]]
    ax[2].plot(x, normal_g, color="#002060", lw=1.6, label="normal g (Fz_sum / W)")
    ax[2].plot(x, [a / G for a in d["ax"][sl]], color="#01A0E9", lw=1.2,
               label="longitudinal g (a_x)")
    ax[2].axhline(1.0, color="#888", ls=":", lw=0.8)
    ax[2].set_ylabel("specific force [g]")
    ax[2].set_xlabel("loop angle [deg]  (0 bottom, 90 = 3 o'clock, 180 top, 270 = 9 o'clock)")
    ax[2].legend(ncol=3, fontsize=8)

    for a in ax:
        for v in (0, 90, 180, 270, 360):
            a.axvline(v, color="#eee", lw=1.0, zorder=-1)
        a.grid(True, color="#f2f2f2")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=120)
    print("wrote", out)


def plot_jump(d, out):
    t = d["t"]
    airborne = [f < 30.0 for f in d["fz_sum"]]

    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    fig.suptitle("L5 spatial-strut — ramp jump (take-off, flight, landing)", fontsize=12)

    def shade(a):
        in_air = False
        t0 = 0.0
        for i, f in enumerate(airborne):
            if f and not in_air:
                t0 = t[i]; in_air = True
            elif not f and in_air:
                a.axvspan(t0, t[i], color="#eaf4fb", zorder=-2); in_air = False
        if in_air:
            a.axvspan(t0, t[-1], color="#eaf4fb", zorder=-2)

    ax[0].plot(t, d["z"], color="#002060", lw=1.6, label="CG height z [m]")
    ax[0].set_ylabel("CG height z [m]")
    shade(ax[0]); ax[0].legend(fontsize=8, loc="upper right")
    ax[0].set_title("shaded = airborne (Fz_sum < 30 N)", fontsize=9)

    for i, w in enumerate(WHEELS):
        ax[1].plot(t, [1000.0 * c for c in d[f"comp_{w.lower()}"]],
                   color=WC[i], label=w, lw=1.3)
    ax[1].axhline(120.0, color="#888", ls="--", lw=0.7)
    ax[1].axhline(-122.6, color="#888", ls="--", lw=0.7)
    ax[1].set_ylabel("susp. compression [mm]")
    ax[1].legend(ncol=4, fontsize=8); shade(ax[1])
    ax[1].set_title("ramp compression -> airborne droop -> landing spike -> settle",
                    fontsize=9)

    for i, w in enumerate(WHEELS):
        ax[2].plot(t, d[f"fz_{w.lower()}"], color=WC[i], label=w, lw=1.3)
    ax[2].set_ylabel("tire Fz [N]")
    ax[2].set_xlabel("time [s]")
    ax[2].legend(ncol=4, fontsize=8); shade(ax[2])

    for a in ax:
        a.grid(True, color="#f2f2f2")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=120)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["loop", "jump"], required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    d = load(args.csv)
    (plot_loop if args.mode == "loop" else plot_jump)(d, args.out)


if __name__ == "__main__":
    main()
