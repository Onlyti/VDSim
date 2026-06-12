"""
Validation runner — runs the ISO maneuver suite on a given vehicle/tire
config and writes a markdown + figures report.

Usage:
    python3 apps/validation/run_validation.py \\
        --vehicle configs/parts/body/sedan.yaml \\
        --tire    configs/parts/tire/kinematic_fallback.yaml \\
        --level   L3 \\
        --out     apps/validation/results/sedan

Default tire is kinematic_fallback (legacy ISO baseline). Catalog default
tire.default_pacejka uses LuGre; pass --tire configs/parts/tire/default_pacejka.yaml
to re-baseline ISO under LuGre.
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import vdsim
from iso_7401 import run_iso_7401, format_report as fmt_7401
from iso_4138 import run_iso_4138, format_report as fmt_4138
from iso_3888 import run_iso_3888, format_report as fmt_3888, dlc_waypoints


def make_dyn(level: str):
    if level == "L1": return vdsim.create_bicycle()
    if level == "L3": return vdsim.create_fourteen_dof()
    return vdsim.create_seven_dof()


def plot_7401(r, out_png):
    traj = r.trajectory
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(traj["t"], np.degrees(traj["steer"]), "k-", lw=1.5)
    axes[0].set_ylabel("steer [deg]"); axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"ISO 7401 step-steer @ {r.v_target*3.6:.0f} km/h, "
                       f"δ = {r.steer_deg:+.1f}°")
    axes[1].plot(traj["t"], np.degrees(traj["r"]), "b-", lw=1.5)
    axes[1].axhline(np.degrees(r.psi_dot_ss), color="b", ls="--", alpha=0.5,
                    label=f"ψ̇_ss = {np.degrees(r.psi_dot_ss):.2f}°/s")
    axes[1].axhline(np.degrees(r.psi_dot_peak), color="r", ls=":", alpha=0.5,
                    label=f"ψ̇_peak = {np.degrees(r.psi_dot_peak):.2f}°/s")
    axes[1].set_ylabel("yaw rate [deg/s]"); axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="lower right", fontsize=8)
    axes[2].plot(traj["t"], traj["ay"], "g-", lw=1.5)
    axes[2].axhline(r.a_y_ss, color="g", ls="--", alpha=0.5,
                    label=f"a_y_ss = {r.a_y_ss:.2f} m/s²")
    axes[2].set_xlabel("t [s]"); axes[2].set_ylabel("a_y [m/s²]")
    axes[2].grid(True, alpha=0.3); axes[2].legend(loc="lower right", fontsize=8)
    fig.tight_layout(); fig.savefig(out_png, dpi=120); plt.close(fig)


def plot_3888(r, out_png):
    """Top view of vehicle path through DLC cone course + speed timeline."""
    traj = r.trajectory
    wp = dlc_waypoints()
    fig, axes = plt.subplots(2, 1, figsize=(11, 6),
                              gridspec_kw={"height_ratios": [2, 1]})
    ax = axes[0]
    # Cone gates (rough): draw lane corridors as fill
    ax.plot(wp[:, 0], wp[:, 1] + 1.0, "k--", lw=0.5, alpha=0.5)
    ax.plot(wp[:, 0], wp[:, 1] - 1.0, "k--", lw=0.5, alpha=0.5)
    ax.plot(wp[:, 0], wp[:, 1], "g:", lw=1.0, label="target center")
    ax.plot(traj["x"], traj["y"], "b-", lw=1.6, label="vehicle path")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"ISO 3888-2 DLC @ {r.v_entry_kmh:.0f} km/h — "
                  f"{'PASS' if r.success else 'FAIL'}")
    ax.legend(loc="upper right"); ax.grid(True, alpha=0.3); ax.set_aspect("equal")

    ax2 = axes[1]
    ax2.plot(traj["t"], traj["vx"] * 3.6, "b-", lw=1.5, label="vx [km/h]")
    ax2.axhline(r.v_entry_kmh, color="g", ls="--", alpha=0.5, label="entry")
    ax2b = ax2.twinx()
    ax2b.plot(traj["t"], np.degrees(traj["r"]), "r-", lw=1, alpha=0.7,
              label="yaw rate")
    ax2.set_xlabel("t [s]"); ax2.set_ylabel("speed [km/h]")
    ax2b.set_ylabel("yaw rate [°/s]", color="r")
    ax2.legend(loc="lower right", fontsize=8); ax2.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out_png, dpi=120); plt.close(fig)


def plot_4138(r, out_png, vehicle_params):
    traj = r.trajectory
    L = vehicle_params.wheelbase
    steer = np.array(traj["steer"]); vx = np.array(traj["vx"])
    r_arr = np.array(traj["r"]); ay = np.array(traj["ay"])
    delta_kin = L * r_arr / np.maximum(vx, 1e-3)
    delta_minus_kin = steer - delta_kin

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(np.degrees(steer), ay, "b-", lw=1.5)
    axes[0].set_xlabel("steer δ [deg]"); axes[0].set_ylabel("a_y [m/s²]")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"ISO 4138 ramp @ {r.v_target*3.6:.0f} km/h")
    axes[1].plot(ay, np.degrees(delta_minus_kin), "r-", lw=1.5, label="δ − δ_kin")
    # Linear fit line
    if abs(r.K_rad_per_m_per_s2) > 1e-9:
        xs = np.linspace(0, r.linear_range_ay_max, 100)
        ys = r.K_rad_per_m_per_s2 * xs
        axes[1].plot(xs, np.degrees(ys), "g--", lw=1,
                      label=f"fit K = {r.K_rad_per_g*1000:.2f} mrad/g  "
                            f"({r.handling_type})")
    axes[1].set_xlabel("a_y [m/s²]"); axes[1].set_ylabel("δ − δ_kin [deg]")
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)
    axes[1].set_title("understeer characteristic")
    fig.tight_layout(); fig.savefig(out_png, dpi=120); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/parts/body/sedan.yaml")
    ap.add_argument("--tire",    default="configs/parts/tire/kinematic_fallback.yaml")
    ap.add_argument("--level",   default="L2")
    ap.add_argument("--out",     default="apps/validation/results/run01")
    args = ap.parse_args()

    out_dir = REPO / args.out; out_dir.mkdir(parents=True, exist_ok=True)

    vp = vdsim.VehicleParams.from_yaml(str(REPO / args.vehicle))
    tp = vdsim.TireParams.from_yaml(str(REPO / args.tire))
    sp = vdsim.SolverParams()

    # Run all tests
    results = {}
    # ISO 7401 spec: choose steer to give a_y_ss ≈ 0.4 g (3.9 m/s²); ~5-10°
    # of road-wheel angle at 80 km/h works for typical passenger/sport.
    print("--- Running ISO 7401 step-steer (6°, 80 km/h) ---")
    dyn = make_dyn(args.level); dyn.initialize(vp, tp, sp)
    r7401 = run_iso_7401(dyn, vp, v_target_kmh=80.0, steer_deg=6.0,
                          t_post=6.0)
    print(fmt_7401(r7401))
    plot_7401(r7401, out_dir / "iso_7401_step.png")
    results["iso_7401"] = r7401

    print("--- Running ISO 4138 slow ramp (4° max, 80 km/h) ---")
    dyn = make_dyn(args.level); dyn.initialize(vp, tp, sp)
    r4138 = run_iso_4138(dyn, vp, v_target_kmh=80.0, steer_max_deg=4.0)
    print(fmt_4138(r4138))
    plot_4138(r4138, out_dir / "iso_4138_ramp.png", vp)
    results["iso_4138"] = r4138

    print("--- Running ISO 3888-2 Double Lane Change (60 km/h) ---")
    dyn = make_dyn(args.level); dyn.initialize(vp, tp, sp)
    r3888 = run_iso_3888(dyn, vp, v_entry_kmh=60.0)
    print(fmt_3888(r3888))
    plot_3888(r3888, out_dir / "iso_3888_dlc.png")
    results["iso_3888"] = r3888

    # Write markdown report
    report = out_dir / "REPORT.md"
    with open(report, "w") as f:
        f.write(f"# VDSim ISO maneuver validation report\n\n")
        f.write(f"- Vehicle config : `{args.vehicle}`\n")
        f.write(f"- Tire config    : `{args.tire}`\n")
        f.write(f"- Dynamics level : `{args.level}`\n")
        f.write(f"- Mass           : `{vp.mass:.0f} kg`\n")
        f.write(f"- Wheelbase      : `{vp.wheelbase:.3f} m`\n")
        f.write(f"\n## ISO 7401 — step-steer transient response\n\n")
        for line in fmt_7401(r7401).splitlines():
            f.write("    " + line + "\n")
        f.write(f"\n![iso_7401](iso_7401_step.png)\n\n")
        f.write(f"## ISO 4138 — steady-state circular driving\n\n")
        for line in fmt_4138(r4138).splitlines():
            f.write("    " + line + "\n")
        f.write(f"\n![iso_4138](iso_4138_ramp.png)\n\n")
        f.write(f"## ISO 3888-2 — Double Lane Change\n\n")
        for line in fmt_3888(r3888).splitlines():
            f.write("    " + line + "\n")
        f.write(f"\n![iso_3888](iso_3888_dlc.png)\n")
    print(f"[validate] report -> {report}")


if __name__ == "__main__":
    main()
