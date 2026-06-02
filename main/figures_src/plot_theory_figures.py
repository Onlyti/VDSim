"""
Generate learning figures for the theory chapters.

Schematic figures (frames, geometry, free-body) are drawn directly.
Physics figures (tire curves, understeer) call the actual vdsim model so
the figure IS a verification of the implementation.

Output: docs/theory/figures/*.png   (referenced from docs/theory/*.md)
Run:    python3 docs/figures_src/plot_theory_figures.py
"""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle, Ellipse
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
OUT = REPO / "docs" / "theory" / "figures"
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(REPO / "build" / "python"))

try:
    import vdsim
    HAVE_VDSIM = True
except ImportError:
    HAVE_VDSIM = False
    print("[fig] vdsim not built — physics figures will be skipped")


def _save(fig, name):
    path = OUT / name
    fig.tight_layout()
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"[fig] {path.relative_to(REPO)}")


def arrow(ax, x0, y0, x1, y1, color="k", lw=1.8, ls="-"):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1),
                 arrowstyle="-|>", mutation_scale=14,
                 color=color, lw=lw, linestyle=ls))


# ---------------------------------------------------------------------------
# 01 — ISO 8855 frame + wheel index + slip angle
# ---------------------------------------------------------------------------
def fig_01_frames():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    # (a) body frame + wheel index (top view)
    ax1.set_title("(a) ISO 8855 body frame + wheel index (top view)")
    car = Rectangle((-1.2, -0.7), 2.4, 1.4, fill=False, ec="#333", lw=1.5)
    ax1.add_patch(car)
    arrow(ax1, 0, 0, 1.6, 0, color="#c62828")   # +x forward
    arrow(ax1, 0, 0, 0, 1.1, color="#1565c0")   # +y left
    ax1.text(1.65, 0, "+x (forward)", color="#c62828", va="center", fontsize=10)
    ax1.text(0, 1.18, "+y (left)", color="#1565c0", ha="center", fontsize=10)
    ax1.text(0.05, -0.18, "+z (up, out of page) ⊙", fontsize=9, color="#388e3c")
    # wheels FL=0 FR=1 RL=2 RR=3
    wheels = {"FL=0": (1.0, 0.7), "FR=1": (1.0, -0.7),
              "RL=2": (-1.0, 0.7), "RR=3": (-1.0, -0.7)}
    for lbl, (wx, wy) in wheels.items():
        ax1.add_patch(Rectangle((wx-0.18, wy-0.1), 0.36, 0.2,
                       fc="#212121"))
        ax1.text(wx, wy + (0.28 if wy > 0 else -0.34), lbl,
                 ha="center", fontsize=9, fontweight="bold")
    ax1.set_xlim(-2, 2.6); ax1.set_ylim(-1.4, 1.6)
    ax1.set_aspect("equal"); ax1.axis("off")

    # (b) slip angle definition
    ax2.set_title("(b) slip angle α (ISO 8855 sign)")
    # wheel heading along +x; velocity vector tilted by alpha
    arrow(ax2, 0, 0, 1.4, 0, color="#333")          # wheel forward (x_wheel)
    alpha = math.radians(-18)   # negative alpha (left turn)
    vlen = 1.5
    arrow(ax2, 0, 0, vlen*math.cos(alpha), vlen*math.sin(alpha), color="#1565c0")
    ax2.text(1.45, 0.0, "wheel heading (x_wheel)", fontsize=9, va="center")
    ax2.text(vlen*math.cos(alpha)+0.05, vlen*math.sin(alpha),
             "velocity v", color="#1565c0", fontsize=9, va="center")
    # alpha arc
    th = np.linspace(alpha, 0, 30)
    ax2.plot(0.55*np.cos(th), 0.55*np.sin(th), "k-", lw=1)
    ax2.text(0.62*math.cos(alpha/2), 0.62*math.sin(alpha/2),
             "α<0", fontsize=10, color="#c62828")
    ax2.text(0, -0.95,
             "α = atan2(v_y_wheel, v_x_wheel)\nFy = −D·sin(...) → left turn α<0, Fy>0",
             fontsize=8.5, ha="center", family="monospace")
    ax2.set_xlim(-0.4, 2.0); ax2.set_ylim(-1.1, 0.7)
    ax2.set_aspect("equal"); ax2.axis("off")
    _save(fig, "01_frames.png")


# ---------------------------------------------------------------------------
# 02 — Coriolis term in body frame
# ---------------------------------------------------------------------------
def fig_02_coriolis():
    fig, ax = plt.subplots(figsize=(6.5, 5))
    ax.set_title("Body-frame acceleration: a = v̇_body + ω × v_body")
    # body velocity
    arrow(ax, 0, 0, 1.6, 0.4, color="#1565c0")
    ax.text(1.65, 0.42, "v_body", color="#1565c0", fontsize=10)
    # yaw rate (curved)
    th = np.linspace(0.3, 2.6, 40)
    ax.plot(0.5*np.cos(th), 0.5*np.sin(th), "#388e3c", lw=1.8)
    arrow(ax, 0.5*math.cos(2.6), 0.5*math.sin(2.6),
          0.5*math.cos(2.75), 0.5*math.sin(2.75), color="#388e3c")
    ax.text(-0.55, 0.6, "ω = r (yaw rate)", color="#388e3c", fontsize=10)
    # omega x v (perpendicular, centripetal)
    arrow(ax, 1.6, 0.4, 1.6-0.45, 0.4+1.0, color="#c62828", ls="--")
    ax.text(1.0, 1.35, "ω × v_body\n(Coriolis / centripetal)",
            color="#c62828", fontsize=9.5)
    ax.text(0, -1.3,
            "IMU measures v̇_body + ω×v_body, NOT just v̇_body.\n"
            "ω×v = (−r·vy, r·vx, 0)  for planar (p=q=0).",
            fontsize=9, ha="center", family="monospace")
    ax.set_xlim(-1.2, 2.4); ax.set_ylim(-1.7, 1.8)
    ax.set_aspect("equal"); ax.axis("off")
    _save(fig, "02_coriolis.png")


# ---------------------------------------------------------------------------
# 03 — Pacejka tire (real model): Fy-alpha + friction ellipse
# ---------------------------------------------------------------------------
def fig_03_tire():
    if not HAVE_VDSIM:
        return
    tp = vdsim.TireParams.from_yaml(
        str(REPO / "configs/tires/default_pacejka.yaml"))
    tire = vdsim.create_pacejka_mf96()
    tire.initialize(tp)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

    # (a) Fy vs alpha at several Fz
    alphas = np.linspace(-0.30, 0.30, 200)
    for Fz in (2000, 4000, 6000, 8000):
        Fy = []
        for a in alphas:
            inp = vdsim.TireInput()
            inp.Fz = float(Fz); inp.kappa = 0.0; inp.alpha = float(a)
            inp.mu_long = 1.0; inp.mu_lat = 1.0
            Fy.append(tire.compute(inp).Fy)
        ax1.plot(np.degrees(alphas), np.array(Fy)/1000.0,
                 label=f"Fz={Fz} N")
    ax1.set_xlabel("slip angle α [deg]")
    ax1.set_ylabel("lateral force Fy [kN]")
    ax1.set_title("(a) Pacejka MF96 — Fy vs α (load sensitivity)")
    ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)
    ax1.axhline(0, color="#999", lw=0.5); ax1.axvline(0, color="#999", lw=0.5)

    # (b) friction ellipse — combined Fx,Fy sweep at Fz=4000
    Fz = 4000.0
    pts_x, pts_y = [], []
    for kappa in np.linspace(-0.3, 0.3, 25):
        for a in np.linspace(-0.3, 0.3, 25):
            inp = vdsim.TireInput()
            inp.Fz = Fz; inp.kappa = float(kappa); inp.alpha = float(a)
            inp.mu_long = 1.0; inp.mu_lat = 1.0
            o = tire.compute(inp)
            pts_x.append(o.Fx/1000.0); pts_y.append(o.Fy/1000.0)
    ax2.scatter(pts_x, pts_y, s=4, alpha=0.4, color="#1565c0")
    # bounding ellipse Fx_max, Fy_max
    fxm = tp.D_long * Fz * 1.0 / 1000.0
    fym = tp.D_lat  * Fz * 1.0 / 1000.0
    ax2.add_patch(Ellipse((0, 0), 2*fxm, 2*fym, fill=False,
                           ec="#c62828", lw=1.5, ls="--"))
    ax2.set_xlabel("Fx [kN]"); ax2.set_ylabel("Fy [kN]")
    ax2.set_title("(b) friction ellipse (combined slip, Fz=4 kN)")
    ax2.grid(True, alpha=0.3); ax2.set_aspect("equal")
    _save(fig, "03_tire.png")


# ---------------------------------------------------------------------------
# 04a — single-track geometry
# ---------------------------------------------------------------------------
def fig_04_bicycle_geom():
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.set_title("Single-track (bicycle) geometry")
    a, b = 1.2, 1.5
    # body line
    ax.plot([-b, a], [0, 0], "k-", lw=1.2)
    ax.plot(0, 0, "ko", ms=6); ax.text(0, -0.22, "CG", ha="center", fontsize=9)
    # rear wheel
    ax.add_patch(Rectangle((-b-0.18, -0.12), 0.36, 0.24, fc="#212121"))
    ax.text(-b, -0.4, "rear", ha="center", fontsize=9)
    # front wheel steered
    d = math.radians(20)
    fw = 0.28
    ax.add_patch(FancyArrowPatch((a-fw*math.cos(d), -fw*math.sin(d)),
                 (a+fw*math.cos(d), fw*math.sin(d)),
                 arrowstyle="-", color="#212121", lw=6))
    ax.text(a, -0.4, "front (δ)", ha="center", fontsize=9)
    # dims
    ax.annotate("", (0,0.35),(-b,0.35), arrowprops=dict(arrowstyle="<->"))
    ax.text(-b/2, 0.45, "b", ha="center", fontsize=10)
    ax.annotate("", (a,0.35),(0,0.35), arrowprops=dict(arrowstyle="<->"))
    ax.text(a/2, 0.45, "a", ha="center", fontsize=10)
    ax.text(a/2-0.3, 0.75, "L = a + b (wheelbase)", fontsize=9)
    # slip angle markers
    ax.text(-b-0.05, 0.18, "α_r", color="#c62828", fontsize=10)
    ax.text(a-0.05, 0.55, "α_f", color="#c62828", fontsize=10)
    ax.set_xlim(-b-0.7, a+0.9); ax.set_ylim(-0.7, 1.0)
    ax.set_aspect("equal"); ax.axis("off")
    _save(fig, "04_bicycle_geom.png")


# ---------------------------------------------------------------------------
# 04b — understeer: yaw-rate gain vs speed (real model sweep)
# ---------------------------------------------------------------------------
def fig_04_understeer():
    if not HAVE_VDSIM:
        return
    vp = vdsim.VehicleParams.from_yaml(str(REPO/"configs/vehicles/sports.yaml"))
    tp = vdsim.TireParams.from_yaml(str(REPO/"configs/tires/default_pacejka.yaml"))
    sp = vdsim.SolverParams()

    delta = 0.03
    speeds = np.linspace(5, 40, 12)
    r_gain, r_neutral = [], []
    for v in speeds:
        dyn = vdsim.create_seven_dof(); dyn.initialize(vp, tp, sp)
        s0 = vdsim.State(); s0.velocity = [float(v), 0, 0]
        w = v / vp.wheel_radius_nominal
        s0.wheel_spin = [w, w, w, w]; dyn.reset(s0)
        c = [vdsim.ContactPoint() for _ in range(4)]
        for p in c: p.is_valid=True; p.normal=[0,0,1]; p.mu_long=p.mu_lat=1.0
        cmd = vdsim.CmdL4(); cmd.steer_angle_wheel = delta
        for _ in range(int(6.0/0.005)): dyn.step(cmd, c, 0.005)
        r_gain.append(dyn.state().yaw_rate() / delta)
        r_neutral.append(v / vp.wheelbase)   # neutral-steer Ackermann

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(speeds, r_gain, "o-", color="#1565c0", label="VDSim L2 (sports)")
    ax.plot(speeds, r_neutral, "--", color="#888",
            label="neutral steer  v/L")
    ax.set_xlabel("speed vx [m/s]")
    ax.set_ylabel("yaw-rate gain  r/δ  [1/s]")
    ax.set_title("Understeer: yaw-rate gain vs speed\n"
                 "(below neutral line ⇒ understeer)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    _save(fig, "04_understeer.png")


# ---------------------------------------------------------------------------
# 05 — weight transfer (4 wheel Fz) schematic
# ---------------------------------------------------------------------------
def fig_05_weight_transfer():
    fig, ax = plt.subplots(figsize=(6.5, 5))
    ax.set_title("Weight transfer in a left turn (ay > 0)\n"
                 "outer (right) wheels loaded")
    car = Rectangle((-1.0, -0.6), 2.0, 1.2, fill=False, ec="#333", lw=1.5)
    ax.add_patch(car)
    arrow(ax, 0, 0, 0, 1.0, color="#1565c0")
    ax.text(0.05, 1.05, "ay (centripetal, +y)", color="#1565c0", fontsize=9)
    # wheel circles sized by Fz (right side bigger)
    data = {"FL": (0.85, 0.6, 0.16), "FR": (0.85, -0.6, 0.28),
            "RL": (-0.85, 0.6, 0.18), "RR": (-0.85, -0.6, 0.26)}
    for lbl, (wx, wy, rad) in data.items():
        ax.add_patch(Circle((wx, wy), rad, fc="#90caf9", ec="#1565c0"))
        ax.text(wx, wy, lbl, ha="center", va="center", fontsize=8,
                fontweight="bold")
    ax.text(0, -1.25, "circle size ∝ Fz\n"
            "ΔFz_lat = (m·ay·h_cg / Tw) · share",
            ha="center", fontsize=9, family="monospace")
    ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.7, 1.4)
    ax.set_aspect("equal"); ax.axis("off")
    _save(fig, "05_weight_transfer.png")


# ---------------------------------------------------------------------------
# 06 — quarter-car (sprung/unsprung spring-damper)
# ---------------------------------------------------------------------------
def fig_06_quarter_car():
    fig, ax = plt.subplots(figsize=(4.5, 6))
    ax.set_title("Quarter-car (Ld3 corner)")
    # sprung mass
    ax.add_patch(Rectangle((-0.6, 2.2), 1.2, 0.6, fc="#bbdefb", ec="#1565c0"))
    ax.text(0, 2.5, "sprung  m_s", ha="center", fontsize=9)
    # spring + damper sprung-unsprung
    ax.plot([-0.3,-0.3],[1.4,2.2], "k-", lw=1)  # spring side
    ax.text(-0.55, 1.8, "k_i", fontsize=9)
    ax.plot([0.3,0.3],[1.4,2.2], "k-", lw=1)
    ax.text(0.38, 1.8, "c_i", fontsize=9)
    # unsprung mass
    ax.add_patch(Rectangle((-0.45, 0.9), 0.9, 0.5, fc="#ffe0b2", ec="#e65100"))
    ax.text(0, 1.15, "unsprung  m_u", ha="center", fontsize=8)
    # tire spring
    ax.plot([0,0],[0.2,0.9], "k-", lw=1)
    ax.text(0.1, 0.5, "k_tire", fontsize=9)
    # ground
    ax.plot([-0.8,0.8],[0.2,0.2], "k-", lw=2)
    for gx in np.linspace(-0.7,0.7,8):
        ax.plot([gx,gx-0.1],[0.2,0.1],"k-",lw=0.7)
    ax.text(0, -0.15, "z_s, φ, θ  +  z_u (×4)\n= 14-DOF",
            ha="center", fontsize=9, family="monospace")
    ax.set_xlim(-1.0, 1.0); ax.set_ylim(-0.4, 3.0)
    ax.set_aspect("equal"); ax.axis("off")
    _save(fig, "06_quarter_car.png")


# ---------------------------------------------------------------------------
# 09 — pure pursuit lookahead geometry
# ---------------------------------------------------------------------------
def fig_09_pure_pursuit():
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Pure Pursuit lookahead geometry")
    # vehicle at origin, heading +x
    arrow(ax, 0, 0, 1.2, 0, color="#333")
    ax.plot(0, 0, "ks", ms=8); ax.text(0, -0.3, "rear axle", fontsize=9)
    # lookahead point
    Ld = 3.0; dyb = 1.3
    dxb = math.sqrt(Ld**2 - dyb**2)
    ax.plot(dxb, dyb, "o", color="#c62828", ms=8)
    ax.text(dxb+0.1, dyb, "lookahead pt", color="#c62828", fontsize=9)
    ax.plot([0, dxb],[0,dyb], "--", color="#888")
    ax.text(dxb/2-0.3, dyb/2+0.1, "Ld", fontsize=10)
    # arc through origin and lookahead
    R = Ld**2/(2*dyb)
    cx, cy = 0, R
    th = np.linspace(-0.5, 1.0, 50)
    ax.plot(cx + R*np.sin(th), cy - R*np.cos(th), "-", color="#1565c0", lw=1.5)
    ax.plot(cx, cy, "+", color="#1565c0", ms=10)
    ax.text(cx+0.1, cy, f"ICR (R={R:.1f})", color="#1565c0", fontsize=9)
    ax.text(0, -1.1, "κ = 2·dy_b / Ld² ,  δ = atan(κ·L)",
            ha="center", fontsize=10, family="monospace")
    ax.set_xlim(-0.5, 4.0); ax.set_ylim(-1.4, R+0.5)
    ax.set_aspect("equal"); ax.axis("off")
    _save(fig, "09_pure_pursuit.png")


def main():
    fig_01_frames()
    fig_02_coriolis()
    fig_03_tire()
    fig_04_bicycle_geom()
    fig_04_understeer()
    fig_05_weight_transfer()
    fig_06_quarter_car()
    fig_09_pure_pursuit()
    print("[fig] done")


if __name__ == "__main__":
    main()
