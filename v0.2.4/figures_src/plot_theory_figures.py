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


# ---------------------------------------------------------------------------
# helper: build a dynamics + flat contacts + initial state
# ---------------------------------------------------------------------------
def _make(level, vehicle="sports", v0=0.0):
    vp = vdsim.VehicleParams.from_yaml(str(REPO/f"configs/vehicles/{vehicle}.yaml"))
    tp = vdsim.TireParams.from_yaml(str(REPO/"configs/tires/default_pacejka.yaml"))
    sp = vdsim.SolverParams()
    dyn = {"L1": vdsim.create_bicycle, "L3": vdsim.create_fourteen_dof}.get(
        level, vdsim.create_seven_dof)()
    dyn.initialize(vp, tp, sp)
    s0 = vdsim.State(); s0.velocity = [float(v0), 0, 0]
    if v0 > 0:
        w = v0 / vp.wheel_radius_nominal
        s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)
    return dyn, vp, tp


def _contacts(mu=1.0):
    c = [vdsim.ContactPoint() for _ in range(4)]
    for p in c:
        p.is_valid = True; p.normal = [0, 0, 1]; p.mu_long = mu; p.mu_lat = mu
    return c


# ---------------------------------------------------------------------------
# 03b — tire Mz + Fx (real model, the other two outputs)
# ---------------------------------------------------------------------------
def fig_03b_tire_mz_fx():
    if not HAVE_VDSIM: return
    tp = vdsim.TireParams.from_yaml(str(REPO/"configs/tires/default_pacejka.yaml"))
    tire = vdsim.create_pacejka_mf96(); tire.initialize(tp)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    alphas = np.linspace(-0.30, 0.30, 200)
    Mz, Fy = [], []
    for a in alphas:
        inp = vdsim.TireInput(); inp.Fz=4000.0; inp.kappa=0.0; inp.alpha=float(a)
        inp.mu_long=1.0; inp.mu_lat=1.0
        o = tire.compute(inp); Mz.append(o.Mz); Fy.append(o.Fy)
    ax1.plot(np.degrees(alphas), Mz, color="#6a1b9a")
    ax1.set_xlabel("slip angle α [deg]"); ax1.set_ylabel("aligning moment Mz [N·m]")
    ax1.set_title("(a) self-aligning moment Mz (Fz=4 kN)")
    ax1.grid(True, alpha=0.3); ax1.axhline(0,color="#999",lw=0.5); ax1.axvline(0,color="#999",lw=0.5)
    # Fx vs kappa
    kaps = np.linspace(-0.3, 0.3, 200)
    for Fz in (2000, 4000, 6000):
        Fx=[]
        for k in kaps:
            inp=vdsim.TireInput(); inp.Fz=float(Fz); inp.kappa=float(k); inp.alpha=0.0
            inp.mu_long=1.0; inp.mu_lat=1.0
            Fx.append(tire.compute(inp).Fx/1000.0)
        ax2.plot(kaps, Fx, label=f"Fz={Fz} N")
    ax2.set_xlabel("slip ratio κ"); ax2.set_ylabel("longitudinal force Fx [kN]")
    ax2.set_title("(b) Fx vs κ (longitudinal)")
    ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
    ax2.axhline(0,color="#999",lw=0.5); ax2.axvline(0,color="#999",lw=0.5)
    _save(fig, "03b_tire_mz_fx.png")


# ---------------------------------------------------------------------------
# 04b — step steer yaw-rate time response (real)
# ---------------------------------------------------------------------------
def fig_04_step_response():
    if not HAVE_VDSIM: return
    fig, ax = plt.subplots(figsize=(7,4.5))
    dt=0.005
    for v0 in (10, 20, 30):
        dyn,_,_ = _make("L1", v0=v0); c=_contacts()
        cmd=vdsim.CmdL4()
        t=[]; r=[]
        tt=0.0
        for i in range(int(4.0/dt)):
            cmd.steer_angle_wheel = 0.0 if tt<0.5 else 0.04
            dyn.step(cmd,c,dt); tt+=dt
            t.append(tt); r.append(dyn.state().yaw_rate())
        ax.plot(t, np.degrees(r), label=f"vx={v0} m/s")
    ax.axvline(0.5, color="#999", ls="--", lw=0.8, label="step @ 0.5s")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw rate [deg/s]")
    ax.set_title("Step-steer transient (Ld1, δ=0.04 rad)\nhigher speed → larger SS yaw rate")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    _save(fig, "04_step_response.png")


# ---------------------------------------------------------------------------
# 05b — Ackermann inner/outer angle (real)
# ---------------------------------------------------------------------------
def fig_05_ackermann():
    if not HAVE_VDSIM: return
    fig, ax = plt.subplots(figsize=(7,4.5))
    steers = np.linspace(0.0, 0.45, 40)
    vp = vdsim.VehicleParams.from_yaml(str(REPO/"configs/vehicles/sports.yaml"))
    L, Tw = vp.wheelbase, vp.track_front
    for pct,lbl in [(0,"0% (parallel)"),(50,"50%"),(100,"100% (perfect)")]:
        din=[]; dout=[]
        for d in steers:
            if d<1e-6: din.append(0); dout.append(0); continue
            td=math.tan(d)
            d_in = math.atan(td/(1-td*Tw/2/L))
            d_out= math.atan(td/(1+td*Tw/2/L))
            f=pct/100
            din.append(math.degrees(d+f*(d_in-d)))
            dout.append(math.degrees(d+f*(d_out-d)))
        ln,=ax.plot(np.degrees(steers), din, label=f"inner {lbl}")
        ax.plot(np.degrees(steers), dout, "--", color=ln.get_color())
    ax.set_xlabel("average steer δ [deg]"); ax.set_ylabel("wheel angle [deg]")
    ax.set_title("Ackermann: inner (solid) vs outer (dashed)\nper interpolation %")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    _save(fig, "05_ackermann.png")


# ---------------------------------------------------------------------------
# 06b — roll transient + pitch under brake (real L3)
# ---------------------------------------------------------------------------
def fig_06_transient():
    if not HAVE_VDSIM: return
    fig,(ax1,ax2)=plt.subplots(1,2,figsize=(12,4.5))
    dt=0.005
    # roll: step steer
    dyn,_,_=_make("L3", v0=20); c=_contacts()
    cmd=vdsim.CmdL4(); t=[];roll=[]; tt=0
    for i in range(int(4.0/dt)):
        cmd.steer_angle_wheel = 0.0 if tt<0.5 else 0.06
        dyn.step(cmd,c,dt); tt+=dt
        t.append(tt); roll.append(math.degrees(dyn.roll_angle_qs()))
    ax1.plot(t, roll, color="#c62828")
    ax1.axvline(0.5,color="#999",ls="--",lw=0.8)
    ax1.set_xlabel("time [s]"); ax1.set_ylabel("roll [deg]")
    ax1.set_title("(a) roll transient — step steer (L3, 20 m/s)")
    ax1.grid(True, alpha=0.3)
    # pitch: brake
    dyn,_,_=_make("L3", v0=25); c=_contacts()
    cmd=vdsim.CmdL4(); t=[];pitch=[]; tt=0
    for i in range(int(3.0/dt)):
        cmd.brake = 0.0 if tt<0.5 else 0.6
        dyn.step(cmd,c,dt); tt+=dt
        t.append(tt); pitch.append(math.degrees(dyn.pitch_angle_qs()))
    ax2.plot(t,pitch,color="#1565c0")
    ax2.axvline(0.5,color="#999",ls="--",lw=0.8)
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("pitch [deg]")
    ax2.set_title("(b) pitch transient — brake (L3, 25 m/s)\nnose dive")
    ax2.grid(True, alpha=0.3)
    _save(fig, "06_transient.png")


# ---------------------------------------------------------------------------
# 08b — cascade PID vx tracking (real)
# ---------------------------------------------------------------------------
def fig_08_pid():
    if not HAVE_VDSIM: return
    try:
        vp=vdsim.VehicleParams.from_yaml(str(REPO/"configs/vehicles/sports.yaml"))
        tp=vdsim.TireParams.from_yaml(str(REPO/"configs/tires/default_pacejka.yaml"))
        sp=vdsim.SolverParams()
        dyn=vdsim.create_seven_dof(); dyn.initialize(vp,tp,sp)
        s0=vdsim.State(); s0.velocity=[5,0,0]
        w=5/vp.wheel_radius_nominal; s0.wheel_spin=[w,w,w,w]; dyn.reset(s0)
        vxc=vdsim.LongVxController(); vxc.initialize(vdsim.LongVxGains())
        axc=vdsim.LongAxController(); axc.initialize(vdsim.LongAxGains())
    except Exception as e:
        print(f"[fig] 08 skip: {e}"); return
    c=_contacts(); dt=0.01
    t=[];vx=[];tgt=[]
    tt=0
    for i in range(int(12/dt)):
        v_target = 10.0 if tt<6 else 18.0
        ax_t = vxc.update(v_target, dyn.state().vx(), dt)
        out = axc.update(ax_t, dyn.ax_body_est(), dt)
        cmd=vdsim.CmdL4()
        # out may be tuple or struct; handle both
        try: cmd.throttle, cmd.brake = out
        except TypeError: cmd.throttle=out.throttle; cmd.brake=out.brake
        dyn.step(cmd,c,dt); tt+=dt
        t.append(tt); vx.append(dyn.state().vx()); tgt.append(v_target)
    fig,ax=plt.subplots(figsize=(7,4.5))
    ax.plot(t,tgt,"--",color="#888",label="v_target")
    ax.plot(t,vx,color="#1565c0",label="vx (VDSim L2)")
    ax.set_xlabel("time [s]"); ax.set_ylabel("speed [m/s]")
    ax.set_title("Cascade PID speed tracking (Lc6→Lc5→throttle/brake)")
    ax.grid(True,alpha=0.3); ax.legend(fontsize=9)
    _save(fig,"08_pid_tracking.png")


# ---------------------------------------------------------------------------
# 09b — figure-8 path tracking trajectory (real)
# ---------------------------------------------------------------------------
def fig_09_figure8():
    if not HAVE_VDSIM: return
    # build figure-8 waypoints
    R=20.0; pts=[]
    for i in range(80):
        th=2*math.pi*i/80; pts.append((R-R*math.cos(th), R*math.sin(th)))
    for i in range(80):
        th=2*math.pi*i/80; pts.append((-R+R*math.cos(th), R*math.sin(th)))
    pts=np.array(pts)
    dyn,vp,_=_make("L2", v0=8); c=_contacts(); dt=0.01
    def pp(x,y,yaw,vx,prev):
        Ld=max(2.0,0.45*max(vx,1)); idx=prev; n=len(pts)
        while idx<n:
            if math.hypot(pts[idx][0]-x,pts[idx][1]-y)>=Ld: break
            idx+=1
        if idx>=n: idx=n-1
        cp,sp_=math.cos(yaw),math.sin(yaw)
        dx=cp*(pts[idx][0]-x)+sp_*(pts[idx][1]-y)
        dy=-sp_*(pts[idx][0]-x)+cp*(pts[idx][1]-y)
        l2=dx*dx+dy*dy
        if l2<1e-6: return 0.0,idx
        return max(-0.5,min(0.5,math.atan(2*dy/l2*vp.wheelbase))),idx
    xs=[];ys=[]; prev=0
    for i in range(int(28/dt)):
        st=dyn.state(); x,y=st.position[0],st.position[1]
        steer,prev=pp(x,y,st.yaw(),st.vx(),prev)
        e=8-st.vx()
        cmd=vdsim.CmdL4(); cmd.steer_angle_wheel=steer
        cmd.throttle=max(0,min(1,0.1+0.3*e)); cmd.brake=0 if e>-1 else 0.2
        dyn.step(cmd,c,dt); xs.append(x); ys.append(y)
    fig,ax=plt.subplots(figsize=(6.5,5.5))
    ax.plot(pts[:,0],pts[:,1],"--",color="#bbb",label="reference path")
    ax.plot(xs,ys,color="#1565c0",lw=1.5,label="VDSim L2 + pure pursuit")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Figure-8 path tracking (L2 + pure pursuit, v=8 m/s)")
    ax.legend(fontsize=9); ax.grid(True,alpha=0.3); ax.set_aspect("equal")
    _save(fig,"09_figure8.png")


# ---------------------------------------------------------------------------
# 11b — Euler vs RK4 energy/accuracy (real on a vehicle)
# ---------------------------------------------------------------------------
def fig_11_integrator():
    if not HAVE_VDSIM: return
    fig,ax=plt.subplots(figsize=(7,4.5))
    # reference: tiny dt RK4
    def run(integrator, outer_dt):
        vp=vdsim.VehicleParams.from_yaml(str(REPO/"configs/vehicles/sports.yaml"))
        tp=vdsim.TireParams.from_yaml(str(REPO/"configs/tires/default_pacejka.yaml"))
        sp=vdsim.SolverParams(); sp.integrator=integrator
        sp.max_substep_dt=outer_dt; sp.max_substeps=1
        dyn=vdsim.create_seven_dof(); dyn.initialize(vp,tp,sp)
        s0=vdsim.State(); s0.velocity=[15,0,0]
        w=15/vp.wheel_radius_nominal; s0.wheel_spin=[w,w,w,w]; dyn.reset(s0)
        c=_contacts(); cmd=vdsim.CmdL4(); cmd.steer_angle_wheel=0.05
        t=[];r=[]; tt=0
        n=int(5.0/outer_dt)
        for i in range(n):
            dyn.step(cmd,c,outer_dt); tt+=outer_dt
            t.append(tt); r.append(dyn.state().yaw_rate())
        return np.array(t),np.array(r)
    t_ref,r_ref = run(vdsim.Integrator.RK4, 0.001)
    ax.plot(t_ref,np.degrees(r_ref),color="#333",lw=2,label="RK4 1ms (reference)")
    for integ,odt,style,lbl in [(vdsim.Integrator.Euler,0.02,"--","Euler 20ms"),
                                  (vdsim.Integrator.RK4,0.02,"-.","RK4 20ms")]:
        t,r=run(integ,odt)
        ax.plot(t,np.degrees(r),style,label=lbl)
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw rate [deg/s]")
    ax.set_title("Integrator comparison — step steer\n(coarse Euler drifts, RK4 stays close)")
    ax.grid(True,alpha=0.3); ax.legend(fontsize=9)
    _save(fig,"11_integrator.png")


# ---------------------------------------------------------------------------
# 13 — four suspension types (schematic comparison)
# ---------------------------------------------------------------------------
def fig_13_suspension_types():
    fig,axes=plt.subplots(1,4,figsize=(15,3.8))
    titles=["Double wishbone","MacPherson","Trailing arm","5-link"]
    for ax,title in zip(axes,titles):
        ax.set_title(title, fontsize=10); ax.axis("off")
        ax.set_xlim(-1,1); ax.set_ylim(-1,1.2)
        # chassis (top), knuckle (right), ground
        ax.plot([-0.8,-0.8],[-0.6,0.8],"k-",lw=2)  # chassis
        ax.add_patch(Circle((0.6,0.1),0.25,fill=False,ec="#212121",lw=2))  # wheel
    # DW: UCA + LCA
    a=axes[0]; a.plot([-0.8,0.5],[0.5,0.35],"#c62828",lw=2); a.plot([-0.8,0.5],[-0.3,-0.05],"#1565c0",lw=2)
    a.plot([0.5,0.5],[-0.05,0.35],"#37474f",lw=1.5,ls="--")
    # MacPherson: strut + LCA
    a=axes[1]; a.plot([-0.4,0.55],[0.9,0.1],"#c62828",lw=2.5); a.plot([-0.8,0.5],[-0.3,-0.05],"#1565c0",lw=2)
    # Trailing arm: single arm
    a=axes[2]; a.plot([-0.8,0.6],[0.0,0.1],"#1565c0",lw=2.5)
    # 5-link
    a=axes[3]
    for y0,y1,col in [(0.6,0.35,"#c62828"),(0.5,0.3,"#c62828"),(-0.2,-0.05,"#1565c0"),(-0.3,-0.1,"#1565c0"),(0.1,0.15,"#6a1b9a")]:
        a.plot([-0.8,0.5],[y0,y1],col,lw=1.5)
    fig.suptitle("Ld4 suspension topologies (same ISuspensionKinematics runtime)", fontsize=11)
    _save(fig,"13_suspension_types.png")


# ---------------------------------------------------------------------------
# 10 — driver model (reaction delay + noise) schematic
# ---------------------------------------------------------------------------
def fig_10_driver():
    fig,ax=plt.subplots(figsize=(8,4))
    t=np.linspace(0,4,400)
    ideal=0.05*(t>1.0)
    delay=0.05*(t>1.15)  # 150ms reaction
    rng=np.random.default_rng(0)
    noisy=delay+0.004*rng.standard_normal(len(t))*(t>1.15)
    ax.plot(t,np.degrees(ideal),"--",color="#888",label="ideal controller")
    ax.plot(t,np.degrees(delay),color="#1565c0",lw=1.5,label="+150ms reaction delay")
    ax.plot(t,np.degrees(noisy),color="#c62828",lw=0.8,alpha=0.7,label="+Gaussian noise")
    ax.set_xlabel("time [s]"); ax.set_ylabel("steer [deg]")
    ax.set_title("Driver model: reaction delay (ring buffer) + Box-Muller noise")
    ax.grid(True,alpha=0.3); ax.legend(fontsize=9)
    _save(fig,"10_driver.png")


def main():
    fig_01_frames()
    fig_02_coriolis()
    fig_03_tire()
    fig_03b_tire_mz_fx()
    fig_04_bicycle_geom()
    fig_04_understeer()
    fig_04_step_response()
    fig_05_weight_transfer()
    fig_05_ackermann()
    fig_06_quarter_car()
    fig_06_transient()
    fig_09_pure_pursuit()
    fig_10_driver()
    fig_11_integrator()
    fig_13_suspension_types()
    print("[fig] done")


if __name__ == "__main__":
    main()
