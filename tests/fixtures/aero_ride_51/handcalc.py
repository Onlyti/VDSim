#!/usr/bin/env python3
"""Hand calculations of work order 51 (aero ride-height / pitch slopes).

Independent of the vdsim package (no ``import vdsim``); standard library only.
Every expected value of B2, B3'-L3, B3'-L5 and B3b in DEFINITION.md comes from
the functions below.  The file is fixed by its sha256 in DEFINITION.md before
any measurement.

Conventions (DEFINITION.md section 1):
  * element coordinates: origin on the ground below the front axle, x forward
    (rear elements negative), z up [m]
  * body origin x: x_o = -cg_to_front (CG and sprung-body origin share x)
  * dz: static-relative heave of the body origin [m], + up
  * theta: static-relative pitch [rad], ISO 8855, + nose down
  * h_i - h_ref_i = dz - (x_i - x_o) * theta
  * ClA_eff_i = ClA_i + dClA_dh_i * (h_i - h_ref_i) + dClA_dpitch_i * theta

Usage:
  handcalc.py b2      > b2_expected.json      # fixed grid, committed with the gate
  handcalc.py b3l3                             # B3'-L3 injected-state expectation
  handcalc.py predict                          # B3'-L5 / B3b size check (info only)
  handcalc.py b3l5  measured.json              # verdict of one B3'-L5 pair of runs
  handcalc.py b3b   measured.json              # verdict of one B3b pair of runs
"""
import json
import sys

TOL_B3 = 0.02          # B3' / B3b relative gate
GUARD_N = 10.0         # |expected| below this -> not judgeable -> stop and report

# Fixed example car (configs/vehicles/fs_aero_example.yaml at faf59f2).
FS = {
    "mass": 280.0, "wheelbase": 1.55, "cg_to_front": 0.82, "cg_height": 0.28,
    "rho": 1.165, "spring": 35000.0, "tire_kz": 220000.0,
    "elements": [
        {"name": "front_wing", "ClA": 0.70, "CdA": 0.12, "x": 0.550, "z": 0.100},
        {"name": "rear_wing",  "ClA": 1.50, "CdA": 0.55, "x": -1.520, "z": 1.000},
        {"name": "undertray",  "ClA": 0.80, "CdA": 0.05, "x": -0.750, "z": 0.060},
        {"name": "body",       "ClA": 0.00, "CdA": 0.75, "x": -0.775, "z": 0.300},
    ],
}


def q_of(rho, vx):
    """Dynamic pressure 0.5 * rho * vx^2 [Pa]."""
    return 0.5 * rho * vx * vx


def cla_eff(e, dz, theta, x_o):
    """Effective ClA of element e at static-relative (dz, theta) [m^2]."""
    dh = dz - (e["x"] - x_o) * theta
    return e["ClA"] + e.get("dClA_dh", 0.0) * dh + e.get("dClA_dpitch", 0.0) * theta


def rigid_split(elements, q, L, dz=0.0, theta=0.0, x_o=0.0):
    """Front / rear axle aero load increase [N], drag pitch moment included.

    F_i = q * ClA_eff_i, s_i = (x_i + L) / L, M_d = sum q * CdA_i * z_i,
    F_f = sum F_i s_i - M_d / L, F_r = sum F_i - F_f.
    """
    ff = 0.0
    tot = 0.0
    md = 0.0
    for e in elements:
        f = q * cla_eff(e, dz, theta, x_o)
        s = (e["x"] + L) / L
        ff += f * s
        tot += f
        md += q * e["CdA"] * e["z"]
    ff -= md / L
    return ff, tot - ff


# ---------------------------------------------------------------- B2
B2_VX = 20.0
B2_DZ = [-0.020, 0.0, 0.015]
B2_TH = [-0.010, 0.0, 0.020]
B2_SLOPES = {
    "front_wing": {"h_ref": 0.100, "dClA_dh": -1.5, "dClA_dpitch": 30.0},
    "rear_wing":  {"h_ref": 1.000, "dClA_dh": 0.0,  "dClA_dpitch": -4.0},
    "undertray":  {"h_ref": 0.040, "dClA_dh": -2.0, "dClA_dpitch": 0.0},
}


def b2():
    """Expected element downforce [N] on the fixed (dz, theta) grid."""
    els = []
    for e in FS["elements"]:
        d = dict(e)
        d.update(B2_SLOPES.get(e["name"], {}))
        els.append(d)
    x_o = -FS["cg_to_front"]
    q = q_of(FS["rho"], B2_VX)
    cases = []
    for dz in B2_DZ:
        for th in B2_TH:
            cases.append({
                "dz": dz, "theta": th,
                "fz": {e["name"]: q * cla_eff(e, dz, th, x_o) for e in els},
                "h": {e["name"]: e.get("h_ref", e["z"]) + dz - (e["x"] - x_o) * th for e in els},
            })
    return {"vx": B2_VX, "rho": FS["rho"], "x_o": x_o, "slopes": B2_SLOPES,
            "rel_tol": 1e-9, "cases": cases}


# ---------------------------------------------------------------- B3'-L3
B3L3 = {"vx": 35.0, "dz": -0.010, "theta": 0.010,
        "undertray": {"h_ref": 0.040, "dClA_dh": -2.0, "dClA_dpitch": 0.0}}


def b3l3():
    """Sum-of-tyre-Fz difference (slope on - off) at the first tick [N]."""
    q = q_of(FS["rho"], B3L3["vx"])
    u = next(e for e in FS["elements"] if e["name"] == "undertray")
    dh = B3L3["dz"] - (u["x"] + FS["cg_to_front"]) * B3L3["theta"]
    d = q * B3L3["undertray"]["dClA_dh"] * dh
    return {"q": q, "dh_undertray": dh, "dF_down_expected": d,
            "guard_ok": abs(d) >= GUARD_N, "rel_tol": TOL_B3}


# ---------------------------------------------------------------- B3'-L5
def b3l5(m):
    """Self-sag fixed point of the undertray slope.

    m keys (window means of the two runs, same level / path / command):
      q_on, q_off [Pa]; dzf_off, dzr_off [m] = front / rear axle-point heave of
      the slope-0 run relative to the static reference (+ up);
      sumfz_on, sumfz_off [N] = window mean of the four tyre Fz;
      dh_u_on [m] (info: measured undertray h - h_ref in the slope run).
    """
    L = FS["wheelbase"]
    els = FS["elements"]
    u = next(e for e in els if e["name"] == "undertray")
    s = (u["x"] + L) / L
    ff0, fr0 = rigid_split(els, m["q_off"], L)
    kf = ff0 / (-m["dzf_off"])
    kr = fr0 / (-m["dzr_off"])
    dclh = -2.0
    D = m["q_on"] * dclh
    dh0 = s * m["dzf_off"] + (1.0 - s) * m["dzr_off"]
    dh = dh0 / (1.0 + D * (s * s / kf + (1.0 - s) * (1.0 - s) / kr))
    cla_sum = sum(e["ClA"] for e in els)
    exp = cla_sum * (m["q_on"] - m["q_off"]) + D * dh
    meas = m["sumfz_on"] - m["sumfz_off"]
    out = {"K_f": kf, "K_r": kr, "s_u": s, "dh_u_fixed_point": dh,
           "dF_down_expected": exp, "dF_down_measured": meas,
           "rel_err": meas / exp - 1.0 if exp != 0.0 else float("nan"),
           "guard_ok": abs(exp) >= GUARD_N}
    if "dh_u_on" in m:
        out["info_dF_from_measured_dh"] = cla_sum * (m["q_on"] - m["q_off"]) + D * m["dh_u_on"]
    out["pass"] = bool(out["guard_ok"] and abs(out["rel_err"]) <= TOL_B3)
    return out


# ---------------------------------------------------------------- B3b
B3B = {"v0": 32.0, "ax_target": -4.0, "t_win": [1.0, 3.0],
       "front_wing": {"h_ref": 0.100, "dClA_dh": 0.0, "dClA_dpitch": 30.0}}


def b3b(m):
    """Axle-load difference of the front-wing pitch slope during steady braking.

    m keys: per-tick window arrays of both runs:
      on/off: {"q": [...], "theta": [...] (on only), "ax": [...],
               "fz_front": [...], "fz_rear": [...]}
    Expected per axle: <F_aero,on> - <F_aero,off> -/+ m * (<ax_on> - <ax_off>) * h / L
    with F_aero from rigid_split() per tick (drag moment included).
    """
    L = FS["wheelbase"]
    x_o = -FS["cg_to_front"]
    els = []
    for e in FS["elements"]:
        d = dict(e)
        if e["name"] == "front_wing":
            d.update(B3B["front_wing"])
        els.append(d)

    def mean(v):
        return sum(v) / len(v)

    on, off = m["on"], m["off"]
    fon = [rigid_split(els, q, L, 0.0, th, x_o) for q, th in zip(on["q"], on["theta"])]
    foff = [rigid_split(FS["elements"], q, L) for q in off["q"]]
    dax = mean(on["ax"]) - mean(off["ax"])
    inert = FS["mass"] * dax * FS["cg_height"] / L
    ef = mean([f[0] for f in fon]) - mean([f[0] for f in foff]) - inert
    er = mean([f[1] for f in fon]) - mean([f[1] for f in foff]) + inert
    mf = mean(on["fz_front"]) - mean(off["fz_front"])
    mr = mean(on["fz_rear"]) - mean(off["fz_rear"])
    out = {"dF_front_expected": ef, "dF_rear_expected": er,
           "dF_front_measured": mf, "dF_rear_measured": mr,
           "rel_err_front": mf / ef - 1.0, "rel_err_rear": mr / er - 1.0,
           "guard_ok": abs(ef) >= GUARD_N and abs(er) >= GUARD_N}
    out["pass"] = bool(out["guard_ok"] and abs(out["rel_err_front"]) <= TOL_B3
                       and abs(out["rel_err_rear"]) <= TOL_B3)
    return out


# ---------------------------------------------------------------- size check
def predict():
    """Order-of-magnitude check of the gates before any run (info only).

    Axle rate = 2 x (spring in series with tyre), motion ratio 1; static ride
    pitch from 4 m/s^2 braking by the same rate.  Not a judgement value.
    """
    L = FS["wheelbase"]
    k_wheel = FS["spring"] * FS["tire_kz"] / (FS["spring"] + FS["tire_kz"])
    k_axle = 2.0 * k_wheel
    q35 = q_of(FS["rho"], 35.0)
    ff, fr = rigid_split(FS["elements"], q35, L)
    s = (-0.750 + L) / L
    dzf, dzr = -ff / k_axle, -fr / k_axle
    D = q35 * -2.0
    dh0 = s * dzf + (1 - s) * dzr
    dh = dh0 / (1.0 + D * (s * s / k_axle + (1 - s) * (1 - s) / k_axle))
    dt = FS["mass"] * 4.0 * FS["cg_height"] / L
    theta = 2.0 * (dt / 2.0) / k_wheel / L
    qb = q_of(FS["rho"], 26.0)
    dfw = qb * B3B["front_wing"]["dClA_dpitch"] * theta
    sw = (0.550 + L) / L
    return {"k_axle": k_axle, "b3l5_dF_pred": D * dh, "b3l5_dh_u_pred": dh,
            "b3b_theta_pred": theta, "b3b_dF_front_pred": dfw * sw,
            "b3b_dF_rear_pred": dfw * (1 - sw), "b3l3": b3l3()}


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd = argv[1]
    if cmd == "b2":
        out = b2()
    elif cmd == "b3l3":
        out = b3l3()
    elif cmd == "predict":
        out = predict()
    elif cmd in ("b3l5", "b3b"):
        with open(argv[2]) as f:
            m = json.load(f)
        out = b3l5(m) if cmd == "b3l5" else b3b(m)
    else:
        print(__doc__)
        return 2
    json.dump(out, sys.stdout, indent=1, sort_keys=True)
    sys.stdout.write("\n")
    if isinstance(out, dict) and "pass" in out:
        return 0 if out["pass"] else 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
