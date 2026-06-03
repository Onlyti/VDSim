# Road-roughness presets

PSD road profiles for ride / durability / component-load studies. Each preset
defines a displacement PSD `Gd(n)` over spatial frequency `n` [cycles/m]; VDSim
synthesizes a seeded profile (independent left/right tracks) fed to the L3 ride
model as per-wheel `road_dz`.

ISO 8608 alone classifies a road by one number (`Gd(n0)`, class A–H) at a fixed
waviness `w=2` — but two roads of the *same class* can have very different
spectra. These presets break that limit two ways:

- **analytic** — `Gd(n)=gd_n0·(n/0.1)^-w`, optionally **dual-slope** (`w` below
  `n_break`, `waviness_high` above) so a surface can carry strong short-wavelength
  content (Belgian pavé, cobblestone) that a single slope cannot.
- **table** — measured `(n, Gd)` pairs (log-log interpolated), e.g. a washboard
  spectral peak, or a proving-ground **RLDA** spectrum plugged in directly.

Load from Python via `examples/road_profile.py` (`ground_from_preset` /
`ground_from_csv`) or simulate with `vdsim.make_sim_session_psd(...)`.

| preset | type | character |
|---|---|---|
| `smooth_highway` | analytic | ISO ~A/B, low roughness |
| `minor_road` | analytic | ISO ~C average road |
| `belgian_pave` | analytic dual-slope | short-period, durability |
| `cobblestone` | analytic dual-slope | rough broadband |
| `washboard` | table | corrugation peak (~2 cyc/m) |

The numeric values are **representative defaults** — replace `gd_n0`, `waviness`,
or the table with measured data for fidelity. Demo ride response (FL Fz std,
L3 sedan @ 20 m/s): smooth 2.3 N < minor 2.9 < pavé 3.4 < cobble 5.6 < washboard 9.6 N.
