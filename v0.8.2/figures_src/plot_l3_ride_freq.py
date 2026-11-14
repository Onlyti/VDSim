"""Task 51: L3 unsprung mass wheel-hop frequency from FFT of susp trace."""
from pathlib import Path
import subprocess
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_l3_demo"
OUT = REPO / "docs/tasks/51_l3_ride_freq/figures"
OUT.mkdir(parents=True, exist_ok=True)

# Run L3 on the throttle/brake sequence — brake transient excites the
# wheel-hop modes.
subprocess.run([str(BIN),
                str(REPO / "configs/vehicles/sedan.yaml"),
                str(REPO / "configs/tires/default_pacejka.yaml"),
                str(REPO / "configs/scenarios/throttle_brake_sequence.yaml"),
                "/tmp/l3_brake.csv"], check=True, capture_output=True)
df = pd.read_csv("/tmp/l3_brake.csv").iloc[1:]
# Window the brake transient (4-7 s of throttle_brake_sequence)
t = df["t"].values; m = (t > 4.0) & (t < 7.0)
t_w = t[m] - 4.0
fl = df["susp_FL"].values[m]
fl -= np.mean(fl)

# FFT
dt = t_w[1] - t_w[0]
N = len(fl)
Y = np.fft.rfft(fl)
freqs = np.fft.rfftfreq(N, dt)
mag = np.abs(Y) / N

# Theoretical wheel-hop: sqrt(k_tire / m_u) / (2 pi)
k_tire = 220000.0; m_u = 40.0
f_hop_pred = np.sqrt(k_tire / m_u) / (2 * np.pi)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
ax1.plot(t_w, fl * 1000.0, color="#4F81BD")
ax1.set_xlabel("t [s] (since brake start)")
ax1.set_ylabel("FL susp compression deviation [mm]")
ax1.set_title("L3 ride transient (brake step)")
ax1.grid(True, alpha=0.3)

ax2.plot(freqs, mag * 1000.0, color="#DC291E")
ax2.axvline(f_hop_pred, color="black", linestyle="--",
            label=f"wheel-hop pred = {f_hop_pred:.1f} Hz")
ax2.set_xlim(0, 30)
ax2.set_xlabel("freq [Hz]"); ax2.set_ylabel("|FFT|")
ax2.set_title("Spectrum of susp deviation")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=9)

plt.tight_layout()
plt.savefig(OUT / "ride_fft.png", dpi=120); plt.close(fig)

# Find peak frequency in 5-20 Hz window
band = (freqs > 5.0) & (freqs < 20.0)
peak_f = freqs[band][np.argmax(mag[band])] if band.any() else float("nan")
print(f"theoretical wheel-hop: {f_hop_pred:.2f} Hz")
print(f"measured peak in band: {peak_f:.2f} Hz")
print(f"figures -> {OUT}")
