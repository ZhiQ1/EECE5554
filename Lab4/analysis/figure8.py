from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(exist_ok=True)

d = np.load(ROOT / "data/car/car_drive.npz")

fig, ax = plt.subplots(figsize=(11, 5))

for key in [
    "vehicle_velocity__vehicle_velocity_propulsion",
    "vehicle_velocity__vehicle_velocity_brake",
]:
    t = np.asarray(d["vehicle_velocity__t"], dtype=float)
    v = np.asarray(d[key], dtype=float)
    if len(t) == len(v):
        ax.plot(t - t[0], v, label=key.split("__")[1], alpha=0.8)

for key in [
    "wheel_speeds__front_left",
    "wheel_speeds__front_right",
    "wheel_speeds__rear_left",
    "wheel_speeds__rear_right",
]:
    t = np.asarray(d["wheel_speeds__t"], dtype=float)
    v = np.asarray(d[key], dtype=float)
    if len(t) == len(v):
        ax.plot(t - t[0], v, label=key.split("__")[1],
                linewidth=0.7, alpha=0.5)

ax.set_xlabel("Time since Car PC recording start (s)")
ax.set_ylabel("Reported speed (native units)")
ax.set_title("Figure 8 — Mustang Vehicle and Wheel Speed Signals")
ax.legend(ncol=2, fontsize=8)
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(OUT / "figure8_vehicle_velocity.png", dpi=200)
plt.close(fig)

print("Saved Figure 8")
