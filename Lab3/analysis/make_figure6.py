from pathlib import Path
import runpy

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab3"
FIGURES = ROOT / "analysis/figures"

# Reuse the already-tested motion data loader.
result = runpy.run_path(
    str(ROOT / "analysis/make_motion_overview.py")
)

t = result["t"]
gyro = result["gyro"]       # deg/s
accel = result["accel"]     # m/s^2
yaw = result["yaw"]         # degrees

# Candidate windows identified from the motion overview.
TURN = (17.5, 22.0)
TIP = (38.0, 43.0)
TAPS = (69.0, 75.0)

fig, axes = plt.subplots(
    3, 1,
    figsize=(12, 12)
)

# Figure 6a: horizontal turn
ax = axes[0]
mask = (t >= TURN[0]) & (t <= TURN[1])

line1, = ax.plot(
    t[mask],
    gyro[mask, 2],
    color="tab:green",
    label="Gyroscope Z"
)

ax.set_ylabel("Angular velocity (deg/s)")
ax.set_title("Figure 6a. Horizontal Turn")

ax2 = ax.twinx()

line2, = ax2.plot(
    t[mask],
    yaw[mask],
    color="tab:blue",
    label="Yaw"
)

ax2.set_ylabel("Yaw (degrees)")
ax.legend([line1, line2], ["Gyroscope Z", "Yaw"])
ax.set_xlabel("Time (s)")
ax.grid(alpha=0.3)


# Figure 6b: tip
ax = axes[1]
mask = (t >= TIP[0]) & (t <= TIP[1])

for i, axis in enumerate("XYZ"):
    ax.plot(
        t[mask],
        accel[mask, i],
        label=f"Accel {axis}"
    )

ax.set_title("Figure 6b. Tip Onto Side")
ax.set_ylabel("Acceleration (m/s²)")
ax.set_xlabel("Time (s)")
ax.legend()
ax.grid(alpha=0.3)


# Figure 6c: candidate tap interval
ax = axes[2]
mask = (t >= TAPS[0]) & (t <= TAPS[1])

magnitude = np.linalg.norm(accel, axis=1)

ax.plot(
    t[mask],
    magnitude[mask],
    label="Acceleration magnitude",
    color="tab:purple"
)

ax.axhline(
    9.81,
    linestyle="--",
    color="gray",
    label="9.81 m/s² reference"
)

ax.set_title(
    "Figure 6c. Table Taps: Acceleration Transients"
)
ax.set_ylabel("Acceleration magnitude (m/s²)")
ax.set_xlabel("Time (s)")
ax.legend()
ax.grid(alpha=0.3)

fig.suptitle(
    "Figure 6. VN-100 Motion Experiment",
    fontsize=16
)

fig.tight_layout()

path = FIGURES / "figure6_motion_analysis.png"
fig.savefig(path, dpi=250, bbox_inches="tight")
plt.close(fig)

print("\n========== FIGURE 6 ==========")
print("Saved:", path)

for name, (start, end) in [
    ("Turn", TURN),
    ("Tip", TIP),
    ("Tap candidate", TAPS),
]:
    mask = (t >= start) & (t <= end)
    print(
        f"{name}: {start}-{end} s, "
        f"{np.count_nonzero(mask)} samples"
    )

print("\n========== TAP CANDIDATE DETAILS ==========")

mask = (t >= TAPS[0]) & (t <= TAPS[1])

for time, value in zip(
    t[mask],
    magnitude[mask]
):
    if abs(value - 9.81) > 1.0:
        print(
            f"Time={time:.3f} s, "
            f"Accel magnitude={value:.3f} m/s²"
        )

print("\nFigure 6 candidate completed.")
