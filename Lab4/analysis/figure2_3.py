import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path.home() / "EECE5554/Lab4/analysis"))
import figure1 as raw

OUT = Path.home() / "EECE5554/Lab4/analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

xy = np.column_stack((raw.mx, raw.my)) * 1e6  # Tesla -> microtesla

# Estimate a 2D hard-iron offset.
offset = (xy.max(axis=0) + xy.min(axis=0)) / 2
centered = xy - offset

# Approximate soft-iron correction using covariance whitening.
cov = np.cov(centered, rowvar=False)
eigvals, eigvecs = np.linalg.eigh(cov)
eigvals = np.maximum(eigvals, 1e-10)

W = eigvecs @ np.diag(1 / np.sqrt(eigvals)) @ eigvecs.T
corrected = centered @ W.T

# Normalize to the median magnitude of the centered measurements.
target_radius = np.median(np.linalg.norm(centered, axis=1))
corrected *= target_radius / np.median(
    np.linalg.norm(corrected, axis=1)
)

# Figure 2: raw versus calibrated XY.
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].scatter(xy[:, 0], xy[:, 1], s=2, alpha=0.4)
axes[0].scatter(*offset, marker="x", s=100, color="red",
                label="Estimated hard-iron offset")
axes[0].set_title("Raw Magnetometer")
axes[0].legend()

axes[1].scatter(corrected[:, 0], corrected[:, 1], s=2, alpha=0.4)
axes[1].set_title("After 2D Calibration")

for ax in axes:
    ax.set_xlabel("Magnetic Field X (µT)")
    ax.set_ylabel("Magnetic Field Y (µT)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(OUT / "figure2_magnetometer_calibration.png", dpi=200)
plt.close(fig)

# Figure 3: compare magnetic-field magnitude variation.
r_before = np.linalg.norm(centered, axis=1)
r_after = np.linalg.norm(corrected, axis=1)

fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(raw.t, r_before, linewidth=0.8, alpha=0.8,
        label="Hard-iron corrected only")
ax.plot(raw.t, r_after, linewidth=0.8, alpha=0.8,
        label="After soft-iron approximation")
ax.set_xlabel("Time (s)")
ax.set_ylabel("XY Magnetic Magnitude (µT)")
ax.set_title("Magnetometer Magnitude Consistency")
ax.legend()
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "figure3_magnetometer_consistency.png", dpi=200)
plt.close(fig)

print("Hard-iron offset (µT):", offset)
print("Magnitude std before (µT):", np.std(r_before))
print("Magnitude std after (µT):", np.std(r_after))
print("Saved Figure 2 and Figure 3")
