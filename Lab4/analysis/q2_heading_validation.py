from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d

OUT = Path.home() / "EECE5554/Lab4/analysis"
d = np.load(OUT / "heading_corrected_data.npz")

t = np.asarray(d["t"], dtype=float)
mag = np.asarray(d["magnetic_heading"], dtype=float)
gyro = np.asarray(d["gyro_heading"], dtype=float)
vn = np.asarray(d["vn_yaw"], dtype=float)

dt = float(np.median(np.diff(t)))
fs = 1 / dt

def wrap(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def smooth(x, seconds):
    return gaussian_filter1d(x, sigma=seconds*fs)

def turning_correlation(window):
    # Smooth first, then estimate angular velocity.
    m = np.gradient(smooth(mag, window), t)
    g = np.gradient(smooth(gyro, window), t)

    # Exclude near-stationary periods.
    moving = np.abs(g) > 0.04

    if moving.sum() < 30:
        return np.nan, moving.sum()

    return np.corrcoef(m[moving], g[moving])[0, 1], moving.sum()

print("===== Heading direction check =====")
for window in (0.5, 1.0, 3.0):
    r, n = turning_correlation(window)
    print(f"Smooth {window:.1f}s: r={r:.4f}, moving samples={n}")

def lowpass(signal, fc):
    tau = 1 / (2*np.pi*fc)
    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, len(t)):
        alpha = (t[i]-t[i-1]) / (tau+t[i]-t[i-1])
        out[i] = out[i-1]+alpha*(signal[i]-out[i-1])
    return out

def circular_offset(a):
    return np.arctan2(np.mean(np.sin(a)), np.mean(np.cos(a)))

fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

print("\n===== Cutoff comparison =====")
for fc in (0.00159, 0.0159, 0.159):
    mag_lp = lowpass(mag, fc)
    gyro_hp = gyro - lowpass(gyro, fc)
    fused = mag_lp + gyro_hp

    # VN-100 yaw used only for Figure 7's comparison panel.
    raw_difference = wrap(fused-vn)
    offset = circular_offset(raw_difference)
    residual = wrap(raw_difference-offset)

    rms = np.degrees(np.sqrt(np.mean(residual**2)))
    median = np.degrees(np.median(np.abs(residual)))

    print(
        f"fc={fc:.5f} Hz | "
        f"offset={np.degrees(offset):.2f} deg | "
        f"RMS after offset={rms:.2f} deg | "
        f"median={median:.2f} deg"
    )

    axes[0].plot(
        t, np.degrees(wrap(fused)),
        lw=0.7, label=f"{fc:g} Hz"
    )
    axes[1].plot(
        t, np.degrees(residual),
        lw=0.7, label=f"{fc:g} Hz"
    )

axes[0].set_title("Complementary Filter Cutoff Comparison")
axes[0].set_ylabel("Wrapped heading (degrees)")
axes[0].legend()

axes[1].set_title(
    "Difference from VN-100 After Removing Constant Offset"
)
axes[1].set_ylabel("Heading difference (degrees)")
axes[1].set_xlabel("Laptop recording time (s)")
axes[1].legend()

for ax in axes:
    ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(
    OUT / "figures/q2_cutoff_comparison.png",
    dpi=200
)
plt.close(fig)

print("\nSaved q2_cutoff_comparison.png")
