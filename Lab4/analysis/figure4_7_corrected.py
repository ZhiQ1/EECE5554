from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis"
FIG = OUT / "figures"
FIG.mkdir(exist_ok=True)

def read_imu(name):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data" / name), storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    typ = get_message(types["/imu"])
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/imu"]))
    rows = []
    t0 = None

    while reader.has_next():
        _, raw, ns = reader.read_next()
        if t0 is None:
            t0 = ns
        m = deserialize_message(raw, typ)
        mag = m.mag_field.magnetic_field
        gyro = m.imu.angular_velocity
        q = m.imu.orientation
        rows.append([
            (ns-t0)*1e-9,
            mag.x*1e6, mag.y*1e6,
            gyro.z,
            q.x, q.y, q.z, q.w
        ])
    a = np.asarray(rows, dtype=float)
    return a

def fit_ellipse(m):
    # Teacher's SVD ellipse fit, using horizontal µT readings.
    x, y = m[:, 0], m[:, 1]
    D = np.column_stack([
        x*x, x*y, y*y, x, y, np.ones(len(x))
    ])
    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    q = Vt[-1]

    Q = np.array([
        [q[0], q[1]/2],
        [q[1]/2, q[2]]
    ])
    linear = q[3:5]
    center = np.linalg.solve(2*Q, -linear)
    denominator = center @ Q @ center - q[5]
    M = Q / denominator

    eigenvalues, eigenvectors = np.linalg.eigh(M)

    if np.any(eigenvalues <= 0):
        raise RuntimeError(
            "SVD fit is not a valid ellipse. "
            "Check whether the car completed full circles."
        )

    radius = np.linalg.det(M)**(-0.25)
    S = (
        eigenvectors
        @ np.diag(np.sqrt(eigenvalues))
        @ eigenvectors.T
    ) * radius

    # The ellipse semiaxes are 1/sqrt(eigenvalues).
    lengths = 1/np.sqrt(eigenvalues)
    axis_ratio = max(lengths)/min(lengths)
    major = eigenvectors[:, np.argmin(eigenvalues)]
    tilt = np.degrees(np.arctan2(major[1], major[0]))

    return center, S, radius, axis_ratio, tilt, M

circles = read_imu("car_circles")
drive = read_imu("car_drive")

center, S, radius, axis_ratio, tilt, M = fit_ellipse(
    circles[:, 1:3]
)

raw_c = circles[:, 1:3]
cal_c = (raw_c-center) @ S.T

raw_d = drive[:, 1:3]
cal_d = (raw_d-center) @ S.T

# Figure 4: actual SVD fitted ellipse, raw and calibrated.
theta = np.linspace(0, 2*np.pi, 400)
unit = np.column_stack([np.cos(theta), np.sin(theta)])
ellipse = center + unit @ np.linalg.inv(S).T * radius

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].scatter(raw_c[:, 0], raw_c[:, 1], s=1, alpha=.25)
axes[0].plot(ellipse[:, 0], ellipse[:, 1],
             "r", lw=2, label="SVD ellipse fit")
axes[0].scatter(*center, marker="x", c="black", label="Hard iron")
axes[0].set_title("Figure 4(a) — Raw Car Magnetometer")
axes[0].legend()

axes[1].scatter(cal_c[:, 0], cal_c[:, 1], s=1, alpha=.25)
axes[1].plot(radius*np.cos(theta), radius*np.sin(theta),
             "r", lw=2, label="Calibrated circle")
axes[1].set_title("Figure 4(b) — Calibrated Magnetometer")
axes[1].legend()

for ax in axes:
    ax.set_xlabel("Magnetic X (µT)")
    ax.set_ylabel("Magnetic Y (µT)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=.2)
fig.tight_layout()
fig.savefig(FIG/"figure4_svd_calibration.png", dpi=200)
plt.close(fig)

t = drive[:, 0]
gyro_z = drive[:, 3]

# Magnetic heading, clockwise from magnetic north for x-forward,
# y-right convention. This axis convention must be verified.
heading_raw = np.unwrap(np.arctan2(-raw_d[:, 1], raw_d[:, 0]))
heading_cal = np.unwrap(np.arctan2(-cal_d[:, 1], cal_d[:, 0]))

def wrap_degrees(angle):
    return (np.degrees(angle)+180) % 360-180

# Figure 5.
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(t, wrap_degrees(heading_raw),
        label="Uncalibrated magnetic heading", lw=.7)
ax.plot(t, wrap_degrees(heading_cal),
        label="Calibrated magnetic heading", lw=.8)
ax.set_title("Figure 5 — Magnetometer Heading Before/After Calibration")
ax.set_xlabel("Time since laptop recording began (s)")
ax.set_ylabel("Magnetic heading (degrees)")
ax.legend()
ax.grid(alpha=.2)
fig.tight_layout()
fig.savefig(FIG/"figure5_car_heading_calibration.png", dpi=200)
plt.close(fig)

# Gyroscope heading; remove stationary zero-rate bias.
bias = float(np.median(gyro_z[t <= 20]))
gyro_heading = cumulative_trapezoid(
    gyro_z-bias, t, initial=0
)

# Align magnetic heading modulo full revolutions with integrated gyro.
# Keep a common initial offset for comparable plots.
mag0 = heading_cal[0]
gyro_absolute = gyro_heading + mag0
mag_aligned = heading_cal + 2*np.pi*np.round(
    (gyro_absolute-heading_cal)/(2*np.pi)
)

# Figure 6.
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(t, np.degrees(gyro_absolute), lw=1,
        label="Integrated gyro heading")
ax.plot(t, np.degrees(mag_aligned), lw=.8,
        label="SVD-calibrated magnetometer heading")
ax.set_xlabel("Time since laptop recording began (s)")
ax.set_ylabel("Unwrapped heading (degrees)")
ax.set_title("Figure 6 — Gyro vs Calibrated Magnetometer")
ax.legend()
ax.grid(alpha=.2)
fig.tight_layout()
fig.savefig(FIG/"figure6_svd_heading_comparison.png", dpi=200)
plt.close(fig)

# LPF/HPF complementary filter, as required by the teacher.
fc = 0.0159  # 10-second approximate time constant
tau = 1/(2*np.pi*fc)

def lowpass(signal, times):
    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, len(times)):
        dt = max(0., times[i]-times[i-1])
        alpha = dt/(tau+dt)
        out[i] = out[i-1]+alpha*(signal[i]-out[i-1])
    return out

mag_lpf = lowpass(mag_aligned, t)
gyro_lpf = lowpass(gyro_absolute, t)
gyro_hpf = gyro_absolute-gyro_lpf
fused = mag_lpf+gyro_hpf

# VN-100 yaw is used only for the FINAL panel of Figure 7.
qx, qy, qz, qw = drive[:, 4], drive[:, 5], drive[:, 6], drive[:, 7]
vn_yaw = np.arctan2(
    2*(qw*qz+qx*qy),
    1-2*(qy*qy+qz*qz)
)

fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

axes[0].plot(t, np.degrees(mag_lpf), lw=.8)
axes[0].set_title("Figure 7(a) — Low-Pass Magnetometer Heading")
axes[0].set_ylabel("Degrees")

axes[1].plot(t, np.degrees(gyro_hpf), lw=.8)
axes[1].set_title("Figure 7(b) — High-Pass Gyroscope Heading")
axes[1].set_ylabel("Degrees")

axes[2].plot(t, wrap_degrees(fused), lw=.8)
axes[2].set_title("Figure 7(c) — Complementary Filter")
axes[2].set_ylabel("Degrees")

axes[3].plot(t, wrap_degrees(fused), lw=.8,
             label="Our complementary filter")
axes[3].plot(t, wrap_degrees(vn_yaw), lw=.7, alpha=.7,
             label="VN-100 yaw")
axes[3].set_title("Figure 7(d) — Comparison with VN-100")
axes[3].set_xlabel("Time since laptop recording began (s)")
axes[3].set_ylabel("Degrees")
axes[3].legend(fontsize=8)

for ax in axes:
    ax.grid(alpha=.2)

fig.tight_layout()
fig.savefig(FIG/"figure7_four_panel_complementary.png", dpi=200)
plt.close(fig)

np.savez(
    OUT/"heading_corrected_data.npz",
    t=t,
    gyro_heading=gyro_absolute,
    magnetic_heading=mag_aligned,
    complementary_heading=fused,
    vn_yaw=vn_yaw
)

report = f"""
========== Figures 4–7 / Heading ==========
Car circle samples: {len(circles)}
Car drive samples: {len(drive)}
SVD hard-iron center X: {center[0]:.4f} µT
SVD hard-iron center Y: {center[1]:.4f} µT
SVD ellipse axis ratio: {axis_ratio:.4f}
SVD ellipse tilt: {tilt:.2f} degrees
Calibrated circle radius: {radius:.4f} µT
Gyro stationary bias: {bias:.7f} rad/s
Complementary cutoff: {fc:.4f} Hz
Approximate filter time constant: {tau:.2f} s
WARNING: Heading axis signs require visual verification.
"""
print(report)
(OUT/"heading_corrected_results.txt").write_text(report)
print("Saved Figures 4–7 and heading data.")
