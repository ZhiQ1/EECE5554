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

d = np.load(OUT / "q4_velocity_data.npz")
t = np.asarray(d["t"], dtype=float)
v = np.asarray(d["v_corrected"], dtype=float)
omega = np.asarray(d["gyro_z"], dtype=float)
vcar = np.asarray(d["v_car"], dtype=float)

# Laptop ROS Bag GNSS: preserve quality flags.
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(ROOT / "data/car_drive"),
        storage_id="mcap"
    ),
    rosbag2_py.ConverterOptions("cdr", "cdr")
)

types = {x.name: x.type for x in reader.get_all_topics_and_types()}
gps_type = get_message(types["/gps"])
reader.set_filter(rosbag2_py.StorageFilter(topics=["/gps"]))

gps_rows = []

while reader.has_next():
    _, raw, stamp = reader.read_next()
    msg = deserialize_message(raw, gps_type)
    gps_rows.append([
        stamp * 1e-9,
        msg.utm_easting,
        msg.utm_northing,
        msg.fix_quality
    ])

gps = np.asarray(gps_rows, dtype=float)

# Match the first bag time to the q4 laptop-relative clock.
# The first IMU message usually starts near the beginning.
first_gps_abs = gps[0, 0]
gps_t_relative = gps[:, 0] - first_gps_abs

# Use the q4 GPS timestamps, which share the exact laptop clock.
# q4 stores GPS velocity midpoints, not raw fix timestamps.
# Derive absolute-to-relative offset by matching GPS sample time
# against the GPS-speed midpoint sequence.
gps_speed_mid = np.asarray(d["gps_t"], dtype=float)
if len(gps_t_relative) - 1 != len(gps_speed_mid):
    raise RuntimeError("GPS sample count mismatch")

gps_t = gps_t_relative + (
    gps_speed_mid[0] -
    (gps_t_relative[0] + gps_t_relative[1]) / 2
)

gps_xy = gps[:, 1:3]
gps_xy = gps_xy - gps_xy[0]
quality = gps[:, 3].astype(int)

# Find vehicle movement start from independent Mustang speed.
moving = (
    np.isfinite(vcar) &
    (vcar > 1.0)
)

# Require approximately one continuous second of movement.
fs = 1.0 / np.median(np.diff(t))
min_run = max(1, int(fs))

starts = np.flatnonzero(
    np.convolve(
        moving.astype(int),
        np.ones(min_run, dtype=int),
        mode="same"
    ) >= min_run
)

if len(starts) == 0:
    raise RuntimeError("Could not detect movement start")

start_idx = int(starts[0])
t_start = float(t[start_idx])

# Estimate stationary gyro bias from first 20 s.
bias = float(np.median(omega[t <= 20]))
omega_corrected = omega - bias

# VN-100 gyro z points down: positive means right turn.
yaw = cumulative_trapezoid(
    omega_corrected,
    t,
    initial=0.0
)
yaw -= yaw[start_idx]

# Heading 0 means north. Positive yaw turns east (right).
# This is a gyro-only heading; not yet complementary filtered.
speed = np.maximum(v, 0.0)

velocity_east = speed * np.sin(yaw)
velocity_north = speed * np.cos(yaw)

dr_east = cumulative_trapezoid(
    velocity_east,
    t,
    initial=0.0
)
dr_north = cumulative_trapezoid(
    velocity_north,
    t,
    initial=0.0
)

dr = np.column_stack([dr_east, dr_north])
dr = dr - dr[start_idx]

# GPS track at laptop IMU timestamps.
gps_at_t = np.column_stack([
    np.interp(t, gps_t, gps_xy[:, 0]),
    np.interp(t, gps_t, gps_xy[:, 1])
])
gps_at_t -= gps_at_t[start_idx]

# Calibrate heading using only the first 15 to 100 m
# of GPS movement, without changing scale.
gps_step = np.linalg.norm(
    np.diff(gps_at_t, axis=0), axis=1
)
gps_step[:start_idx] = 0.0
gps_distance = np.r_[
    0.0, np.cumsum(gps_step)
]

calibration = (
    (np.arange(len(t)) >= start_idx) &
    (gps_distance >= 15.0) &
    (gps_distance <= 100.0)
)

if calibration.sum() < 20:
    raise RuntimeError(
        "Insufficient moving samples for heading calibration"
    )

# Estimate only initial rotation (no scale correction).
A = dr[calibration]
B = gps_at_t[calibration]

cross = np.sum(
    A[:, 0] * B[:, 1] -
    A[:, 1] * B[:, 0]
)
dot = np.sum(
    A[:, 0] * B[:, 0] +
    A[:, 1] * B[:, 1]
)

rotation_angle = float(np.arctan2(cross, dot))

c = np.cos(rotation_angle)
s = np.sin(rotation_angle)
R = np.array([
    [c, -s],
    [s, c]
])

dr_aligned = dr @ R.T

# Evaluate at real GPS fix timestamps AFTER movement start.
evaluation = (
    (gps_t >= t_start) &
    (gps_t <= t[-1])
)

te = gps_t[evaluation]
gps_eval = gps_xy[evaluation]

# Make the GNSS evaluation use the same starting position.
gps_start = np.array([
    np.interp(t_start, gps_t, gps_xy[:, 0]),
    np.interp(t_start, gps_t, gps_xy[:, 1])
])
gps_eval = gps_eval - gps_start

dr_eval = np.column_stack([
    np.interp(te, t, dr_aligned[:, 0]),
    np.interp(te, t, dr_aligned[:, 1])
])

q_eval = quality[evaluation]
error = np.linalg.norm(dr_eval - gps_eval, axis=1)
elapsed = te - t_start

# Path lengths after vehicle movement starts.
dr_length = float(np.sum(
    np.linalg.norm(
        np.diff(dr_aligned[start_idx:], axis=0),
        axis=1
    )
))
gps_length = float(np.sum(
    np.linalg.norm(np.diff(gps_eval, axis=0), axis=1)
))
scale_factor = (
    dr_length / gps_length if gps_length > 0 else np.nan
)

def first_exceed(threshold):
    bad = np.flatnonzero(error > threshold)
    return (
        float(elapsed[bad[0]])
        if len(bad)
        else float(elapsed[-1])
    )

t2 = first_exceed(2.0)
t10 = first_exceed(10.0)
end_error = float(error[-1])

unique, counts = np.unique(q_eval, return_counts=True)
quality_counts = dict(zip(
    unique.astype(int).tolist(),
    counts.astype(int).tolist()
))

# Figure 10: trajectory and position error.
fig, axes = plt.subplots(
    1, 2, figsize=(14, 6)
)

ax = axes[0]
ax.plot(
    dr_eval[:, 0], dr_eval[:, 1],
    label="IMU dead reckoning (gyro heading)",
    lw=1.3, color="tab:blue"
)

# Plot recorded GNSS fixes by quality.
quality_labels = {
    1: "Standalone",
    2: "Differential GPS",
    4: "RTK Fixed",
    5: "RTK Float"
}

for q in np.unique(q_eval):
    mask = q_eval == q
    ax.scatter(
        gps_eval[mask, 0],
        gps_eval[mask, 1],
        s=7,
        alpha=0.75,
        label=f"GNSS quality {q}: "
              f"{quality_labels.get(int(q), 'Other')}"
    )

ax.scatter(
    [0], [0], marker="*",
    s=80, color="black", label="Start"
)
ax.set_xlabel("East displacement (m)")
ax.set_ylabel("North displacement (m)")
ax.set_title(
    "Figure 10(a) — Dead Reckoning vs GNSS"
)
ax.set_aspect("equal", adjustable="datalim")
ax.legend(fontsize=8)
ax.grid(alpha=0.2)

ax = axes[1]
ax.plot(
    elapsed, error,
    label="Position error vs recorded GNSS",
    lw=1.2
)
ax.axhline(
    2, color="tab:green",
    linestyle="--", label="2 m threshold"
)
ax.axhline(
    10, color="tab:red",
    linestyle="--", label="10 m threshold"
)
ax.set_xlabel("Time since vehicle movement began (s)")
ax.set_ylabel("Position error (m)")
ax.set_title(
    "Figure 10(b) — Dead-Reckoning Error"
)
ax.legend(fontsize=8)
ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(
    FIG / "figure10_dead_reckoning.png",
    dpi=200
)
plt.close(fig)

summary = f"""
========== Figure 10 / Question 6 ==========

Movement start: {t_start:.2f} s
Initial heading alignment: {np.degrees(rotation_angle):.2f} degrees
Stationary gyro bias: {bias:.6f} rad/s

GNSS path length: {gps_length:.2f} m
IMU dead-reckoning path length: {dr_length:.2f} m
Scale factor (IMU/GNSS): {scale_factor:.4f}

First exceed 2 m: {t2:.2f} s after movement
First exceed 10 m: {t10:.2f} s after movement
Endpoint position error: {end_error:.2f} m
Maximum position error: {np.max(error):.2f} m
Median position error: {np.median(error):.2f} m

GNSS fix quality counts: {quality_counts}

IMPORTANT:
- Reference GNSS is not RTK Fixed.
- This baseline uses corrected IMU velocity and gyro-only heading.
- Initial heading was aligned using the first 15-100 m of GNSS.
- The beginning of that interval is therefore not independent validation.
- No scale correction was applied.
- GNSS antenna-to-base_link lever arm has not been corrected.
- Physical antenna measurements and local magnetic declination
  are not yet available.
"""

print(summary)

(OUT / "q6_dead_reckoning_results.txt").write_text(summary)

np.savez(
    OUT / "q6_dead_reckoning_data.npz",
    t=te,
    elapsed=elapsed,
    gnss_xy=gps_eval,
    dead_reckoning_xy=dr_eval,
    error=error,
    quality=q_eval
)

print("Saved Figure 10 and Q6 results.")
