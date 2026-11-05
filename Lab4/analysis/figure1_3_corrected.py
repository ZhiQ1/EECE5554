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

# Approximate Seattle declination from the instructor's notes.
# This is NOT an independently verified NOAA value.
DECLINATION_DEG = 15.0

def read_bag(name):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data" / name), storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )

    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    imu_type = get_message(types["/imu"])
    gps_type = get_message(types["/gps"])

    imu_rows, gps_rows = [], []
    first_ns = None

    while reader.has_next():
        topic, raw, ns = reader.read_next()
        if first_ns is None:
            first_ns = ns

        t = (ns - first_ns) * 1e-9

        if topic == "/imu":
            m = deserialize_message(raw, imu_type)
            mag = m.mag_field.magnetic_field
            gyro = m.imu.angular_velocity

            imu_rows.append([
                t,
                mag.x * 1e6,
                mag.y * 1e6,
                gyro.z
            ])

        elif topic == "/gps":
            m = deserialize_message(raw, gps_type)
            gps_rows.append([
                t,
                m.utm_easting,
                m.utm_northing,
                m.fix_quality
            ])

    return (
        np.asarray(imu_rows, dtype=float),
        np.asarray(gps_rows, dtype=float)
    )

def fit_ellipse(m):
    """Normalized algebraic SVD ellipse fit; honest circle fallback."""
    x, y = np.asarray(m, dtype=float).T

    # Normalize coordinates to improve numerical conditioning.
    mean = np.array([x.mean(), y.mean()])
    scale = np.array([x.std(), y.std()])
    if np.any(scale < 1e-9):
        raise RuntimeError("Magnetometer readings have insufficient variation")

    normalized = (m - mean) / scale
    u, v = normalized.T

    D = np.column_stack([
        u*u, u*v, v*v, u, v, np.ones(len(u))
    ])

    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    q = Vt[-1]

    Q = np.array([
        [q[0], q[1]/2],
        [q[1]/2, q[2]]
    ])

    try:
        cn = np.linalg.solve(2*Q, -q[3:5])
        denominator = cn @ Q @ cn - q[5]

        Mn = Q / denominator
        A = np.diag(1/scale)
        M = A @ Mn @ A

        eigenvalues, eigenvectors = np.linalg.eigh(M)

        if not np.all(np.isfinite(eigenvalues)) or np.any(eigenvalues <= 0):
            raise ValueError("Non-positive ellipse eigenvalue")

        center = mean + scale*cn
        radius = float(np.linalg.det(M)**(-0.25))

        soft = (
            eigenvectors @ np.diag(np.sqrt(eigenvalues))
            @ eigenvectors.T
        ) * radius

        lengths = 1/np.sqrt(eigenvalues)
        ratio = float(max(lengths)/min(lengths))

        major = eigenvectors[:, np.argmin(eigenvalues)]
        tilt = float(np.degrees(np.arctan2(major[1], major[0])))

        print("Walking calibration: VALID normalized SVD ellipse")
        return center, soft, radius, ratio, tilt

    except (np.linalg.LinAlgError, ValueError) as err:
        print("WARNING: Normalized SVD did not produce a valid ellipse.")
        print("Reason:", err)
        print("Using a circle-only fallback: soft-iron distortion NOT estimated.")

        # Algebraic least-squares circle fit.
        A = np.column_stack([
            2*x, 2*y, np.ones(len(x))
        ])
        b = x*x + y*y

        solution = np.linalg.lstsq(A, b, rcond=None)[0]
        center = solution[:2]

        distances = np.linalg.norm(m-center, axis=1)
        radius = float(np.mean(distances))

        soft = np.eye(2)
        ratio = float("nan")
        tilt = float("nan")

        print("Circle-only center:", center)
        print("Circle-only mean radius:", radius)

        return center, soft, radius, ratio, tilt

def wrap(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def path_from_heading(t, heading, velocity):
    ve = velocity * np.sin(heading)
    vn = velocity * np.cos(heading)

    return np.column_stack([
        cumulative_trapezoid(ve, t, initial=0),
        cumulative_trapezoid(vn, t, initial=0)
    ])

circle, circle_gps = read_bag("walk_circle")
square, square_gps = read_bag("walk_square")

print("Walk circle IMU samples:", len(circle))
print("Walk square IMU samples:", len(square))
print("Walk square GPS samples:", len(square_gps))

# -----------------------------------------------------
# FIGURE 1: SVD ellipse fit on walk_circle.
# -----------------------------------------------------

raw_circle = circle[:, 1:3]
center, soft, radius, ratio, tilt = fit_ellipse(raw_circle)

cal_circle = (raw_circle - center) @ soft.T

theta = np.linspace(0, 2*np.pi, 500)
unit = np.column_stack([
    np.cos(theta),
    np.sin(theta)
])

fitted_ellipse = (
    center +
    radius * unit @ np.linalg.inv(soft).T
)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].scatter(
    raw_circle[:, 0], raw_circle[:, 1],
    s=2, alpha=0.25, label="Raw readings"
)

axes[0].plot(
    fitted_ellipse[:, 0],
    fitted_ellipse[:, 1],
    "r", lw=2, label="Ellipse fit (or circle fallback)"
)

axes[0].scatter(
    center[0], center[1],
    marker="x", s=80, color="black",
    label="Hard-iron center"
)

axes[0].set_title("Figure 1(a) — Raw Walking Magnetometer")
axes[0].legend(fontsize=8)

axes[1].scatter(
    cal_circle[:, 0], cal_circle[:, 1],
    s=2, alpha=0.25, label="Calibrated readings"
)

axes[1].plot(
    radius*np.cos(theta),
    radius*np.sin(theta),
    "r", lw=2, label="Fitted calibrated circle"
)

axes[1].set_title("Figure 1(b) — Calibrated Magnetometer")
axes[1].legend(fontsize=8)

for ax in axes:
    ax.set_xlabel("Magnetic X (µT)")
    ax.set_ylabel("Magnetic Y (µT)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(FIG / "figure1_svd_walking.png", dpi=200)
plt.close(fig)

# -----------------------------------------------------
# FIGURE 2: three heading estimates on walk_square.
# -----------------------------------------------------

t = square[:, 0]
raw_mag = square[:, 1:3]
cal_mag = (raw_mag - center) @ soft.T

raw_heading = np.unwrap(
    np.arctan2(-raw_mag[:, 1], raw_mag[:, 0])
)

mag_heading = np.unwrap(
    np.arctan2(-cal_mag[:, 1], cal_mag[:, 0])
)

gyro_z = square[:, 3]

# The notebook specifies starting at magnetic heading
# during the initial stationary period.
initial = t <= 5
mag_start = float(np.arctan2(
    np.mean(np.sin(mag_heading[initial])),
    np.mean(np.cos(mag_heading[initial]))
))

gyro_heading = (
    mag_start +
    cumulative_trapezoid(gyro_z, t, initial=0)
)

# Match only the initial 2π branch of the two curves.
mag_heading += 2*np.pi * np.round(
    (gyro_heading[0]-mag_heading[0])/(2*np.pi)
)

raw_heading += 2*np.pi * np.round(
    (gyro_heading[0]-raw_heading[0])/(2*np.pi)
)

fig, ax = plt.subplots(figsize=(12, 5))

ax.plot(
    t, np.degrees(raw_heading),
    label="Raw magnetometer", lw=0.8
)
ax.plot(
    t, np.degrees(mag_heading),
    label="Calibrated magnetometer (fallback if needed)", lw=1
)
ax.plot(
    t, np.degrees(gyro_heading),
    label="Integrated gyroscope Z", lw=1
)

ax.set_xlabel("Time since walking recording start (s)")
ax.set_ylabel("Unwrapped heading (degrees)")
ax.set_title("Figure 2 — Walk Square: Three Heading Estimates")
ax.legend()
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(FIG / "figure2_square_headings.png", dpi=200)
plt.close(fig)

# -----------------------------------------------------
# FIGURE 3: GNSS track and walking dead reckoning.
# -----------------------------------------------------

gt = square_gps[:, 0]
gps_xy = square_gps[:, 1:3].copy()
gps_xy -= gps_xy[0]

steps = np.linalg.norm(
    np.diff(gps_xy, axis=0), axis=1
)

dt = np.diff(gt)
segment_speed = np.divide(
    steps, dt,
    out=np.zeros_like(steps),
    where=dt > 0
)

# Follow teacher's rule: count GPS moves >= 0.5 m.
# Exclude implausibly high steps caused by poor GNSS fixes.
moving_intervals = (
    (steps >= 0.5) &
    (segment_speed <= 4.0) &
    (dt > 0)
)

distance_walked = float(
    np.sum(steps[moving_intervals])
)

walking_time = float(
    np.sum(dt[moving_intervals])
)

if walking_time <= 0:
    raise RuntimeError(
        "No valid walking GPS intervals: cannot estimate speed"
    )

average_speed = distance_walked / walking_time

# Map the GNSS moving intervals to the IMU timestamps.
interval_index = np.searchsorted(
    gt, t, side="right"
) - 1

valid_intervals = (
    (interval_index >= 0) &
    (interval_index < len(moving_intervals))
)

walking_mask = np.zeros(len(t), dtype=bool)
walking_mask[valid_intervals] = moving_intervals[
    interval_index[valid_intervals]
]

v = np.where(walking_mask, average_speed, 0.0)

declination = np.radians(DECLINATION_DEG)

mag_track = path_from_heading(
    t, mag_heading + declination, v
)

gyro_track = path_from_heading(
    t, gyro_heading + declination, v
)

# Translate tracks to first GNSS position (all relative to origin).
# Do not secretly rotate or rescale either track.
fig, ax = plt.subplots(figsize=(8, 8))

ax.plot(
    gps_xy[:, 0], gps_xy[:, 1],
    "k-", lw=2,
    label="Recorded GNSS track (not RTK Fixed)"
)

ax.plot(
    mag_track[:, 0], mag_track[:, 1],
    lw=1.2,
    label="DR: calibrated magnetic heading"
)

ax.plot(
    gyro_track[:, 0], gyro_track[:, 1],
    lw=1.2,
    label="DR: integrated gyro heading"
)

ax.scatter(
    [0], [0], color="green",
    marker="*", s=100, label="Start"
)

ax.set_xlabel("East displacement (m)")
ax.set_ylabel("North displacement (m)")
ax.set_title("Figure 3 — Walking Dead Reckoning vs GNSS")
ax.set_aspect("equal", adjustable="datalim")
ax.legend(fontsize=9)
ax.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(FIG / "figure3_square_tracks.png", dpi=200)
plt.close(fig)

# Endpoint closure and alignment diagnostics.
gps_end = gps_xy[-1]
mag_end = mag_track[-1]
gyro_end = gyro_track[-1]

gps_closure = float(np.linalg.norm(gps_end))
mag_closure = float(np.linalg.norm(mag_end))
gyro_closure = float(np.linalg.norm(gyro_end))

# Estimate orientation discrepancy from the first sustained
# moving stretch; this is reported, NOT applied to Figure 3.
moving_gps = np.flatnonzero(
    np.linalg.norm(gps_xy-gps_xy[0], axis=1) >= 15
)

rotation_text = "Unavailable"
if len(moving_gps):
    j = moving_gps[0]
    gps_direction = np.arctan2(
        gps_xy[j, 0], gps_xy[j, 1]
    )
    mag_direction = np.arctan2(
        mag_track[-1, 0] - mag_track[0, 0],
        mag_track[-1, 1] - mag_track[0, 1]
    )
    # Whole-track average direction is not a reliable rotation
    # for a closed square, so do not report this as alignment.

quality1, counts1 = np.unique(
    circle_gps[:, 3].astype(int), return_counts=True
)
quality2, counts2 = np.unique(
    square_gps[:, 3].astype(int), return_counts=True
)

report = f"""
========== Figures 1–3 / Question A2 ==========

walk_circle:
  IMU samples: {len(circle)}
  GPS samples: {len(circle_gps)}
  Fitted hard-iron X: {center[0]:.4f} µT
  Fitted hard-iron Y: {center[1]:.4f} µT
  Soft-iron axis ratio: {ratio:.4f}
  Ellipse major-axis tilt: {tilt:.2f} degrees
  Calibrated radius: {radius:.4f} µT
  Calibrated radial standard deviation:
      {np.std(np.linalg.norm(cal_circle,axis=1)):.4f} µT
  GNSS quality counts:
      {dict(zip(quality1.tolist(),counts1.tolist()))}

walk_square:
  IMU samples: {len(square)}
  GPS samples: {len(square_gps)}
  Estimated walking distance: {distance_walked:.2f} m
  Estimated moving time: {walking_time:.2f} s
  Average walking speed: {average_speed:.3f} m/s
  GNSS endpoint closure: {gps_closure:.2f} m
  Magnetic DR endpoint closure: {mag_closure:.2f} m
  Gyro DR endpoint closure: {gyro_closure:.2f} m
  GNSS quality counts:
      {dict(zip(quality2.tolist(),counts2.tolist()))}

Assumed magnetic declination: {DECLINATION_DEG:.1f} degrees
(approximate value from teacher notes, NOT NOAA-verified)

IMPORTANT:
- Walking GNSS does not satisfy the RTK Fixed requirement.
- Moving intervals are estimated from GNSS steps >= 0.5 m
  and <= 4 m/s, so poor-quality GNSS may affect the result.
- Figure 3 tracks are not GPS-aligned or scale-corrected.
- GNSS-to-DR rotation has not been robustly estimated.
- Sources of hard iron and physical walking observations
  must be confirmed from actual field notes.
"""

print(report)

(OUT / "walking_results.txt").write_text(report)

np.savez(
    OUT / "walking_analysis_data.npz",
    circle_raw=raw_circle,
    circle_calibrated=cal_circle,
    center=center,
    soft=soft,
    radius=radius,
    square_t=t,
    raw_heading=raw_heading,
    magnetic_heading=mag_heading,
    gyro_heading=gyro_heading,
    walking_velocity=v,
    gps_t=gt,
    gps_xy=gps_xy,
    magnetic_track=mag_track,
    gyro_track=gyro_track
)

print("Saved corrected Figures 1–3 and A2 results.")
