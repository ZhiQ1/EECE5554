from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from scipy.ndimage import median_filter
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis"
(OUT / "figures").mkdir(exist_ok=True)

# Q8: Car-PC relative time = Laptop relative time + L.
L = 5.4213
G = 9.80665

# Read the laptop ROS Bag using the teacher's message fields.
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(ROOT / "data/car_drive"),
        storage_id="mcap"
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
        q = m.imu.orientation
        a = m.imu.linear_acceleration
        w = m.imu.angular_velocity

        # Quaternion -> pitch, matching the teacher's formula.
        pitch = np.arcsin(np.clip(
            2 * (q.w*q.y - q.z*q.x), -1, 1
        ))

        imu_rows.append([
            t,
            a.x,
            a.y,
            w.z,
            pitch
        ])

    elif topic == "/gps":
        m = deserialize_message(raw, gps_type)
        gps_rows.append([
            t,
            m.utm_easting,
            m.utm_northing,
            m.fix_quality
        ])

imu = np.asarray(imu_rows, dtype=float)
gps = np.asarray(gps_rows, dtype=float)

t = imu[:, 0]
ax = imu[:, 1]
gyro_z = imu[:, 3]
pitch = imu[:, 4]

tg = gps[:, 0]
east = gps[:, 1]
north = gps[:, 2]
quality = gps[:, 3].astype(int)

# GNSS velocity from UTM differences.
dtg = np.diff(tg)
distance = np.hypot(np.diff(east), np.diff(north))

vgps = np.divide(
    distance, dtg,
    out=np.zeros_like(distance),
    where=dtg > 0
)
tvgps = (tg[:-1] + tg[1:]) / 2
qgps = quality[1:]

# Filter only implausible GPS speed spikes for unit estimation.
vgps_clean = vgps.copy()
vgps_clean[(vgps < 0) | (vgps > 45)] = np.nan

car = np.load(
    ROOT / "data/car/car_drive.npz",
    allow_pickle=False
)

# Try available vehicle speed channels and plausible units.
speed_keys = [
    "vehicle_velocity__vehicle_velocity_propulsion",
    "vehicle_velocity__vehicle_velocity_brake",
    "wheel_speeds__front_left",
    "wheel_speeds__front_right",
    "wheel_speeds__rear_left",
    "wheel_speeds__rear_right",
]

candidates = []
for key in speed_keys:
    if key not in car.files:
        continue

    tk = key.split("__")[0] + "__t"
    if tk not in car.files:
        continue

    ct = np.asarray(car[tk], dtype=float)
    cv = np.asarray(car[key], dtype=float)

    if cv.ndim != 1 or len(cv) != len(ct):
        continue

    # Match car data to laptop time using Q8.
    v_native = np.interp(
        tvgps + L, ct, cv,
        left=np.nan, right=np.nan
    )

    for unit, factor in [
        ("m/s", 1.0),
        ("km/h", 1/3.6),
        ("mph", 0.44704),
    ]:
        vs = np.abs(v_native) * factor

        valid = (
            np.isfinite(vgps_clean) &
            np.isfinite(vs) &
            (vgps_clean > 3) &
            (vgps_clean < 35)
        )

        if valid.sum() < 20:
            continue

        mae = np.median(
            np.abs(vs[valid] - vgps_clean[valid])
        )

        candidates.append((mae, key, unit, factor))

if not candidates:
    raise RuntimeError(
        "Could not identify vehicle speed units. "
        "Inspect Car PC speed channels."
    )

candidates.sort(key=lambda x: x[0])
mae, speed_key, speed_unit, factor = candidates[0]

car_t = np.asarray(
    car[speed_key.split("__")[0] + "__t"],
    dtype=float
)
car_speed = np.abs(np.asarray(car[speed_key], dtype=float)) * factor

# Car speed in laptop time.
vcar = np.interp(
    t + L, car_t, car_speed,
    left=np.nan, right=np.nan
)

# Initial stationary acceleration bias.
initial = t <= 20
bias = float(np.median(ax[initial]))
pitch0 = float(np.median(pitch[initial]))

# Stage 1: uncorrected acceleration integration.
v_raw = cumulative_trapezoid(
    ax, t, initial=0.0
)

# Stage 2: remove initial accelerometer bias.
a_bias = ax - bias
v_bias = cumulative_trapezoid(
    a_bias, t, initial=0.0
)

# Stage 3: correct varying gravity projection.
# Use pitch relative to the initial stationary orientation:
# the stationary component is already included in bias.
hill_term = G * (
    np.sin(pitch) - np.sin(pitch0)
)
a_hill = a_bias - hill_term

v_hill = cumulative_trapezoid(
    a_hill, t, initial=0.0
)

# Stage 4: zero-velocity updates using observed car stops.
# These are stop constraints, not GPS speed inserted into integration.
stationary = (
    np.isfinite(vcar) &
    (vcar < 0.5) &
    (np.abs(gyro_z) < 0.04)
)

# Suppress isolated stop detections.
stationary = median_filter(
    stationary.astype(np.uint8),
    size=81
).astype(bool)

# Find stationary runs of at least 1 second.
starts = np.where(np.diff(
    np.r_[False, stationary, False].astype(int)
) == 1)[0]
ends = np.where(np.diff(
    np.r_[False, stationary, False].astype(int)
) == -1)[0]

anchors = [0]
for start, end in zip(starts, ends):
    if end - start >= 40:
        anchors.append((start + end - 1) // 2)

anchors = np.unique(np.asarray(anchors, dtype=int))

# Force zero velocity at each known stationary anchor,
# removing accumulated drift by interpolation.
if len(anchors) >= 2:
    drift = np.interp(
        t, t[anchors], v_hill[anchors]
    )
else:
    drift = np.zeros_like(v_hill)

v_corrected = np.maximum(
    v_hill - drift, 0.0
)
v_corrected[stationary] = 0.0

# For fair comparison, interpolate GNSS speed onto IMU times.
gps_at_imu = np.interp(
    t, tvgps, vgps_clean,
    left=np.nan, right=np.nan
)

def rms_vs_gps(pred):
    valid = (
        np.isfinite(pred) &
        np.isfinite(gps_at_imu) &
        (gps_at_imu <= 35)
    )
    if valid.sum() == 0:
        return np.nan
    return float(np.sqrt(np.mean(
        (pred[valid] - gps_at_imu[valid])**2
    )))

rms_raw = rms_vs_gps(v_raw)
rms_bias = rms_vs_gps(v_bias)
rms_hill = rms_vs_gps(v_hill)
rms_final = rms_vs_gps(v_corrected)
rms_car = rms_vs_gps(vcar)

# Figure 8: exact two-panel structure requested.
fig, axes = plt.subplots(
    2, 1, figsize=(12, 8), sharex=True
)

axes[0].plot(
    t, v_raw,
    label="Raw accelerometer integration",
    linewidth=1
)
axes[0].set_ylabel("Forward velocity (m/s)")
axes[0].set_title(
    "Figure 8(a) — Velocity Before Corrections"
)
axes[0].legend()
axes[0].grid(alpha=0.25)

axes[1].plot(
    t, v_corrected,
    label="Accelerometer: bias + hill + stop corrected",
    linewidth=1.2
)
axes[1].plot(
    tvgps, vgps_clean,
    label="Recorded GNSS speed (quality not RTK-fixed)",
    linewidth=0.8,
    alpha=0.7
)
axes[1].plot(
    t, vcar,
    label=f"Mustang speed ({speed_key})",
    linewidth=0.9,
    alpha=0.8
)

axes[1].set_xlabel("Laptop recording time (s)")
axes[1].set_ylabel("Forward speed (m/s)")
axes[1].set_title(
    "Figure 8(b) — Corrected Velocity and References"
)
axes[1].legend(fontsize=8)
axes[1].grid(alpha=0.25)

fig.tight_layout()
fig.savefig(
    OUT / "figures/figure8_corrected_velocity.png",
    dpi=200
)
plt.close(fig)

results = f"""
========== Figure 8 / Question 4 ==========

Laptop IMU samples: {len(t)}
GNSS samples: {len(tg)}

Selected Mustang speed: {speed_key}
Inferred speed unit: {speed_unit}
Unit conversion factor to m/s: {factor:.6f}
Median absolute speed difference: {mae:.3f} m/s

Initial 20 s accelerometer bias: {bias:.6f} m/s^2
Initial pitch: {np.degrees(pitch0):.3f} degrees
Maximum absolute hill correction: {np.max(np.abs(hill_term)):.4f} m/s^2
Detected stationary anchors: {len(anchors)}

RMS speed error relative to recorded GNSS:
  Raw integration:          {rms_raw:.3f} m/s
  After bias correction:    {rms_bias:.3f} m/s
  After hill correction:    {rms_hill:.3f} m/s
  After stop correction:    {rms_final:.3f} m/s
  Mustang speed:            {rms_car:.3f} m/s

GNSS fix quality counts:
{dict(zip(*np.unique(quality, return_counts=True)))}

IMPORTANT:
- Reference GNSS has no RTK Fixed coverage.
- Speed unit was inferred by comparing candidate signals.
- Stop constraints use vehicle speed and gyro.
- This is a preliminary implementation requiring plot inspection.
"""

print(results)

(OUT / "q4_velocity_results.txt").write_text(results)

np.savez(
    OUT / "q4_velocity_data.npz",
    t=t,
    v_raw=v_raw,
    v_bias=v_bias,
    v_hill=v_hill,
    v_corrected=v_corrected,
    v_car=vcar,
    gps_t=tvgps,
    gps_v=vgps_clean,
    gps_quality=qgps,
    pitch=pitch,
    acceleration=ax,
    hill_term=hill_term,
    gyro_z=gyro_z
)

print("Saved Figure 8 and Q4 results.")
