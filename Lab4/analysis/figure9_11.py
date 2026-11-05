from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import correlate, correlation_lags
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

car = np.load(ROOT / "data/car/car_drive.npz", allow_pickle=False)

def normalize_time(t):
    t = np.asarray(t, dtype=float).ravel()
    if len(t) < 2:
        raise ValueError("Insufficient timestamps")
    step = np.median(np.diff(t))
    if step > 1e6:
        t = t * 1e-9
    return t - t[0]

def read_laptop_bag():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data/car_drive"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    imu_type = get_message(types["/imu"])
    gps_type = get_message(types["/gps"])

    imu, gps = [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == "/imu":
            m = deserialize_message(raw, imu_type)
            imu.append([
                stamp * 1e-9,
                m.imu.angular_velocity.z,
                m.imu.linear_acceleration.x,
                m.imu.linear_acceleration.y
            ])
        elif topic == "/gps":
            m = deserialize_message(raw, gps_type)
            gps.append([
                stamp * 1e-9,
                m.utm_easting,
                m.utm_northing
            ])
    return np.asarray(imu), np.asarray(gps)

imu, gps = read_laptop_bag()
print("Laptop IMU:", imu.shape, "GPS:", gps.shape)

# Figure 9: vehicle motion response, using Car PC signals.
tgyro = normalize_time(car["car_imu__t"])
gyro = np.asarray(car["car_imu__gyro"], dtype=float)
if gyro.ndim != 2 or gyro.shape[1] < 3:
    raise ValueError(f"Unexpected car gyro shape: {gyro.shape}")

ts = normalize_time(car["steering__t"])
steer = np.asarray(car["steering__steering_wheel_angle"], dtype=float).ravel()

fig, ax = plt.subplots(figsize=(11, 4))
ax.plot(tgyro, gyro[:, 2], lw=0.8, label="Car gyro Z (rad/s)")
ax.set_xlabel("Car PC time since recording start (s)")
ax.set_ylabel("Yaw angular velocity (rad/s)")
ax2 = ax.twinx()
ax2.plot(ts, steer, lw=0.7, alpha=0.6, color="tab:orange",
         label="Steering wheel angle (native units)")
ax2.set_ylabel("Steering wheel angle (native units)")
ax.set_title("Figure 9 — Steering and Vehicle Yaw Response")
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "figure9_motion_response.png", dpi=180)
plt.close(fig)
print("Saved Figure 9")

# Figure 10: GPS path and a gyro-heading / GPS-speed reconstruction.
# Diagnostic only: speed comes from GPS, not from accelerometer integration.
tg = gps[:, 0] - gps[0, 0]
east = gps[:, 1] - gps[0, 1]
north = gps[:, 2] - gps[0, 2]
ti = imu[:, 0] - gps[0, 0]

# GPS-derived speed from consecutive position fixes.
dtg = np.diff(tg)
steps = np.hypot(np.diff(east), np.diff(north))
speed = np.divide(steps, dtg, out=np.zeros_like(steps),
                  where=dtg > 0)
speed_t = (tg[:-1] + tg[1:]) / 2

# Integrate IMU yaw relative to initial direction.
yaw = np.zeros(len(ti))
yaw[1:] = np.cumsum(
    0.5 * (imu[1:, 1] + imu[:-1, 1]) * np.diff(ti)
)
yaw_g = np.interp(tg, ti, yaw)

# Choose initial heading from early nonstationary GPS displacement.
moving = np.flatnonzero(steps > 0.5)
if len(moving) == 0:
    raise ValueError("GPS track contains no measurable movement")
j = moving[0]
heading0 = np.arctan2(np.diff(north)[j], np.diff(east)[j])

# Both signs are tested because VN-100 axis orientation needs validation.
paths = {}
for sign in (1, -1):
    h = heading0 + sign * (yaw_g - yaw_g[j])
    dx = speed * dtg * np.cos((h[:-1] + h[1:]) / 2)
    dy = speed * dtg * np.sin((h[:-1] + h[1:]) / 2)
    paths[sign] = (
        np.r_[0, np.cumsum(dx)],
        np.r_[0, np.cumsum(dy)]
    )

fig, ax = plt.subplots(figsize=(7, 7))
ax.plot(east, north, label="GPS recorded path", lw=2)
for sign in (1, -1):
    x, y = paths[sign]
    ax.plot(x, y, lw=1, alpha=0.75,
            label=f"Gyro + GPS speed (yaw sign {sign:+d})")
ax.scatter([0], [0], marker="o", label="Start")
ax.set_xlabel("East displacement (m)")
ax.set_ylabel("North displacement (m)")
ax.set_aspect("equal", adjustable="datalim")
ax.set_title("Figure 10 — Preliminary Path Reconstruction")
ax.legend(fontsize=8)
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "figure10_preliminary_path.png", dpi=180)
plt.close(fig)
print("Saved Figure 10 (GPS-assisted diagnostic, NOT IMU-only dead reckoning)")

# Figure 11: two-clock alignment from matching gyro turn patterns.
# Resample separately to a common relative 10 Hz grid.
tcar = normalize_time(car["car_imu__t"])
gz_car = gyro[:, 2]

tlaptop = imu[:, 0] - imu[0, 0]
gz_laptop = imu[:, 1]

duration = min(tcar[-1], tlaptop[-1])
grid = np.arange(0, duration, 0.1)
a = np.interp(grid, tcar, gz_car)
b = np.interp(grid, tlaptop, gz_laptop)

def standardize(v):
    v = v - np.median(v)
    return v / (np.std(v) + 1e-12)

a = standardize(a)
b = standardize(b)

# Restrict offset search to +/- 60 seconds.
lags = correlation_lags(len(a), len(b), mode="full")
valid = np.abs(lags * 0.1) <= 60
results = []
for sign in (1, -1):
    corr = correlate(a, sign * b, mode="full", method="fft")
    # Normalize roughly by overlap length to compare shifted matches.
    overlap = len(a) - np.abs(lags)
    score = corr / np.maximum(overlap, 1)
    score[~valid] = -np.inf
    k = int(np.argmax(score))
    results.append((float(score[k]), int(lags[k]), sign))

best_score, best_lag, best_sign = max(results)
offset = best_lag * 0.1

fig, ax = plt.subplots(figsize=(11, 4))
ax.plot(grid, a, lw=0.9, label="Car PC gyro Z (normalized)")
shifted = np.interp(grid - offset, grid, best_sign * b,
                    left=np.nan, right=np.nan)
ax.plot(grid, shifted, lw=0.8, alpha=0.75,
        label=f"Laptop gyro (shift={offset:.1f}s, sign={best_sign:+d})")
ax.set_xlabel("Car PC relative time (s)")
ax.set_ylabel("Normalized gyro Z")
ax.set_title("Figure 11 — Preliminary Two-Computer Clock Alignment")
ax.legend(fontsize=8)
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "figure11_clock_alignment.png", dpi=180)
plt.close(fig)

print("Saved Figure 11")
print(f"Estimated relative lag: {offset:.2f} s")
print(f"Gyro sign factor: {best_sign:+d}")
print(f"Correlation score: {best_score:.3f}")
print("WARNING: Inspect the alignment visually before accepting the offset.")
