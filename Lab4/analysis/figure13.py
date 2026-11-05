from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

car = np.load(ROOT / "data/car/car_drive.npz", allow_pickle=False)
lidar = np.load(ROOT / "analysis/lidar_odometry.npz", allow_pickle=False)

def seconds(x):
    x = np.asarray(x, dtype=float).ravel()
    if len(x) > 1 and np.median(np.diff(x)) > 1e6:
        x = x * 1e-9
    return x

tl = seconds(lidar["t"])
pl = np.asarray(lidar["xy"], dtype=float)

tg = seconds(car["car_gps__t"])
lat = np.asarray(car["car_gps__lat"], dtype=float).ravel()
lon = np.asarray(car["car_gps__lon"], dtype=float).ravel()

ti = seconds(car["car_imu__t"])
gyro = np.asarray(car["car_imu__gyro"], dtype=float)
accel = np.asarray(car["car_imu__accel"], dtype=float)

valid = (
    np.isfinite(tg) &
    np.isfinite(lat) &
    np.isfinite(lon) &
    (np.abs(lat) <= 90) &
    (np.abs(lon) <= 180) &
    (np.abs(lat) > 1)
)
tg, lat, lon = tg[valid], lat[valid], lon[valid]

order = np.argsort(tg)
tg, lat, lon = tg[order], lat[order], lon[order]

if len(tg) < 30:
    raise RuntimeError("Insufficient car GPS fixes")

# Local east/north coordinates (meters).
R = 6371000.0
lat0 = np.deg2rad(lat[0])
east = R * np.cos(lat0) * np.deg2rad(lon - lon[0])
north = R * np.deg2rad(lat - lat[0])
pg = np.column_stack([east, north])

# Use only the common interval of the car PC sensors.
start_common = max(tl.min(), tg.min(), ti.min())
end_common = min(tl.max(), tg.max(), ti.max())

if end_common - start_common < 120:
    raise RuntimeError(
        f"Only {end_common-start_common:.1f}s common data. "
        "Check timestamps before aligning the sensors."
    )

# Pick a 90-second interval with 20 seconds of pre-outage history.
t0 = start_common + max(25.0, (end_common-start_common-90.0)*0.35)
t1 = t0 + 90.0

if t1 > end_common:
    raise RuntimeError("Cannot fit a 90-second outage interval")

def interp_xy(t, values, query):
    return np.column_stack([
        np.interp(query, t, values[:, 0]),
        np.interp(query, t, values[:, 1])
    ])

# Pre-outage GPS is allowed for initialization.
pre_t = tg[(tg >= t0-20) & (tg <= t0)]
if len(pre_t) < 5:
    raise RuntimeError("Not enough GPS samples before outage")

pre_gps = interp_xy(tg, pg, pre_t)
pre_lidar = interp_xy(tl, pl, pre_t)

# Fit a rigid 2D transform using PRE-outage points only.
A = pre_lidar - pre_lidar.mean(axis=0)
B = pre_gps - pre_gps.mean(axis=0)
U, _, Vt = np.linalg.svd(A.T @ B)
rot = U @ np.diag([1, np.linalg.det(U @ Vt)]) @ Vt

# Row-vector mapping: p_gps = p_lidar @ rot + translation.
translation = pre_gps.mean(axis=0) - pre_lidar.mean(axis=0) @ rot

# Evaluation grid at 1 Hz.
t = np.arange(t0, t1 + 0.001, 1.0)
truth = interp_xy(tg, pg, t)
truth -= truth[0]

# LiDAR-only: incremental ICP odometry initialized before outage.
lidar_path = interp_xy(tl, pl, t) @ rot + translation
lidar_path -= lidar_path[0]

# Estimate initial heading and speed from GPS BEFORE outage.
p0 = interp_xy(tg, pg, np.array([t0]))[0]
p_before = interp_xy(tg, pg, np.array([t0-5]))[0]
v0_vec = (p0 - p_before) / 5.0
v0 = np.linalg.norm(v0_vec)

if v0 < 0.3:
    print("WARNING: Outage begins near a stop; heading initialization is uncertain")

heading0 = np.arctan2(v0_vec[1], v0_vec[0])

# Car IMU: integrate yaw rate and forward acceleration.
# This is an approximate IMU-only baseline: vehicle axes and
# gravity/bias compensation require separate validation.
yawrate = np.interp(t, ti, gyro[:, 2])
forward_acc = np.interp(t, ti, accel[:, 0])

# Initial stationary bias proxy, if available.
early = accel[(ti >= start_common) & (ti <= start_common + 15), 0]
bias = np.median(early) if len(early) > 10 else 0.0
forward_acc = forward_acc - bias

dt = np.diff(t)
yaw = np.zeros(len(t))
yaw[1:] = np.cumsum(
    (yawrate[1:] + yawrate[:-1]) * 0.5 * dt
)

speed_imu = np.empty(len(t))
speed_imu[0] = v0
for k in range(1, len(t)):
    speed_imu[k] = speed_imu[k-1] + (
        forward_acc[k-1] + forward_acc[k]
    ) * 0.5 * dt[k-1]

def integrate(speed, heading):
    xy = np.zeros((len(t), 2))
    for k in range(1, len(t)):
        ds = 0.5*(speed[k-1] + speed[k]) * (t[k]-t[k-1])
        angle = 0.5*(heading[k-1] + heading[k])
        xy[k] = xy[k-1] + ds*np.array([
            np.cos(angle), np.sin(angle)
        ])
    return xy

imu_path = integrate(speed_imu, heading0 + yaw)

# Fusion: LiDAR displacement magnitude plus car IMU heading.
# This is an exploratory fusion method, not a validated estimator.
lidar_ds = np.linalg.norm(np.diff(lidar_path, axis=0), axis=1)
fusion = np.zeros_like(lidar_path)
for k in range(1, len(t)):
    angle = heading0 + 0.5*(yaw[k-1] + yaw[k])
    fusion[k] = fusion[k-1] + lidar_ds[k-1]*np.array([
        np.cos(angle), np.sin(angle)
    ])

results = {
    "IMU only (approx.)": imu_path,
    "LiDAR ICP": lidar_path,
    "LiDAR distance + gyro heading": fusion
}

fig, axes = plt.subplots(1, 2, figsize=(13, 5))

axes[0].plot(truth[:, 0], truth[:, 1],
             "k-", linewidth=2, label="Car GPS reference")
for label, path in results.items():
    axes[0].plot(path[:, 0], path[:, 1], label=label)

axes[0].set_xlabel("East displacement (m)")
axes[0].set_ylabel("North displacement (m)")
axes[0].set_title("90-second Simulated GNSS Outage")
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].grid(alpha=0.2)
axes[0].legend(fontsize=8)

for label, path in results.items():
    error = np.linalg.norm(path-truth, axis=1)
    axes[1].plot(t-t0, error, label=label)
    print(f"{label}:")
    for sec in (30, 60, 90):
        idx = int(np.argmin(np.abs((t - t0) - sec)))
        print(f"  {sec} s error = {error[idx]:.2f} m")

axes[1].set_xlabel("Time since GNSS outage (s)")
axes[1].set_yscale("log")
axes[1].set_ylim(bottom=0.01)
axes[1].set_ylabel("Position error vs car GPS (m, log scale)")
axes[1].set_title("Position Error During Outage")
axes[1].grid(alpha=0.2)
axes[1].legend(fontsize=8)

fig.tight_layout()
fig.savefig(OUT / "figure13_gnss_outage.png", dpi=200)
plt.close(fig)

print(f"Outage interval: {t0:.2f} to {t1:.2f}")
print("Reference: Mustang car GPS (NOT RTK fixed)")
print("Saved Figure 13")
