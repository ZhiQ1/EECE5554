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

tl = np.asarray(lidar["t"], dtype=float)
pl = np.asarray(lidar["xy"], dtype=float)

tg = np.asarray(car["car_gps__t"], dtype=float)
lat = np.asarray(car["car_gps__lat"], dtype=float)
lon = np.asarray(car["car_gps__lon"], dtype=float)

valid = (
    np.isfinite(tg) &
    np.isfinite(lat) &
    np.isfinite(lon) &
    (np.abs(lat) > 1) &
    (np.abs(lat) <= 90) &
    (np.abs(lon) <= 180)
)

tg = tg[valid]
lat = lat[valid]
lon = lon[valid]

order = np.argsort(tg)
tg, lat, lon = tg[order], lat[order], lon[order]

# Convert GPS coordinates to local East/North (meters).
R_earth = 6371000.0
lat0 = np.deg2rad(lat[0])

east = R_earth * np.cos(lat0) * np.deg2rad(lon - lon[0])
north = R_earth * np.deg2rad(lat - lat[0])

pg = np.column_stack([east, north])

# Compare at GPS timestamps, keeping only common coverage.
common = (tg >= tl[0]) & (tg <= tl[-1])
t = tg[common]
gps = pg[common]

if len(t) < 40:
    raise RuntimeError("Not enough overlapping GPS and LiDAR samples")

def interpolate_xy(source_t, source_xy, query_t):
    return np.column_stack([
        np.interp(query_t, source_t, source_xy[:, 0]),
        np.interp(query_t, source_t, source_xy[:, 1])
    ])

estimated = interpolate_xy(tl, pl, t)

# Determine initial orientation from the first MOVING section.
# Do not estimate heading from the stationary first 30 seconds.
gps_steps = np.linalg.norm(np.diff(gps, axis=0), axis=1)
distance = np.r_[0.0, np.cumsum(gps_steps)]

# Use the first 15-100 m of actual movement.
calibration = (distance >= 15.0) & (distance <= 100.0)

if calibration.sum() < 5:
    raise RuntimeError("Not enough moving GPS points for heading alignment")

A = estimated[calibration]
B = gps[calibration]

A_center = A.mean(axis=0)
B_center = B.mean(axis=0)

U, _, Vt = np.linalg.svd(
    (A - A_center).T @ (B - B_center)
)

D = np.diag([1.0, np.linalg.det(U @ Vt)])
rotation = U @ D @ Vt

# Apply rotation to the entire LiDAR trajectory.
# Keep the original start point as the origin.
aligned = (estimated - estimated[0]) @ rotation + gps[0]

angle = np.degrees(
    np.arctan2(rotation[0, 1], rotation[0, 0])
)

print(f"Initial alignment rotation: {angle:.2f} degrees")
print(f"Alignment used {calibration.sum()} moving samples")
print(
    f"Calibration interval: "
    f"{t[calibration][0]-t[0]:.1f} to "
    f"{t[calibration][-1]-t[0]:.1f} s"
)
print("WARNING: Orientation calibrated using the first 100 m of GPS.")
print("Evaluate independent odometry accuracy after this interval.")

# Both paths start at the same point.
gps = gps - gps[0]
aligned = aligned - aligned[0]

error = np.linalg.norm(aligned - gps, axis=1)
elapsed = t - t[0]

def first_exceedance(threshold):
    bad = np.flatnonzero(error > threshold)
    if len(bad) == 0:
        return float(elapsed[-1]), True
    return float(elapsed[bad[0]]), False

def time_below(threshold):
    dt = np.diff(elapsed)
    return float(np.sum(dt[error[:-1] <= threshold]))

gps_length = float(np.sum(np.linalg.norm(np.diff(gps, axis=0), axis=1)))
lidar_length = float(np.sum(np.linalg.norm(np.diff(aligned, axis=0), axis=1)))

end_error = float(error[-1])
end_percent = end_error / gps_length * 100 if gps_length > 0 else np.nan

t2, all2 = first_exceedance(2)
t10, all10 = first_exceedance(10)

print("\n========= Q9 LiDAR Evaluation =========")
print(f"Evaluation duration: {elapsed[-1]:.2f} s")
print(f"GPS path length: {gps_length:.2f} m")
print(f"LiDAR path length: {lidar_length:.2f} m")
print(f"Scale ratio (LiDAR/GPS): {lidar_length/gps_length:.4f}")
print(f"Endpoint position error: {end_error:.2f} m")
print(f"Endpoint error / GPS distance: {end_percent:.3f}%")
print(f"First exceed 2 m: {t2:.2f} s")
print(f"First exceed 10 m: {t10:.2f} s")
print(f"Total sampled time within 2 m: {time_below(2):.2f} s")
print(f"Total sampled time within 10 m: {time_below(10):.2f} s")
print(f"Maximum position error: {np.max(error):.2f} m")
print(f"Median position error: {np.median(error):.2f} m")
print("Reference: Mustang GPS, NOT RTK Fixed")

# Figure 12: trajectory comparison and error threshold plot.
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

ax = axes[0]
ax.plot(gps[:, 0], gps[:, 1],
        color="black", lw=1.7, label="Mustang GPS reference")
ax.plot(aligned[:, 0], aligned[:, 1],
        color="tab:blue", lw=1.4, label="LiDAR ICP odometry")
ax.scatter([0], [0], s=50, marker="o",
           color="tab:green", zorder=5, label="Start")
ax.set_xlabel("East displacement (m)")
ax.set_ylabel("North displacement (m)")
ax.set_title("Figure 12(a) — LiDAR vs GPS Trajectory")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(alpha=0.2)
ax.legend(fontsize=9)

ax = axes[1]
ax.plot(elapsed, error, lw=1.2,
        color="tab:blue", label="LiDAR position error")
ax.axhline(2, linestyle="--", color="tab:green",
           label="2 m threshold")
ax.axhline(10, linestyle="--", color="tab:red",
           label="10 m threshold")
ax.set_xlabel("Time since evaluation start (s)")
ax.set_ylabel("Position error relative to GPS (m)")
ax.set_title("Figure 12(b) — LiDAR Position Error")
ax.grid(alpha=0.2)
ax.legend(fontsize=9)

fig.tight_layout()
fig.savefig(OUT / "figure12_lidar_vs_gps.png", dpi=200)
plt.close(fig)

summary = f"""Q9 LiDAR GPS-referenced evaluation
Evaluation duration: {elapsed[-1]:.2f} s
GPS path length: {gps_length:.2f} m
LiDAR path length: {lidar_length:.2f} m
Scale ratio: {lidar_length/gps_length:.4f}
Endpoint position error: {end_error:.2f} m
Endpoint error percentage: {end_percent:.3f}%
First exceed 2 m: {t2:.2f} s
First exceed 10 m: {t10:.2f} s
Total sampled time within 2 m: {time_below(2):.2f} s
Total sampled time within 10 m: {time_below(10):.2f} s
Maximum error: {np.max(error):.2f} m
Median error: {np.median(error):.2f} m
Reference: Mustang GPS; RTK Fixed unavailable.
Alignment: rigid transform fitted on initial 30 seconds only.
"""

(ROOT / "analysis/q9_lidar_results.txt").write_text(summary)

np.savez(
    ROOT / "analysis/q9_lidar_evaluation.npz",
    t=t,
    gps_xy=gps,
    lidar_xy=aligned,
    error=error
)

print("\nSaved Figure 12 GPS comparison")
print("Saved Q9 results and evaluation data")
