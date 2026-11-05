from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

d = np.load(ROOT / "data/car/car_drive.npz", allow_pickle=False)

points = d["scans__xy"]
offsets = d["scans__offsets"]
times = d["scans__t"]

print("LiDAR scans:", len(times))
print("LiDAR points:", len(points))

def scan(i, max_points=800):
    p = points[offsets[i]:offsets[i+1]]
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(np.linalg.norm(p, axis=1) > 2) &
          (np.linalg.norm(p, axis=1) < 45)]
    if len(p) > max_points:
        ids = np.linspace(0, len(p)-1, max_points).astype(int)
        p = p[ids]
    return p.astype(float)

def transform(p, theta, shift):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return p @ R.T + shift

def icp(source, target, theta0=0.0, shift0=None, iterations=15):
    if len(source) < 30 or len(target) < 30:
        return theta0, np.zeros(2), np.nan

    theta = float(theta0)
    shift = np.zeros(2) if shift0 is None else np.array(shift0, dtype=float)

    tree = cKDTree(target)

    for _ in range(iterations):
        moved = transform(source, theta, shift)
        distances, indices = tree.query(moved)

        threshold = min(2.0, np.percentile(distances, 80))
        good = distances <= threshold

        if good.sum() < 20:
            break

        A = moved[good]
        B = target[indices[good]]

        ca = A.mean(axis=0)
        cb = B.mean(axis=0)

        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        delta_theta = np.arctan2(R[1, 0], R[0, 0])
        delta_shift = cb - R @ ca

        shift = R @ shift + delta_shift
        theta += delta_theta

        if abs(delta_theta) < 1e-5 and np.linalg.norm(delta_shift) < 1e-4:
            break

    final = transform(source, theta, shift)
    distances, _ = tree.query(final)
    residual = np.median(distances)

    return theta, shift, residual

# Use Car PC speed and gyro as ICP initialization.
speed_t = d["vehicle_velocity__t"]
speed = d["vehicle_velocity__vehicle_velocity_propulsion"]

gyro_t = d["lidar_imu__t"]
gyro_z = d["lidar_imu__gyro"][:, 2]
gyro_z = np.nan_to_num(gyro_z)

# Analyze every second LiDAR frame (~1 second apart).
ids = np.arange(0, len(times), 2)
if ids[-1] != len(times)-1:
    ids = np.r_[ids, len(times)-1]

trajectory = np.zeros((len(ids), 2))
heading = np.zeros(len(ids))
residuals = np.full(len(ids), np.nan)

for k in range(1, len(ids)):
    i0, i1 = ids[k-1], ids[k]

    old = scan(i0)
    new = scan(i1)

    t0, t1 = times[i0], times[i1]
    dt = t1 - t0

    mid = (t0 + t1) / 2
    v = np.interp(mid, speed_t, speed)
    omega = np.interp(mid, gyro_t, gyro_z)

    # Initial transformation from new scan coordinates to old scan.
    theta0 = omega * dt
    shift0 = np.array([v * dt, 0.0])

    theta, shift, error = icp(
        new, old, theta0=theta0, shift0=shift0
    )

    # Convert local displacement into a global trajectory.
    c, s = np.cos(heading[k-1]), np.sin(heading[k-1])
    R = np.array([[c, -s], [s, c]])

    trajectory[k] = trajectory[k-1] + R @ shift
    heading[k] = heading[k-1] + theta
    residuals[k] = error

    if k % 100 == 0:
        print(f"ICP {k}/{len(ids)-1}, median residual={error:.3f} m", flush=True)

np.savez(
    ROOT / "analysis/lidar_odometry.npz",
    t=times[ids],
    xy=trajectory,
    heading=heading,
    residual=residuals
)

fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].plot(trajectory[:, 0], trajectory[:, 1], linewidth=1.3)
axes[0].scatter(trajectory[0, 0], trajectory[0, 1],
                marker="o", label="Start")
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].set_xlabel("Relative X (m)")
axes[0].set_ylabel("Relative Y (m)")
axes[0].set_title("LiDAR ICP Odometry: Estimated Path")
axes[0].legend()
axes[0].grid(alpha=0.25)

axes[1].plot(times[ids], residuals, linewidth=1)
axes[1].set_xlabel("Car PC Time (s)")
axes[1].set_ylabel("Median Nearest-Neighbor Residual (m)")
axes[1].set_title("ICP Scan-Matching Residual")
axes[1].grid(alpha=0.25)

fig.tight_layout()
fig.savefig(OUT / "figure12_lidar_icp.png", dpi=200)
plt.close(fig)

print("Saved Figure 12")
print("Median ICP residual:", np.nanmedian(residuals))
print("Estimated path length (m):",
      np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1)))
print("Saved analysis/lidar_odometry.npz")
