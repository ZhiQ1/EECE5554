from argparse import ArgumentParser
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

parser = ArgumentParser()
parser.add_argument("--data", type=Path, default=Path.home() / "data/team1_drive")
parser.add_argument("--kit", type=Path, default=Path.home() / "f26-dr-kit")
parser.add_argument("--output", type=Path, default=Path.cwd())
args = parser.parse_args()
data_dir, kit_dir = args.data, args.kit
args.output.mkdir(parents=True, exist_ok=True)

def load(path, time_column):
    rows = np.genfromtxt(path, delimiter=",", names=True)
    rows = rows[rows[time_column].argsort()]
    _, indices = np.unique(rows[time_column], return_index=True)
    return rows[indices]

imu = load(data_dir / "team1_drive_vn100.csv", "t_laptop")
can = load(data_dir / "team1_drive_can.csv", "t_car")
ref = load(kit_dir / "reference/team1_drive_gnss.csv", "t")

imu_t = imu["t_laptop"] - 2.8687
can_t = can["t_car"]
ref_t = ref["t"]
start = max(imu_t[0], can_t[0], ref_t[0])
end = min(imu_t[-1], can_t[-1], ref_t[-1])
if end <= start:
    raise SystemExit("No common time interval.")

inside = (imu_t > start) & (imu_t < end)
t = np.r_[start, imu_t[inside], end]
dt = np.diff(t)
speed = np.interp(t, can_t, can["speed_mps"])
gyro_z = np.interp(t, imu_t, imu["gz"])

x, y, z, w = (imu[key] for key in ("qx", "qy", "qz", "qw"))
quaternion_yaw = np.unwrap(np.arctan2(
    2 * (w * z + x * y), w*w + x*x - y*y - z*z
))
yaw_observation = np.interp(t, imu_t, quaternion_yaw)
yaw = np.empty_like(t)
yaw[0] = yaw_observation[0]
for i, step in enumerate(dt, start=1):
    prediction = yaw[i - 1] + 0.5 * (gyro_z[i - 1] + gyro_z[i]) * step
    residual = yaw_observation[i] - prediction
    residual = np.arctan2(np.sin(residual), np.cos(residual))
    yaw[i] = prediction + (1 - np.exp(-step / 2.0)) * residual

ref_e = np.interp(t, ref_t, ref["easting"])
ref_n = np.interp(t, ref_t, ref["northing"])
delta_e = ref_e - ref_e[0]
delta_n = ref_n - ref_n[0]
candidates = np.flatnonzero(np.hypot(delta_e, delta_n) >= 15)
if candidates.size == 0:
    raise SystemExit("No 15 m displacement for initial heading alignment.")
anchor = candidates[0]

# Convert clockwise NED heading to counterclockwise angle from UTM east.
heading = -yaw
course = np.arctan2(delta_n[anchor], delta_e[anchor])
initial_motion = np.sum(
    speed[:anchor + 1] * np.exp(1j * heading[:anchor + 1])
)
heading += course - np.angle(initial_motion)

velocity_e = speed * np.cos(heading)
velocity_n = speed * np.sin(heading)
east = np.r_[0, np.cumsum(0.5 * (velocity_e[:-1] + velocity_e[1:]) * dt)]
north = np.r_[0, np.cumsum(0.5 * (velocity_n[:-1] + velocity_n[1:]) * dt)]

fig, ax = plt.subplots(figsize=(8, 7))
ax.plot(delta_e, delta_n, label="DGPS reference", linewidth=2)
ax.plot(east, north, label="dr_vn100", linewidth=1.4)
ax.scatter(0, 0, color="black", s=20, label="Start", zorder=3)
ax.set(xlabel="East from start (m)", ylabel="North from start (m)",
       title="team1_drive · Dead reckoning vs. DGPS · No simulated outage")
ax.set_aspect("equal", adjustable="box")
ax.grid(alpha=0.25)
ax.legend()
fig.tight_layout()
output = args.output / "team1_drive_dr_vn100_vs_dgps.png"
fig.savefig(output, dpi=180)
print(f"Duration: {end - start:.1f} s")
print(f"Endpoint separation: {np.hypot(east[-1] - delta_e[-1], north[-1] - delta_n[-1]):.1f} m")
print(output)
