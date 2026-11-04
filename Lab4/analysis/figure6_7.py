from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

def read_imu(name):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data" / name),
            storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )

    types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }
    msg_type = get_message(types["/imu"])
    samples = []

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic != "/imu":
            continue

        m = deserialize_message(raw, msg_type)
        samples.append([
            stamp * 1e-9,
            m.imu.angular_velocity.z,
            m.mag_field.magnetic_field.x * 1e6,
            m.mag_field.magnetic_field.y * 1e6
        ])

    return np.asarray(samples, dtype=float)

def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Use the same car calibration for the car drive.
circles = read_imu("car_circles")
drive = read_imu("car_drive")

cal_xy = circles[:, 2:4]
offset = (cal_xy.max(axis=0) + cal_xy.min(axis=0)) / 2
centered = cal_xy - offset

cov = np.cov(centered, rowvar=False)
values, vectors = np.linalg.eigh(cov)
values = np.maximum(values, 1e-10)
correction = (
    vectors @ np.diag(1 / np.sqrt(values)) @ vectors.T
)

drive_xy = (drive[:, 2:4] - offset) @ correction.T

# Bag-recording timestamps, relative to start.
t = drive[:, 0] - drive[0, 0]
dt = np.diff(t, prepend=t[0])
dt[0] = 0

# VN-100 z axis points down, so negate gyro z
# for a counterclockwise-positive planar yaw convention.
gyro_z = -drive[:, 1]

gyro_heading = np.zeros(len(t))
gyro_heading[1:] = np.cumsum(
    0.5 * (gyro_z[1:] + gyro_z[:-1]) * np.diff(t)
)

# Magnetic heading, expressed relative to its initial heading.
mag_heading = np.unwrap(
    np.arctan2(-drive_xy[:, 1], drive_xy[:, 0])
)
mag_heading -= mag_heading[0]

# Complementary filter:
# gyro predicts rapid changes; magnetometer corrects slow drift.
tau = 10.0  # correction time constant, seconds
fused = np.zeros(len(t))

for i in range(1, len(t)):
    prediction = fused[i-1] + 0.5 * (
        gyro_z[i] + gyro_z[i-1]
    ) * dt[i]

    alpha = np.exp(-dt[i] / tau)

    error = wrap(mag_heading[i] - prediction)
    fused[i] = prediction + (1 - alpha) * error

# Figure 6
fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t, np.rad2deg(gyro_heading),
        label="Integrated gyroscope", linewidth=1.5)
ax.plot(t, np.rad2deg(mag_heading),
        label="Calibrated magnetometer", linewidth=1, alpha=0.75)

ax.set_title("Figure 6: Gyroscope vs Magnetometer Heading")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Relative heading (degrees)")
ax.legend()
ax.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(OUT / "figure6_heading_comparison.png", dpi=200)
plt.close(fig)

# Figure 7
fig, ax = plt.subplots(figsize=(11, 5))
ax.plot(t, np.rad2deg(gyro_heading),
        label="Gyroscope only", linewidth=1, alpha=0.8)
ax.plot(t, np.rad2deg(fused),
        label="Complementary filter", linewidth=1.6)
ax.plot(t, np.rad2deg(mag_heading),
        label="Magnetometer", linewidth=0.8, alpha=0.45)

ax.set_title("Figure 7: Complementary Filter Heading")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Relative heading (degrees)")
ax.legend()
ax.grid(alpha=0.25)
fig.tight_layout()
fig.savefig(OUT / "figure7_complementary_heading.png", dpi=200)
plt.close(fig)

print("Car IMU samples:", len(t))
print("Duration (s):", round(float(t[-1]), 2))
print("Gyroscope total heading (degrees):",
      round(float(np.rad2deg(gyro_heading[-1])), 2))
print("Magnetometer final heading (degrees):",
      round(float(np.rad2deg(mag_heading[-1])), 2))
print("Complementary final heading (degrees):",
      round(float(np.rad2deg(fused[-1])), 2))
print("Calibration offset (microtesla):", offset)
print("Saved Figure 6 and Figure 7")
