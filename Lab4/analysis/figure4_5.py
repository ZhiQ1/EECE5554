from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis/figures"
OUT.mkdir(parents=True, exist_ok=True)

def load_mag(name):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data" / name),
            storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )

    types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    msg_type = get_message(types["/imu"])
    data = []

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic != "/imu":
            continue

        msg = deserialize_message(raw, msg_type)
        m = msg.mag_field.magnetic_field

        data.append([m.x, m.y, m.z])

    return np.asarray(data) * 1e6

def calibrate(xy):
    offset = (xy.max(axis=0) + xy.min(axis=0)) / 2
    centered = xy - offset

    cov = np.cov(centered, rowvar=False)
    values, vectors = np.linalg.eigh(cov)
    values = np.maximum(values, 1e-10)

    transform = vectors @ np.diag(
        1 / np.sqrt(values)
    ) @ vectors.T

    corrected = centered @ transform.T

    radius = np.median(np.linalg.norm(centered, axis=1))
    corrected *= radius / np.median(
        np.linalg.norm(corrected, axis=1)
    )

    return corrected, offset

walk = load_mag("walk_circle")
car = load_mag("car_circles")

walk_cal, walk_offset = calibrate(walk[:, :2])
car_cal, car_offset = calibrate(car[:, :2])

# Figure 4: car magnetometer calibration
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].scatter(
    car[:, 0], car[:, 1],
    s=2, alpha=0.4
)
axes[0].set_title("Car: Raw Magnetometer")

axes[1].scatter(
    car_cal[:, 0], car_cal[:, 1],
    s=2, alpha=0.4
)
axes[1].set_title("Car: Calibrated Magnetometer")

for ax in axes:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Magnetic Field X (µT)")
    ax.set_ylabel("Magnetic Field Y (µT)")
    ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(
    OUT / "figure4_car_magnetometer.png",
    dpi=200
)
plt.close(fig)

# Figure 5: walking versus car calibration
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

axes[0].scatter(
    walk_cal[:, 0], walk_cal[:, 1],
    s=2, alpha=0.4
)
axes[0].set_title("Walking Calibration")

axes[1].scatter(
    car_cal[:, 0], car_cal[:, 1],
    s=2, alpha=0.4
)
axes[1].set_title("Car Calibration")

for ax in axes:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Corrected X (µT)")
    ax.set_ylabel("Corrected Y (µT)")
    ax.grid(alpha=0.2)

fig.tight_layout()
fig.savefig(
    OUT / "figure5_walking_vs_car.png",
    dpi=200
)
plt.close(fig)

print("Walking samples:", len(walk))
print("Car samples:", len(car))
print("Walking hard-iron offset (µT):", walk_offset)
print("Car hard-iron offset (µT):", car_offset)
print("Saved Figure 4 and Figure 5")
