from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554" / "Lab4"
BAG = ROOT / "data" / "walk_circle"
OUT = ROOT / "analysis" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(BAG), storage_id="mcap"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
)

types = {
    topic.name: topic.type
    for topic in reader.get_all_topics_and_types()
}

if "/imu" not in types:
    raise RuntimeError("No /imu topic found")

msg_type = get_message(types["/imu"])

t, mx, my, mz = [], [], [], []

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic != "/imu":
        continue

    msg = deserialize_message(data, msg_type)

    t.append(timestamp * 1e-9)
    mx.append(msg.mag_field.magnetic_field.x)
    my.append(msg.mag_field.magnetic_field.y)
    mz.append(msg.mag_field.magnetic_field.z)

t = np.asarray(t)
mx = np.asarray(mx)
my = np.asarray(my)
mz = np.asarray(mz)

t -= t[0]

fig, ax = plt.subplots(figsize=(7, 6))
ax.scatter(mx, my, s=3, alpha=0.5)
ax.set_xlabel("Magnetometer X (T)")
ax.set_ylabel("Magnetometer Y (T)")
ax.set_title("Walking Circle: Raw Magnetometer XY")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(alpha=0.25)
fig.tight_layout()

output = OUT / "figure1_raw_magnetometer.png"
fig.savefig(output, dpi=200)
plt.close(fig)

print("IMU messages:", len(t))
print("Duration (s):", round(t[-1], 2))
print("Magnetometer X range:", mx.min(), mx.max())
print("Magnetometer Y range:", my.min(), my.max())
print("Magnetometer Z range:", mz.min(), mz.max())
print("Saved:", output)
