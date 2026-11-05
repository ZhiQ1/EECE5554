from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis"
(OUT / "figures").mkdir(exist_ok=True)

vdata = np.load(OUT / "q4_velocity_data.npz")
t = np.asarray(vdata["t"], dtype=float)
v_adjusted = np.asarray(vdata["v_corrected"], dtype=float)
v_car = np.asarray(vdata["v_car"], dtype=float)
omega = np.asarray(vdata["gyro_z"], dtype=float)

# Read lateral acceleration from the same laptop recording.
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(ROOT / "data/car_drive"),
        storage_id="mcap"),
    rosbag2_py.ConverterOptions("cdr", "cdr")
)

types = {x.name: x.type for x in reader.get_all_topics_and_types()}
msg_type = get_message(types["/imu"])
reader.set_filter(rosbag2_py.StorageFilter(topics=["/imu"]))

lateral = []
while reader.has_next():
    _, raw, _ = reader.read_next()
    msg = deserialize_message(raw, msg_type)
    lateral.append(msg.imu.linear_acceleration.y)

ay = np.asarray(lateral, dtype=float)
if len(ay) != len(t):
    raise RuntimeError("IMU sample count mismatch")

dt = float(np.median(np.diff(t)))
fs = 1.0 / dt

# 1 Hz low-pass filtering.
sos = butter(4, 1.0, btype="lowpass", fs=fs, output="sos")

def lowpass(x):
    return sosfiltfilt(sos, np.asarray(x, dtype=float))

omega_f = lowpass(omega)
ay_f = lowpass(ay)

# Initial static lateral acceleration is treated as sensor offset.
static = t <= 20.0
ay_offset = float(np.median(ay_f[static]))
ay_obs = ay_f - ay_offset

# Angular acceleration.
omega_dot = np.gradient(omega_f, t)
omega_dot = lowpass(omega_dot)

def evaluate_model(speed, name):
    speed = np.asarray(speed, dtype=float)
    valid_speed = np.isfinite(speed)

    if valid_speed.sum() < 100:
        raise RuntimeError(f"Insufficient velocity data for {name}")

    speed_f = np.interp(
        t, t[valid_speed], speed[valid_speed]
    )
    speed_f = lowpass(speed_f)

    # Teacher's basic model.
    centripetal = lowpass(omega_f * speed_f)

    # Use moving samples with meaningful angular acceleration.
    moving = (
        valid_speed &
        (speed_f > 2.0) &
        np.isfinite(ay_obs) &
        np.isfinite(omega_dot)
    )

    if moving.sum() < 100:
        raise RuntimeError(f"Insufficient moving samples for {name}")

    residual = ay_obs - centripetal

    # Fit x_c by least squares:
    # a_y - omega*V = omega_dot*x_c.
    x = omega_dot[moving]
    y = residual[moving]

    denominator = float(np.dot(x, x))
    xc = float(np.dot(x, y) / denominator) if denominator > 1e-10 else np.nan

    predicted_basic = centripetal
    predicted_full = centripetal + omega_dot * xc

    def rms(pred):
        return float(np.sqrt(np.mean(
            (ay_obs[moving] - pred[moving])**2
        )))

    return {
        "name": name,
        "xc": xc,
        "rms_basic": rms(predicted_basic),
        "rms_full": rms(predicted_full),
        "basic": predicted_basic,
        "full": predicted_full,
        "moving": moving,
        "speed": speed_f
    }

adjusted = evaluate_model(v_adjusted, "Corrected IMU velocity")
mustang = evaluate_model(v_car, "Mustang vehicle velocity")

# Select a turn with substantial omega, excluding initial stationary time.
turn_candidates = np.where(
    (t > 30) &
    (t < t[-1] - 20) &
    (np.abs(omega_f) > 0.10) &
    (mustang["speed"] > 3)
)[0]

if len(turn_candidates):
    center = float(t[turn_candidates[np.argmax(
        np.abs(omega_f[turn_candidates])
    )]])
else:
    center = float(t[len(t)//2])

zoom = (t >= center - 10) & (t <= center + 10)

fig, axes = plt.subplots(2, 1, figsize=(13, 8))

axes[0].plot(
    t, ay_obs, label="Observed lateral acceleration",
    linewidth=0.8, color="black"
)
axes[0].plot(
    t, adjusted["basic"],
    label="ωV (corrected IMU velocity)",
    linewidth=1.1
)
axes[0].plot(
    t, adjusted["full"],
    label="ωV + ω̇ x_c (fitted model)",
    linewidth=1.0, alpha=0.8
)
axes[0].set_title("Figure 9(a) — Lateral Acceleration: Full Drive")
axes[0].set_ylabel("Lateral acceleration (m/s²)")
axes[0].grid(alpha=0.2)
axes[0].legend(fontsize=9)

axes[1].plot(
    t[zoom], ay_obs[zoom],
    label="Observed lateral acceleration",
    linewidth=1.2, color="black"
)
axes[1].plot(
    t[zoom], adjusted["basic"][zoom],
    label="ωV",
    linewidth=1.1
)
axes[1].plot(
    t[zoom], adjusted["full"][zoom],
    label="ωV + ω̇ x_c",
    linewidth=1.1
)
axes[1].set_title(
    f"Figure 9(b) — Turn Detail at t={center:.1f} s"
)
axes[1].set_xlabel("Laptop recording time (s)")
axes[1].set_ylabel("Lateral acceleration (m/s²)")
axes[1].grid(alpha=0.2)
axes[1].legend(fontsize=9)

fig.tight_layout()
fig.savefig(
    OUT / "figures/figure9_motion_model.png",
    dpi=200
)
plt.close(fig)

lines = [
    "========== Figure 9 / Question 5 ==========",
    f"Sampling frequency: {fs:.2f} Hz",
    "Low-pass cutoff: 1 Hz",
    f"Initial lateral acceleration offset: {ay_offset:.5f} m/s²",
    f"Selected turn center: {center:.2f} s",
]

for result in (adjusted, mustang):
    lines.extend([
        "",
        result["name"],
        f"Estimated x_c: {result['xc']:.4f} m",
        f"RMS without lever-arm term: {result['rms_basic']:.4f} m/s²",
        f"RMS with lever-arm term: {result['rms_full']:.4f} m/s²",
        f"Moving samples used: {result['moving'].sum()}"
    ])

lines.extend([
    "",
    "IMPORTANT:",
    "The physical tape-measured x_c is not available.",
    "Check the frame orientation and accelerometer sign if",
    "the fitted lever arm is implausible.",
    "The corrected IMU speed is used for the primary model.",
    "The Mustang-speed model is a diagnostic comparison."
])

report = "\n".join(lines) + "\n"
print(report)

(OUT / "q5_motion_model_results.txt").write_text(report)

np.savez(
    OUT / "q5_motion_model_data.npz",
    t=t,
    ay_observed=ay_obs,
    omega=omega_f,
    omega_dot=omega_dot,
    predicted_basic=adjusted["basic"],
    predicted_full=adjusted["full"],
    estimated_xc=adjusted["xc"]
)

print("Saved Figure 9 and Q5 results.")
