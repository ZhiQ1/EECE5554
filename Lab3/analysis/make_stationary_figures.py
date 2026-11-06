import ast
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab3"
ANALYSIS = ROOT / "analysis"
FIGURES = ANALYSIS / "figures"
NOTEBOOK = ANALYSIS / "lab3_analysis.ipynb"

FIGURES.mkdir(exist_ok=True)

# Extract the original rosbag reader from the instructor notebook.
nb = json.loads(NOTEBOOK.read_text())
reader_function = None

for cell in nb["cells"]:
    if cell["cell_type"] != "code":
        continue

    src = "".join(cell["source"])

    try:
        tree = ast.parse(src)
    except SyntaxError:
        continue

    for node in tree.body:
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "read_imu_bag"
        ):
            reader_function = node

if reader_function is None:
    raise RuntimeError("Cannot find read_imu_bag in notebook")

module = ast.Module(
    body=[reader_function],
    type_ignores=[]
)

namespace = {
    "np": np,
    "Path": Path,
}

exec(
    compile(
        ast.fix_missing_locations(module),
        str(NOTEBOOK),
        "exec"
    ),
    namespace
)

read_imu_bag = namespace["read_imu_bag"]

print("Reading stationary rosbag...")

stat = read_imu_bag(ROOT / "data/stationary")

t = stat["t"]
gyro = np.rad2deg(stat["gyro"])
accel = stat["accel"]
quat = stat["quat"]

print("Samples:", len(t))
print("Duration:", round(t[-1], 3), "seconds")


def save_figure(fig, name):
    path = FIGURES / name
    fig.savefig(path, dpi=250, bbox_inches="tight")
    plt.close(fig)
    print("Saved:", path)


def plot_three_axes(t, values, title, ylabel, filename):
    fig, ax = plt.subplots(figsize=(11, 5))

    for i, axis in enumerate("XYZ"):
        ax.plot(
            t,
            values[:, i],
            linewidth=0.65,
            label=axis
        )

    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid(alpha=0.3)

    fig.tight_layout()
    save_figure(fig, filename)


# Figure 1: Gyroscope
plot_three_axes(
    t,
    gyro,
    "Figure 1. Stationary Gyroscope Measurements",
    "Angular Velocity (deg/s)",
    "figure1_stationary_gyro.png"
)

# Figure 2: Accelerometer
plot_three_axes(
    t,
    accel,
    "Figure 2. Stationary Accelerometer Measurements",
    "Linear Acceleration (m/s²)",
    "figure2_stationary_accel.png"
)

# Figure 3: Quaternion -> Euler angles (ZYX)
qx, qy, qz, qw = quat.T

yaw = np.arctan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy**2 + qz**2)
)

pitch = np.arcsin(
    np.clip(
        2 * (qw * qy - qz * qx),
        -1,
        1
    )
)

roll = np.arctan2(
    2 * (qw * qx + qy * qz),
    1 - 2 * (qx**2 + qy**2)
)

angles = np.column_stack([
    np.rad2deg(np.unwrap(yaw)),
    np.rad2deg(pitch),
    np.rad2deg(roll)
])

fig, ax = plt.subplots(figsize=(11, 5))

for i, label in enumerate(["Yaw", "Pitch", "Roll"]):
    ax.plot(
        t,
        angles[:, i],
        linewidth=0.8,
        label=label
    )

ax.set_title("Figure 3. Stationary Orientation")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Orientation (degrees)")
ax.legend()
ax.grid(alpha=0.3)

fig.tight_layout()
save_figure(fig, "figure3_stationary_orientation.png")


# Figure 4: Noise histograms with fitted Gaussian curves
fig, axes = plt.subplots(2, 3, figsize=(14, 8))

datasets = [
    (stat["gyro"], "Gyroscope", "rad/s"),
    (accel, "Accelerometer", "m/s²")
]

for row, (values, sensor, unit) in enumerate(datasets):
    for col, axis in enumerate("XYZ"):
        ax = axes[row, col]

        x = values[:, col]
        centered = x - np.mean(x)
        sigma = np.std(centered)

        ax.hist(
            centered,
            bins=65,
            density=True,
            alpha=0.65,
            label="Measured"
        )

        if sigma > 0:
            grid = np.linspace(
                np.min(centered),
                np.max(centered),
                400
            )

            gaussian = (
                np.exp(-0.5 * (grid / sigma)**2)
                / (sigma * np.sqrt(2 * np.pi))
            )

            ax.plot(
                grid,
                gaussian,
                linewidth=2,
                label="Gaussian"
            )

        ax.set_title(f"{sensor} {axis}, σ={sigma:.3e}")
        ax.set_xlabel(f"Mean-subtracted noise ({unit})")
        ax.set_ylabel("Probability Density")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)

fig.suptitle(
    "Figure 4. Stationary Sensor Noise Distributions",
    fontsize=15
)

fig.tight_layout()
save_figure(fig, "figure4_noise_histograms.png")


print("\n========== STATIONARY STATISTICS ==========")

for values, sensor, unit in datasets:
    print(f"\n{sensor} ({unit})")

    for i, axis in enumerate("XYZ"):
        print(
            f"{axis}: "
            f"mean={np.mean(values[:, i]):.8e}, "
            f"std={np.std(values[:, i]):.8e}"
        )

print("\nFigures 1-4 completed.")
