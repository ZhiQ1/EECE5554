import ast
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab3"
NOTEBOOK = ROOT / "analysis/lab3_analysis.ipynb"
OUT = ROOT / "analysis/figures/motion_overview.png"

nb = json.loads(NOTEBOOK.read_text())

reader_function = None

for cell in nb["cells"]:
    if cell["cell_type"] != "code":
        continue

    try:
        tree = ast.parse("".join(cell["source"]))
    except SyntaxError:
        continue

    for node in tree.body:
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "read_imu_bag"
        ):
            reader_function = node

if reader_function is None:
    raise RuntimeError("read_imu_bag not found")

module = ast.Module(
    body=[reader_function],
    type_ignores=[]
)

ns = {"np": np, "Path": Path}

exec(
    compile(
        ast.fix_missing_locations(module),
        str(NOTEBOOK),
        "exec"
    ),
    ns
)

motion = ns["read_imu_bag"](ROOT / "data/motion")

t = motion["t"]
gyro = np.rad2deg(motion["gyro"])
accel = motion["accel"]

qx, qy, qz, qw = motion["quat"].T

yaw = np.rad2deg(
    np.unwrap(
        np.arctan2(
            2 * (qw * qz + qx * qy),
            1 - 2 * (qy**2 + qz**2)
        )
    )
)

fig, axes = plt.subplots(
    4, 1,
    figsize=(14, 11),
    sharex=True
)

for i, axis in enumerate("XYZ"):
    axes[0].plot(
        t, gyro[:, i],
        label=f"Gyro {axis}"
    )

axes[0].set_ylabel("Angular velocity (deg/s)")
axes[0].legend()
axes[0].set_title("Motion Overview: Full Recording")

axes[1].plot(t, yaw)
axes[1].set_ylabel("Yaw (degrees)")

for i, axis in enumerate("XYZ"):
    axes[2].plot(
        t, accel[:, i],
        label=f"Accel {axis}"
    )

axes[2].set_ylabel("Acceleration (m/s²)")
axes[2].legend()

axes[3].plot(
    t,
    np.linalg.norm(accel, axis=1)
)

axes[3].set_ylabel("Accel magnitude (m/s²)")
axes[3].set_xlabel("Time (s)")

for ax in axes:
    ax.grid(alpha=0.3)
    ax.set_xticks(np.arange(0, 105, 5))

fig.tight_layout()
fig.savefig(OUT, dpi=200)
plt.close(fig)

print("Samples:", len(t))
print("Duration:", round(t[-1], 3), "s")
print("Saved:", OUT)
