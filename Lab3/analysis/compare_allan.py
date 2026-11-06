import ast
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "EECE5554/Lab3"
ANALYSIS = ROOT / "analysis"

NOTEBOOK = ANALYSIS / "lab3_analysis.ipynb"
LONG_CSV = Path.home() / "lab3_long_data/gyro_long.csv"

# Use the nominal rate from the instructor notebook.
FS = 40.0

# Reuse the instructor's functions.
required = {"read_imu_bag", "allan_deviation"}
found = {}

nb = json.loads(NOTEBOOK.read_text())

for cell in nb["cells"]:
    if cell["cell_type"] != "code":
        continue

    source = "".join(cell["source"])

    try:
        tree = ast.parse(source)
    except SyntaxError:
        continue

    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            if node.name in required:
                found[node.name] = node

if required - found.keys():
    raise RuntimeError(
        f"Missing functions: {required - found.keys()}"
    )

module = ast.Module(
    body=list(found.values()),
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

read_imu_bag = ns["read_imu_bag"]
allan_deviation = ns["allan_deviation"]

# Own stationary data: use the first 300 seconds.
stationary = read_imu_bag(ROOT / "data/stationary")

mask = stationary["t"] < 300.0

own_z = stationary["gyro"][mask, 2]

# Long dataset: validated raw VNYMR measurements.
long_data = np.loadtxt(
    LONG_CSV,
    delimiter=",",
    skiprows=1
)

long_z = long_data[:, 3]

# Allan deviation for both recordings.
tau_own, adev_own = allan_deviation(own_z, FS)
tau_long, adev_long = allan_deviation(long_z, FS)

# Compare the curves where both are defined.
common = tau_own[
    (tau_own >= tau_long.min()) &
    (tau_own <= tau_long.max())
]

long_interp = np.interp(
    np.log10(common),
    np.log10(tau_long),
    np.log10(adev_long)
)

own_interp = np.interp(
    np.log10(common),
    np.log10(tau_own),
    np.log10(adev_own)
)

ratio = 10 ** (own_interp - long_interp)

print("========== DATA ==========")
print("Own stationary samples:", len(own_z))
print("Long data samples:", len(long_z))
print("Nominal sampling rate:", FS, "Hz")

print("\n========== ALLAN COMPARISON ==========")
print("Own maximum tau:", f"{tau_own.max():.3f} s")
print("Long maximum tau:", f"{tau_long.max():.3f} s")
print(
    "Common tau range:",
    f"{common.min():.3f} to {common.max():.3f} s"
)

print("\nSelected averaging times:")

for target in [0.1, 1, 3, 10, 30]:
    if target > common.max():
        continue

    idx = np.argmin(np.abs(common - target))

    print(
        f"tau={common[idx]:.3f} s, "
        f"own/long={ratio[idx]:.3f}"
    )

# Instructor-provided approximate relative error.
print("\n========== RELATIVE ERROR ESTIMATE ==========")

L = len(own_z)

for tau in [1, 10, 30, 100]:
    m = max(1, round(tau * FS))

    if L / m > 1:
        error = 1 / np.sqrt(2 * (L / m - 1))

        print(
            f"tau={tau} s: "
            f"relative error ≈ {100 * error:.1f}%"
        )

# Comparison plot.
fig, ax = plt.subplots(figsize=(10, 6))

ax.loglog(
    tau_long,
    adev_long,
    linewidth=2,
    label="Provided 5-hour recording (Gyro Z)"
)

ax.loglog(
    tau_own,
    adev_own,
    linewidth=2,
    label="Own 5-minute recording (Gyro Z)"
)

ax.set_title(
    "Allan Deviation Comparison: 5 Minutes vs. 5 Hours"
)

ax.set_xlabel("Averaging time τ (s)")
ax.set_ylabel("Allan deviation (rad/s)")
ax.legend()
ax.grid(True, which="both", alpha=0.3)

fig.tight_layout()

output = ANALYSIS / "figures/allan_5min_vs_5hour.png"

fig.savefig(output, dpi=250)
plt.close(fig)

print("\nSaved:", output)
print("Question 4 analysis completed.")
