import ast
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HOME = Path.home()
ANALYSIS = HOME / "EECE5554/Lab3/analysis"
NOTEBOOK = ANALYSIS / "lab3_analysis.ipynb"
CSV = HOME / "lab3_long_data/gyro_long.csv"
FIGURES = ANALYSIS / "figures"

FIGURES.mkdir(exist_ok=True)

# Use the nominal sample rate specified in the instructor notebook.
FS = 40.0

# Reuse the instructor's original functions.
required = {
    "allan_deviation",
    "local_slope",
    "noise_parameters",
    "plot_allan",
}

nb = json.loads(NOTEBOOK.read_text())
functions = {}

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
                functions[node.name] = node

missing = required - functions.keys()

if missing:
    raise RuntimeError(
        f"Missing instructor functions: {sorted(missing)}"
    )

module = ast.Module(
    body=list(functions.values()),
    type_ignores=[]
)

namespace = {"np": np, "plt": plt}

exec(
    compile(
        ast.fix_missing_locations(module),
        str(NOTEBOOK),
        "exec"
    ),
    namespace
)

allan_deviation = namespace["allan_deviation"]
noise_parameters = namespace["noise_parameters"]
plot_allan = namespace["plot_allan"]

# Load validated gyroscope readings.
data = np.loadtxt(
    CSV,
    delimiter=",",
    skiprows=1
)

gyro = data[:, 1:4]

print("========== INPUT ==========")
print("Valid samples:", len(gyro))
print("Nominal sampling rate:", FS, "Hz")

# Compute Allan deviation.
params = {}
fig, ax = plt.subplots(figsize=(10, 7))

for i, axis in enumerate("xyz"):
    tau, adev = allan_deviation(gyro[:, i], FS)

    p = noise_parameters(tau, adev)
    params[axis] = p

    plot_allan(
        ax,
        tau,
        adev,
        p,
        f"Gyro {axis.upper()}"
    )

    print(f"\n========== GYRO {axis.upper()} ==========")
    print(f"N = {p['N']:.8e} rad/s/sqrt(Hz)")
    print(f"B = {p['B']:.8e} rad/s")
    print(f"K = {p['K']:.8e} rad/s/sqrt(s)")
    print(f"Tau minimum = {p['tau_B']:.3f} s")

ax.set_title(
    "Figure 5. Allan Deviation of VN-100 Gyroscope"
)

fig.tight_layout()

figure_path = FIGURES / "figure5_allan_deviation.png"

fig.savefig(
    figure_path,
    dpi=250,
    bbox_inches="tight"
)

plt.close(fig)

# Save machine-readable noise parameters.
output = {
    "data_set": "imu_bag_1",
    "sample_rate_hz": FS,
    "units": {
        "N": "rad/s/sqrt(Hz)",
        "B": "rad/s",
        "K": "rad/s/sqrt(s)",
        "tau_B": "s"
    }
}

for axis, p in params.items():
    output[axis] = {
        k: float(v) if np.isfinite(v) else None
        for k, v in p.items()
    }

json_path = ANALYSIS / "noise_parameters.json"

json_path.write_text(
    json.dumps(output, indent=2) + "\n"
)

print("\n========== OUTPUT FILES ==========")
print(figure_path)
print(json_path)
print("\nFigure 5 completed.")
