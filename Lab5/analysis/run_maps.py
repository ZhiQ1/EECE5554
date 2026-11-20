import json
import ast
import numpy as np
from pathlib import Path

nb = json.loads(
    Path("Lab5/analysis/lab5_analysis.ipynb").read_text()
)

ns = {"__name__": "__main__"}

# Load required functions and data.
cells = [
    9, 13, 17, 19, 21, 23, 25,
    31, 32, 34, 37, 46, 54, 56, 58, 59
]

for i in cells:
    print(f"Loading cell {i}", flush=True)
    src = "".join(nb["cells"][i].get("source", []))
    exec(compile(src, f"cell_{i}", "exec"), ns)

# Load existing calibration results.
z = np.load("Lab5/analysis/figure7_scan.npz")

ns["pitch_corr"] = float(z["pitch_corr"])
ns["yaw_corr"] = float(z["yaw_corr"])

ns["T_corr"] = ns["adjust_camera"](
    ns["fs"]["base_T_cam"],
    pitch_down_deg=ns["pitch_corr"],
    yaw_left_deg=ns["yaw_corr"]
)

ns["t_cor"] = ns["t_lap"] - ns["L_cam"]


# AUTOLOAD_NOTEBOOK_FUNCTIONS
# Load function/class definitions without running their experiment cells.
for idx, cell in enumerate(nb["cells"]):
    if cell["cell_type"] != "code":
        continue

    source = "".join(cell.get("source", []))

    try:
        tree = ast.parse(source)
    except SyntaxError:
        continue

    definitions = [
        node for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        and not node.decorator_list
    ]

    for node in definitions:
        if node.name in ns:
            continue

        module = ast.fix_missing_locations(
            ast.Module(body=[node], type_ignores=[])
        )

        try:
            exec(compile(module, f"function_cell_{idx}", "exec"), ns)
        except (NameError, TypeError):
            pass

print("frame_weight loaded:", "frame_weight" in ns)
print("GroundMap loaded:", "GroundMap" in ns)

# Run only the mapping cell.
print("\n=== GENERATING FIGURES 8 AND 9 ===", flush=True)

src = "".join(nb["cells"][62].get("source", []))
exec(compile(src, "cell_62", "exec"), ns)

# Copy generated figures to the requested directory.
import shutil

destination = Path("Lab5/figures")
destination.mkdir(parents=True, exist_ok=True)

for number in [8, 9]:
    filename = f"fig{number:02d}.png"
    source = Path("Lab5/analysis/figures") / filename

    if source.exists():
        shutil.copy2(source, destination / filename)
        print("SUCCESS:", (destination / filename).resolve())
    else:
        print("WARNING: Figure not found:", source)

print("\n=== DONE ===")
