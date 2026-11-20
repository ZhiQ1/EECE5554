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


# Run Figure 10 recording cell.
print("\n=== GENERATING RERUN RECORDING ===", flush=True)

# Load any additional notebook definitions.
import ast

for idx, cell in enumerate(nb["cells"]):
    if cell["cell_type"] != "code":
        continue

    source = "".join(cell.get("source", []))

    try:
        tree = ast.parse(source)
    except SyntaxError:
        continue

    for node in tree.body:
        if not isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            continue
        if node.decorator_list or node.name in ns:
            continue

        module = ast.fix_missing_locations(
            ast.Module(body=[node], type_ignores=[])
        )

        try:
            exec(compile(module, f"defs_{idx}", "exec"), ns)
        except (NameError, TypeError):
            pass


# Restore cached corrected visual odometry.
ns["vo_cor"] = ns["run_odometry"](
    ns["T_corr"],
    Path("Lab5/analysis/vo_corrected.npz")
)

print("Corrected VO loaded:", len(ns["vo_cor"]), "pairs", flush=True)

ns["RERUN_TIMELINE"] = "time"
src = "".join(nb["cells"][64].get("source", []))
exec(compile(src, "cell_64", "exec"), ns)

print("\nSUCCESS: Lab5/analysis/lab5_drive.rrd")
