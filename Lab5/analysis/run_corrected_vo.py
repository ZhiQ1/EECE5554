import json
import numpy as np
from pathlib import Path

nb = json.loads(
    Path("Lab5/analysis/lab5_analysis.ipynb").read_text()
)
ns = {"__name__": "__main__"}

cells = [9, 13, 17, 19, 21, 23, 25, 31, 32,
         34, 37, 46, 54, 56, 58, 59]

for i in cells:
    print(f"Loading cell {i}", flush=True)
    src = "".join(nb["cells"][i].get("source", []))
    exec(compile(src, f"cell_{i}", "exec"), ns)

ns["grid"] = ns["bev_grid"]()

z = np.load("Lab5/analysis/figure7_scan.npz")
pitch = float(z["pitch_corr"])
yaw = float(z["yaw_corr"])

T_corr = ns["adjust_camera"](
    ns["fs"]["base_T_cam"],
    pitch_down_deg=pitch,
    yaw_left_deg=yaw
)

print("\n=== COMPUTING CORRECTED VO ===", flush=True)
print(f"Pitch = {pitch:+.4f} deg")
print(f"Yaw = {yaw:+.4f} deg")

out = Path("Lab5/analysis/vo_corrected.npz")
vo_cor = ns["run_odometry"](T_corr, out)

valid = np.isfinite(vo_cor[:, 0])

print("\n=== RESULTS ===")
print("Total pairs:", len(vo_cor))
print("Valid pairs:", int(valid.sum()))
print("Success rate:", f"{valid.mean()*100:.1f}%")
print("Saved:", out.resolve())
