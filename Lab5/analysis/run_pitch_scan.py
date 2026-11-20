import json
import numpy as np
from pathlib import Path

nb = json.loads(
    Path("Lab5/analysis/lab5_analysis.ipynb").read_text()
)

ns = {"__name__": "__main__"}

for i in [9, 13, 17, 19, 21, 23, 25, 31, 32, 34, 37, 46, 54, 56, 58, 59]:
    print(f"Loading cell {i}", flush=True)
    src = "".join(nb["cells"][i].get("source", []))
    exec(compile(src, f"cell_{i}", "exec"), ns)

# Initialize the BEV grid using the same default configuration.
ns["grid"] = ns["bev_grid"]()

# Load Figure 7 setup and helper functions.
src = "".join(nb["cells"][60].get("source", []))
setup = src.split("pitches = np.arange(")[0]
exec(compile(setup, "cell_60_setup", "exec"), ns)

print("\n========== PITCH SCAN ==========", flush=True)

pitches = np.arange(-1.0, 1.001, 0.25)
med = []

for pitch in pitches:
    T = ns["adjust_camera"](
        ns["fs"]["base_T_cam"],
        pitch_down_deg=float(pitch)
    )

    motions = ns["motions_for"](ns["sel"], T)
    ratio = ns["med_ratio"](motions, ns["sel"])

    med.append(ratio)

    print(
        f"Pitch {pitch:+.2f} deg: "
        f"median VO/GPS ratio = {ratio:.4f}",
        flush=True
    )

med = np.asarray(med)

if not np.isfinite(med).any():
    raise RuntimeError("No valid pitch measurements")

cross = np.flatnonzero(
    np.isfinite(med[:-1])
    & np.isfinite(med[1:])
    & ((med[:-1] - 1) * (med[1:] - 1) <= 0)
)

print("\n========== CALIBRATION RESULT ==========")
print("Selected pairs:", len(ns["sel"]))

if cross.size:
    j = int(cross[0])

    if abs(med[j + 1] - med[j]) < 1e-9:
        pitch_corr = float(pitches[j])
    else:
        pitch_corr = float(
            pitches[j]
            + (1 - med[j])
            * (pitches[j + 1] - pitches[j])
            / (med[j + 1] - med[j])
        )

    print(f"Pitch crossing found: {pitch_corr:+.4f} deg")

else:
    j = int(np.nanargmin(np.abs(med - 1)))
    pitch_corr = float(pitches[j])

    print("WARNING: ratio does not cross 1.")
    print("Using the best tested pitch, without extrapolation.")
    print(f"Best tested pitch: {pitch_corr:+.4f} deg")
    print(f"Ratio at best pitch: {med[j]:.4f}")

T_pitch = ns["adjust_camera"](
    ns["fs"]["base_T_cam"],
    pitch_down_deg=pitch_corr
)

mo = ns["motions_for"](ns["sel"], T_pitch)

direction = np.degrees(
    np.arctan2(mo[:, 1], mo[:, 0])
)

yaw_corr = -float(np.nanmedian(direction))

print(f"Exploratory yaw correction: {yaw_corr:+.4f} deg")

out = Path("Lab5/analysis/figure7_scan.npz")

np.savez(
    out,
    pitches=pitches,
    ratios=med,
    pitch_corr=pitch_corr,
    yaw_corr=yaw_corr,
    selected_pairs=ns["sel"],
    crossing_found=bool(cross.size)
)

print(f"\nSaved: {out.resolve()}")
