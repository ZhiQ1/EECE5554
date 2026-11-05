from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home() / "EECE5554/Lab4"
OUT = ROOT / "analysis"
RESULTS = []

def log(s):
    print(s, flush=True)
    RESULTS.append(s)

def load_laptop(name):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(ROOT / "data" / name),
            storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr")
    )

    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    msg_type = get_message(types["/imu"])
    data = []

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == "/imu":
            msg = deserialize_message(raw, msg_type)
            data.append((stamp * 1e-9, msg.imu.angular_velocity.z))

    a = np.asarray(data, dtype=float)
    a[:, 0] -= a[0, 0]
    return a[:, 0], a[:, 1]

def load_car(name):
    p = np.load(ROOT / "data/car" / (name + ".npz"))
    t = np.asarray(p["lidar_imu__t"], dtype=float).ravel()
    gyro = np.asarray(p["lidar_imu__gyro"], dtype=float)

    if np.median(np.diff(t)) > 1e6:
        t *= 1e-9

    t -= t[0]
    return t, gyro[:, 2]

def correlation_at(lag, tl, gl, tc, gc):
    # Instructor convention: car time = laptop time + L
    times = tl + lag
    valid = (times >= tc[0]) & (times <= tc[-1])

    if valid.sum() < 200:
        return np.nan

    a = gl[valid]
    b = np.interp(times[valid], tc, gc)

    a = a - a.mean()
    b = b - b.mean()

    denom = np.linalg.norm(a) * np.linalg.norm(b)
    if denom < 1e-12:
        return np.nan

    return float(np.dot(a, b) / denom)

def find_lag(tl, gl, tc, gc):
    # VN-100 z down, Ouster z up: reverse Ouster sign.
    gc = -gc

    # Resample laptop gyro at 100 Hz.
    grid = np.arange(tl[0], tl[-1], 0.01)
    values = np.interp(grid, tl, gl)

    # Coarse search.
    coarse = np.arange(-60, 60.001, 0.2)
    scores = np.array([
        correlation_at(L, grid, values, tc, gc)
        for L in coarse
    ])

    if np.all(~np.isfinite(scores)):
        raise RuntimeError("No valid overlap for clock alignment")

    k = np.nanargmax(scores)
    best = coarse[k]

    # Fine search at 0.01 s.
    fine = np.arange(best - 0.3, best + 0.301, 0.01)
    fine_scores = np.array([
        correlation_at(L, grid, values, tc, gc)
        for L in fine
    ])

    j = np.nanargmax(fine_scores)
    lag = float(fine[j])
    peak = float(fine_scores[j])

    # Refine peak using a parabola.
    if 0 < j < len(fine) - 1:
        y1, y2, y3 = fine_scores[j-1:j+2]
        denom = y1 - 2*y2 + y3
        if np.isfinite(denom) and abs(denom) > 1e-12:
            delta = 0.5 * (y1-y3) / denom
            if abs(delta) <= 1:
                lag += delta * 0.01

    peak = correlation_at(lag, grid, values, tc, gc)
    wrong_sign = correlation_at(lag, grid, values, tc, -gc)

    return lag, peak, wrong_sign

fig, axes = plt.subplots(2, 1, figsize=(12, 8))

for index, name in enumerate(["car_circles", "car_drive"]):
    tl, gl = load_laptop(name)
    tc, gc = load_car(name)

    lag, corr, wrong = find_lag(tl, gl, tc, gc)

    log(f"\n{name}")
    log(f"  Clock lag L = {lag:.4f} s")
    log(f"  Correct-sign Pearson r = {corr:.4f}")
    log(f"  Wrong-sign Pearson r = {wrong:.4f}")

    # Plot aligned gyros with instructor's sign convention.
    aligned_car = np.interp(
        tl + lag, tc, -gc,
        left=np.nan, right=np.nan
    )

    axes[index].plot(tl, gl, label="VN-100 gyro Z", lw=1)
    axes[index].plot(
        tl, aligned_car, label="Ouster gyro Z (sign flipped)",
        lw=1, alpha=0.7
    )
    axes[index].set_title(
        f"{name}: L={lag:.3f}s, Pearson r={corr:.3f}"
    )
    axes[index].set_xlabel("Laptop relative time (s)")
    axes[index].set_ylabel("Angular velocity (rad/s)")
    axes[index].legend()
    axes[index].grid(alpha=0.2)

    if name == "car_drive":
        middle = len(tl) // 2

        l1, r1, _ = find_lag(
            tl[:middle], gl[:middle], tc, gc
        )
        l2, r2, _ = find_lag(
            tl[middle:], gl[middle:], tc, gc
        )

        center1 = np.mean([tl[0], tl[middle-1]])
        center2 = np.mean([tl[middle], tl[-1]])

        drift_ppm = (
            (l2-l1) / (center2-center1) * 1e6
        )

        log("\nCar drive clock drift")
        log(f"  First-half L = {l1:.4f} s, r={r1:.4f}")
        log(f"  Second-half L = {l2:.4f} s, r={r2:.4f}")
        log(f"  Clock drift = {drift_ppm:.2f} ppm")

fig.tight_layout()
fig.savefig(
    OUT / "figures/figure11_clock_alignment_corrected.png",
    dpi=200
)
plt.close(fig)

(OUT / "q8_clock_results.txt").write_text(
    "\n".join(RESULTS) + "\n"
)

log("\nSaved corrected Figure 11 and Q8 results.")
