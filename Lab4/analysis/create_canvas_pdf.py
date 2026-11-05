from pathlib import Path
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path.home() / "EECE5554/Lab4"
FIG = ROOT / "analysis/figures"
PDF = ROOT / "zhang.zhiqi2_lab4_figures.pdf"

figures = [
    ("figure1_svd_walking.png", "Figure 1 — Walking magnetometer calibration"),
    ("figure2_square_headings.png", "Figure 2 — Walking square heading estimates"),
    ("figure3_square_tracks.png", "Figure 3 — Walking dead reckoning"),
    ("figure4_svd_calibration.png", "Figure 4 — Car magnetometer calibration"),
    ("figure5_car_heading_calibration.png", "Figure 5 — Car magnetic heading"),
    ("figure6_svd_heading_comparison.png", "Figure 6 — Gyro vs magnetometer heading"),
    ("figure7_four_panel_complementary.png", "Figure 7 — Complementary filter"),
    ("figure8_corrected_velocity.png", "Figure 8 — Forward velocity"),
    ("figure9_motion_model.png", "Figure 9 — Vehicle motion model"),
    ("figure10_dead_reckoning.png", "Figure 10 — Dead reckoning vs GNSS"),
    ("figure11_clock_alignment_corrected.png", "Figure 11 — Two-computer clock alignment"),
    ("figure12_lidar_vs_gps.png", "Figure 12 — LiDAR ICP vs GPS"),
    ("figure13_gnss_outage.png", "Figure 13 — Simulated 90-second GNSS outage"),
]

missing = [name for name, _ in figures if not (FIG / name).exists()]
if missing:
    raise FileNotFoundError("Missing figures:\n" + "\n".join(missing))

with PdfPages(PDF) as pdf:

    cover, ax = plt.subplots(figsize=(11, 8.5))
    ax.axis("off")

    ax.text(
        0.5, 0.76,
        "EECE 5554 — Lab 4",
        ha="center", fontsize=27, fontweight="bold"
    )
    ax.text(
        0.5, 0.66,
        "Dead Reckoning with IMU, GNSS, and LiDAR",
        ha="center", fontsize=17
    )
    ax.text(
        0.5, 0.53,
        "Zhiqi Zhang  |  zhang.zhiqi2",
        ha="center", fontsize=14
    )
    ax.text(
        0.5, 0.43,
        "https://github.com/ZhiQ1/EECE5554",
        ha="center", fontsize=13
    )
    ax.text(
        0.5, 0.25,
        "Data limitation: RTK Fixed was unavailable in the recorded datasets.\n"
        "GNSS-referenced plots should not be interpreted as RTK-accuracy validation.",
        ha="center", fontsize=11
    )

    pdf.savefig(cover)
    plt.close(cover)

    for index, (filename, title) in enumerate(figures, 1):
        image = Image.open(FIG / filename).convert("RGB")

        page, ax = plt.subplots(figsize=(11.7, 8.3))
        ax.imshow(image)
        ax.axis("off")
        ax.set_title(title, fontsize=14, pad=16)

        page.tight_layout()
        pdf.savefig(page, dpi=180)
        plt.close(page)

        print(f"Added {index}/13: {filename}")

print(f"\nCreated: {PDF}")
print("Total pages: 14 (cover + 13 figures)")
