from pathlib import Path
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

root = Path.home() / "EECE5554/Lab3"
figures = root / "analysis/figures"
output = root / "zhang.zhiqi2_lab3_figures.pdf"

names = [
    "figure1_stationary_gyro.png",
    "figure2_stationary_accel.png",
    "figure3_stationary_orientation.png",
    "figure4_noise_histograms.png",
    "figure5_allan_deviation.png",
    "figure6_motion_analysis.png",
]

with PdfPages(output) as pdf:

    # Cover page
    fig = plt.figure(figsize=(8.5, 11))
    fig.text(.12, .80, "EECE 5554 — Lab 3", fontsize=24)
    fig.text(.12, .74, "VN-100 IMU Analysis", fontsize=17)
    fig.text(.12, .67, "Zhiqi Zhang", fontsize=13)
    fig.text(.12, .62, "zhang.zhiqi2", fontsize=12)
    fig.text(
        .12, .54,
        "github.com/greengreerfine/EECE5554",
        fontsize=12
    )
    pdf.savefig(fig)
    plt.close(fig)

    for name in names:
        path = figures / name
        if not path.exists():
            raise FileNotFoundError(path)

        img = mpimg.imread(path)

        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.imshow(img)
        ax.axis("off")
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

print("Created:", output)
