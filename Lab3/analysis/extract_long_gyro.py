from pathlib import Path
from collections import Counter
import csv
import math

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


HOME = Path.home()

INPUT = HOME / "lab3_long_data/imu_bag_1/imu_bag_1_0.mcap"
OUTPUT = HOME / "lab3_long_data/gyro_long.csv"


def checksum(body):
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return f"{value:02X}"


def parse_vnymr(raw):
    """Validate VNYMR and extract three gyro axes in rad/s."""
    sentence = raw.strip()

    if not sentence.startswith("$VNYMR,"):
        return None

    if sentence.count("*") != 1:
        return None

    body, given = sentence[1:].split("*", 1)

    if len(given) != 2 or given.upper() != checksum(body):
        return None

    fields = body.split(",")

    if len(fields) != 13 or fields[0] != "VNYMR":
        return None

    try:
        values = [float(x) for x in fields[1:]]
    except ValueError:
        return None

    if not all(math.isfinite(x) for x in values):
        return None

    return values[9:12]


def main():
    counts = Counter()
    first_time = None
    last_time = None
    previous_time = None
    max_gap = 0.0

    with INPUT.open("rb") as source, OUTPUT.open(
        "w", newline=""
    ) as target:

        reader = make_reader(
            source,
            decoder_factories=[DecoderFactory()]
        )

        writer = csv.writer(target)
        writer.writerow(["time_s", "gyro_x", "gyro_y", "gyro_z"])

        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            if channel.topic != "/imu":
                continue

            counts["total"] += 1

            raw = ros_msg.raw
            gyro = parse_vnymr(raw)

            if gyro is None:
                counts["rejected"] += 1
                continue

            t = message.log_time * 1e-9

            if first_time is None:
                first_time = t

            if previous_time is not None:
                gap = t - previous_time
                max_gap = max(max_gap, gap)
                if gap <= 0:
                    counts["non_positive_dt"] += 1

            previous_time = t
            last_time = t

            writer.writerow([
                f"{t - first_time:.9f}",
                *gyro
            ])

            counts["valid"] += 1

            if counts["total"] % 100000 == 0:
                print(
                    f"Processed {counts['total']:,} messages",
                    flush=True
                )

    print("\n========== EXTRACTION SUMMARY ==========")
    print("Total:", counts["total"])
    print("Valid:", counts["valid"])
    print("Rejected:", counts["rejected"])
    print("Non-positive time intervals:",
          counts["non_positive_dt"])

    if counts["total"]:
        print(
            "Rejected percentage:",
            f"{100 * counts['rejected'] / counts['total']:.4f}%"
        )

    if first_time is not None and last_time > first_time:
        duration = last_time - first_time
        print("Duration:", f"{duration:.3f} s")
        print(
            "Average valid sample rate:",
            f"{(counts['valid'] - 1) / duration:.4f} Hz"
        )
        print("Largest valid-sample gap:",
              f"{max_gap:.6f} s")

    print("Saved:", OUTPUT)


if __name__ == "__main__":
    main()
