"""Compare DGPS course change with time-aligned gyro yaw on moving windows."""
from argparse import ArgumentParser
from pathlib import Path

import numpy as np
import yaml


def load(path, clock):
    rows = np.sort(np.genfromtxt(path, delimiter=",", names=True), order=clock)
    _, unique = np.unique(rows[clock], return_index=True)
    return rows[unique]


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path.home() / "data/team1_drive")
    parser.add_argument("--kit", type=Path, default=Path.home() / "f26-dr-kit")
    args = parser.parse_args()
    vn = load(args.data / "team1_drive_vn100.csv", "t_laptop")
    ouster = load(args.data / "team1_drive_ouster_imu.csv", "t_car")
    can = load(args.data / "team1_drive_can.csv", "t_car")
    ref = load(args.kit / "reference/team1_drive_gnss.csv", "t")

    offset = yaml.safe_load((args.kit / "clock/team1_drive.yaml").read_text())
    frames = yaml.safe_load((args.kit / "frames.yaml").read_text())
    roll, pitch, _ = frames["os_imu"]["rpy"]
    vertical = np.array([-np.sin(pitch), np.cos(pitch)*np.sin(roll),
                         np.cos(pitch)*np.cos(roll)])
    ouster_yaw = vertical @ np.vstack([ouster["gx"], ouster["gy"], ouster["gz"]])
    # Lab 4 mounts the VN-100 level with +X forward and +Z down.
    # A 180-degree roll converts forward-right-down to forward-left-up.
    vn_to_base = np.diag([1, -1, -1])
    vn_gyro = vn_to_base @ np.vstack([vn["gx"], vn["gy"], vn["gz"]])
    vn_yaw = vn_gyro[2]
    vn_time = vn["t_laptop"] + offset["offset_s"]
    can_time, os_time, ref_time = can["t_car"], ouster["t_car"], ref["t"]
    clocks = (vn_time, can_time, os_time, ref_time)
    start = max(clock[0] for clock in clocks)
    end = min(clock[-1] for clock in clocks)
    centers = np.arange(start + 5, end - 5, 0.25)

    def position(time):
        return (np.interp(time, ref_time, ref["easting"])
                + 1j * np.interp(time, ref_time, ref["northing"]))

    incoming = position(centers - 1) - position(centers - 5)
    outgoing = position(centers + 5) - position(centers + 1)
    heading = np.rad2deg(np.angle(outgoing * np.conj(incoming)))
    speed = np.interp(centers[:, None] + np.arange(-5, 5.01, 0.25),
                      can_time, can["speed_mps"])
    moving = ((speed.min(axis=1) >= 2) & (np.abs(incoming) >= 8)
              & (np.abs(outgoing) >= 8))

    for label, sign in (("left", 1), ("right", -1)):
        candidates = np.flatnonzero(moving & (sign * heading >= 8))
        if not candidates.size:
            print(f"{label}: no DGPS candidate")
            continue
        index = candidates[np.argmax(sign * heading[candidates])]
        time = centers[index] + np.linspace(-3, 3, 121)
        gyro_angles = []
        for clock, yaw in ((os_time, ouster_yaw), (vn_time, vn_yaw)):
            rates = np.interp(time, clock, yaw)
            angle = np.sum((rates[1:] + rates[:-1]) * np.diff(time) / 2)
            gyro_angles.append(np.rad2deg(angle))
        steering = np.median(np.interp(time, can_time, can["steering_wheel_angle_rad"]))
        print(f"{label}: {time[0]:.3f} to {time[-1]:.3f} car PC seconds")
        print(f"DGPS {heading[index]:+.1f}°, Ouster {gyro_angles[0]:+.1f}°, "
              f"VN-100 {gyro_angles[1]:+.1f}°, median steering {steering:+.3f} rad")


if __name__ == "__main__":
    main()
