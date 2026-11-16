# Week 1 — Bridging GNSS Outages

[Full-drive dr_vn100 vs. DGPS](team1_drive_dr_vn100_vs_dgps.png) covers 542.5 s on team1_drive, ending at the last CAN sample. The endpoint separation is 120.9 m. The estimate uses CAN wheel speed and a complementary filter combining VN-100 gyro and quaternion yaw. DGPS sets the initial position and aligns heading over the first 15 m of displacement. There are no subsequent DGPS corrections or simulated outage windows. The filter uses a 2 s time constant and assumes NED yaw.

## Gyro sign check

VN-100 time was shifted by −2.8687 s onto the car PC clock. Ouster angular velocity was projected onto base_link +Z using the kit mounting rotation. The VN-100 projection uses the provisional −gz mapping. Both turns support the vertical-axis sign. The complete VN-100 mounting rotation still needs confirmation from Lab 4.

| Turn | Car PC time (s) | DGPS course change | Ouster integrated yaw | VN-100 integrated yaw | Median CAN steering |
| --- | --- | ---: | ---: | ---: | ---: |
| Left | 1793654100.594–1793654106.594 | +21.4° | +24.3° | +25.9° | +0.573 rad |
| Right | 1793654285.344–1793654291.344 | −78.7° | −81.3° | −79.9° | −3.807 rad |

DGPS uses travel directions estimated over four-second chords centered at the gyro integration endpoints, so the course changes and integrated angles can differ.

## Reproduce

Use Python 3 with NumPy, Matplotlib and PyYAML. Keep the three OneDrive CSVs in `~/data/team1_drive` and the course kit in `~/f26-dr-kit`, or pass `--data` and `--kit` explicitly. Run from this directory:

```bash
python3 check_gyro_signs.py
python3 plot_dr_vn100.py
```

## Break log

- OneDrive saved the CSVs inside an extracted download subdirectory. Located the files and copied them to the external data directory.
- The first distance-only DGPS turn detector identified a false left-turn candidate. Switched to moving time windows with CAN speed and minimum DGPS chord-length checks. This found a moderate left turn confirmed by both gyros.
- The track diverges from DGPS over the latter part of the route. Kept the measured drift in the plot and did not fit the trajectory to the rest of the reference.
