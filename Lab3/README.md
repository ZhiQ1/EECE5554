# EECE 5554 — Lab 3: IMU

## Setup

I used a VectorNav VN-100 IMU with ROS 2 Jazzy on Ubuntu 24.04. I wrote a driver to read VNYMR messages from the serial port, check the checksum, and publish the data to `/imu` at 40 Hz.

I recorded the stationary data indoors at NEU's Terry Building in Seattle. The sensor was on a table, about 40 cm away from my computer. I recorded the motion data by myself.

The stationary recording contains 15,307 messages over 383.1 seconds. The motion recording contains 3,985 messages over 100.0 seconds. My driver passed all 24 self-check tests.

## Question 1 — Gravity and Magnetic Field

The average accelerometer readings were:

| Axis | Acceleration (m/s²) |
|---|---:|
| X | -1.0895 |
| Y | 0.4199 |
| Z | -9.6439 |

The total acceleration magnitude was 9.7143 m/s², which is close to 9.81 m/s².

Most of the acceleration was measured along the negative Z axis. Since an accelerometer measures specific force, this suggests that the sensor's positive Z axis was pointing mostly downward. This is different from the usual ROS REP 103 convention, where Z points upward.

The average magnetic field readings were:

| Axis | Magnetic field (µT) |
|---|---:|
| X | 28.562 |
| Y | -14.624 |
| Z | 117.292 |

The total magnetic field magnitude was 121.602 µT. Using the accelerometer to estimate the downward direction, I calculated a magnetic inclination of 81.53°.

NOAA's World Magnetic Model estimates the Earth's magnetic field based on location and date. NOAA gives a typical total field strength of about 25–65 µT worldwide. My measurement of 121.602 µT was much higher than this range.

The data was recorded indoors, so nearby metal objects or electronic equipment may have affected the magnetometer. Sensor calibration could also be a factor.

Reference: NOAA World Magnetic Model 2025 (WMM2025).

## Question 2 — Noise

I calculated the mean and standard deviation using the full stationary recording.

| Sensor | Axis | Mean | Standard deviation |
|---|---|---:|---:|
| Gyro (rad/s) | X | 0.00019911 | 0.01722750 |
| Gyro (rad/s) | Y | 0.00000671 | 0.00338973 |
| Gyro (rad/s) | Z | 0.00017585 | 0.00306858 |
| Accel (m/s²) | X | -1.08948083 | 0.03648668 |
| Accel (m/s²) | Y | 0.41987346 | 0.10615671 |
| Accel (m/s²) | Z | -9.64391644 | 0.02218515 |

The histograms are not perfectly Gaussian. The gyroscope data contains some large spikes, especially around 281 seconds. The accelerometer Y axis also shows a slow change over time.

These changes could be caused by movement or vibration during recording. They increase the standard deviation, so the results include more than just the sensor's random noise.

## Question 3 — Allan Deviation

I used the provided five-hour recording to calculate Allan deviation for all three gyroscope axes.

The recording contained 717,785 messages. After checking the raw VNYMR messages, I rejected 8 invalid messages and used the remaining 717,777 samples.

I used the nominal sampling rate of 40 Hz for the calculations.

| Axis | N (rad/s/√Hz) | N (deg/√h) | B (rad/s) | B (deg/h) | K | Minimum τ (s) |
|---|---:|---:|---:|---:|---|---:|
| X | 1.0452e-4 | 0.3593 | 4.3812e-6 | 0.9037 | Not resolved | 1602.78 |
| Y | 1.1742e-4 | 0.4037 | 4.2151e-6 | 0.8694 | Not resolved | 1794.40 |
| Z | 9.9679e-5 | 0.3427 | 3.7590e-6 | 0.7753 | Not resolved | 1794.40 |

The VN-100 specifications list a gyroscope noise density of 0.0035 deg/s/√Hz and an in-run bias stability below 10 deg/h.

My N values were higher than the listed noise density, while all three B values were below 10 deg/h.

The Y axis had the largest N, and the X axis had the largest B.

I could not find a clear +1/2 slope in the Allan deviation curves, so I marked K as not resolved.

The VNYMR angular velocity readings are corrected by the VN-100's onboard bias estimate. This correction can reduce slow bias drift, so the long-τ part of the Allan deviation curve may be flatter or lower than it would be for raw gyroscope data.

Reference: VectorNav VN-100 IMU/AHRS specifications.

## Question 4 — Why Five Hours?

I compared the Z-axis Allan deviation from the first five minutes of my stationary recording with the provided five-hour recording.

Both curves generally decrease as the averaging time increases, but they do not overlap closely.

At τ = 0.1 s, my Allan deviation was about 5.60 times the five-hour result. At τ = 1.025 s, it was about 5.17 times higher.

The difference may be related to disturbances in my stationary recording or different recording conditions.

My five-minute recording only allowed Allan deviation calculations up to about 29.95 seconds, while the five-hour recording reached 1794.4 seconds.

Using the relative error approximation from the instructions, the estimated uncertainty for my five-minute recording was:

| Averaging time τ | Relative error |
|---|---:|
| 1 s | 4.1% |
| 10 s | 13.1% |
| 30 s | 23.6% |

As the averaging time increases, fewer clusters are available for the calculation, so the result becomes less reliable.

Five minutes is not enough to observe the longer time scales needed for a reliable bias instability estimate. It is also too short to determine whether the curve eventually develops the +1/2 slope associated with rate random walk.

## Question 5 — From Turning Rate to Heading

I used the horizontal turn between 17.5 and 22.0 seconds.

Integrating Gyro Z gave -71.2141°, while the VN-100's yaw changed by -71.4550°. The difference was 0.2409°.

Both values were negative, so the rotation direction matched. The small difference could be caused by sensor noise and the VN-100's orientation estimation.

I also used the Z-axis N and B values to estimate how much heading error could build up if a robot only used the gyroscope.

| Time | Random walk | Bias drift | Combined error |
|---|---:|---:|---:|
| 1 minute | 0.0442° | 0.0129° | 0.0461° |
| 1 hour | 0.3427° | 0.7753° | 0.8477° |

After one minute, random walk is the larger source of error. After one hour, bias drift becomes more important.

This shows why a robot cannot rely on gyroscope integration alone for a long time. Even small errors will build up, so other sensors are needed to correct the heading.

## Motion Experiment

For the horizontal turn, the Z-axis angular velocity changed in the same direction as yaw.

When I tilted the sensor, the acceleration shifted between the Y and Z axes because the sensor's orientation relative to gravity changed.

The table taps produced short acceleration disturbances. Shaking the sensor caused much larger changes in angular velocity and acceleration.

## Files

- `analysis/figures/` — Figures 1–6 and the Allan comparison
- `analysis/noise_parameters.json` — Allan deviation parameters
- `analysis/` — Python analysis scripts
- `data/stationary/` — Stationary Rosbag
- `data/motion/` — Motion Rosbag
- `zhang.zhiqi2_lab3.txt` — Self-check report
- `zhang.zhiqi2_lab3_figures.pdf` — Figures PDF

The five-hour recording was kept outside the repository because of its size.

## References

1. NOAA National Centers for Environmental Information, World Magnetic Model 2025.
2. VectorNav Technologies, VN-100 IMU/AHRS Specifications.
3. ROS REP 103, Standard Units of Measure and Coordinate Conventions.
