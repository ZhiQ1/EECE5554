# EECE 5554 — Lab 3: IMU

## Setup

I used a VectorNav VN-100 IMU with ROS 2 Jazzy on Ubuntu 24.04. I wrote a driver that reads VNYMR messages from the serial port, checks the checksum, and publishes the data to `/imu` at 40 Hz.

I recorded the stationary data in a room at NEU's Terry Building in Seattle. The sensor was on a table, about 40 cm away from my computer. I recorded the motion data by myself.

The stationary recording contains 15,307 messages over 383.1 seconds. The motion recording contains 3,985 messages over 100.0 seconds. My driver passed all 24 self-check tests.

## Question 1 — Gravity and Magnetic Field

The Z-axis accelerometer reading was closest to gravity, with a mean of -9.6439 m/s². The X and Y means were -1.0895 and 0.4199 m/s². The total acceleration magnitude was 9.7143 m/s².

The negative Z reading suggests that the sensor's positive Z axis was pointing mostly downward. This is different from the usual ROS REP 103 convention, where Z points upward.

The measured magnetic field was (28.562, -14.624, 117.292) µT. Its total magnitude was 121.602 µT, and the calculated inclination was 81.53°.

The magnetic field reading is unusually high compared with the expected Earth's magnetic field. Since I recorded indoors, nearby metal objects, electronic equipment, or sensor calibration errors may have affected the measurement.

**NOAA comparison:** [Add NOAA field strength and inclination for Seattle on the recording date.]

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

These changes are probably related to movement or vibration during recording, rather than just random sensor noise. They also increase the standard deviation.

## Question 3 — Allan Deviation

I used the provided five-hour recording to calculate Allan deviation for all three gyroscope axes. The recording contained 717,785 messages, and I rejected 8 messages during checksum validation.

The calculations used the nominal sampling rate of 40 Hz.

| Axis | N (deg/√h) | B (deg/h) | K | Minimum τ (s) |
|---|---:|---:|---|---:|
| X | 0.3593 | 0.9037 | Not resolved | 1602.78 |
| Y | 0.4037 | 0.8694 | Not resolved | 1794.40 |
| Z | 0.3427 | 0.7753 | Not resolved | 1794.40 |

The datasheet reference in the assignment gives a gyro noise density of 0.0035 deg/s/√Hz and an in-run bias stability below 10 deg/h.

My N values are higher than the reference noise density, while all three B values are below 10 deg/h. Y has the largest N, and X has the largest B.

I could not identify a clear +1/2 slope in the Allan deviation curves, so K was not resolved.

The VNYMR angular velocity readings are corrected by the VN-100's onboard bias estimate. This correction can reduce slow bias drift, which may make the long-τ part of the Allan deviation curve flatter or lower than it would be for raw gyroscope measurements.

## Question 4 — Why Five Hours?

I compared the Z-axis Allan deviation from the first five minutes of my stationary recording with the provided five-hour recording.

Both curves generally decrease as the averaging time increases, but they do not overlap closely.

At τ = 0.1 s, my Allan deviation was about 5.60 times the five-hour result. At τ = 1.025 s, it was about 5.17 times higher. The difference may be related to disturbances in my stationary recording or different recording conditions.

My five-minute recording only allowed Allan deviation calculations up to about 29.95 seconds, while the five-hour recording reached 1794.4 seconds.

Using the relative error approximation from the instructions, the estimated uncertainty for my five-minute recording increases from about 4.1% at τ = 1 s to 23.6% at τ = 30 s.

Five minutes is not enough to observe the much longer time scales needed for a reliable bias instability estimate. It is also too short to determine whether the curve eventually develops the +1/2 slope associated with rate random walk.

## Question 5 — From Turning Rate to Heading

I used the horizontal