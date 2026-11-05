# EECE 5554 — Lab 4: Dead Reckoning

**Student:** Zhiqi Zhang  
**Northeastern username:** zhang.zhiqi2  
**Team:** 3  
**Role:** Navigator  
**Repository:** https://github.com/ZhiQ1/EECE5554

## 1. Overview

This lab investigates inertial navigation, magnetometer calibration,
complementary filtering, vehicle motion models, clock synchronization,
LiDAR ICP odometry, and navigation during a simulated GNSS outage.

The analysis uses four laptop ROS 2 recordings and two Mustang Mach-E
Car PC datasets. The final car_drive recording corresponds to the
team's third driving take.

Figures 1–13 are provided in the Canvas figures PDF. Reproducible
analysis scripts are located in `analysis/`.

### Data quality and limitations

The automated self-check passed 18 of 21 checks. The two walking bags
failed the initial-stationary and RTK-quality requirements.

The car_drive GNSS observations include 530 quality-2 fixes and
13 quality-1 fixes, with no quality-4 RTK Fixed or quality-5 RTK Float
fixes. The walking recordings also contain no RTK Fixed fixes.

Therefore, GPS-referenced trajectory errors are **not** measurements
against centimeter-accurate RTK ground truth. The results below report
the measurements actually available, including unsuccessful experiments.

Several analyses remain exploratory, notably the gyro-only heading
baseline in Figure 10 and the non-RTK outage test in Figure 13.

## 2. Results Table

| Quantity | Measured result | Units / notes |
|---|---:|---|
| Walking hard iron, x | 10.8472 | µT; normalized SVD ellipse fit |
| Walking hard iron, y | -21.0482 | µT |
| Walking soft-iron axis ratio | 1.3006 | dimensionless |
| Walking ellipse major-axis tilt | -151.73 | degrees; equivalent axis orientation modulo 180° |
| Walking calibrated radius | 26.3514 | µT |
| Walking radial standard deviation | 5.1375 | µT |
| Car hard iron, x | -73.6720 | µT; SVD ellipse fit |
| Car hard iron, y | 75.0507 | µT |
| Car soft-iron axis ratio | 1.0645 | dimensionless |
| Car ellipse tilt | 171.31 | degrees |
| Car calibrated radius | 19.2048 | µT |
| Selected complementary-filter cutoff | 0.0159 | Hz |
| Initial forward-acceleration bias | 0.4050 | m/s²; first 20 s |
| Initial pitch | 2.331 | degrees |
| Maximum absolute hill correction | 1.9064 | m/s² |
| Estimated IMU forward offset x_c (corrected IMU speed) | -0.9254 | m; estimated |
| Estimated x_c using Mustang speed | -0.4721 | m; diagnostic |
| Tape-measured IMU x_c | Not measured | m; not recorded in provided results |
| Tape-measured antenna x_a, y_a | Not measured | m; required for base_link correction |
| Heading alignment rotation, Figure 10 | 93.73 | degrees; GPS-assisted initial alignment |
| Magnetic declination used in walking plots | approx. +15 | degrees; instructor approximation, not NOAA-verified |
| IMU/GNSS distance scale factor | 0.5772 | no scale adjustment applied |
| IMU DR first exceeds 2 m | 5.56 | s after movement; GPS reference |
| IMU DR first exceeds 10 m | 41.57 | s after movement; GPS reference |
| IMU DR endpoint error | 302.23 | m; GPS reference |
| car_drive RTK Fixed share | 0 | % |
| car_drive RTK Float share | 0 | % |
| car_drive quality-1 / quality-2 observations | 13 / 530 | samples |
| car_circles laptop/car clock lag L | 3.9795 | s |
| car_circles clock correlation | 0.9999 | Pearson r |
| car_drive laptop/car clock lag L | 5.4213 | s |
| car_drive clock correlation | 0.9979 | Pearson r |
| First-half / second-half lag | 5.4373 / 5.4190 | s |
| Clock drift | -67.47 | ppm |
| GPS UTC-based independent clock lag | Not calculated | s |
| LiDAR ICP median scan residual | 0.2958 | m; not global position error |
| LiDAR/GPS distance ratio | 1.0037 | dimensionless |
| LiDAR first exceeds 2 m | 75.01 | s from recording start; includes stationary interval |
| LiDAR first exceeds 10 m | 100.97 | s from recording start; includes stationary interval |
| LiDAR endpoint position error | 93.50 | m; GPS reference |
| LiDAR endpoint error / GPS distance | 3.492 | % |
| Outage IMU-only error at 30 / 60 / 90 s | 56.92 / 238.10 / 659.89 | m; Mustang GPS reference |
| Outage LiDAR ICP error at 30 / 60 / 90 s | 4.49 / 7.70 / 15.98 | m; Mustang GPS reference |
| Outage LiDAR-distance + gyro error at 30 / 60 / 90 s | 9.73 / 21.01 / 46.52 | m; Mustang GPS reference |

The Figure 10 and Figure 12 starting orientations were calibrated
using an early moving segment of GPS observations. Accuracy within
these calibration intervals is not an independent validation result.

## 3. Analysis Questions

### Question A1 — What the dry run caught

The walking-circle recording contains 9,123 IMU measurements and 228
GNSS fixes, all quality 1. The walking-square recording contains
6,315 IMU measurements and 158 GNSS fixes: 123 quality 1 and
35 quality 2. Neither walk reached quality-4 RTK Fixed during the
recorded period, so time to first RTK Fixed cannot be reported as a
successful acquisition; its fixed share is 0% in both walks.

The self-check identified insufficient initial stationary periods
and inadequate RTK quality in both recordings. The car recordings
and Car PC data passed their self-check items. I served as the navigator during the car experiment. The VN-100 was secured in the trunk. The walking data did not meet the required RTK quality, so we retained the recorded measurements and reported the limitations.

### Question A2 — Calibration on foot

A normalized NumPy SVD fit to walk_circle gave hard-iron center
(10.8472, -21.0482) µT, axis ratio 1.3006, and calibrated
radius 26.3514 µT. The radial spread remained 5.1375 µT.
This radius is appreciably higher than the approximately 19 µT
horizontal field stated in the instructor notes for Seattle;
a date-specific NOAA value was not verified.

The walk-square GNSS closure error was 6.76 m. Dead-reckoning
endpoint closure was 50.83 m using calibrated magnetic heading
and 12.36 m using integrated gyro heading. These are closure
errors, not necessarily the position errors relative to GNSS at
the same endpoint. The magnetic interference source was not
identified from field notes, and a robust GNSS/DR rotation-angle
estimate was not available. The GNSS was not RTK Fixed.

### Question 1 — Calibration in the car

We fitted the car_circles horizontal magnetic field using a
six-parameter algebraic ellipse and NumPy SVD. The fitted hard-iron
center was (-73.6720, 75.0507) µT; the soft-iron axis ratio
was 1.0645 and the calibrated radius was 19.2048 µT.
The calibration was applied separately to car_drive.

The car's hard-iron offsets differ greatly from the walking
offsets, consistent with a substantially different magnetic
environment. Possible contributors include the steel body,
motors, wiring, and other vehicle components; the individual
source cannot be isolated from these recordings alone.

### Question 2 — The complementary filter

The main figure uses cutoff 0.0159 Hz, equivalent to a time
constant of approximately 10.01 s. A tenfold lower cutoff,
0.00159 Hz, produced 1.95° RMS disagreement with VN-100 yaw
after removing a fitted constant heading offset. The selected
0.0159 Hz produced 4.96°, and a tenfold higher cutoff,
0.159 Hz, produced 6.69°.

Thus the higher cutoff admitted more low-frequency magnetic
disturbance in this dataset, while the lower cutoff retained
the gyro contribution over longer periods and agreed more
closely with VN-100 in this diagnostic. The 0.00159 Hz test
is slightly outside the instructor's suggested 0.002–0.2 Hz
range. These RMS values are relative comparisons after offset
removal, not absolute heading-accuracy measurements. The main
cutoff remains 0.0159 Hz as an initial choice.

### Question 3 — Which heading to trust

The integrated gyro responds smoothly to turns but accumulates
bias. The magnetometer offers a heading reference but is
disturbed by the vehicle's magnetic environment. A
complementary filter combines these different characteristics.

With approximately 1 s smoothing, magnetometer and gyro
turn-rate correlation was 0.9664; with 3 s smoothing, it
was 0.9900. The initially low unfiltered correlation
was therefore dominated by short-term variations.

The complementary heading and VN-100 yaw differed by an
approximately -83.46° constant offset. After removing
that offset for diagnostic comparison only, the RMS
difference at 0.0159 Hz was 4.96°. The fixed offset means
that absolute heading-frame agreement remains unresolved.
VN-100 yaw was used only for the allowed comparison.

### Question 4 — Velocity

Integrating forward acceleration without correction gave
87.304 m/s RMS difference from recorded GNSS-derived speed.
Removing the initial 20 s acceleration bias of 0.405 m/s²
reduced the RMS difference to 44.459 m/s. Accounting for
pitch-dependent gravity reduced it to 7.730 m/s; applying
detected stop constraints reduced it to 2.487 m/s.

The maximum absolute modeled hill correction was
1.9064 m/s². Mustang's brake-velocity channel was inferred
to be in m/s and had a 0.102 m/s RMS difference from the
GNSS-derived speed. This vehicle signal remains much
more accurate than integrated IMU speed in the present
analysis. In particular, residual acceleration bias caused
substantial cumulative-speed error. GNSS quality was
insufficient to treat this as an RTK-accuracy comparison.

### Question 5 — The motion model

A 1 Hz low-pass filter was applied to measured lateral
acceleration and the predicted centripetal acceleration
omega*V. With corrected IMU speed, the RMS mismatch
was 0.3450 m/s² before the angular-acceleration
lever-arm term, and 0.3436 m/s² afterward. Least squares
estimated x_c = -0.9254 m.

Using Mustang speed as a diagnostic gave 0.3101 m/s²
without the correction, 0.3094 m/s² with it, and
x_c = -0.4721 m. The improvement is small. Sensor-frame
uncertainty, pitch/roll, imperfect velocity, road
vibration, and tire slip can contribute. The VN-100 was secured in the trunk, behind the passenger cabin. The fitted offset was negative, but its physical accuracy cannot be checked without the measured position relative to the rear axle.

### Question 6 — How long can you dead-reckon?

The gyro-heading and corrected-IMU-speed baseline used a
93.73° initial alignment rotation. Its traveled distance
was 1551.18 m compared with 2687.48 m for GNSS, giving
scale factor 0.5772 without rescaling.

The position error first exceeded 2 m after 5.56 s
and 10 m after 41.57 s from detected movement start.
Endpoint error was 302.23 m. These estimates are relative
to recorded GNSS, not RTK Fixed. The first moving segment
was used to estimate initial orientation, so early errors
are not an entirely independent test.

The driving bag contained 0% RTK Fixed and 0% RTK Float,
with 13 quality-1 and 530 quality-2 observations.
The antenna-to-base_link offset was not measured, so its
effect on the 2 m threshold cannot be quantified.
The IMU speed estimate, not simply its heading, was a
major limitation: it integrated to only 57.7% of the
GNSS path length. On these results, dependable IMU-only
navigation through a long tunnel is not demonstrated.

### Question 7 — Where the magnetometer failed

The car magnetometer experienced a substantial hard-iron
offset and short-term heading disturbances. Smoothing
magnetic and gyro turn rates improved their correlation
from the unfiltered 0.2661 to 0.9664 at 1 s.

A route-by-route list of intervals exceeding 5° has not
yet been extracted. The driving route was in the Kerry Park area of Seattle. Driving was generally smooth. The magnetic heading showed deviations, but the available data do not establish which nearby objects or road features caused them. Large traction and braking
currents in an electric vehicle are plausible sources
of magnetic interference, but that relationship was not
quantitatively verified here.

### Question 8 — Two clocks

Using the opposite Z-axis conventions of the VN-100 and
Ouster gyros, cross-correlation yielded clock lag
L = 3.9795 s for car_circles (r = 0.9999) and
L = 5.4213 s for car_drive (r = 0.9979).

The car_drive first- and second-half lags were
5.4373 s and 5.4190 s, corresponding to estimated drift
of -67.47 ppm. Without flipping the gyro sign, the
correlations were approximately -0.9999 and -0.9979
instead of positive peaks.

The independent GPS UTC lag was not calculated. Therefore,
agreement between gyro-derived and GPS-derived lag cannot
be quantified. Gyro cross-correlation is more precise than
hand-recorded start times because it matches physical turns
observed by both computers rather than relying on manually
noted recording start times.

### Question 9 — LiDAR odometry

Speed-initialized ICP was run on the recorded LiDAR scans.
The median nearest-neighbor residual was 0.2958 m,
which measures scan fit rather than trajectory accuracy.

After rigid orientation alignment using the early moving
GPS segment, the LiDAR/GPS path-length ratio was 1.0037.
Endpoint position error was 93.50 m, corresponding to
3.492% of the GPS distance. Error first exceeded 2 m
at 75.01 s and 10 m at 100.97 s from recording start;
these times include the approximately 39 s stationary
beginning and the GPS-assisted calibration interval.

The maximum GPS-relative position error was 139.03 m.
These results are not an RTK Fixed comparison.
The driving took place around Kerry Park and was generally smooth. The current ICP implementation did not retain point-pair
share and final matched-pair RMS per scan. Consequently,
the lowest-overlap segment and its geographic cause cannot
be established from the saved residuals. Motion within
one scan introduces distortion: on straight sections it
stretches the scan along the motion direction, while during
turns it can additionally bend the apparent geometry.
A quantitative hill-by-hill LiDAR/accelerometer speed
comparison was not completed.

### Question 10 — The outage

A simulated 90 s GNSS outage was evaluated against
Mustang GPS rather than the required RTK Fixed data.

| Method | 30 s | 60 s | 90 s |
|---|---:|---:|---:|
| Approximate IMU-only | 56.92 m | 238.10 m | 659.89 m |
| LiDAR ICP | 4.49 m | 7.70 m | 15.98 m |
| LiDAR distance + gyro heading | 9.73 m | 21.01 m | 46.52 m |

The approximate IMU-only estimator failed fastest, mainly
because of acceleration integration and residual bias.
The LiDAR ICP trajectory performed best in this interval.

Contrary to the proposed fusion advantage, replacing
LiDAR turns with integrated gyro heading increased the
error in this experiment. This suggests shortcomings in
heading alignment or gyro-based turn integration; an
improvement is not claimed.

A 2 km tunnel at 20 m/s takes approximately 100 s.
For this dataset, LiDAR ICP would be the preferable
starting point, but a single 90 s test is insufficient
to guarantee safe navigation for 100 s or more.
A production system would additionally require robust
motion compensation, sensor-failure detection, calibrated
wheel odometry, map matching or localization, and
uncertainty-aware sensor fusion.

Because the dataset lacked a continuous RTK Fixed interval,
the prescribed RTK-ground-truth outage experiment could
not be completed exactly as specified.

## 4. Reproducibility and Submission

- `analysis/`: analysis scripts, intermediate results, and figures.
- `data/walk_circle/`, `data/walk_square/`: walking ROS bags.
- `data/car_circles/`, `data/car_drive/`: laptop car ROS bags.
- `data/car/car_circles.npz`, `data/car/car_drive.npz`: Mustang Car PC data.
- `ros2_ws/src/`: ROS 2 packages and launch configuration.
- `zhang.zhiqi2_lab4.txt`: self-check report.
- `zhang.zhiqi2_lab4_figures.pdf`: Canvas figures PDF.

The original self-check failures and unresolved measurement gaps
are reported rather than replaced with fabricated results.
