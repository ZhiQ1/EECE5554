# EECE 5554 — Lab 5

## Results

| Set | Photos | Median matches | Median inliers | RMS (px) | RANSAC tries |
|---|---:|---:|---:|---:|---:|
| mural_50 | 5 | 1193.50 | 1090.00 | 1.24 | 16.50 |
| mural_20 | 3 | 314.50 | 271.50 | 0.86 | 22.00 |
| rotate | 3 | 154.50 | 133.50 | 0.88 | 11.50 |
| brick | 4 | 149.00 | 117.00 | 0.92 | 16.00 |

## Data and assumptions

- Four phone-photo sets totaling 15 images.
- The revised rotate set passes the overlap check.
- The mural_20 first pair has 41% overlap, exceeding the 40% limit.
- The frame set contains 1947 images over 389 seconds at 5 Hz.
- The frame timestamps do not align with the available Lab 4 car file.
- Lab 4 clock offset: L = 5.4213 s.
- Antenna offsets x_a = 1.55 m and y_a = 0 m are course-example assumptions, not Team 3 measurements.
- Figures 1–10 have been generated. Figures 6–10 use the available car_drive recording and GPS/DGPS reference; this is not RTK ground truth.

## Analysis questions

### Q1. Normalization
Hartley normalization centers image points and scales their average distance before DLT, improving numerical conditioning. The measured condition numbers require verification from the notebook output.

### Q2. RANSAC
The experiment uses matched image features and RANSAC to reject outliers. The expected four-point sample count depends on the inlier fraction; actual trials also reflect random sampling and stopping criteria. The default threshold is 3 px. Threshold comparison figures were generated.

### Q3. Plane versus rotation
A planar scene admits a homography despite camera translation. Pure camera rotation also admits a homography for scenes with depth. The revised rotate photos pass the overlap check; quantitative rotation misfits still require verification.

### Q4. Overlap and repeated patterns
The mural_50 overlaps are 56%, 60%, 55%, and 62%. The revised mural_20 overlaps are 41% and 22%. Smaller overlaps provide fewer shared constraints and can increase accumulated stitching drift. Repetitive brick patterns can create ambiguous matches.

### Q5. Bird's-eye view
The road is modeled as a plane. Objects extending above that plane, changes in road elevation, and imperfect camera calibration violate the model. Figure 5 shows the initial bird's-eye projection. Pixel-to-ground resolution measurements remain to be reported.

### Q6. Camera clock
The camera frame set runs at 5 Hz. Estimated camera lag is 0.0928 s, with angular-velocity correlation 0.8067, using the available recording. Exposure, encoding, transmission, and buffering can introduce latency.

### Q7. Pitch and yaw
Small angular calibration errors can create substantial ground-projection errors at long distances. The exploratory pitch correction is -0.4300 degrees and yaw correction is +2.5506 degrees. These are calibration estimates, not independently verified ground truth.

### Q8. Error budget
Figure 9 reports mean spread: corrected 43.93, time +0.2 s 42.70, no lever arm 43.55, heading +1 degree 44.14, and pitch +0.5 degree 52.65. Lower spread is not always obtained with the nominal corrected configuration.

### Q9. Better mapping camera
The current frame set is 1224 x 1024 at 5 Hz. A higher frame rate could reduce displacement between adjacent frames, while reliable hardware timestamps or synchronization would help motion compensation. A global shutter could reduce motion distortion. Further quantitative comparison requires synchronized motion data.

### Q10. Sensor alignment
Figure 10 includes nominal and corrected camera views with LiDAR overlays, together with the Rerun recording. Timing offsets primarily become more important during motion, while angular calibration errors change projected image positions.

## Submission limitations

Figures 1–10 have been generated. Remaining limitations include the mural_20 overlap violation, exploratory calibration estimates, and GPS/DGPS rather than RTK ground truth.
