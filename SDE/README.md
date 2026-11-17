# EECE 5554 System Design Exercise: S9 Wearable Navigation Aid

Zhiqi Zhang

A body-worn system guides a blind pedestrian with no cane from a bus stop to a room inside a building, warning of obstacles from the ground to head height and giving turn cues by vibration, with no phone in hand. Two 8 × 8 time-of-flight sensors, a 60 GHz radar, a room-number camera, and a safety microcontroller sit on a chest unit; a foot IMU with zero-velocity updates dead-reckons indoors; a phone in a pocket runs estimation and guidance; two wristbands vibrate left, right, or both for stop. The door-arrival budget closes at 0.82 m (95 percent) against 1 m, the worn mass is 292 g against 300 g, and runtime is 13.4 h on an aged pack at 2.8 °C once the radar is duty-cycled. Moving obstacles need 5.2 m (people) and 23 m (vehicles) of warning, beyond the given 3 m. The largest open risk is small drops and thin branches that the ToF sensors cannot resolve at 3 m.

| Path | Contents |
|---|---|
| `design_journal.md` | Pass 1 notes for Parts 1 to 6, assumptions, design loops, sources |
| `analysis/sde_analysis.ipynb` | Tools 1 to 11 run with the S9 numbers and the Lab 3 gyro parameters |
| `analysis/figures/` | every figure in the report and the slides |
| `report/` | report source and PDF |
| `slides/` | slides PDF |
