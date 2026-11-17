# EECE 5554 — System Design Exercise (S9)
## Design Journal — Wearable Navigation Aid

**Student:** Zhiqi Zhang
**Scenario:** S9 — Wearable Navigation Aid for a Blind Pedestrian
**Repository:** https://github.com/ZhiQ1/EECE5554

---

## 0. Fixed Client Requirements and Starting Assumptions

### Fixed requirements (do not relax)
- **G1:** Warn of any obstacle from the ground to head height within 3 m, at a walking speed of 1.4 m/s.
- **G2:** Arrive within 1 m of the room door.
- **G3:** All worn parts together weigh under 300 g and run 12 h on one charge.
- **G4:** The user never needs to look at or hold a phone.

Mission: a body-worn system guides a blind user from a bus stop to a room inside a building, with turn cues by vibration, and warns of obstacles from the ground up to head height. Setting: a sidewalk, a building entrance, a lobby, an elevator or stairs, corridors, open doors, and low branches.

### Open questions and the assumptions taken
| ID | Question | Decision |
|---|---|---|
| Q1 | Does G1's 3 m also apply to moving obstacles? | Yes. For moving obstacles, 3 m is a minimum, and R7 and R8 set the larger ranges they need |
| Q2 | Does a phone in a pocket count toward the 300 g? | No: the phone is the user's own device |
| Q4 | With no prior map, how is the destination door found? | By reading its room number on site. The destination is set before the trip on the phone by a helper, or chosen with the wristband buttons |

### Assumptions register
| ID | Assumption | Why needed | Evidence | Confidence | How to test |
|---|---|---|---|---|---|
| A1 | No white cane; the system warns over the full height, ground to head | Which heights the system owns | Design choice | | |
| A2 | Vehicles are in scope | Closing speeds far above walking speed | Design choice | | |
| A3 | Nothing installed in the building; indoors the system localizes on walls and doors it senses | Indoor fix source | Design choice | | |
| A4 | No prior map | Planning, final approach | Design choice | | |
| A5 | One user, 1.65 m tall; chest unit at 1.25 m | Vertical coverage (Tool 2) | Design choice | Medium | Measure the chest height |
| A6 | Phone carried in a trouser or coat pocket | GNSS quality, G4 | Design choice | | |
| A7 | Stop from the cue: 1.466 m at 1.4 m/s | Human part of Tool 1 | E8 control group, mean + 2 SD = 1.113 m at 1.22 m/s, × (1.4/1.22)² = 1.317 | Low: older adults, auditory cue, speed scaling | 20 stop-cue trials with a vibration cue at 1.4 m/s |
| A8 | Phone GNSS accuracy 3 m (1σ) outdoors | Tool 5 fix noise | Lab 1 standalone precision about 1.5 m, doubled for pocket and buildings | Low | RTK receiver as truth |
| A9 | Foot-IMU step-length error 3 percent; lateral noise 0.01 m/√s | Tool 4 indoor drift | none yet | Low | T8 indoor walks |
| A10 | Radar detects a car at 23 m in the duty-cycled configuration | R8 | TI lists traffic monitoring as an application; range not on the datasheet | Low | T6 |
| A11 | Radar duty-cycled to 0.6 W average | R4 | estimate | Low | Power meter on the radar board |
| A12 | The client is an assistive technology company | Who decides that the system works | Design choice | | |

### Design decision and iteration log
| # | Original decision | Test or calculation that challenged it | Change | Evidence |
|---|---|---|---|---|
| 1 | Human stopping model: steps to stop × walking step length | Full E8 paper: measured stopping distance 0.669 m, about half of the 1.29 to 1.89 m the method gave; E8 distances already include the reaction | Use E8's measured distance from the cue directly (A7) | E8 Tables 1 and 2 |
| 2 | G1's 3 m taken as enough for all obstacles | R7: an oncoming person needs 5.17 m; R8: a vehicle needs 23.0 m | Moving obstacles get their own sensor: radar | Tool 1 |
| 3 | R9: warn of a 13 mm drop at 3 m | Tool 2: a 13 mm drop at 3 m changes the ground range by 3.4 cm inside a 0.29 m ToF zone | R9 revised to 150 mm; 13 to 150 mm left as residual risk | Tool 2, E12 |
| 4 | Radar at its datasheet power, 1.19 W | Tool 8: 9.2 h on an aged pack at 2.8 °C, under 12 h | Radar duty-cycled to 0.6 W: 13.4 h | Tool 8 |
| 5 | Indoor legs of any length on dead reckoning | Tool 4: a 60 m leg reaches 3.15 m at 95 percent, over 2 m | R10: a structure fix at least every 35 m | Tool 4 |

---

## 1. Problem Definition

### 1.1 Operational concept — Pass 1

1. **Who is the client, and what would make them say the system works?**
   - Client (A12): an assistive technology company that will sell the device to blind pedestrians.
   - Stakeholders: the blind user (reach the right room alone, never be hurt, clear cues, light and lasting the day); an orientation and mobility instructor (can it be taught and trusted); pedestrians, cyclists, and drivers (not walked into); building staff (nothing installed, nothing in the way); whoever charges the device (one charge a day).
   - Success, measured on 20 trial missions: at least 19 end within 1 m of the correct door (R2); no contact with any obstacle (R1); no phone handling (R5); 12 h on an aged pack at 2.8 °C and under 300 g worn (R3, R4).

2. **Walk through one normal mission. What must the system sense at each step?**

   | Stage | What the user does | What the system must know | Absolute fix source | Cue |
   |---|---|---|---|---|
   | Bus stop | Steps off, starts the mission | destination, start position, heading | phone GNSS | mission started; first direction |
   | Sidewalk | Walks at up to 1.4 m/s | position, heading; obstacles ground to 1.65 m within 3 m; drops; people and vehicles | phone GNSS, degraded by the body and buildings | turns, warnings, stop |
   | Entrance | Finds and passes the door | where the entrance is; door state; position as GNSS fades | last GNSS fix, then none | door ahead |
   | Lobby | Crosses an open space with people | position relative to the entrance; elevator or stairs; people moving | walls and doors it senses | directions, warnings |
   | Elevator or stairs | Changes floor | first step or elevator door; stair edges; which floor | barometer; floor signs | edge warning; floor reached |
   | Corridor | Walks past many doors | position along the corridor; walls and doors; side obstacles; target door | walls and door frames | directions, warnings |
   | Room door | Stops | the correct door; within 1 m, 95 percent | the door itself, sensed | arrival |

   The mission is done when the target room number has been read and the user is within 1 m of that door.

3. **Ten ways the world can be worse than normal, and the three that worry me most**
   1. A low branch or an overhanging sign at head height.
   2. A downward drop: a curb edge, or the top of a staircase.
   3. A vehicle crossing the sidewalk at a driveway.
   4. An oncoming pedestrian or cyclist who does not yield.
   5. Phone GNSS biased by several meters between tall buildings.
   6. A glass entrance door or glass wall.
   7. Rain, frequent in Seattle winters.
   8. The elevator out of service.
   9. The wrong door or the wrong floor, with no map.
   10. The user misreads a vibration cue, or does not feel it while walking.

   Most worrying: event 2, because with no cane the system is the only protection and a missing surface is hard to sense; event 3, because the closing speed leaves far less warning time than 3 m gives (R8) and the harm is the most severe; event 9, because it is wrong but plausible, so neither the system nor the user notices.

4. **Out of scope:** deciding when to cross a street (the user's orientation and mobility skills and accessible pedestrian signals; the system still warns of vehicles); snow, ice, and nights below 0 °C; doors that need a key card; voice interaction, emergency calls, and more than one user; charging hardware.

### 1.2 Operational concept worksheet
- Users and stakeholders: see 1.1 Q1.
- Mission: see 1.1 Q2.
- Operating envelope: indoors and outdoors; daylight, dusk, streetlights, indoor light, and full darkness; dry and rain; 2.8 to 25.6 °C (Seattle December average low and August average high, E10); concrete sidewalks, curb ramps, indoor floors, stairs; ordinary pedestrian traffic.
- Off-nominal events:

  | Event | What the system does |
  |---|---|
  | Obstacle or drop within the warning range | Stop cue on both bands, then a direction cue |
  | Vehicle approaching across the path | Stop cue at once |
  | Position uncertainty too large | "Position uncertain" pattern; guidance paused; warnings continue |
  | Target door not identified | "Not confirmed" pattern; the user asks a person nearby |
  | Phone crash or Bluetooth loss | Bands play "guidance unavailable"; the chest unit keeps the stop warning |
  | Battery low | "Low battery" pattern once, at the Tool 8 reserve |

### 1.3 Requirements
| ID | Requirement | Value | Type | Source or calculation | Verification |
|---|---|---|---|---|---|
| R1 | Warn of an obstacle anywhere from the ground to head height ahead | within 3 m at 1.4 m/s; head height 1.65 m (A5) | Given | G1 | T1 |
| R2 | Arrival position at the room door | within 1 m, 95 percent, horizontal radius (k = 2.45) | Given | G2 | T2 |
| R3 | Mass of all worn parts | under 300 g | Given | G3 | Inspection: weigh every part |
| R4 | Runtime of all worn parts | at least 12 h on an aged pack at 2.8 °C | Given | G3, E10 | Tool 8, then T3 |
| R5 | No phone handling | at no point in the mission | Given | G4 | Demonstration during T2 |
| R6 | Near-field latency, obstacle appearing to vibration | at most 0.989 s | Derived from R1 | (3 − 1.466 − 0.15) / 1.4 | Analysis, then T4 |
| R7 | Warning range for an oncoming person who does not yield | at least 4.22 m + 2.71 m/s × t_system; 5.17 m at 0.35 s | Derived from R1, Q1 | Tool 1, E9 | Analysis, then T5 |
| R8 | Warning range for a vehicle crossing at a driveway | at least 19.41 m + 10.34 m/s × t_system; 23.0 m at 0.35 s | Derived from R1, A2 | Tool 1, E11 | Analysis, then T6 |
| R9 | Warn of a downward change in level | 150 mm or more, at 3 m or more | Derived from R1, A1 | E12; R6 at 3 m; revised in loop 3 | T7 |
| R10 | Interval between indoor structure fixes | at most 35 m of walking | Derived from R2 | Tool 4: 2D 95 percent error passes 2 m at 38.1 m | Analysis, then T8 |

R4 uses an average low: Seattle-Tacoma averages about 7 nights at or below 0 °C in each of December and January (E10). Those nights are outside the design envelope.

### 1.4 Tool 1 — Reaction time and warning range
Model:

$$d_{detect} \ge (v + v_o)\,t_{system} + d_{human} + v_o\,t_{human} + d_{margin}$$

| Input | Value | Source |
|---|---|---|
| v | 1.4 m/s | G1 |
| d_human | 1.466 m | A7 |
| t_human | 1.99 s | E8, mean + 2 SD = 1.17 + 2 × 0.41 |
| d_margin | 0.15 m | design choice |
| t_system, near | 0.187 s: ToF frame 0.067 + MCU 0.02 + BLE 0.05 + motor 0.05 | ToF frame from DS13754; the rest estimated, T4 |
| t_system, far | 0.35 s: radar 0.10 + tracking 0.10 + BLE 0.10 + motor 0.05 | estimated, T4 |
| v_o, person | 1.31 m/s | E9 usual pace |
| v_o, vehicle | 8.94 m/s (20 mph) | E11 |

Results: static obstacle 1.88 m (G1's 3 m holds); R6 0.989 s; R7 5.17 m; R8 23.0 m. Figure: `analysis/figures/tool1_warning_range.png`.

The notebook's `stopping_distance` assumes a constant braking deceleration, which a walking person does not have, so the human part uses E8's measured distance from the cue. The earlier conversion from steps to stop gave twice the measured distance and was dropped (loop 1).

---

## 2. System Architecture

### 2.1 Pass 1
1. **Single point of failure:** the phone, which runs estimation, perception, and guidance; the wristbands, the only output; the Bluetooth links; the chest battery.
2. **Where the system decides it is unsafe:** the safety microcontroller decides from raw ToF ranges (obstacle or drop within 3 m, or ToF data missing) and drives the stop pattern itself. The phone decides from estimates (position uncertainty, arrival unconfirmed, low battery) and pauses guidance while warnings continue.
3. **Functions that survive a crash of the main computer:** near-field obstacle and drop warning (R1, R9) and the "guidance unavailable" cue, on the safety microcontroller and the wristband watchdogs. Long-range radar warning is lost.
4. **Most compute:** perception on the phone: radar tracking for R7 and R8, and room-number reading. It must stay within the 0.35 s far-field budget.

### 2.2 Functional architecture
| Block | Function | Output | Requirement |
|---|---|---|---|
| Near-field ToF pair | ranges ground to 1.65 m, out to 4 m, 15 Hz | 2 × 64 zones | R1, R9 |
| Radar | people and vehicles with closing speed, 10 Hz | tracks | R7, R8 |
| Sign camera with IR LED | room numbers near doors, 1 Hz | image crop | Q4, R2 |
| Foot IMU | steps with zero-velocity updates, 2 Hz | step vectors | R2, R10 |
| Phone GNSS | outdoor position, 1 Hz | fix with quality | R2 |
| Barometer | floor, 10 Hz | height | R2 |
| Wristband buttons | start, confirm, choose a preset destination | events | R5 |
| Pose filter (EKF) | position, heading, floor | pose with covariance | R2 |
| Local map | walls, door frames, numbers read | map | A4 |
| Radar tracker | closing tracks in the path | threats | R7, R8 |
| Door reader | target door identified | door pose | R2 |
| Planner and cue generator | turns, arrival, warnings | vibration patterns | R2, R5 |
| Safety MCU | stop from ToF alone; phone heartbeat watchdog | stop pattern | R1, R6 |
| Two wristbands | left, right, both for stop; link watchdog | vibration | R5 |

### 2.3 Pipeline
Slide 2 figure: six sensors feed the safety microcontroller (ToF) and the phone app (everything else); both drive the two wristbands over Bluetooth Low Energy. The safety path, ToF to safety MCU to wristbands, does not pass through the phone.

### 2.4 Physical architecture
| Part | What runs on it | How it connects |
|---|---|---|
| Phone in a pocket | GNSS, pose filter, map, tracker, door reader, planner, cue generator | BLE to every worn part |
| Chest unit | 2 ToF, radar, sign camera, barometer, safety MCU (nRF52840), three 18650 cells | sensors wired to the MCU; BLE to the phone and the bands; MCU on its own regulator |
| Two wristbands | MCU, LRA motor, own cell; link watchdog; motor current check at start and every 60 s | BLE |
| Foot IMU pod | step detection with ZUPT | BLE |

### 2.5 Operating modes
| Mode | Entered when | Behavior | Left when |
|---|---|---|---|
| Normal | sensors healthy; 95 percent position bound under 1.2 m outdoors, 1 m within 5 m of the target door | guidance and all warnings | |
| Degraded position | bound over those thresholds | guidance paused, "position uncertain" pattern; warnings continue | bound under the threshold for 5 s |
| Degraded perception | ToF valid zones under 50 percent, or radar or camera data missing, for 2 sensor periods | "slow down" pattern; near-field warnings continue | data valid again for 5 s |
| Safe fallback | no phone heartbeat for 2 periods (0.2 s), or no ToF data | "guidance unavailable"; near-field stop continues if ToF data is present, otherwise a continuous stop pattern | the phone restarts and the user confirms with a button |
| Arrival unconfirmed | within 1 m of the expected door, number not matched | "not confirmed" pattern | number read and matched |
| Low battery | charge at the Tool 8 reserve | "low battery" pattern once | charging |

The 1.2 m outdoor threshold is the minimum clear width of a pedestrian access route in PROWAG (E12).

### 2.6 ROS 2 plan (final-project prototype on a laptop standing in for the phone)
| Node | Package | Topic | Message type | Rate |
|---|---|---|---|---|
| IMU driver | Lab 3 `vn_driver` | `/imu` | `sensor_msgs/Imu` | 100 Hz |
| Step detector, ZUPT | own | `/steps` | `geometry_msgs/TwistWithCovarianceStamped` | 2 Hz |
| Phone GNSS bridge | own | `/gnss/fix` | `sensor_msgs/NavSatFix` | 1 Hz |
| ToF driver | own | `/tof/low`, `/tof/high` | `sensor_msgs/PointCloud2` | 15 Hz |
| Radar driver | own | `/radar/points` | `sensor_msgs/PointCloud2` | 10 Hz |
| Pose filter | robot_localization | `/odometry/filtered` | `nav_msgs/Odometry` | 30 Hz |
| Tracker, door reader, planner, cues | own | `/tracks`, `/door`, `/cue` | own | 10 Hz |

---

## 3. Sensor Selection

### 3.1 Pass 1
1. **Cheapest sensor that could meet each need:** near obstacles, a ToF array; far moving objects, a mmWave radar; door identity, a small camera; steps, a MEMS IMU; floor, a barometer; outdoor position, the phone's own GNSS.
2. **Hardest need:** R8, 23 m in the dark on a 300 g, 12 h budget. Without it, vehicles would fall back to the user's own skills, as street crossing already does.
3. **Two sensors that fail under the same condition:** the two ToF sensors, both infrared, both blinded by strong sunlight. The radar uses radio waves and covers large objects.
4. **Placement and what each cannot see:** ToF on the chest cannot see the ground nearer than 0.52 m or head height nearer than 0.97 m, and puts only 0.17 zones on a 5 cm pole at 3 m.

### 3.2 Sensing needs
| Need | Requirement | Quantity | Value |
|---|---|---|---|
| Static obstacles ground to head | R1, R6 | range; vertical coverage | 3 m; 0 to 1.65 m from 1 to 3 m |
| Path width | R1 | horizontal field of view | 11.4° at 3 m, 33.4° at 1 m (0.6 m path) |
| Oncoming people | R7 | range with closing speed | 5.17 m |
| Vehicles | R8 | range with closing speed | 23.0 m |
| Drops | R9 | range change at 3 m | 0.39 m for 150 mm |
| Darkness | 1.2 envelope | works with no light | yes |
| Door identity | Q4 | read a room number | at 1 to 2 m |
| Indoor dead reckoning | R2, R10 | step length and heading | 3 percent step error or better |

### 3.3 Tool 2 — Field of view and blind zones
Chest unit at 1.25 m. Two VL53L5CX, 45° × 45°, 8 × 8 zones (DS13754). The search found the low sensor at 45° down and the high sensor level: together they cover the ground to 1.65 m from 1 to 3 m. Ground visible from 0.52 m; head height from 0.97 m. A 13 mm drop at 3 m changes the ground range by 3.4 cm; a 150 mm drop by 39 cm. Figure: `analysis/figures/tool2_coverage.png`.

### 3.4 Tool 3 — Trade studies
**Study A, far-field moving objects (R7, R8).** Weights: range 0.30 (R8 is the hardest number), closing speed 0.25 (R7, R8 need it), power 0.20 (R4), rain 0.15 (Seattle), wearability 0.10. Must-meet: range 23.0 m, darkness, at most 2 W. Only the TI IWR6843AOP passes (1.19 W at 24 percent duty, SWRS237C); lidar, ToF, depth camera, and ultrasonics fail on range. Switch if T6 shows the radar cannot hold a car at 23 m.

**Study B, near-field obstacles and drops (R1, R9).** Weights: vertical coverage 0.25 (G1), power 0.20, range 0.15, thin objects 0.15, drops 0.15, wearability 0.10. Must-meet: 3 m, darkness, at most 1.5 W. Two VL53L5CX score 0.664 and win 99.9 percent of 5,000 weight perturbations; the runner-up, three VL53L5CX, scores 0.633; RPLIDAR C1 0.371; the depth camera fails darkness and power; ultrasonics fail range. Switch to three if T1 shows head-height misses between 1 and 3 m.

Figures: `analysis/figures/tool3_studyA.png`, `tool3_studyB.png`.

### 3.5 Placement and prototype
- Chest unit at 1.25 m: ToF low (45° down), ToF high (level), radar facing forward, sign camera with IR LED, barometer.
- Wristbands on both wrists; foot IMU on the shoe.
- Riskiest assumption: the human stopping distance (A7), 77 percent of the R2 budget.
- Smallest prototype: a phone on the wrist that vibrates at a random time, tape marks on a hallway floor, and video; 20 walks at 1.4 m/s. The full prototype adds the course VN-100 on a shoe and a radar evaluation board, $328.80 out of pocket (Tool 9).

---

## 4. Localization, Mapping & Navigation

### 4.1 Pass 1
1. **Largest error source and the cost to halve it:** the user's stopping spread, 0.292 m (1σ), 77 percent of the R2 budget. Halving it needs no hardware: a calibration walk per user to set the cue lead. The door error would fall from 0.82 m to about 0.53 m.
2. **Weakest absolute fix:** indoors, between the entrance and the door: no GNSS and nothing installed. At most 35 m between structure fixes (R10), about 25 s of walking.
3. **When the map is wrong:** there is no prior map; a misread or missing room number leaves arrival unconfirmed, and the user asks a person nearby.

### 4.2 Methods
| Function | Choice | Alternative | Why it lost |
|---|---|---|---|
| Localization | EKF: foot-IMU steps with ZUPT, phone GNSS outdoors, wall and door-frame fixes indoors | Visual-inertial odometry on a chest camera | fails in full darkness, which is in scope |
| Mapping | No prior map; a local map of walls, doors, and room numbers built on site | A floor plan from the building | A4 |
| Navigation | Route as a list of turns; wristband cues; final approach on the door that is read | Turn-by-turn on a global map | needs the map A4 rules out |

### 4.3 Estimator design
| Item | Value |
|---|---|
| States | x, y, heading, gyro bias, floor |
| Process model | foot-IMU step vectors with ZUPT |
| Measurements | phone GNSS 1 Hz, 3 m (A8), used only when its quality field shows a fix; wall-line heading and door-frame position from ToF, 15 Hz; barometric height 10 Hz |
| Frames | `map` (built on site), `odom`, `base_link` at the chest; `foot_imu`, `tof_low`, `tof_high`, `radar`, `camera` |
| Start | standstill at the bus stop for the gyro bias; heading from the first GNSS track |
| Outlier rejection | chi-square gate at 99 percent on every update; three GNSS rejections in a row enter degraded position |

All entries in the R2 budget are accuracy: the stopping distances in E8 are measured against the floor, the truth, and the mean stop is compensated by the cue lead.

### 4.4 Tool 4 — Drift
Gyro: Lab 3, z axis (`~/EECE5554/Lab3/analysis/noise_parameters.json`), N = 0.343 deg/√h, B = 0.8 deg/h. Indoors from the last structure fix, with 3 percent step-length error and 0.01 m/√s lateral noise (A9): the step-length term dominates; the 2D 95 percent error passes 2 m after 27.2 s, 38.1 m. A 60 m leg reaches 3.15 m. Figure: `analysis/figures/tool4_indoor_drift.png`.

### 4.5 Tool 5 — Pose entering the building
Outdoors at 1.4 m/s with 3 m phone GNSS at 1 Hz, after 200 s: position 0.43 m, heading 0.16°, gyro bias 0.8 deg/h (1σ). The heading is good because a long walk on many noisy fixes still shows the direction of travel. Longest indoor interval between structure fixes that keeps 2 m: 38.1 m, set as R10 at 35 m.

### 4.6 Tool 6 — Error budget for R2
| Error source | 1σ (m) | Precision or accuracy | Evidence | Mitigation |
|---|---|---|---|---|
| User stopping variability | 0.292 | accuracy | E8 SD 0.222 m × (1.4/1.22)² | per-user calibration of the cue lead |
| Door position from sensing | 0.100 | accuracy | assumption, T2 | number read twice |
| Dead reckoning over the last 3 m | 0.092 | accuracy | Tool 4 | short final leg |
| Cue latency jitter, 50 ms × 1.4 m/s | 0.070 | accuracy | assumption, T4 | time stamps from one clock |
| Chest to body-centre offset | 0.050 | accuracy | assumption | tape measure |

Total 0.334 m (1σ), 0.82 m at 95 percent (k = 2.45), against 1.00 m: met with 0.18 m to spare. The stopping spread may grow to 0.375 m before R2 fails. Figure: `analysis/figures/tool6_door_budget.png`.

### 4.7 Mapping, planning, and control
The local map holds wall lines and door frames from the ToF sensors and the numbers read at each door, rebuilt each trip. The route is a list of turns and landmarks set before the trip. The final approach starts when the target number is read twice; the stop cue leads the door by the expected stopping distance, 1.47 m. If localization degrades, guidance pauses while warnings continue.

---

## 5. System Integration

### 5.1 Pass 1
1. **Which clock stamps each measurement:** the phone clock is the reference; the chest MCU and wristbands stamp with their own clocks, synchronized from the phone heartbeat.
2. **Closest to its limit:** mass, 292 g of 300 g. Then runtime, 13.4 h of 12 h.
3. **Worst-case runtime:** 13.4 h on an aged pack at 2.8 °C with the radar duty-cycled; 9.2 h at full radar duty.
4. **First three tests with course hardware:** stop-cue trials at 1.4 m/s (A7); VN-100 on a shoe over indoor walks of 20 to 60 m (T8); latency from an obstacle appearing to the motor signal (T4).

### 5.2 Frames and calibration
`map → odom → base_link` (chest) → `tof_low` (45° down), `tof_high` (level), `radar`, `camera`; the foot IMU through the step model. Static transforms from the chest-unit CAD, checked with a tape measure.

| What | How | How often | Acceptance |
|---|---|---|---|
| Gyro bias | 5 s standstill | every start | change below 10 deg/h |
| Step length | walk a measured 20 m line | weekly | error under 3 percent |
| ToF pitch | wall at a known distance | after any knock | under 1° |
| Chest height | tape measure | when the strap changes | ±2 cm |

### 5.3 Timing
A 10 ms stamp error is 1.4 cm at 1.4 m/s, and about 10 cm against a car closing at 10 m/s. The budget carries 50 ms of cue jitter, 7 cm. Bluetooth latency is measured in T4 and compensated.

### 5.4 Tool 7 — Data rates
| Stream | Hz | Size | Rate |
|---|---|---|---|
| Radar tracks | 10 | 320 B | 25.6 kbit/s |
| ToF zones | 15 | 384 B | 46.1 kbit/s |
| Sign image crop, near doors | 1 | 5 kB | 40.0 kbit/s |
| Steps, barometer, heartbeat, cues | | | 3.8 kbit/s |

Total 0.12 Mbit/s, 23 percent of an assumed 0.5 Mbit/s BLE throughput. Full 20 kB images would double it, so only the sign region is sent.

### 5.5 Compute
Radar signal processing on the radar's own DSP; ToF stop decisions on the safety MCU; tracking, filtering, number reading, and planning on the phone. The far-field path must fit in 0.35 s; T4 measures it.

### 5.6 Tool 8 — Power and runtime
| Component | Average (W) | Peak (W) | Source |
|---|---|---|---|
| mmWave radar | 1.19, duty-cycled to 0.60 | 1.75 | SWRS237C; 0.60 estimated (A11) |
| ToF sensors, 2 | 0.60 | 0.60 | estimate |
| Sign camera, 10 percent duty | 0.03 | 0.30 | estimate |
| Safety MCU and BLE | 0.03 | 0.06 | estimate |
| Barometer | 0.001 | 0.001 | estimate |

Three INR18650-35E cells (3.6 V, 3.35 Ah minimum, 8 A continuous each), 90 percent usable, aged 0.8, cold 0.85, 15 percent reserve, converter 90 percent: full-duty radar 2.06 W, 9.2 h, fails; duty-cycled 1.40 W, 13.4 h, meets. Peak 0.84 A.

### 5.7 Tool 9 — Mass and cost
| Component | Count | Mass (g) | Cost (USD) | Source |
|---|---|---|---|---|
| Li-ion cell, INR18650-35E | 3 | 150 | | datasheet, 50 g max each |
| Radar module | 1 | 20 | 178.80 (EVM) | DigiKey; mass estimated |
| ToF breakout | 2 | 6 | 50.00 | estimate |
| Sign camera with IR LED | 1 | 10 | | estimate |
| Safety MCU with BLE, barometer | 1 | 6 | 25.00 | estimate |
| Chest enclosure and strap | 1 | 45 | 15.00 | estimate |
| Wristband | 2 | 40 | 20.00 (motors) | estimate |
| Foot IMU pod | 1 | 15 | course VN-100 | course, Lab 3 |

Worn mass 292 g against 300 g. Prototype out of pocket $328.80, including the sign camera ($15), barometer ($10), and three cells ($15), all estimated prices.

### 5.8 Verification plan and traceability
| Requirement | Design element | Test | Pass | In the final project |
|---|---|---|---|---|
| R1 | ToF pair, safety MCU | T1: 20 approaches to obstacles at three heights | first cue at 3 m or more every time | Yes |
| R2, R5 | door reader, pose filter | T2: 20 missions | 19 of 20 within 1 m; no phone handling | Partly |
| R3 | mass budget | weigh every worn part | under 300 g | Yes |
| R4 | pack, radar duty cycle | T3: full discharge | 12 h after derating | Yes, at room temperature |
| R6 | safety path | T4: 20 timed obstacle appearances | each at most 0.989 s | Yes |
| R7 | radar | T5: 20 approaches by a second person | first cue at 5.17 m or more | Yes |
| R8 | radar | T6: moving target or recorded vehicle data | detection at 23 m or more | Partly |
| R9 | ToF low | T7: 20 approaches to a 150 mm step down | first cue at 3 m or more | Yes |
| R10 | foot IMU, ToF fixes | T8: indoor walks of 20 to 60 m | under 2 m at 95 percent | Yes |

Tool 10: every requirement R1 to R10 has a design element and a test, and every element serves a requirement. No problems found.

---

## 6. Failure Analysis

### 6.1 Pass 1
1. **Single failure that could hurt someone:** a failed wristband motor, since the bands are the only output. Each band's MCU checks its motor current at start and every 60 s; the other band then plays "guidance unavailable".
2. **Safety functions after a crash of the main computer:** the near-field stop on the safety MCU, and the wristband link watchdogs.
3. **Two sensors with a shared weakness:** the two ToF sensors, blinded together by sunlight.

### 6.2 Tool 11 — Ranking
| Failure mode | S | O | D | RPN | After |
|---|---|---|---|---|---|
| Drop of 13 to 150 mm missed | 9 | 5 | 8 | 360 | 216 |
| Thin branch or pole at head height missed | 9 | 5 | 7 | 315 | 216 |
| GNSS biased between buildings | 6 | 7 | 7 | 294 | 168 |
| Radar false alarms in crowds | 7 | 6 | 6 | 252 | 112 |
| User misreads a vibration cue | 8 | 4 | 7 | 224 | 96 |
| Vehicle above 20 mph or radar range short | 10 | 3 | 6 | 180 | 100 |
| Sun blinds the ToF sensors | 9 | 4 | 5 | 180 | 108 |
| Wrong door, number misread | 6 | 4 | 7 | 168 | 72 |
| Indoor step-length drift | 5 | 6 | 5 | 150 | 45 |
| Floor misidentified by the barometer | 5 | 4 | 5 | 100 | 30 |
| Wristband motor fails | 9 | 2 | 5 | 90 | 36 |
| Phone app crashes or BLE drops | 8 | 4 | 2 | 64 | 32 |
| Battery runs out early, aged and cold | 6 | 3 | 3 | 54 | 24 |

Top five: the first five rows. Severity 9 or 10: drop missed, thin object missed, vehicle, sun, wristband motor.

### 6.3 Detection and recovery
| Failure | Detection and threshold | Response | Mode | Leaves when |
|---|---|---|---|---|
| Drop missed | foot IMU stumble: vertical acceleration spike with no step pattern | stop cue | Safe fallback | user confirms |
| Thin object missed | radar return at head height with no ToF return | slow-down cue | Degraded perception | no conflict for 5 s |
| GNSS biased | 99 percent innovation gate; 3 rejections in a row | reject and count | Degraded position | bound under 1.2 m for 5 s |
| Radar false alarms | track confirmed over 3 frames; closing tracks in the path cone only | fewer cues | Normal | |
| Cue misread | no button confirmation of a turn within 3 s | repeat the cue | Normal | |
| Vehicle | radar closing speed over 8.94 m/s | stop cue at once | Normal | |
| Sun blinds ToF | valid zones under 50 percent for 0.5 s | slow-down cue; radar still warns | Degraded perception | over 80 percent for 5 s |
| Wristband motor | motor current at start and every 60 s | other band plays "guidance unavailable" | Safe fallback | repaired |
| Phone crash or BLE loss | no message for 2 periods, seen by the bands and the chest MCU | "guidance unavailable"; chest stop continues | Safe fallback | phone restarts, user confirms |

### 6.4 Residual risk
- Drops of 13 to 150 mm and thin branches are below what 8 × 8 ToF zones resolve at 3 m.
- The stopping distance (A7) comes from older adults with a sound cue and is 77 percent of the R2 budget. The first test is 20 stop-cue trials at 1.4 m/s with a vibration cue.
- Radar range on a car (A10) and its duty-cycled power (A11) are not datasheet numbers: T6 and T3.
- The mass margin is 8 g, and most masses are estimates.

---

## 9. Evidence and Reference Log
| Ref | Source | Accessed | Supports | Used in |
|---|---|---|---|---|
| E1 | `EECE5554_SDE_instructions.ipynb` | November 2026 | G1 to G4, mission, Tools 1 to 11 | all |
| E2 | `EECE5554_SDE_report_template.md` | November 2026 | report structure | report |
| E3 | `EECE5554_SDE_example.md` | November 2026 | depth and format | report |
| E4 | Lab 3 noise parameters, `~/EECE5554/Lab3/analysis/noise_parameters.json`, data set imu_bag_1 | November 2026 | gyro z axis N and B | 4.4, 4.5 |
| E6 | Chapwouo Tchakouté, Tremblay, Menelas, Sensors 18(7):2088, 2018. doi:10.3390/s18072088 | November 2026 | vibration response time while walking, 0.1466 ± 0.0332 s | 1.4 |
| E7 | Bao et al., PLoS ONE 14(8):e0219737, 2019. doi:10.1371/journal.pone.0219737 | November 2026 | vibrotactile reaction times 200 to 2000 ms by application | 1.4 |
| E8 | Boripuntakul, Lord, Methapatara, Sungkarat, Walailak J Sci & Tech 14(1):35–42, 2017 | November 2026 | unplanned stop from the cue: 1.17 ± 0.41 s, 0.669 ± 0.222 m at 1.22 m/s | A7, 1.4, 4.6 |
| E9 | Murtagh et al., Sports Med 51(1):125–141, 2021. doi:10.1007/s40279-020-01351-3 | November 2026 | usual outdoor pace 1.31 m/s | R7 |
| E10 | NOAA U.S. Climate Normals 1991–2020, Seattle-Tacoma (USW00024233), via climate-zone.com | November 2026 | December average low 37 °F, August average high 78 °F | R4, 1.2 |
| E11 | NCHRP Web-Only Document 151, Geometric Design of Driveways, 2010. doi:10.17226/17637 | November 2026 | few vehicles above 20 mph entering a driveway | R8 |
| E12 | U.S. Access Board, PROWAG Final Rule, 2023, via CP Tech Center summary | November 2026 | changes in level: 6.4 mm vertical, 13 mm beveled, above 13 mm ramped, above 150 mm per R407 | R9, 2.5 |
| E13 | Texas Instruments IWR6843AOP datasheet SWRS237C; IWR6843AOPEVM DigiKey listing | November 2026 | 1.19 W at 24 percent duty; $178.80 | 3.4, 5.6 |
| E14 | STMicroelectronics VL53L5CX datasheet DS13754 and UM2884 | November 2026 | 4 m, 8 × 8 zones, 45° × 45°, 15 Hz | 3.3 |
| E15 | Samsung SDI INR18650-35E specification | November 2026 | 3.35 Ah minimum, 3.6 V, 8 A, 50 g | 5.6 |
