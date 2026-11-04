# VIO on the 10-30 flight bags (analysis 2026-11-01)

Bags: `~/mr_logs/vio_random_ppm_flight.bag` (teleop, 159 s, live OpenVINS inside) and `~/mr_logs/vio_corkscrew3.bag`
(MPC corkscrew-3, 97 s). Replays are deterministic (`ros1_serial_msckf`, one tracker thread) with the adopted
`config/arducam_msp` (interim F7 extrinsic prior, extrinsics + time offset online), IMU `/falcon/fc/imu`, scored
against `/natnet_ros/kingfisher/odom`. Outputs: `~/record/arena_eval/eval_1030/` (this file, `analysis_*.txt`,
`feature_depths.txt`, `image_stats.txt`, `imu_sanity.txt`, `frames/`), replays in `sweep/` (`results.md`,
`traj_*.txt`, `score_*.txt`, `log_*.txt`, `viz_*.bag` with `/ov_msckf/loop_extrinsic`, `points_*`, `odomimu`).

## Flight timelines (mocap)
- teleop: on the ground until 74.3 s (motors off until ~72 s; static init at 71.8 s), hover 0.85 m until 106 s
  (speed 0.07 m/s), teleop 106-150 s up to 2.4 m/s and 23 deg, landing 156.3 s.
- corkscrew: static init 12.5 s (motors already running), takeoff 15.0 s, hover 1.3 m until 37 s, corkscrew
  37-41 s (roll to 179 deg, 5.5 m/s, 530-1100 deg/s, gyro peak 14 rad/s), hover 41-52 s, slow motion to 2 m/s,
  landing 65.4 s, on the ground until 97 s.

## Sensor streams
IMU 199.4 / 200.0 Hz, three 17-26 ms gaps in the teleop bag, none in the corkscrew bag; |acc| at rest 9.94 (motors
off) / 9.88 (motors on), hover vibration 0.51-0.56 m/s^2 3-D std, units and axes as before (`imu_sanity.txt`).
Camera 32.0 Hz, one dropped frame (corkscrew 25.8 s), frames live (consecutive frames differ), brightness 54-60,
sharpness similar in both bags (`image_stats.txt`, `frames/*.png`).

## 1. Scores with the adopted config (`sweep/results.md`)
| bag | init | ATE RMS | early-aligned drift (fit on first 10 s after init) |
|---|---|---|---|
| teleop, LIVE in-flight estimate | 71.9 s | 341 m | 10-20 s 20.8 m, 20-30 s 49 m, 30-45 s 152 m |
| teleop, offline replay | 71.8 s | 259 m | 10-20 s 20.7 m, 20-30 s 48.8 m, 30-45 s 140 m |
| corkscrew, offline replay | 12.5 s | 6.3 m (landing drift) | 10-20 s 0.06 m, 20-30 s 0.73 m (contains the roll), 30-45 s 1.64 m, 45-60 s 0.87 m, 60-90 s 11 m (on the ground after landing) |

The replay reproduces the in-flight divergence, so it is in the data + config, not in FalconPilot's copy
(their `estimator_config.yaml` equals this one apart from comments).

## 2. Online extrinsics (`analysis_*_base.txt`, p_CinI in mm, prior 68.6 / -40.5 / 60.9)
- teleop: unchanged for 35 s (0.5 mm), i.e. no information at all; moves to 66 / -69 / 76 only after 107 s when the
  filter is already hundreds of metres off. Meaningless.
- corkscrew: 53 / -34 / 69 after 5 s, 42 / -26 / 74 before the roll (33 mm from the prior, rotation 1.3 deg),
  jumps to 118 / -26 / 62 during the roll and ends at 151 / -7 / 105 (99 mm). Time offset -3.0 ms (prior -1.1 ms).
- 10-26 convergence for comparison: 69 / -41 / 61 (the interim prior itself).

## 3. Prior moved to the corkscrew convergence (42 / -26 / 74), `sweep/results.md` rows `moved`, `moved_fixed`
| variant | teleop ATE | corkscrew ATE | corkscrew early drift 10-20 / 20-30 / 30-45 / 45-60 s |
|---|---|---|---|
| adopted prior, online (base) | 259 m | 6.28 m | 0.06 / 0.73 / 1.64 / 0.87 |
| moved prior, online | 254 m | 6.17 m | 0.07 / 0.75 / 1.74 / 0.90 |
| moved prior, extrinsics fixed | - | 17.4 m | 0.06 / 1.28 / 3.47 / 3.94 |

The extrinsic is not the 10-30 problem: moving the prior changes nothing on the teleop bag and nothing on the
corkscrew bag, and freezing it at the converged value is worse. Nothing in these bags indicates that the camera
moved by centimetres in the crash; the 3 cm online wander is the same order as on 10-26.

## 4. Why the teleop flight diverged: initialised on the ground during motor spool-up, then no parallax
Not the extrinsic (section 3), not the IMU stream (`imu_sanity.txt`), not the images (`image_stats.txt`,
`frames/`), not FalconPilot's config (identical). Facts from `sweep/log_vio_random_ppm_flight_base.txt`,
`feature_depths.txt`, `spinup_imu.txt`:
- The static initialiser fired at 71.8 s on the arming jolt, 2.5 s before takeoff (74.3 s). On 10-26 the same
  thing happened but the takeoff followed 0.8-0.9 s later (`spinup_1026.txt`); on the corkscrew bag 2.5 s later
  as well, but with a shorter transient (below).
- Right after the jolt the accelerometer mean reads 0.3-0.7 m/s^2 BELOW gravity for ~2 s while the drone is
  still on the ground (|acc| 9.05-9.55 at 72.0-73.75 s; the body-x mean also steps from -1.34 to -0.4, i.e. the
  airframe settles/tilts on its gear as the motors spool up). The corkscrew bag shows the same transient for only
  ~0.75 s (9.35-9.58 at 12.75-13.25 s), the 10-26 bags for ~0.25 s.
- The camera sees a static scene with zero parallax during those seconds, so nothing can be triangulated; the
  velocity state integrates the transient: -0.4 m/s at 72.8 s, -1.0 m/s at takeoff, -2.0 m/s from 78 s on
  (`sweep/viz_*_base.bag`, odomimu twist). With that velocity error the clone geometry never matches the far
  arena scene again and NOT ONE feature passed into an update in 88 s (`points_msckf`/`points_slam` empty on all
  2306 updates); the filter ran on the IMU alone: 6 m at takeoff, 80 m by 105 s, all -z (the dive FalconPilot saw).
- The ZUPT did not save it: it was accepted on 1795 frames (disparity gate, 0.04-0.3 px with ~110 far features)
  but this fork's ZUPT only applies the IMU rest model to biases/orientation (`explicitly_enforce_zero_motion =
  false`); it does not pin velocity. Variants (`sweep/results.md`): ZUPT off -> 4051 m (attitude 45 deg RMS, worse);
  IMU-only ZUPT gate -> identical to base (both gates accept in hover); `init_imu_thresh` 1.0 -> identical (the
  jolt exceeds 1.0 anyway).
- Proof by delayed start (`sweep/results.md` rows `*_s73.0`, `*_s74.0`): the same bag and config with the estimator
  started at 73 s (motors idling, transient mostly over): the static initialiser never sees a still window (idle
  vibration 0.5-0.7 m/s^2 > thresh 0.3) and the dynamic initialiser fires at 106.6 s when the teleop motion starts;
  the 53 s teleop segment then scores **0.64 m ATE** (thresh 1.0: **0.58 m**, 2.1 deg) - the 10-26 class. Started at
  74 s: 1.45 m.

10-26 regression of the ZUPT variants (`sweep_1026/results_1026.md`): base reproduces the 10-29 numbers exactly
(10:45 0.600 m; 10:40 hover drift 0.02/0.05/0.24/0.43/0.89 m); ZUPT off 0.99 m / better hover (0.03/0.03/0.12/0.14)
but worse landing; IMU-only ZUPT gate 247 m on 10:45. Keep the ZUPT settings as they are.

## 5. The corkscrew (`analysis_corkscrew_base.txt`, `feature_depths.txt`)
Alignment fitted on the 22-35 s hover, yaw from the whole flight:
| segment | mean / max position error | mean orientation error | features per update (msckf / slam) |
|---|---|---|---|
| 15-37 s takeoff + hover | 0.09 / 0.44 m | 0.4 deg | 0-1 / 1-20 |
| 37-41 s corkscrew (179 deg roll, 5.5 m/s, 14 rad/s) | 0.59 / 1.35 m | 2.2 deg | 3-5 / 0 (SLAM map lost, 7-14 of ~30 updates per second empty) |
| 41-52 s hover after | 1.68 / 1.84 m | 1.5 deg | 0-1 / 40-59 (new map) |
| 52-65 s slow box, landing | 0.86 / 1.74 m | 1.7 deg | 0-1 / 34-98 |
| 65-97 s on the ground | 8.7 / 25 m | 1.7 deg | 0 (the usual post-landing drift) |

The filter did not diverge: the roll cost about 1.5 m of position and 2 deg of attitude, the velocity stayed sane
(VIO 4-7 m/s vs mocap 5.5), tracking resumed within a second and the error decreased again during the following
motion. The online extrinsic absorbed part of the roll (jumped 8 cm), which is why the post-roll offset stayed at
1.7 m; a Kalibr-verified fixed extrinsic would remove that absorption. So: VIO through a corkscrew is survivable
with the present setup, at the metre level; a mocap/attitude fallback during the manoeuvre is still the safe choice.

## 6. Recommendations
1. Never let the VIO initialise and then idle armed on the ground. Either take off within ~1 s of arming (10-26
   pattern), or start/reset OpenVINS only once the motors idle so the init happens at the takeoff jerk or at the
   first motion (`init_imu_thresh` 1.0 helps the static init fire on a brisk takeoff; otherwise the dynamic init
   fires at the first real motion, meaning no estimate during a ground idle / early hover).
2. FalconPilot's bad-init guard (VIO |v| > 0.3 m/s while the IMU says still) would have flagged this flight at
   ~73 s, before takeoff; keep it and refuse the VIO source for the run when it trips.
3. Keep the ZUPT settings; the code-level fix (zero-motion enforcement in the ZUPT, vibration-robust stillness for
   the static init) is a later OpenVINS change.
4. The extrinsic: no evidence of a crash shift; the Kalibr F7 re-run remains the right next step (fixes the 3 cm
   online wander and the post-roll absorption). Procedure: `config/arducam_msp/MANUAL.md` sections 4 (collecting
   the data) and 5 (running Kalibr, installing the result) = `~/record/kalibr_arducam/Kingfisher_VIO_Manual.pdf`
   (10-29 render); `README.md` section 4 is the short version.
5. Fixed today in the tree (uncommitted): `estimator_config.yaml` inline comments with ": " moved to their own
   lines (OpenCV's YAML reader made `calib_cam_intrinsics` a map; the 10-26 numbers are unchanged by it);
   `arena_sweep.sh` records `loop_extrinsic`/points during replays (`RECORD_TOPICS`), config source override
   (`CFG_SRC`), attitude topic (`ATT_TOPIC`), ZUPT/threshold variants; `arducam_msp_sensors.launch` defaults to the F7
   port /dev/ttyTHS0 with `imu_inflight` 3.
