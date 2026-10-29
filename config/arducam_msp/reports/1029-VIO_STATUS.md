# VIO on the kingfisher: OpenVINS vs sqrt-VINS, arena results and flight recommendation

Session 1029 (2026-10-29). Data: `~/mr_logs/vio_mocap_2026-10-26-*.bag` (arena flights on agilicious with
mocap, live OpenVINS logged), `~/record/kalibr_arducam/` (handheld). Every number below comes from a kept
script output in `~/record/arena_eval/`. Nothing in this document has flown on VIO yet.

## Summary

**What this is.** A mono visual-inertial odometry (VIO) stack on the kingfisher: Arducam OV9782 global-shutter
camera (640x400 grey, 32 Hz) and the flight controller's IMU read over MSP (200 Hz), calibrated with Kalibr
on 2026-10-11, running OpenVINS or its square-root sibling sqrt-VINS on the Jetson Orin. Output: the IMU pose at
200 Hz in a gravity-aligned world fixed at the first pose (`/vio_world_align/odom_world`), ready for a pose
consumer such as FalconPilot's server (which today reads mocap only).

**What was measured.** Handheld, the stack closes 100-200 m loops within 0.5-1.4 % and matches the FC attitude
to 0.8 deg RMS. In the arena flights of 10-26 the live OpenVINS was unusable (5.6-12.7 m RMS against mocap).
Replaying those bags offline reproduces the failure exactly, which makes it a tuning/calibration problem, not a
runtime one, and allows an offline fix campaign with mocap as truth. The campaign found one dominant cause and a fix: the camera-IMU extrinsic calibrated on the old flight controller
no longer matches the new one. With the prior corrected (and more SLAM landmarks, a longer window, intrinsics
fixed) the 219 s / 2.9 m/s flight scores **0.6 m RMS against mocap** instead of 5.6 m and survives its fast
segment; the hover flight holds **0.3 m over 60 s** and only fails at a hard landing. sqrt-VINS, tried on the same
flight with the same corrections, diverges within 45 s. Recommendation: OpenVINS, with a Kalibr cam-IMU re-run on
the F7 airframe as the first action.

**What changed since the handheld tests.** The flight controller was replaced on 10-25 (Kakute H7 destroyed in a
crash -> Kakute F7 on the Jetson UART `/dev/ttyTHS0`). Same IMU class, scales re-verified on the bench, but the
IMU now sits at a different place on the airframe than the one Kalibr calibrated against, and the airframe
vibrates: hovering, the IMU sees 0.55 m/s^2 and 0.03-0.2 rad/s of motion that handheld data never had.

**What remains.** Re-calibrate cam-IMU on the F7 (10 min of recording, laptop run), one mocap flight with the corrected config
logged (expect < 0.3 m in hover), then VIO in the loop via the state mux, and a `POSE_TOPIC` override in
FalconPilot. Landing and flips stay on fallback logic (mocap or FC attitude hold), not on VIO.

## 1. The system

| element | value |
|---|---|
| camera | Arducam OV9782, global shutter, 1280x800 MJPG captured, published 640x400 mono8 at 32 Hz, exposure 1.0 ms, gain 10, V4L2 kernel timestamps |
| IMU | Betaflight FC gyro+accel over MSP, 200 Hz, 0.061035 deg/s per count, 2024 counts/g, stamp = receive - 2 ms; now a Kakute F7 on /dev/ttyTHS0 at 1 Mbaud (H7 on CH340 until 10-25) |
| calibration | Kalibr 2026-10-11 (H7 airframe): camera 0.28 px, cam-IMU 0.35 px, time offset -1.1 ms, camera 63 mm ahead / 58 mm above the IMU, optical axis 8.6 deg up |
| estimators | OpenVINS (open_vins `yan-arducam-vio`, config `config/arducam_msp`) and sqrt-VINS (`sqrtVINS`, config `config/arducam_msp`, same calibration files) |
| frames | body: camera-forward +x, left +y, up +z; raw estimator world: z up, arbitrary heading; `vio_world_align`: origin + heading at the first pose |
| compute | OpenVINS 11.6 ms/frame mean on the Orin (one third of a core); sqrt-VINS single precision |

## 2. How the data is produced and judged

| step | command |
|---|---|
| sensors (bench) | `roslaunch ov_msckf arducam_msp_sensors.launch` (camera + FC-IMU bridge; FC `serial_update_rate_hz = 1000`) |
| sensors (FalconPilot owns the FC) | `roslaunch ov_msckf arducam.launch` for the camera; IMU comes from the server as `/falcon/fc/imu` |
| record handheld | `rosrun ov_msckf record_kalibr.sh imucam <name>`; 3 s still, brisk lift, loop, back on the mark |
| record a flight | `rosbag record -O <file> /arducam/image_raw /kingfisher/msp/imu /kingfisher/msp/attitude /natnet_ros/kingfisher/odom /natnet_ros/kingfisher/pose /ov_msckf/odomimu /vio_world_align/odom_world` |
| replay + metrics | `rosrun ov_msckf eval_replay.sh <bag> [--srvins] [--video] [--view=tracker]` (loop closure, touchdown error, video with trajectory inset) |
| score against mocap | `rosrun ov_msckf score_vs_mocap.sh <bag> <traj.txt> --plot` (4-DoF alignment, ATE RMS, per-10 s residual, attitude) |
| deterministic replay | `ov_msckf serial.launch` with `num_opencv_threads: 1` (byte-identical runs; used for all sweeps) |
| sweep | `arena_sweep.sh <out.md> <bags...>` (config variants x bags, mocap-scored) |
| feature view | topic `/ov_msckf/trackhist` (sqrt-VINS: `/ov_srvins/trackhist`), image with tracked features, published while subscribed; 3-D features on `points_msckf` / `points_slam` |

## 3. Arena flights of 2026-10-26 (live OpenVINS, agilicious on mocap)

Two flights with live OpenVINS logged (agilicious on mocap, `state_source:=mocap`), scored offline against
mocap with a single 4-DoF alignment (`score_vs_mocap.sh`). The IMU link was clean (200 Hz, no gaps, gravity
9.96 m/s^2 landed), images had ~300 corners per frame (17 % saturated pixels in the brighter flight), and
initialisation was the static one at arming (2 s window: still, then the motor spin-up jerk).

| flight | duration / envelope | live OpenVINS ATE | first 20 s after init | then |
|---|---|---|---|---|
| 10:45:48 | 219 s, 36 m of path, up to 2.9 m/s, hover 113-170 s | 5.6 m RMS (reported std 0.4 m) | 6 cm | 1.5 m at 30 s, 9 m at 60 s, 14 m in the fast segment |
| 10:40:00 | 126 s, hover + short dashes, hard landing at 90 s | 12.7 m RMS | 4 cm | 0.5 m at 30 s, 5 m at 45 s, 54 m after the landing |

Offline deterministic replays (`ros1_serial_msckf`, one tracker thread) reproduce the live numbers to the
centimetre (5.57 m vs 5.58 m), so the fault is systematic. Attitude stayed within 1.3-1.9 deg RMS of the
FC's own estimate throughout, including while the position was metres off: the gyro/attitude side of the
filter is fine, the translation side is not.

**Hover vibration.** Hovering, the IMU shows 0.55 m/s^2 (3-D std) of acceleration and 0.03-0.2 rad/s of
rotation; landed with motors off it is 0.007 m/s^2. The spectrum is below 15 Hz (real airframe motion, the
FC's 25 Hz accelerometer filter has removed the propeller band).

**Online calibration tells the story.** Recording `/ov_msckf/loop_extrinsic` during the replay: the camera
position in the IMU frame starts at the Kalibr prior (63, 3, 58 mm), moves 4 cm sideways within 10 s and
swings by up to 10 cm during the flight, while the rotation stays within 1 deg. The IMU that Kalibr saw on
10-11 was the Kakute H7; the Kakute F7 fitted on 10-25 sits elsewhere on the frame.

## 4. Offline fix campaign (deterministic replays, mocap as truth)

All replays deterministic, both flights, scored against mocap. "Early drift" = position error after a
yaw+translation fit on the first 10 s after initialisation.

**Round 1, IMU noise and filter gates (prior extrinsic):** nothing helps. Noise x2: 7.8 / 21.5 m; x4: 5.2 /
123 m; x8: 5.1 / 41 m; x4 + intrinsics frozen 4.3 / 21 m; chi2 x4, sigma 3 px, init threshold 1.0, ZUPT off:
all worse than or equal to the baseline (5.6 / 12.7 m). The IMU noise model is not the problem.

**Round 2, extrinsic prior moved to where OpenVINS converges (camera 69, -41, 61 mm in the IMU frame):**

| variant | 10:45 ATE | 10:45 early drift 20-30 / 30-45 / 45-60 s | 10:40 ATE | 10:40 early drift 20-30 / 30-45 / 45-60 / 60-90 s |
|---|---|---|---|---|
| baseline (old prior) | 5.57 m | 1.5 / 2.6 / 9.1 | 12.7 m | 0.5 / 5.1 / 1.7 / 4.2 |
| old prior, no online calibration | 6.30 m | 1.5 / 2.6 / 7.2 | 7.09 m | 0.5 / 5.2 / 1.7 / - |
| new prior, extrinsics online | **0.68 m** | 1.4 / 2.3 / 2.3 | 6.65 m | 0.6 / 5.5 / 2.1 / 3.1 |
| new prior, extrinsics fixed | 3.56 m | 1.4 / 2.4 / 5.7 | 7.05 m | 0.5 / 5.0 / 1.6 / - |
| new prior fixed, 300 features | 0.65 m | 1.2 / 2.0 / 2.0 | 6.32 m | 0.1 / 2.3 / 1.8 / 3.1 |
| new prior fixed, max_slam 100, max_clones 20 | **0.59 m** | 1.3 / 2.1 / 2.2 | 6.24 m | **0.1 / 0.3 / 0.5 / 0.9** |

Reading: the extrinsic prior is the dominant fault (10x on the 10:45 flight); with it corrected the filter
survives the 2.9 m/s segment (residual 0.3-1.0 m through it). More SLAM landmarks and a longer window make
the hover drift on 10:40 negligible for 90 s; the remaining 10:40 error is the hard landing at 90 s
(close featureless floor, ZUPT off after start), the same failure as handheld loop2. The 10:45 early drift of
1-2 m after 20 s is partly yaw alignment error over a 30 m path; the globally aligned residual is 0.3-0.6 m.

**Round 3, combinations with the corrected prior (extrinsics + time offset online, intrinsics fixed):**

| variant | 10:45 ATE | 10:40 ATE | 10:40 early drift 20-30 / 30-45 / 45-60 / 60-90 s | ms/frame |
|---|---|---|---|---|
| max_slam 100, max_clones 20, 200 features (**adopted**) | 0.60 m | 6.40 m (landing) | 0.05 / 0.24 / 0.43 / 0.89 | 11.3 / 9.7 |
| same + 300 features | 0.45 m | 6.82 m (landing) | 0.05 / 0.28 / 0.46 / 0.89 | 14.9 / 12.5 |
| same + intrinsics online | 2.32 m | - | - | 11.3 |

Adopted config (`config/arducam_msp/estimator_config.yaml`, `camchain-imucam.yaml`): interim F7 extrinsic
prior, `calib_cam_extrinsics` and `calib_cam_timeoffset` on, `calib_cam_intrinsics` off, `max_slam` 100,
`max_clones` 20, `num_pts` 200 (300 buys 0.15 m on one flight for +3.6 ms/frame; optional), IMU noise as
before (halved Oct-2025 values), static init first with dynamic fallback, ZUPT only before the first motion.

## 5. OpenVINS vs sqrt-VINS

sqrt-VINS (RPNG's square-root reformulation of OpenVINS, single precision, own static/dynamic initialiser) was
set up by the labmate with the same calibration files and a matching config (`sqrtVINS/config/arducam_msp`,
`eval_replay.sh --srvins`). On handheld bags it was comparable to OpenVINS (loop2 1.0 % vs 0.5 % at touchdown,
and it held still after landing where OpenVINS drifted). On the arena flight it is not usable as configured:

| run (flight 10:45) | ATE | early drift 20-30 / 30-45 / 45-60 s |
|---|---|---|
| from bag start (110 s on the ground) | 307 m | diverged while stationary (initialised at 3 s, ZUPT did not hold) |
| started 8 s before takeoff | 35 m | 0.5 / 3.0 / 20 m |
| started 8 s before takeoff, corrected extrinsic | 28 m | 0.3 / 2.2 / 16 m |
| OpenVINS, corrected extrinsic, same flight | 0.6 m | 1.3 / 2.1 / 2.2 m (global residual 0.3-0.6 m) |

The labmate's notes list the open issues (initialisation timing, a probable data race, ZUPT acceptance into the
first motion). None of the OpenVINS findings transfer automatically. **Recommendation: fly OpenVINS**; keep
sqrt-VINS as a research option once its initialisation is fixed, since its CPU advantage is irrelevant at the
current 11 ms/frame.

## 6. Flight controller filters

The FC's own filters already do what a smoothing filter would: Betaflight's accelerometer low-pass (25 Hz)
and gyro low-passes run before MSP, and the hover spectrum of the 10-26 flights confirms it: over 60 % of the
remaining accelerometer energy is below 15 Hz (peak 6.3 Hz) and 93 % of the gyro energy is below 15 Hz, i.e.
real airframe motion and control activity, not propeller harmonics. Hover levels: accelerometer std
0.15 / 0.50 / 0.19 m/s^2 (x/y/z), gyro std 0.04 / 0.04 / 0.10 rad/s. Adding smoothing on the FC would add delay
(every 10 Hz of cut-off below 25 Hz costs several ms of lag) that the camera-IMU time offset cannot absorb
consistently, and the offline sweep shows that modelling more IMU noise in the estimator does **not** help
(x2: worse, x4: diverged on 10-40). Keep the FC filters as they are, keep `serial_update_rate_hz = 1000`, and
never change them between calibration and flight. The y-axis 0.5 m/s^2 oscillation at 6 Hz is worth a look on
the control side (roll loop), not on the sensor side.

## 7. Integration with FalconPilot

FalconPilot's server reads one topic, `POSE_TOPIC = /natnet_ros/kingfisher/pose` (`geometry_msgs/PoseStamped`,
mocap world, 200 Hz), differentiates it for velocity (12 Hz low-pass) and coasts 0.25 s / holds level after 1 s
without a pose. There is no odometry or IMU input into the state. The only VIO hook is the teleop client's
`ctbr_state_source.py`, which already consumes `/vio_world_align/odom_world` with a mocap guard.

The simplest sound integration is therefore a **PoseStamped in the mocap world frame** produced from VIO, and
a server-side `POSE_TOPIC` parameter (or remap) to select it:

| piece | status |
|---|---|
| VIO -> mocap-world alignment (4 DoF) and IMU -> rigid-body offset | `vio_align.py` (agilicious branch `yan_ACRO_VIO`) from one mocap flight; its yaml feeds `state_mux.py` |
| PoseStamped output with fallback mocap -> VIO -> FC attitude, blended switching, bad-init guard | `state_mux.py` publishes `~pose` (PoseStamped) and `~odometry`; today it is launched by the agilicious VIO launch only |
| FalconPilot side | add a `POSE_TOPIC` override so the server can read `/kingfisher/state_mux/pose` (or run the mux standalone under FalconPilot with `/falcon/fc/imu` and `/falcon/fc/attitude` as inputs); nothing else in the cascade changes, since it only needs a self-consistent gravity-aligned pose |
| what must be true first | VIO hover drift fixed (section 4); re-calibration for the F7 airframe; one more mocap flight with VIO logged to confirm < 0.2 m over a hover |

## 8. Recommendation and next steps

1. **Re-calibrate camera-IMU for the F7 airframe** (the camera intrinsics stay valid; only the two
   cam-IMU bags are needed): FC powered, `roslaunch ov_msckf arducam_msp_sensors.launch` on the bench
   (or the camera node + FalconPilot's `/falcon/fc/imu`), `rosrun ov_msckf record_kalibr.sh imucam f7_run1`
   and `f7_run2` (60-90 s each, all rotation axes, translations, figure-8, target in view), then on the laptop
   `kalibr_calibrate_imu_camera --bag imucam_f7_run1_*.bag --cam cam_wall_*-camchain.yaml --imu
   kalibr_imu.yaml --target aprilgrid_6x6.yaml --bag-freq 20 --timeoffset-padding 0.1 --dont-show-report`,
   install the result as `camchain-imucam.yaml` (re-indent the matrix rows, `%YAML:1.0` first line) and in
   `sqrtVINS/config/arducam_msp/kalibr_imucam_chain.yaml`. Expect the camera about 4 cm to the right of the
   old value. Until then the interim prior (OpenVINS's converged value) is in the config.
2. **Fly the corrected config with mocap, VIO logged** (`state_source:=mocap`, `rosbag record` as in section
   2), hover 1-2 min plus a slow box; expect < 0.3 m against mocap. Then `vio_backup`, then `vio`.
3. **Landing**: switch to mocap or hold the last pose for the touchdown; VIO on a close featureless floor
   drifts after landing (loop2, flight 10:40). ZUPT after start is an option but must be checked for false
   triggers in hover on the flight bags first.
4. **Aggressive flight**: tracking loss above ~3 rad/s is expected at 32 fps; FalconPilot's fallback
   (coast 0.25 s, level hold on the FC attitude after 1 s) and the mux's attitude mode cover it. 60 fps capture
   and 300 features are the next lever if flips must be flown on VIO.
5. **FalconPilot**: `POSE_TOPIC` override to the mux's PoseStamped (section 7).
