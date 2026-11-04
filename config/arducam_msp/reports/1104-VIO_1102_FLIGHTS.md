# VIO on the 11-02 flights (analysis 2026-11-04)

Bags (falcon_vio.launch, mocap mode, live OpenVINS inside): `~/mr_logs/vio_mocap_2026-11-02-09-34-24.bag` (main
flight, 143 s: manual transmitter takeoff, MSP override at 44.3 s, server brake/hold 50-60 s, teleop around the
arena, landing 139 s) and `~/mr_logs/vio_mocap_2026-11-02-09-32-37.bag` (57 s: brisk manual takeoff, aggressive
dive at 27 s, landing 43 s). Outputs: `~/record/arena_eval/eval_1102/` (this file, `timeline.txt`,
`modes_and_spinup.txt`, `check_sync_main.txt`, `score_live_*.txt`, `analysis_live_*.txt`, `live_jumps.txt`,
`image_stats.txt`, `frames/`), deterministic replays in `sweep/` (`results.md`, `traj_*`, `score_*`, `log_*`,
`viz_*.bag` with loop_extrinsic/points/odomimu), 10-26 regression of the 300-feature setting in `sweep_1026/`.
Streams: IMU 199.4 Hz (one 33 ms gap), camera 32.0 Hz, no dropped frames; mocap has 47 glitch samples (jumps up to
36 m/s, 3 s removed from the scoring).

## 1. Live VIO vs mocap, main flight (`score_live_09-34-24.txt`, `analysis_live_main.txt`)
- Init 22.7 s on the arming jolt (motors off until 22.45 s), takeoff 24.5 s: **1.8 s armed on the ground after the
  init**. The 10-30 mechanism recurred in a milder form: the spool-up transient (|acc| 9.7-9.9 for ~1 s) walked the
  state about 2.5 m before the first features came at 25.5 s; the filter then converged (error 2.5 -> 0.2 m by 28 s)
  because the takeoff came early enough and the transient was shorter. On the short flight the init came at the
  takeoff thrust itself (6.4 s, takeoff 6.7 s, still second = the motors-off second before arming) and the first
  20 s were the best VIO seen so far: **2-8 cm** with 90-100 features per update.
- Whole-flight 4-DoF fit: **ATE 1.10 m RMS, max 3.2 m, orientation 2.35 deg RMS**; per 10 s window 2.1 / 1.6 / 1.9 m
  for the first 30 s, then 0.5-0.8 m. Reported position std 0.10-0.15 m throughout (never flagged). Only two
  position jumps > 0.15 m (0.33 m at 25.8 s, 0.35 m at 55.5 s).
- Two regimes: with the alignment fitted on 25-35 s the error is 0.2-1.6 m through the manual phase and the override
  (24.5-52 s, mean 0.6-0.9 m), then a **2.5 m slip during the server's brake/hold at 52-55 s** (error 0.5 -> 3.0 m
  in 4 s while the mocap moved at 0.6 m/s), after which the error stays a constant 2.5-2.9 m for the remaining 80 s
  of teleop (i.e. relative accuracy 0.5-0.8 m, no further drift).
- Short flight: 2-8 cm for 20 s, then at the 27 s dive (pitch 37 deg, 5 m/s, 994 deg/s) every feature was lost
  (0 per update from 28 s on, although 250-300 corners were in the image) and the filter ran away at 5-7 m/s;
  reported std reached 1.0 m only 9 s later. The dive was the cause, not the init.

## 2. Offline replays (`sweep/results.md`, adopted config = 200 features, interim extrinsic online)
| variant | main ATE | main per-10 s (after 30 s) | early drift 10-20 / 20-30 / 30-45 s | short bag |
|---|---|---|---|---|
| live in-flight | 1.10 m | 0.5-0.8 m | 0.61 / 0.79 / 2.30 | diverged at 27 s |
| base replay (reproduces the flight) | 1.15 m | 0.5-0.8 m | 0.62 / 0.79 / 2.32 | 36.7 m (diverged at 27 s) |
| extrinsics fixed at the interim prior | 1.55 m | 0.7-2.7 m | 0.63 / 0.81 / 2.42 | - |
| extrinsics + time offset fixed | 1.46 m | 0.6-2.7 m | 0.63 / 0.79 / 1.98 | - |
| **300 features (`num_pts` 300)** | **0.62 m** | **0.2-0.6 m** | **0.44 / 0.45 / 1.07** | 37.1 m (same divergence at the 27 s dive: 0.21 m at 10-20 s, then 20 m) |

With 300 features the brake slip shrinks from 2.5 m to 1.0-1.4 m (25 SLAM features per update instead of 1-2) and
the ground-idle offset from 2.7 to 1.6 m; orientation 1.97 deg RMS. CPU +3.6 ms/frame (14.9 ms, measured 10-29).
10-26 regression (`sweep_1026/results_1026.md`): flight 10:45 0.453 m ATE (0.600 with 200), flight 10:40 hover drift
0.02 / 0.05 / 0.28 / 0.46 / 0.89 m (0.02 / 0.05 / 0.24 / 0.43 / 0.89 with 200): a consistent gain or neutral, no regression.

## 3. Online extrinsic (p_CinI, prior 69 / -41 / 61 mm, `analysis_replay_main*.txt`)
The estimate runs away in both replays and to different places: 200 features -> 56/-36/168 after 10 s, 28/-34/224 after
20 s, ends 135/16/195 (z 13 cm above the prior); 300 features -> ends 155/9/110. Rotation 1.4-4 deg from the prior,
time offset -2.5 ms. Two runs converging to two different 10-15 cm values means the extrinsic state is absorbing a
model error rather than measuring the mount; yet freezing it at the interim prior is worse (table above), so the
interim prior is not the true mount either. Candidates for the absorbed error: the camera-IMU rotation of the H7
calibration on the F7 airframe, and the accelerometer scale (rest norm 9.94-9.96 with 2024 counts/g, i.e. 1.4 %
high; the static init puts it into the bias, but it varies with attitude).

## 4. Readiness for vio_with_mocap_backup (guard 0.5 m / 1.5 m/s / 10 deg for 0.3 s)
Not with the present config. On this flight the guard would have tripped at takeoff (1.6-2.7 m ground-idle offset
if the alignment is taken at init) or at the brake slip (2.5 m at 200 features, 1.0-1.4 m at 300), and the
steady-state residual of 0.5-0.8 m per 10 s window sits on the threshold. With 300 features the steady state is
0.2-0.6 m, which fits a 0.5 m guard most of the time but not through brakes or dives. What the data says a backup
mode needs: alignment taken only after the VIO has settled (>= 5 s after takeoff), a guard of about 1.0 m / 0.3 s
rather than 0.5 m, the velocity gap as the fast trigger, and the mocap carrying every brake, dive and the landing.
Pure VIO mode is not ready: metre-class errors and a dive-induced divergence with no recovery.

## 5. What to change before the next flight, in order
1. `num_pts: 300` in `config/arducam_msp/estimator_config.yaml` (and FalconPilot's copy): halves the error on this
   flight, 0.45 instead of 0.60 m on 10-26 10:45, neutral on the 10-26 hover; it does not save the dive. Applied to the
   open_vins tree today (uncommitted). Keep `track_frequency` 35, CPU stays under a third of a core.
2. Takeoff discipline: init on the arming jolt and lift off within about a second (the short flight's 0.2 s was
   ideal: still second before arming, jerk = takeoff; there is no lower bound as long as the drone was still before
   arming). 1.8 s on the ground cost 2.5 m here, 2.5 s cost the whole flight on 10-30.
3. Kalibr cam-IMU re-run on the F7 airframe (MANUAL.md sections 4-5): the extrinsic state is absorbing a
   model error and the interim prior is wrong; after the re-run, re-test with extrinsics fixed.
4. Accelerometer scale on the F7: measure |acc| at rest in two attitudes and set `imu_acc_lsb_per_g` so that it reads
   9.81 (currently 9.94-9.96 with 2024). Bench job, 10 minutes, then re-check with a replay.
5. Aggressive manoeuvres (dive at 17 rad/s, corkscrew) lose all features; the filter recovers from a corkscrew in
   hover (10-30) but not from the dive here. Keep mocap/attitude in the loop for those; 60 fps capture is the next
   lever if they must be flown on VIO.
