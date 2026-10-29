#!/usr/bin/env bash
# Deterministic OpenVINS replays (ros1_serial_msckf, 1 tracker thread) of flight bags with config
# variants, scored against mocap.   arena_sweep.sh <out.md> <bag1> [bag2 ...]  (variants via VARIANTS env, space separated)
set -u
OUT="${1:?out.md}"; shift
SRC="$(rospack find ov_msckf)/../config/arducam_msp"; SCR="$(rospack find ov_msckf)/scripts"
W="${SWEEP_TMP:-$HOME/record/arena_eval/sweep}"; mkdir -p "$W"
IMU_TOPIC="${IMU_TOPIC:-/kingfisher/msp/imu}"
declare -A VAR=(
  [base]=""
  [noise_x2]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0253/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00109/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.000913/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 3.42e-05/"
  [noise_x4]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/"
  [noise_x8]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.1014/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00435/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00366/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 1.37e-04/"
  [noise_x4_fixintr]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^calib_cam_intrinsics: true/calib_cam_intrinsics: false/"
  [noise_x4_chi2x4]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^up_msckf_chi2_multipler: 2/up_msckf_chi2_multipler: 4/;s/^up_slam_chi2_multipler: 2/up_slam_chi2_multipler: 4/"
  [noise_x4_sigma3]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^up_msckf_sigma_px: 1.5/up_msckf_sigma_px: 3.0/;s/^up_slam_sigma_px: 1.5/up_slam_sigma_px: 3.0/"
  [noise_x4_thresh1]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^init_imu_thresh: .*/init_imu_thresh: 1.0/"
  [noise_x4_nozupt]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^try_zupt: true/try_zupt: false/"
  [noise_x4_fixall]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.0507/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.00218/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.00183/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 6.84e-05/;s/^calib_cam_intrinsics: true/calib_cam_intrinsics: false/;s/^calib_cam_extrinsics: true/calib_cam_extrinsics: false/;s/^calib_cam_timeoffset: true/calib_cam_timeoffset: false/"
)
VARIANTS="${VARIANTS:-base noise_x2 noise_x4 noise_x8 noise_x4_fixintr noise_x4_chi2x4 noise_x4_sigma3 noise_x4_thresh1 noise_x4_nozupt noise_x4_fixall}"
export ROS_MASTER_URI=http://localhost:11396 ROS_IP=127.0.0.1
[ -s "$OUT" ] || { echo "| variant | bag | init t | ATE RMS m | last 20% m | worst 10 s m | orient RMS deg | per-10s |" > "$OUT"; echo "|---|---|---|---|---|---|---|---|" >> "$OUT"; }
for v in $VARIANTS; do
  D="$W/cfg_$v"; rm -rf "$D"; cp -r "$SRC" "$D"
  sed -i "s#rostopic: /msp/imu#rostopic: $IMU_TOPIC#" "$D/imu.yaml"; sed -i 's/^num_opencv_threads: 4/num_opencv_threads: 1/' "$D/estimator_config.yaml"
  [ -n "${VAR[$v]:-}" ] && sed -i "${VAR[$v]}" "$D/estimator_config.yaml" "$D/imu.yaml"
  for B in "$@"; do
    N="$(basename "${B%.bag}")"; grep -q "^| $v | $N |" "$OUT" && { echo "skip $v $N"; continue; }
    EST="$W/traj_${N}_${v}.txt"; rm -f "$EST"
    DUR=$(rosbag info "$B" 2>/dev/null | awk '/duration:/ {print $NF}' | tr -d '()s' | tail -1); DUR=${DUR%.*}
    timeout $(( DUR + 120 )) roslaunch -p 11396 ov_msckf serial.launch config:=arducam_msp config_path:="$D/estimator_config.yaml" bag:="$B" max_cameras:=1 use_stereo:=false dosave:=true path_est:="$EST" verbosity:=WARNING > "$W/log_${N}_${v}.txt" 2>&1
    R=$("$SCR/score_vs_mocap.sh" "$B" "$EST" 2>&1)
    IT=$(echo "$R" | sed -n 's/.*init at bag t = \([0-9.]*\) s.*/\1/p'); RMS=$(echo "$R" | sed -n 's/.*position residual.*RMS \([0-9.]*\) m.*/\1/p')
    L20=$(echo "$R" | sed -n 's/.*last 20 %: \([0-9.]*\) m.*/\1/p'); OR=$(echo "$R" | sed -n 's/^orientation residual after alignment: RMS \([0-9.]*\) deg.*/\1/p')
    W10=$(echo "$R" | sed -n 's/^position residual RMS per 10 s window \[m\]: //p'); WORST=$(echo "$W10" | tr '|' '\n' | awk '{print $NF}' | sort -n | tail -1)
    echo "| $v | $N | ${IT:-?} | ${RMS:-fail} | ${L20:-?} | ${WORST:-?} | ${OR:-?} | ${W10:-?} |" >> "$OUT"
    echo "done $v $N: ATE ${RMS:-fail} m (init ${IT:-?} s)"
  done
done
echo "sweep finished -> $OUT"
