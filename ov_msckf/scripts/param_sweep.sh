#!/usr/bin/env bash
# Offline parameter sweep: replays one bag through OpenVINS with config variants and tabulates
# error at touchdown / final error / FC-attitude RMS / timing.   usage: param_sweep.sh <bag> <out.md> [variants...]
set -u
BAG="$(realpath "${1:?bag}")"; OUT="${2:?out.md}"; shift 2
SRC="$(rospack find ov_msckf)/../config/arducam_msp"; SCR="$(rospack find ov_msckf)/scripts"
S="${SWEEP_TMP:-/tmp/vio_sweep}"; mkdir -p "$S"
N="$(basename "${BAG%.bag}")"
declare -A VAR=(
  [base]=""
  [noise_half]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.00634/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.000272/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.000229/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 8.6e-06/"
  [noise_double]="s/^  accelerometer_noise_density: .*/  accelerometer_noise_density: 0.02534/;s/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.001088/;s/^  gyroscope_noise_density: .*/  gyroscope_noise_density: 0.000913/;s/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 3.42e-05/"
  [zupt_always]="s/^zupt_only_at_beginning: true/zupt_only_at_beginning: false/"
  [clones_11]="s/^max_clones: 15/max_clones: 11/"
  [clones_20]="s/^max_clones: 15/max_clones: 20/"
  [fix_intrinsics]="s/^calib_cam_intrinsics: true/calib_cam_intrinsics: false/"
  [fix_all_calib]="s/^calib_cam_intrinsics: true/calib_cam_intrinsics: false/;s/^calib_cam_extrinsics: true/calib_cam_extrinsics: false/;s/^calib_cam_timeoffset: true/calib_cam_timeoffset: false/"
  [fast_10]="s/^fast_threshold: 15/fast_threshold: 10/"
  [fast_20]="s/^fast_threshold: 15/fast_threshold: 20/"
  [clahe]="s/^histogram_method: \"HISTOGRAM\"/histogram_method: \"CLAHE\"/"
  [track_20hz]="s/^track_frequency: 35.0/track_frequency: 20.0/"
  [sigma_1.0]="s/^up_msckf_sigma_px: 1.5/up_msckf_sigma_px: 1.0/;s/^up_slam_sigma_px: 1.5/up_slam_sigma_px: 1.0/"
  [sigma_2.0]="s/^up_msckf_sigma_px: 1.5/up_msckf_sigma_px: 2.0/;s/^up_slam_sigma_px: 1.5/up_slam_sigma_px: 2.0/"
  [slam_25]="s/^max_slam: 50/max_slam: 25/"
  [slam_100]="s/^max_slam: 50/max_slam: 100/"
  [num_pts_150]="s/^num_pts: 200.*/num_pts: 150/"
  [init_static_only]="s/^init_dyn_use: true.*/init_dyn_use: false/"
  [init_reccond_1e-12]="s/^init_dyn_min_rec_cond: 1e-20/init_dyn_min_rec_cond: 1e-12/"
  [init_mindeg_15]="s/^init_dyn_min_deg: 10.0/init_dyn_min_deg: 15.0/"
)
[ $# -gt 0 ] && LIST=("$@") || LIST=(base init_static_only init_reccond_1e-12 init_mindeg_15 noise_half noise_double zupt_always clones_11 clones_20 fix_intrinsics fix_all_calib fast_10 fast_20 clahe track_20hz sigma_1.0 sigma_2.0 slam_25 slam_100 num_pts_150)
if [ ! -s "$OUT" ]; then
  echo "| variant | init | path m | touchdown err m (%) | final err m | drift on floor m | roll RMS | pitch RMS | yaw RMS | ms/frame |" > "$OUT"
  echo "|---|---|---|---|---|---|---|---|---|---|" >> "$OUT"
fi
for v in "${LIST[@]}"; do
  grep -q "^| $v |" "$OUT" && { echo "skip $v (already in table)"; continue; }
  D="$S/cfg_$v"; rm -rf "$D"; cp -r "$SRC" "$D"
  if [ -n "${VAR[$v]:-}" ]; then sed -i "${VAR[$v]}" "$D/estimator_config.yaml" "$D/imu.yaml"; fi
  EST="/tmp/traj_${N}.txt"; TIM="$S/timing_${v}.txt"
  R=$("$SCR/eval_replay.sh" "$BAG" config_path:="$D/estimator_config.yaml" dotime:=true path_time:="$TIM" 2>&1)
  INIT=$(echo "$R" | sed -n 's/^init: \([a-z]*\).*/\1/p')
  PATHM=$(echo "$R" | sed -n 's/^path length \([0-9.]*\) m.*/\1/p')
  FINAL=$(echo "$R" | sed -n 's/^path length .*start->end \([0-9.]*\) m.*/\1/p')
  TD=$(echo "$R" | sed -n 's/^at touchdown.*start->end \([0-9.]*\) m (\([0-9.]*\) % of.*/\1 (\2)/p'); [ -z "$TD" ] && TD="n/a"
  DRIFT=$(echo "$R" | sed -n 's/^at touchdown.*afterwards \([0-9.]*\) m.*/\1/p'); [ -z "$DRIFT" ] && DRIFT="-"
  ATT=$(python3 "$SCR/compare_attitude.py" --bag "$BAG" --traj "$EST" 2>/dev/null | sed -n 's/.*\(roll\|pitch\|yaw\) .*RMS diff \([0-9.]*\) deg.*/\2/p' | tr '\n' ' ')
  MS=$(python3 -c "import numpy as np; d=np.loadtxt('$TIM',comments='#',delimiter=','); print('%.1f'%(d[:,-1].mean()*1e3))" 2>/dev/null || echo "-")
  set -- $ATT
  echo "| $v | ${INIT:-?} | ${PATHM:-?} | $TD | ${FINAL:-?} | $DRIFT | ${1:-?} | ${2:-?} | ${3:-?} | $MS |" >> "$OUT"
  cp "$EST" "$S/traj_${N}_${v}.txt" 2>/dev/null
  echo "done $v: touchdown $TD final ${FINAL:-?}"
done
echo "sweep finished -> $OUT"
