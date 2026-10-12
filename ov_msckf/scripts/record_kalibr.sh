#!/usr/bin/env bash
# Record bags for Kalibr / VIO tests on the Jetson (local disk).
#
#   record_kalibr.sh cam        [name] [duration_s]   camera only (intrinsics)
#   record_kalibr.sh imucam     [name] [duration_s]   camera + IMU (cam-IMU extrinsics/time offset)
#   record_kalibr.sh static_imu [name] [duration_s]   IMU only, hours, for Allan variance
#   record_kalibr.sh vio        [name] [duration_s]   camera + IMU + mocap + OpenVINS outputs
#
# Stop with Ctrl-C (or give a duration). Afterwards prints `rosbag info` and runs check_sync.py.
set -u
MODE="${1:-}"; NAME="${2:-run}"; DUR="${3:-}"
OUTDIR="${KALIBR_BAG_DIR:-$HOME/record/kalibr_arducam}"
IMG="${IMAGE_TOPIC:-/arducam/image_raw}"
IMU="${IMU_TOPIC:-/msp/imu}"
case "$MODE" in
  cam)        TOPICS="$IMG" ;;
  imucam)     TOPICS="$IMG $IMU /msp/attitude /arducam/image_color" ;;
  static_imu) TOPICS="$IMU" ;;
  vio)        TOPICS="$IMG $IMU /msp/attitude /arducam/image_color /natnet_ros/kingfisher/pose /natnet_ros/kingfisher/odom /ov_msckf/odomimu /ov_msckf/odomimu_corrected /ov_msckf/poseimu /tf /tf_static" ;;
  *) sed -n 2,10p "$0"; exit 2 ;;
esac
mkdir -p "$OUTDIR"
STAMP=$(date +%Y-%m-%d-%H-%M-%S)
OUT="$OUTDIR/${MODE}_${NAME}_${STAMP}.bag"
DURARG=""; [ -n "$DUR" ] && DURARG="--duration=${DUR}"
echo "Recording $MODE -> $OUT"
echo "Topics: $TOPICS"
echo "Ctrl-C to stop."
trap 'echo' INT   # let rosbag handle Ctrl-C; keep running afterwards for the summary
rosbag record -b 1024 --tcpnodelay $DURARG -O "$OUT" $TOPICS
trap - INT
sleep 1
[ -f "$OUT" ] || { echo "no bag written"; exit 1; }
echo; rosbag info "$OUT"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTRA=""; { [ "$MODE" = "cam" ] || [ "$MODE" = "static_imu" ]; } && EXTRA="--no-flow"
echo; python3 "$SCRIPT_DIR/check_sync.py" "$OUT" --image-topic "$IMG" --imu-topic "$IMU" $EXTRA
