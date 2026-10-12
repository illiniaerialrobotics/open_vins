#!/usr/bin/env bash
# Replay a camera+IMU bag through OpenVINS (config/arducam_msp) and report the trajectory:
# init time, path length, start->end distance (loop-closure error if you returned to the start
# mark), max speed, and warning counts. Runs on a PRIVATE ROS master (port 11399) so live sensor
# nodes cannot mix into the bag data.
#
#   rosrun ov_msckf eval_replay.sh <bag> [--video] [--view=tracker|color|mono] [extra roslaunch args]
#
# --video: renders <bag_dir>/videos/<bag>.mp4: plain camera view (colour if recorded) + top-down
#          trajectory inset. --view=tracker instead shows OpenVINS's feature overlay (recorded
#          from /ov_msckf/trackhist during the replay).
set -u
BAG="$(realpath "${1:?bag path}")"; shift || true   # roslaunch nodes run from ~/.ros, so the path must be absolute
[ -f "$BAG" ] || { echo "bag not found: $BAG"; exit 2; }
VIDEO=0; VIEW="camera"; ARGS=()
for a in "$@"; do case "$a" in --video) VIDEO=1;; --view=*) VIEW="${a#--view=}";; *) ARGS+=("$a");; esac; done
NAME="$(basename "${BAG%.bag}")"
EST="/tmp/traj_${NAME}.txt"; LOG="/tmp/replay_${NAME}.log"; VIZ="/tmp/replay_${NAME}_viz.bag"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rm -f "$EST" "$VIZ"
DUR=$(rosbag info "$BAG" 2>/dev/null | awk '/duration:/ {print $NF}' | tr -d '()s' | tail -1); DUR=${DUR:-120}
export ROS_MASTER_URI=http://localhost:11399 ROS_IP=127.0.0.1
echo "replaying $BAG (bag ${DUR}s) on private master $ROS_MASTER_URI ..."
RATE=1; for a in "${ARGS[@]}"; do case "$a" in bag_rate:=*) RATE="${a#bag_rate:=}";; esac; done
TMO=$(python3 -c "print(int(${DUR%.*}/float('$RATE')+40))")
timeout $TMO roslaunch -p 11399 ov_msckf arducam_msp_vio.launch dobag:=true bag:="$BAG" dosave:=true path_est:="$EST" "${ARGS[@]}" > "$LOG" 2>&1 &
LP=$!
REC=""
if [ "$VIDEO" = 1 ]; then
  sleep 3   # master + ov_msckf up; trackhist is only published while someone subscribes
  if [ "$VIEW" = "tracker" ]; then TOPICS="/ov_msckf/trackhist /ov_msckf/odomimu"; else TOPICS="/ov_msckf/odomimu"; fi
  rosbag record -q -O "$VIZ" $TOPICS > /dev/null 2>&1 &
  REC=$!
fi
wait $LP
[ -n "$REC" ] && { kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null; }
sed -i 's/\x1b\[[0-9;]*m//g' "$LOG"
# rosbag play ending normally also shows as "REQUIRED process has died" (exit code 0); only a non-zero exit is an error
if grep -aE "process has died \[pid [0-9]+, exit code [1-9]" "$LOG" | grep -q "rosbag/play"; then echo "ERROR: rosbag play failed (see $LOG)"; exit 2; fi
NSF=$(grep -ac 'failed static init' "$LOG"); NDY=$(grep -ac 'disparity is' "$LOG")
# both initializers print "successful initialization"; dynamic init is preceded by its "disparity is" attempts
if grep -aq 'successful initialization' "$LOG"; then
  if grep -a -B6 'successful initialization' "$LOG" | grep -aq 'disparity is'; then MODE="dynamic"; else MODE="static"; fi
elif [ -s "$EST" ]; then MODE="static"; else MODE="none"; fi
echo "init: $MODE ($NSF static-init failures, $NDY dynamic-init attempts)"
echo "warnings: out-of-order images $(grep -aci 'out of order' "$LOG"), backward time $(grep -aci 'went backward' "$LOG")"
python3 "$SCRIPT_DIR/traj_metrics.py" "$BAG" "$EST"
echo "log: $LOG"
if [ "$VIDEO" = 1 ]; then
  OUTDIR="$(dirname "$BAG")/videos"; mkdir -p "$OUTDIR"
  python3 "$SCRIPT_DIR/vio_video.py" --viz-bag "$VIZ" --bag "$BAG" --traj "$EST" --view "$VIEW" --out "$OUTDIR/${NAME}.mp4"
fi
