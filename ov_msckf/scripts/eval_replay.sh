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
timeout $(( ${DUR%.*} + 40 )) roslaunch -p 11399 ov_msckf arducam_msp_vio.launch dobag:=true bag:="$BAG" dosave:=true path_est:="$EST" "${ARGS[@]}" > "$LOG" 2>&1 &
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
if grep -aq 'successful initialization' "$LOG"; then MODE="dynamic"; elif [ -s "$EST" ]; then MODE="static"; else MODE="none"; fi
echo "init: $MODE ($NSF static-init failures, $NDY dynamic-init attempts)"
echo "warnings: out-of-order images $(grep -aci 'out of order' "$LOG"), backward time $(grep -aci 'went backward' "$LOG")"
python3 - "$EST" "$BAG" <<'PY'
import sys, os, numpy as np, subprocess
p, bag = sys.argv[1], sys.argv[2]
if not os.path.exists(p) or os.path.getsize(p) < 100:
    print("RESULT: no trajectory -> OpenVINS never initialized (needs a still start, or motion for dynamic init)"); sys.exit(1)
d = np.loadtxt(p, comments="#"); t = d[:,0]; P = d[:,1:4]
t0 = float(subprocess.run(["rosbag","info","-y","-k","start",bag],stdout=subprocess.PIPE).stdout.decode().strip() or t[0])
seg = np.linalg.norm(np.diff(P,axis=0),axis=1); L = seg.sum(); err = np.linalg.norm(P[-1]-P[0])
print("initialized at bag t = %.1f s, tracked %.1f s (%d poses)" % (t[0]-t0, t[-1]-t[0], len(t)))
print("path length %.2f m | start->end %.3f m (%.1f %% of path) | max speed %.2f m/s | max |pos| %.2f m | final z %.3f m"
      % (L, err, 100*err/max(L,1e-6), (seg/np.maximum(np.diff(t),1e-3)).max(), np.linalg.norm(P,axis=1).max(), P[-1,2]-P[0,2]))
# touchdown: start of the final stationary stretch (smoothed speed < 0.15 m/s until the end, >= 2 s long)
v = seg/np.maximum(np.diff(t),1e-3); w = max(1, int(0.5/np.median(np.diff(t)))); vs = np.convolve(v, np.ones(w)/w, mode="same")
still = vs < 0.15; k = len(still)
while k > 0 and still[k-1]: k -= 1
if k < len(still) and t[-1]-t[k] >= 2.0 and k > 0:
    e_td = np.linalg.norm(P[k]-P[0])
    print("at touchdown (t_end-%.1f s): start->end %.3f m (%.1f %% of path); drift while stationary afterwards %.3f m (z %+.3f)"
          % (t[-1]-t[k], e_td, 100*e_td/max(L,1e-6), np.linalg.norm(P[-1]-P[k]), P[-1,2]-P[k,2]))
    err = min(err, e_td)
print("RESULT: %s" % ("GOOD (loop closure < 2 %)" if err/max(L,1e-6) < 0.02 else "CHECK (loop closure >= 2 % of path, or no loop was closed)"))
print("trajectory: %s" % p)
PY
echo "log: $LOG"
if [ "$VIDEO" = 1 ]; then
  OUTDIR="$(dirname "$BAG")/videos"; mkdir -p "$OUTDIR"
  python3 "$SCRIPT_DIR/vio_video.py" --viz-bag "$VIZ" --bag "$BAG" --traj "$EST" --view "$VIEW" --out "$OUTDIR/${NAME}.mp4"
fi
