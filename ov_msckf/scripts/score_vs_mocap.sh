#!/usr/bin/env bash
# Score an estimator trajectory (pose_to_file txt: t tx ty tz qx qy qz qw) against the mocap in a bag.
#   score_vs_mocap.sh <bag> <traj.txt> [--mocap /natnet_ros/kingfisher/odom] [--attitude /kingfisher/msp/attitude] [--plot]
# Packs the trajectory and the mocap/attitude topics into a temporary bag and runs vio_align.py
# (agilicious) on it: one global 4-DoF alignment, position residual RMS (= ATE), per-10 s windows,
# orientation residual, FC attitude check.
set -u
BAG="$(realpath "${1:?bag}")"; TRAJ="$(realpath "${2:?traj}")"; shift 2
MOCAP=/natnet_ros/kingfisher/odom; ATT=/kingfisher/msp/attitude; PLOT=""
while [ $# -gt 0 ]; do case "$1" in --mocap) MOCAP="$2"; shift 2;; --attitude) ATT="$2"; shift 2;; --plot) PLOT="--plot"; shift;; *) shift;; esac; done
TMP="${TRAJ%.txt}_vs_mocap.bag"
python3 - "$BAG" "$TRAJ" "$TMP" "$MOCAP" "$ATT" <<'PY'
import sys, numpy as np, rosbag, rospy
from nav_msgs.msg import Odometry
bag_in, traj, out, mocap, att = sys.argv[1:6]
d = np.loadtxt(traj, comments="#")
if d.ndim != 2 or len(d) < 10:
    print("no trajectory in", traj); sys.exit(1)
bag = rosbag.Bag(out, "w")
for r in d:
    od = Odometry(); od.header.stamp = rospy.Time.from_sec(r[0]); od.header.frame_id = "global"; od.child_frame_id = "imu"
    od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = r[1:4]
    od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = r[4:8]
    bag.write("/vio/odom", od, od.header.stamp)
src = rosbag.Bag(bag_in); t0 = src.get_start_time()
for tp, m, t in src.read_messages(topics=[mocap, att]): bag.write(tp, m, t)
src.close(); bag.close()
print("estimator init at bag t = %.1f s, %d poses, %.1f s" % (d[0, 0] - t0, len(d), d[-1, 0] - d[0, 0]))
PY
[ $? -eq 0 ] || exit 1
python3 "$(rospack find agiros)/scripts/vio_align.py" --bag "$TMP" --mocap "$MOCAP" --vio /vio/odom --attitude "$ATT" $PLOT 2>&1 | grep -E "overlap|orientation residual|position residual|first 20|per 10 s|FC attitude|plot saved|not enough"
rm -f "$TMP"
# ---- early-aligned drift: align yaw+translation on the first 10 s after init, then error over time
python3 - "$BAG" "$TRAJ" "$MOCAP" <<'PY'
import sys, numpy as np, rosbag, math
def q_to_R(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
bag_in, traj, mocap = sys.argv[1:4]
d=np.loadtxt(traj, comments="#"); T=d[:,0]; P=d[:,1:4]; Q=d[:,4:8]
bag=rosbag.Bag(bag_in); Tm=[];Pm=[];Qm=[]
for _,m,_ in bag.read_messages(topics=[mocap]): Tm.append(m.header.stamp.to_sec()); Pm.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]); q=m.pose.pose.orientation; Qm.append([q.x,q.y,q.z,q.w])
bag.close(); Tm=np.array(Tm);Pm=np.array(Pm);Qm=np.array(Qm)
ok=(T>=Tm[0])&(T<=Tm[-1]); T,P,Q=T[ok],P[ok],Q[ok]
if len(T)<50: print("early-aligned drift: not enough overlap"); sys.exit(0)
Pmi=np.stack([np.interp(T,Tm,Pm[:,c]) for c in range(3)],axis=1)
s=T<T[0]+10
yv=np.array([math.atan2(q_to_R(q)[1,0],q_to_R(q)[0,0]) for q in Q[s]]); j=np.clip(np.searchsorted(Tm,T[s]),0,len(Tm)-1)
ym=np.array([math.atan2(q_to_R(q)[1,0],q_to_R(q)[0,0]) for q in Qm[j]])
dy=math.atan2(np.sin(ym-yv).mean(), np.cos(ym-yv).mean()); R=np.array([[math.cos(dy),-math.sin(dy),0],[math.sin(dy),math.cos(dy),0],[0,0,1]])
t=(Pmi[s]-(P[s]@R.T)).mean(0); e=np.linalg.norm(P@R.T+t-Pmi,axis=1); tt=T-T[0]
def m(a,b):
    w=(tt>=a)&(tt<b); return e[w].mean() if w.sum() else float('nan')
print("early-aligned drift [m] (yaw+xyz fitted on the first 10 s after init): 10-20s %.2f | 20-30s %.2f | 30-45s %.2f | 45-60s %.2f | 60-90s %.2f | 90-120s %.2f" % (m(10,20),m(20,30),m(30,45),m(45,60),m(60,90),m(90,120)))
PY
