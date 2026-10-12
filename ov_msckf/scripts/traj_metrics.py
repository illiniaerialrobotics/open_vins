#!/usr/bin/env python3
"""Score an OpenVINS trajectory (pose_to_file txt) against the bag it came from.
Touchdown = start of the final stationary stretch judged from the IMU (|gyro| < 0.06 rad/s and
|acc| within 0.6 m/s^2 of g, sustained to the end, >= 2 s), so a drifting estimate cannot hide it.
  traj_metrics.py <bag> <traj.txt>"""
import sys
import numpy as np
import rosbag

bag_path, traj = sys.argv[1], sys.argv[2]
bag = rosbag.Bag(bag_path)
t0 = bag.get_start_time()
Ti, Wi, Ai = [], [], []
for _, m, _ in bag.read_messages(topics=["/msp/imu"]):
    Ti.append(m.header.stamp.to_sec())
    Wi.append((m.angular_velocity.x ** 2 + m.angular_velocity.y ** 2 + m.angular_velocity.z ** 2) ** 0.5)
    Ai.append((m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2 + m.linear_acceleration.z ** 2) ** 0.5)
bag.close()
Ti, Wi, Ai = np.array(Ti), np.array(Wi), np.array(Ai)

try:
    d = np.loadtxt(traj, comments="#")
except Exception:
    d = np.zeros((0, 8))
if d.ndim != 2 or len(d) < 10:
    print("RESULT: no trajectory -> OpenVINS never initialized (needs a still start, or motion for dynamic init)")
    sys.exit(1)
t, P = d[:, 0], d[:, 1:4]
seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
L = seg.sum(); err_final = np.linalg.norm(P[-1] - P[0])
print("initialized at bag t = %.1f s, tracked %.1f s (%d poses)" % (t[0] - t0, t[-1] - t[0], len(t)))
print("path length %.2f m | start->end %.3f m (%.1f %% of path) | max speed %.2f m/s | max |pos| %.2f m | final z %.3f m"
      % (L, err_final, 100 * err_final / max(L, 1e-6), (seg / np.maximum(np.diff(t), 1e-3)).max(), np.linalg.norm(P, axis=1).max(), P[-1, 2] - P[0, 2]))

err = err_final
if len(Ti) > 400:
    # stationary = |gyro| < 0.12 rad/s and |acc| within 0.5 m/s^2 of g; touchdown = earliest time
    # after which >= 90 % of the samples to the end are stationary (tolerates hand bumps/spikes)
    quiet = (Wi < 0.12) & (np.abs(Ai - 9.80665) < 0.5)
    frac_from = np.cumsum(quiet[::-1])[::-1] / np.arange(len(quiet), 0, -1)
    ok = frac_from > 0.9
    j = len(ok)
    while j > 0 and ok[j - 1]:
        j -= 1
    if j < len(ok) and Ti[-1] - Ti[j] >= 2.0 and Ti[j] > t[0]:
        t_td = Ti[j]
        p_td = np.array([np.interp(t_td, t, P[:, c]) for c in range(3)])
        e_td = np.linalg.norm(p_td - P[0])
        # path length up to touchdown
        kk = int(np.searchsorted(t, t_td))
        L_td = seg[: max(kk - 1, 0)].sum()
        print("at touchdown (bag t = %.1f s, %.1f s before end): start->end %.3f m (%.1f %% of %.1f m path); drift while stationary afterwards %.3f m (z %+.3f)"
              % (t_td - t0, Ti[-1] - t_td, e_td, 100 * e_td / max(L_td, 1e-6), L_td, np.linalg.norm(P[-1] - p_td), P[-1, 2] - p_td[2]))
        err, L = e_td, L_td
    else:
        print("no stationary stretch >= 2 s at the end of the bag (IMU), touchdown metric not available")
print("RESULT: %s" % ("GOOD (loop closure < 2 %)" if err / max(L, 1e-6) < 0.02 else "CHECK (loop closure >= 2 % of path, or no loop was closed)"))
