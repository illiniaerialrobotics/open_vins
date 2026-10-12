#!/usr/bin/env python3
"""
Live monitor for the MSP IMU link: prints every gap in /msp/imu longer than --gap-ms with the
wall-clock time, plus a one-line rate summary every --period seconds. Use it while wiggling the
FC <-> USB-serial wires or during a flight-like load test to catch dropouts.

  rosrun ov_msckf imu_link_monitor.py [--topic /msp/imu] [--gap-ms 15] [--seconds 60]
"""
import argparse
import sys
import time

import rospy
from sensor_msgs.msg import Imu

p = argparse.ArgumentParser()
p.add_argument("--topic", default="/msp/imu")
p.add_argument("--gap-ms", type=float, default=15.0)
p.add_argument("--period", type=float, default=5.0)
p.add_argument("--seconds", type=float, default=60.0)
args = p.parse_args(rospy.myargv()[1:])

rospy.init_node("imu_link_monitor", anonymous=True, disable_signals=True)
state = {"last": None, "n": 0, "gaps": 0, "worst": 0.0, "total_lost": 0.0, "nonmono": 0}


def cb(m):
    t = m.header.stamp.to_sec()
    if state["last"] is not None:
        dt = t - state["last"]
        if dt <= 0:
            state["nonmono"] += 1
        elif dt > args.gap_ms / 1e3:
            state["gaps"] += 1
            state["worst"] = max(state["worst"], dt)
            state["total_lost"] += dt
            print("  %s  GAP %.1f ms" % (time.strftime("%H:%M:%S"), 1e3 * dt), flush=True)
    state["last"] = t
    state["n"] += 1


rospy.Subscriber(args.topic, Imu, cb, queue_size=2000, tcp_nodelay=True)
print("monitoring %s for %.0f s (gap threshold %.0f ms) ..." % (args.topic, args.seconds, args.gap_ms), flush=True)
t0 = time.time()
n_prev = 0
while time.time() - t0 < args.seconds and not rospy.is_shutdown():
    time.sleep(args.period)
    n = state["n"]
    print("  %s  %.1f Hz   gaps so far %d (worst %.0f ms)" % (time.strftime("%H:%M:%S"), (n - n_prev) / args.period, state["gaps"], 1e3 * state["worst"]), flush=True)
    n_prev = n
print("RESULT: %d msgs, %d gaps > %.0f ms, worst %.0f ms, %.2f s lost, non-monotonic %d -> %s"
      % (state["n"], state["gaps"], args.gap_ms, 1e3 * state["worst"], state["total_lost"], state["nonmono"],
         "CLEAN" if state["gaps"] == 0 and state["nonmono"] == 0 else "DROPOUTS SEEN"))
rospy.signal_shutdown("done")
sys.exit(0 if state["gaps"] == 0 else 1)
