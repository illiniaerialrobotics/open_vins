#!/usr/bin/env python3
"""
Compare the flight controller's attitude (/msp/attitude: roll, pitch, yaw in rad, Betaflight
convention) with the VIO orientation from an OpenVINS replay (pose_to_file trajectory:
timestamp tx ty tz qx qy qz qw, IMU orientation in the gravity-aligned global frame).

  compare_attitude.py --bag <bag> --traj /tmp/traj_<bag>.txt [--plot]

Sign conventions differ (Betaflight: pitch nose-up positive, yaw clockwise positive), so each axis
gets its sign fitted by correlation and yaw gets a constant offset removed (FC yaw is relative to
boot). Reports RMS and max difference per axis after that. The FC attitude is a Mahony/complementary
estimate itself (1-2 deg class), so this is a consistency check, not ground truth.
"""
import argparse
import math
import os
import sys

import numpy as np

try:
    import rosbag
    from tf.transformations import euler_from_quaternion
except ImportError:
    print("source /opt/ros/noetic/setup.bash first")
    sys.exit(2)


def unwrap(a):
    return np.unwrap(a)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    T, R, P, Y = [], [], [], []
    for _, m, _ in bag.read_messages(topics=["/msp/attitude"]):
        T.append(m.header.stamp.to_sec()); R.append(m.vector.x); P.append(m.vector.y); Y.append(m.vector.z)
    bag.close()
    if len(T) < 10:
        print("no /msp/attitude in bag"); sys.exit(1)
    Ta, Ra, Pa, Ya = np.array(T), np.array(R), np.array(P), unwrap(np.array(Y))

    d = np.loadtxt(args.traj, comments="#")
    Tv = d[:, 0]
    rpy = np.array([euler_from_quaternion(q) for q in d[:, 4:8]])  # roll, pitch, yaw (ZYX), right-handed, z up
    Rv, Pv, Yv = rpy[:, 0], rpy[:, 1], unwrap(rpy[:, 2])

    # common time range, interpolate VIO (30 Hz) onto FC stamps (20-50 Hz)
    t0, t1 = max(Ta[0], Tv[0]), min(Ta[-1], Tv[-1])
    sel = (Ta >= t0) & (Ta <= t1)
    Ta, Ra, Pa, Ya = Ta[sel], Ra[sel], Pa[sel], Ya[sel]
    Rvi, Pvi, Yvi = (np.interp(Ta, Tv, x) for x in (Rv, Pv, Yv))
    print("overlap %.1f s, %d FC samples (%.1f Hz), VIO %d poses" % (t1 - t0, len(Ta), len(Ta) / max(t1 - t0, 1e-3), len(Tv)))

    def fit(name, fc, vio, remove_offset):
        c = np.corrcoef(fc - fc.mean(), vio - vio.mean())[0, 1]
        sign = 1.0 if c >= 0 else -1.0
        v = sign * vio
        off = np.median(fc - v) if remove_offset else 0.0
        res = fc - (v + off)
        res = (res + np.pi) % (2 * np.pi) - np.pi
        print("  %-5s FC = %+d x VIO %s | corr %.3f | RMS diff %.2f deg | mean %+.2f deg | max |diff| %.2f deg"
              % (name, int(sign), ("%+.1f deg" % math.degrees(off)) if remove_offset else "", abs(c),
                 math.degrees(np.sqrt(np.mean(res ** 2))), math.degrees(res.mean()), math.degrees(np.abs(res).max())))
        return sign, off, res

    print("per-axis comparison (sign fitted; yaw offset removed):")
    sr, orr, rr = fit("roll", Ra, Rvi, remove_offset=True)
    sp, op, rp = fit("pitch", Pa, Pvi, remove_offset=True)
    sy, oy, ry = fit("yaw", Ya, Yvi, remove_offset=True)
    print("  (roll/pitch offsets are mount tilt between FC attitude reference and VIO gravity frame; "
          "yaw offset is arbitrary since FC yaw starts at 0 on boot)")
    # yaw drift: linear trend of the yaw residual
    k = np.polyfit(Ta - Ta[0], ry, 1)[0]
    print("  yaw residual trend %+.3f deg/s over %.0f s (positive: FC yaw drifts ahead of VIO)" % (math.degrees(k), t1 - t0))

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        tt = Ta - Ta[0]
        for a, name, fc, vio, sign, off in [(ax[0], "roll", Ra, Rvi, sr, orr), (ax[1], "pitch", Pa, Pvi, sp, op), (ax[2], "yaw", Ya, Yvi, sy, oy)]:
            a.plot(tt, np.degrees(fc), label="FC (msp/attitude)", lw=0.8)
            a.plot(tt, np.degrees(sign * vio + off), label="VIO (sign/offset fitted)", lw=0.8)
            a.set_ylabel(name + " [deg]"); a.grid(alpha=0.3)
        ax[0].legend(loc="upper right"); ax[2].set_xlabel("time [s]")
        out = os.path.splitext(args.bag)[0] + "_attitude.png"
        fig.tight_layout(); fig.savefig(out, dpi=110); print("plot saved:", out)


if __name__ == "__main__":
    main()
