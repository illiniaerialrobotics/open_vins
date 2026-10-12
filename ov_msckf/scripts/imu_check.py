#!/usr/bin/env python3
"""
Bench checks for the Betaflight MSP IMU stream (sensor_msgs/Imu on /msp/imu) before Kalibr.

Modes
  static   Sit still for ~10 s. Reports rate, dt jitter, gaps, stamp monotonicity, |acc|,
           biases/std, quantisation step (informational) and a suggested init_imu_thresh.
  rotate   Guided handedness + gyro-scale test. For each repetition: ~10 s still, then rotate
           the IMU slowly by about +90 deg about ONE horizontal axis, then hold still 3 s.
           The gyro is integrated (R <- R Exp(w dt)) and compared with the gravity direction
           change seen by the accelerometer. The accelerometer angle is the truth, so the ratio
           gives the real gyro scale regardless of what the bridge currently uses, and the
           rotation-axis signs give the handedness verdict.

Usage
  rosrun ov_msckf imu_check.py static  [--topic /msp/imu] [--seconds 10]
  rosrun ov_msckf imu_check.py rotate  [--repeat 3] [--still 10] [--move 8] [--hold 3]
                                       [--bridge-gyro-scale 1.0]

Run 'rotate' three times: level (rotate about X), level (rotate about Y), on its side (rotate
about Z). Rotations about the gravity axis carry no information, so the axis you rotate about
must be horizontal at the time.
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

G0 = 9.80665
# deg/s per LSB candidates: Betaflight integer deg/s, MultiWii-compat (1/4.096), raw 2000 dps/16.384
SCALE_CANDIDATES = {"1.0 (integer deg/s)": 1.0, "0.2441 (1/4.096)": 1.0 / 4.096, "0.0610 (1/16.384)": 1.0 / 16.384}


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def so3_exp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + skew(w)
    k = w / th
    K = skew(k)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def so3_log(R):
    """Rotation vector of R (angle in [0, pi])."""
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if abs(math.pi - th) < 1e-3:  # near pi: axis from R + I
        A = R + np.eye(3)
        k = A[:, np.argmax(np.linalg.norm(A, axis=0))]
        k = k / np.linalg.norm(k)
        return th * k
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    return th * w


def angle_between(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return math.degrees(math.atan2(np.linalg.norm(np.cross(a, b)), float(np.dot(a, b))))


class Collector:
    def __init__(self, imu_topic, att_topic):
        self.lock = threading.Lock()
        self.imu = []  # (stamp, recv, ax, ay, az, gx, gy, gz)
        self.att = []  # (stamp, roll, pitch, yaw)
        rospy.Subscriber(imu_topic, Imu, self.cb_imu, queue_size=2000, tcp_nodelay=True)
        rospy.Subscriber(att_topic, Vector3Stamped, self.cb_att, queue_size=200)

    def cb_imu(self, m):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.imu.append((m.header.stamp.to_sec(), now,
                             m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                             m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

    def cb_att(self, m):
        with self.lock:
            self.att.append((m.header.stamp.to_sec(), m.vector.x, m.vector.y, m.vector.z))

    def take(self, since=None):
        with self.lock:
            d = np.array(self.imu, dtype=float) if self.imu else np.zeros((0, 8))
            a = np.array(self.att, dtype=float) if self.att else np.zeros((0, 4))
        if since is not None and len(d):
            d = d[d[:, 1] >= since]
            a = a[a[:, 0] >= since] if len(a) else a
        return d, a

    def count(self):
        with self.lock:
            return len(self.imu)


def wait_collect(col, seconds, label):
    t0 = time.time()
    n0 = col.count()
    while time.time() - t0 < seconds and not rospy.is_shutdown():
        time.sleep(0.2)
        sys.stdout.write("\r  [%s] %4.1f s  %5d msgs" % (label, time.time() - t0, col.count() - n0))
        sys.stdout.flush()
    print()


def timing_report(d, nominal_hz=None):
    t = d[:, 0]
    dt = np.diff(t)
    n = len(t)
    dur = t[-1] - t[0]
    rate = (n - 1) / dur if dur > 0 else float("nan")
    nominal = 1.0 / nominal_hz if nominal_hz else (np.median(dt) if len(dt) else float("nan"))
    nonmono = int(np.sum(dt <= 0))
    gaps = int(np.sum(dt > 3 * nominal))
    lat = d[:, 1] - d[:, 0]
    print("  messages            : %d over %.2f s  -> %.1f Hz (median dt %.2f ms)" % (n, dur, rate, 1e3 * np.median(dt)))
    print("  dt jitter           : std %.2f ms, p95 %.2f ms, max %.2f ms" % (1e3 * dt.std(), 1e3 * np.percentile(dt, 95), 1e3 * dt.max()))
    print("  gaps > 3 periods    : %d     non-monotonic stamps: %d" % (gaps, nonmono))
    print("  recv - stamp        : mean %.2f ms, p95 %.2f ms (stamp_offset_sec shows up here)" % (1e3 * lat.mean(), 1e3 * np.percentile(lat, 95)))
    ok = nonmono == 0 and gaps == 0
    return ok, rate


def static_mode(col, args):
    print("\n=== STATIC test: keep the IMU perfectly still for %.0f s ===" % args.seconds)
    time.sleep(1.0)
    t_start = rospy.Time.now().to_sec()
    wait_collect(col, args.seconds, "static")
    d, _ = col.take(since=t_start)
    if len(d) < 20:
        print("ERROR: only %d IMU messages received on the topic. Is the bridge running with publish_imu:=true?" % len(d))
        return 2
    ok_t, rate = timing_report(d)
    acc = d[:, 2:5]
    gyr = d[:, 5:8]
    am = acc.mean(0)
    gm = gyr.mean(0)
    acc_std3 = math.sqrt(float(np.sum(acc.var(0))))
    print("  acc mean [m/s^2]    : %+.3f %+.3f %+.3f   |mean| = %.3f (expect %.3f +- 0.15)" % (am[0], am[1], am[2], np.linalg.norm(am), G0))
    print("  acc std  [m/s^2]    : %.4f %.4f %.4f   3-D std = %.4f" % (acc.std(0)[0], acc.std(0)[1], acc.std(0)[2], acc_std3))
    print("  gyro mean (bias)    : %+.5f %+.5f %+.5f rad/s  (%+.3f %+.3f %+.3f deg/s)" % (gm[0], gm[1], gm[2], *np.degrees(gm)))
    print("  gyro std            : %.5f %.5f %.5f rad/s" % tuple(gyr.std(0)))
    up = int(np.argmax(np.abs(am)))
    print("  gravity axis        : %s%s reads %+.2f  (the 'up' body axis should read +%.2f at rest)" % ("+" if am[up] > 0 else "-", "xyz"[up], am[up], G0))
    steps = np.abs(np.diff(gyr, axis=0)).ravel()
    steps = steps[steps > 1e-9]
    if len(steps):
        q = float(np.min(steps))
        print("  smallest gyro step  : %.6f rad/s = %.4f deg/s  (info only; 1 deg/s -> bridge scale 1.0, 0.244 deg/s -> 1/4.096)" % (q, math.degrees(q)))
    print("  acc scale hint      : imu_acc_lsb_per_g = %.0f would make |acc| = %.3f at this pose (bridge uses 2048 unless set)" % (2048.0 * np.linalg.norm(am) / G0, G0))
    thresh = max(0.5, round(3.0 * acc_std3 + 0.05, 2))
    print("  suggested OpenVINS init_imu_thresh >= %.2f  (>= 3x the static 3-D acc std)" % thresh)
    norm_ok = abs(np.linalg.norm(am) - G0) < 0.15
    print("\nRESULT: timing %s, |acc| %s, up-axis sign %s" % ("OK" if ok_t else "FAIL", "OK" if norm_ok else "FAIL (check imu_acc_lsb_per_g)", "OK" if am[up] > 0 else "NEGATIVE (acc frame reflected or IMU upside down)"))
    return 0 if (ok_t and norm_ok) else 1


def integrate(d, bias, scale=1.0):
    """Integrate body rates (already in rad/s as published) with an extra scale factor."""
    R = np.eye(3)
    t = d[:, 0]
    w = (d[:, 5:8] - bias) * scale
    for k in range(len(t) - 1):
        dt = t[k + 1] - t[k]
        if dt <= 0:
            continue
        R = R @ so3_exp(0.5 * (w[k] + w[k + 1]) * dt)
    return R


def rate_integral(d, bias):
    """Vector integral of (w - bias) dt. For a rotation about one fixed axis its norm is the
    UNWRAPPED total angle (a 4x-too-large scale shows 369 deg here, not 9 deg as Log(R) would)."""
    t = d[:, 0]
    w = d[:, 5:8] - bias
    v = np.zeros(3)
    for k in range(len(t) - 1):
        dt = t[k + 1] - t[k]
        if dt > 0:
            v += 0.5 * (w[k] + w[k + 1]) * dt
    return v


def rotate_once(col, args, idx):
    print("\n=== ROTATE test %d/%d ===" % (idx + 1, args.repeat))
    print("Place the IMU so that the axis you will rotate about is HORIZONTAL.")
    print("Suggested order: 1) level, rotate about X (roll)  2) level, rotate about Y (pitch)")
    print("                 3) on its side, rotate about Z (yaw)")
    input("Press ENTER, then keep it still for %.0f s ..." % args.still)
    t0 = rospy.Time.now().to_sec()
    wait_collect(col, args.still, "still")
    d0, a0 = col.take(since=t0)
    if len(d0) < 20:
        print("ERROR: no IMU data.")
        return None
    bias = d0[:, 5:8].mean(0)
    g0 = d0[:, 2:5].mean(0)
    print("  |g0| = %.3f m/s^2, gyro bias = %s rad/s" % (np.linalg.norm(g0), np.array2string(bias, precision=4)))
    print("NOW rotate slowly by about +90 deg about one horizontal axis (take 2-4 s), then HOLD STILL.")
    print("You have %.0f s for the move, then %.0f s hold are recorded automatically." % (args.move, args.hold))
    t1 = rospy.Time.now().to_sec()
    wait_collect(col, args.move + args.hold, "move+hold")
    d1, a1 = col.take(since=t1)
    t_end = d1[-1, 0]
    hold = d1[d1[:, 0] >= t_end - args.hold]
    g1 = hold[:, 2:5].mean(0)
    gyr_hold_std = float(np.linalg.norm(hold[:, 5:8].std(0)))
    if gyr_hold_std > 0.05:
        print("  WARNING: not still during the hold phase (gyro std %.3f rad/s); result may be off." % gyr_hold_std)
    # truth from the accelerometer
    th_acc = angle_between(g0, g1)
    n_acc = np.cross(g1, g0)  # parallel to the rotation axis (component perpendicular to gravity)
    n_acc = n_acc / max(np.linalg.norm(n_acc), 1e-9)
    # gyro as published (bridge scale): unwrapped angle from the rate integral, then rescaled
    # so the gyro angle matches the accelerometer angle (the truth)
    v_pub = rate_integral(d1, bias)
    th_pub = math.degrees(np.linalg.norm(v_pub))
    n_gyro = v_pub / max(np.linalg.norm(v_pub), 1e-9)
    th_log = math.degrees(np.linalg.norm(so3_log(integrate(d1, bias, 1.0))))
    # truth angle: the accelerometer tilt, or a user-known angle (e.g. an exact 360 deg table turn)
    truth = args.known_deg if args.known_deg > 0 else th_acc
    ratio = truth / th_pub if th_pub > 1e-3 else float("nan")
    implied = args.bridge_gyro_scale * ratio
    R_fix = integrate(d1, bias, ratio)
    consist = angle_between(R_fix.T @ g0, g1) if th_acc > 30 else float("nan")
    k = int(np.argmax(np.abs(n_gyro)))
    sigma = int(np.sign(n_acc[k]) * np.sign(n_gyro[k])) if (abs(n_acc[k]) > 0.2 and th_acc > 30) else 0
    best = min(SCALE_CANDIDATES.items(), key=lambda kv: abs(kv[1] - implied))
    print("  accelerometer: rotation angle %.1f deg, axis %s, |g0| %.3f |g1| %.3f m/s^2" % (th_acc, np.array2string(n_acc, precision=2), np.linalg.norm(g0), np.linalg.norm(g1)))
    print("  gyro (bridge scale %.4f): integrated %.1f deg unwrapped (Log(R) %.1f deg), axis %s" % (args.bridge_gyro_scale, th_pub, th_log, np.array2string(n_gyro, precision=2)))
    print("  truth angle used: %.1f deg (%s)" % (truth, "known --known-deg" if args.known_deg > 0 else "accelerometer tilt"))
    print("  implied imu_gyro_dps_per_lsb = %.4f  (nearest candidate: %s, %.1f%% off)" % (implied, best[0], 100 * abs(best[1] - implied) / best[1]))
    print("  dominant axis %s: sign agreement sigma = %+d   consistency angle(R^T g0, g1) = %.1f deg" % ("xyz"[k], sigma, consist))
    signed = np.degrees(v_pub) * ratio
    print("  signed integrated angle per axis at the implied scale: x %+.1f  y %+.1f  z %+.1f deg" % tuple(signed))
    expect_ok = None
    if args.expect:
        sgn = -1.0 if args.expect[0] == "-" else 1.0
        ax = "xyz".index(args.expect[-1])
        expect_ok = bool(np.sign(signed[ax]) == sgn and abs(signed[ax]) > 0.5 * abs(signed).max())
        print("  expected positive rotation about %s: gyro %s says %+.1f deg -> %s" % (args.expect, "xyz"[ax], signed[ax],
              "OK (sign consistent)" if expect_ok else "FLIPPED or wrong axis"))
    if len(a0) and len(a1):
        rp0 = a0[:, 1:3].mean(0)
        rp1 = a1[-max(1, len(a1) // 5):, 1:3].mean(0)
        print("  msp/attitude delta: roll %+.1f deg, pitch %+.1f deg" % tuple(np.degrees(rp1 - rp0)))
    if th_acc < 30 and args.known_deg <= 0:
        print("  WARNING: the accelerometer saw only %.1f deg of tilt. Rotate about a HORIZONTAL axis by ~90 deg,"
              " or pass --known-deg for an exact table turn." % th_acc)
    return dict(axis="xyz"[k], sigma=sigma, th_acc=truth, th_pub=th_pub, implied=implied, consist=consist, expect_ok=expect_ok)


def rotate_mode(col, args):
    results = []
    for i in range(args.repeat):
        r = rotate_once(col, args, i)
        if r:
            results.append(r)
    if not results:
        return 2
    print("\n=== SUMMARY ===")
    print("  axis  sigma  acc_deg  gyro_deg(pub)  implied_scale  consistency_deg")
    for r in results:
        print("   %s     %+d    %6.1f     %6.1f        %.4f         %5.1f" % (r["axis"], r["sigma"], r["th_acc"], r["th_pub"], r["implied"], r["consist"]))
    impl = np.array([r["implied"] for r in results if r["th_acc"] > 30 and np.isfinite(r["implied"])])
    if len(impl):
        med = float(np.median(impl))
        best = min(SCALE_CANDIDATES.items(), key=lambda kv: abs(kv[1] - med))
        print("\n  gyro scale: median implied imu_gyro_dps_per_lsb = %.4f  -> use  imu_gyro_dps_per_lsb:=%.6f  (%s)" % (med, best[1], best[0]))
        if abs(best[1] - med) / best[1] > 0.05:
            print("  WARNING: >5%% from every known candidate; repeat with a slower, cleaner 90 deg rotation.")
    sig = {r["axis"]: r["sigma"] for r in results if r["sigma"] != 0}
    untested = [r["axis"] for r in results if r["sigma"] == 0]
    if untested:
        print("  no tilt-based handedness information for axis %s in this run (rotation about the gravity axis);"
              " use --expect +z with a known counter-clockwise turn instead." % sorted(set(untested)))
    exp = [r for r in results if r.get("expect_ok") is not None]
    for r in exp:
        print("  known-direction check (--expect %s): %s" % (args.expect, "OK" if r["expect_ok"] else "FAILED -> that gyro axis is sign-flipped"))
    if not sig and not exp:
        return 0
    neg = [a for a, s in sig.items() if s < 0]
    bad_consist = [r for r in results if np.isfinite(r["consist"]) and r["consist"] > 5.0 and r["th_acc"] > 30]
    if sig and not neg and not bad_consist:
        print("  handedness: tilt-tested axes %s consistent -> right-handed, gyro and acc share one frame." % sorted(sig.keys()))
    elif len(neg) == 1:
        print("  handedness: gyro axis %s appears sign-flipped relative to the accelerometer." % neg[0])
    elif len(neg) == 2:
        third = ({"x", "y", "z"} - set(neg)).pop()
        print("  handedness: accelerometer axis %s appears sign-flipped (two gyro axes disagree)." % third)
    elif len(neg) == 3:
        print("  handedness: all three disagree -> acc and gyro frames differ by -I; check the static up-axis sign.")
    if bad_consist:
        print("  consistency > 5 deg on axes %s -> reflection or bad scale; fix and repeat." % [r["axis"] for r in bad_consist])
    tested = set(sig.keys())
    if len(tested) < 3:
        print("  NOTE: only axes %s were tested; run again on the missing axis for a complete verdict." % sorted(tested))
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["static", "rotate"])
    p.add_argument("--topic", default="/msp/imu")
    p.add_argument("--attitude-topic", default="/msp/attitude")
    p.add_argument("--seconds", type=float, default=10.0, help="static: duration")
    p.add_argument("--repeat", type=int, default=3, help="rotate: number of rotations")
    p.add_argument("--still", type=float, default=10.0)
    p.add_argument("--move", type=float, default=10.0)
    p.add_argument("--hold", type=float, default=3.0)
    p.add_argument("--bridge-gyro-scale", type=float, default=1.0,
                   help="imu_gyro_dps_per_lsb the bridge is currently running with (default 1.0)")
    p.add_argument("--expect", default="", choices=["", "+x", "-x", "+y", "-y", "+z", "-z"],
                   help="rotate: axis you KNOW you turned positively about, e.g. +z for a counter-clockwise "
                        "turn seen from above (z up). Checks that gyro axis sign without needing a tilt.")
    p.add_argument("--known-deg", type=float, default=0.0,
                   help="rotate: the exact angle you will turn (e.g. 360 for a full table turn); "
                        "overrides the accelerometer tilt as the truth angle")
    args = p.parse_args(rospy.myargv()[1:])
    rospy.init_node("imu_check", anonymous=True, disable_signals=True)
    col = Collector(args.topic, args.attitude_topic)
    print("Listening on %s (attitude cross-check on %s if present)" % (args.topic, args.attitude_topic))
    time.sleep(1.5)
    if col.count() == 0:
        print("WARNING: no IMU messages yet. Bridge running? publish_imu:=true? Topic name?")
    rc = static_mode(col, args) if args.mode == "static" else rotate_mode(col, args)
    rospy.signal_shutdown("done")
    sys.exit(rc)


if __name__ == "__main__":
    main()
