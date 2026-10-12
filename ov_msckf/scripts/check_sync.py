#!/usr/bin/env python3
"""
Timestamp / synchronisation QA for a camera + IMU rosbag before sending it to Kalibr.

  check_sync.py <bag> [--image-topic /arducam/image_raw] [--imu-topic /msp/imu] [--no-flow] [--plot]

Reports per topic: rate, dt jitter (std/p95/max), gaps, non-monotonic stamps, and
(bag receive time - header stamp) latency. With image flow enabled it also estimates the
camera-IMU time offset coarsely by cross-correlating |gyro| with the mean KLT optical-flow
speed. Convention matches Kalibr: t_imu = t_cam + timeshift_cam_imu.  Expect +-5..10 ms accuracy;
it is a sanity check for the Kalibr value, not a replacement.
"""
import argparse
import math
import os
import sys

import numpy as np

try:
    import rosbag
except ImportError:
    print("rosbag python module not found: source /opt/ros/noetic/setup.bash first")
    sys.exit(2)

PASS = True


def report(name, stamps, recv, nominal_hz=None, gap_factor=3.0):
    global PASS
    t = np.asarray(stamps)
    r = np.asarray(recv)
    if len(t) < 3:
        print("  %-22s: only %d messages" % (name, len(t)))
        PASS = False
        return None
    dt = np.diff(t)
    dur = t[-1] - t[0]
    rate = (len(t) - 1) / dur
    nominal = 1.0 / nominal_hz if nominal_hz else float(np.median(dt))
    nonmono = int(np.sum(dt <= 0))
    gaps = int(np.sum(dt > gap_factor * nominal))
    lat = r - t
    print("  %-22s: %6d msgs, %.1f Hz over %.1f s" % (name, len(t), rate, dur))
    print("    dt: median %.2f ms, std %.2f, p95 %.2f, max %.2f ms" % (1e3 * np.median(dt), 1e3 * dt.std(), 1e3 * np.percentile(dt, 95), 1e3 * dt.max()))
    print("    gaps > %.1f periods: %d   non-monotonic: %d   recv-stamp: mean %.1f ms p95 %.1f ms" % (gap_factor, gaps, nonmono, 1e3 * lat.mean(), 1e3 * np.percentile(lat, 95)))
    if nonmono or gaps:
        PASS = False
    big = np.where(dt > gap_factor * nominal)[0]
    for i in big[:5]:
        print("    gap of %.1f ms at t=%.3f" % (1e3 * dt[i], t[i] - t[0]))
    return dt


def flow_speed(bag, topic, max_side=320):
    """Mean KLT displacement (px/s) between consecutive frames, at the frame's stamp midpoint."""
    import cv2
    prev = None
    prev_t = None
    out_t, out_v = [], []
    for _, m, _ in bag.read_messages(topics=[topic]):
        if m.encoding not in ("mono8", "8UC1"):
            # fall back: take the first channel of 3-channel images
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
            img = np.ascontiguousarray(arr[:, : m.width * 3].reshape(m.height, m.width, 3)[:, :, 1])
        else:
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, : m.width]
        s = max(img.shape) / float(max_side)
        if s > 1.0:
            img = cv2.resize(img, (int(img.shape[1] / s), int(img.shape[0] / s)), interpolation=cv2.INTER_AREA)
        else:
            s = 1.0
        t = m.header.stamp.to_sec()
        if prev is not None and t > prev_t:
            p0 = cv2.goodFeaturesToTrack(prev, 150, 0.01, 8)
            if p0 is not None and len(p0) >= 10:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, img, p0, None, winSize=(15, 15), maxLevel=3)
                ok = st.ravel() == 1
                if ok.sum() >= 10:
                    d = np.linalg.norm((p1 - p0)[ok].reshape(-1, 2), axis=1)
                    v = float(np.median(d)) * s / (t - prev_t)
                    out_t.append(0.5 * (t + prev_t))
                    out_v.append(v)
        prev, prev_t = img, t
    return np.array(out_t), np.array(out_v)


def xcorr_offset(t_cam, v_cam, t_imu, w_imu, max_lag=0.15, fs=200.0):
    t0 = max(t_cam[0], t_imu[0])
    t1 = min(t_cam[-1], t_imu[-1])
    if t1 - t0 < 5.0:
        return None
    grid = np.arange(t0, t1, 1.0 / fs)
    a = np.interp(grid, t_cam, v_cam)
    b = np.interp(grid, t_imu, w_imu)
    a = (a - a.mean()) / (a.std() + 1e-9)
    b = (b - b.mean()) / (b.std() + 1e-9)
    nlag = int(max_lag * fs)
    lags = np.arange(-nlag, nlag + 1)
    c = np.array([np.mean(a[max(0, -l): len(a) - max(0, l)] * b[max(0, l): len(b) - max(0, -l)]) for l in lags])
    k = int(np.argmax(c))
    # parabolic refinement
    if 0 < k < len(c) - 1:
        den = c[k - 1] - 2 * c[k] + c[k + 1]
        frac = 0.5 * (c[k - 1] - c[k + 1]) / den if abs(den) > 1e-12 else 0.0
    else:
        frac = 0.0
    lag = (lags[k] + frac) / fs
    return lag, float(c[k]), lags / fs, c


def main():
    global PASS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--image-topic", default="/arducam/image_raw")
    ap.add_argument("--imu-topic", default="/msp/imu")
    ap.add_argument("--imu-hz", type=float, default=None, help="nominal IMU rate for the gap test (default: median)")
    ap.add_argument("--cam-hz", type=float, default=None)
    ap.add_argument("--no-flow", action="store_true", help="skip the optical-flow offset estimate")
    ap.add_argument("--plot", action="store_true", help="save <bag>_sync.png next to the bag")
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    info = bag.get_type_and_topic_info()[1]
    print("Bag: %s" % args.bag)
    for tp, ti in sorted(info.items()):
        print("  topic %-32s %-40s %7d msgs" % (tp, ti.msg_type, ti.message_count))
    imu_t, imu_r, imu_w = [], [], []
    if args.imu_topic in info:
        for _, m, rt in bag.read_messages(topics=[args.imu_topic]):
            imu_t.append(m.header.stamp.to_sec())
            imu_r.append(rt.to_sec())
            imu_w.append(math.sqrt(m.angular_velocity.x ** 2 + m.angular_velocity.y ** 2 + m.angular_velocity.z ** 2))
    cam_t, cam_r = [], []
    if args.image_topic in info:
        for _, m, rt in bag.read_messages(topics=[args.image_topic]):
            cam_t.append(m.header.stamp.to_sec())
            cam_r.append(rt.to_sec())
    print("\nTiming:")
    if imu_t:
        report("IMU " + args.imu_topic, imu_t, imu_r, args.imu_hz, gap_factor=3.0)
    else:
        print("  IMU topic %s not in bag" % args.imu_topic)
    if cam_t:
        report("CAM " + args.image_topic, cam_t, cam_r, args.cam_hz, gap_factor=1.5)
    else:
        print("  image topic %s not in bag" % args.image_topic)
    if imu_t and cam_t:
        ov0, ov1 = max(imu_t[0], cam_t[0]), min(imu_t[-1], cam_t[-1])
        print("  overlap: %.1f s (cam starts %+.2f s after imu)" % (ov1 - ov0, cam_t[0] - imu_t[0]))

    lagres = None
    if imu_t and cam_t and not args.no_flow:
        print("\nOptical-flow vs |gyro| cross-correlation (coarse time offset):")
        t_cam, v_cam = flow_speed(bag, args.image_topic)
        if len(t_cam) > 50:
            lagres = xcorr_offset(t_cam, v_cam, np.array(imu_t), np.array(imu_w))
            if lagres:
                lag, peak, lags, c = lagres
                print("  best lag = %+.1f ms  (peak corr %.2f)  ->  timeshift_cam_imu ~ %+.4f s  (t_imu = t_cam + shift)" % (1e3 * lag, peak, lag))
                if peak < 0.3:
                    print("  WARNING: weak correlation; needs rotation-rich motion with texture in view. Treat as unreliable.")
                elif abs(lag) > 0.05:
                    print("  NOTE: |offset| > 50 ms; pass --timeoffset-padding >= %.2f to kalibr_calibrate_imu_camera" % (abs(lag) + 0.05))
            else:
                print("  not enough overlap for a correlation")
        else:
            print("  too few flow samples (%d)" % len(t_cam))
    bag.close()

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3 if lagres else 2, figsize=(14, 4))
        if imu_t:
            ax[0].hist(1e3 * np.diff(imu_t), bins=100)
            ax[0].set_title("IMU dt [ms]")
        if cam_t:
            ax[1].hist(1e3 * np.diff(cam_t), bins=100)
            ax[1].set_title("camera dt [ms]")
        if lagres:
            ax[2].plot(1e3 * lagres[2], lagres[3])
            ax[2].axvline(1e3 * lagres[0], color="r")
            ax[2].set_title("xcorr(flow, |gyro|) vs lag [ms]")
        out = os.path.splitext(args.bag)[0] + "_sync.png"
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        print("plot saved: %s" % out)

    print("\nRESULT: %s" % ("PASS" if PASS else "FAIL (gaps or non-monotonic stamps; fix before calibrating)"))
    sys.exit(0 if PASS else 1)


if __name__ == "__main__":
    main()
