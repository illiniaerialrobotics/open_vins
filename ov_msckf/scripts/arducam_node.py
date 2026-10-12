#!/usr/bin/env python3
"""
Arducam (UVC/MJPG) camera publisher for VIO: mono8 sensor_msgs/Image with V4L2 buffer timestamps.

Design (see config/arducam_msp/README.md):
  * cv2.VideoCapture(V4L2), MJPG at capture_width x capture_height @ fps.
  * grab() dequeues the buffer; CAP_PROP_POS_MSEC then holds the kernel buffer timestamp
    (CLOCK_MONOTONIC for uvcvideo). We convert it to wall/ROS time with a per-frame
    time.time()-time.monotonic() offset, so decode and USB jitter never reach header.stamp.
  * retrieve() with CAP_PROP_CONVERT_RGB=0 returns the raw JPEG; cv2.imdecode with
    IMREAD_REDUCED_GRAYSCALE_<decode_reduce> yields the downscaled grey image directly.
  * Exposure/gain are set with v4l2-ctl after streaming starts.
  * No cv_bridge (pip OpenCV 4.10 vs system OpenCV 4.2 clash); the Image message is filled by hand.
"""
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

REDUCE_FLAGS = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
                4: cv2.IMREAD_REDUCED_GRAYSCALE_4, 8: cv2.IMREAD_REDUCED_GRAYSCALE_8}


def v4l2_set(dev, controls):
    """controls: list of (name, value). Applied one by one so a missing control does not block the rest."""
    for name, val in controls:
        if val is None:
            continue
        try:
            r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", "%s=%s" % (name, val)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=3.0)
            if r.returncode != 0:
                rospy.logwarn("v4l2-ctl %s=%s failed: %s", name, val, r.stdout.decode(errors="ignore").strip())
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("v4l2-ctl %s=%s error: %s", name, val, e)


def v4l2_get(dev, names):
    try:
        r = subprocess.run(["v4l2-ctl", "-d", dev, "-C", ",".join(names)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=3.0)
        return r.stdout.decode(errors="ignore").strip().replace("\n", ", ")
    except Exception as e:  # noqa: BLE001
        return "v4l2-ctl error: %s" % e


class ArducamNode:
    def __init__(self):
        self.dev = rospy.get_param("~device", "/dev/video0")
        self.cw = int(rospy.get_param("~capture_width", 1280))
        self.ch = int(rospy.get_param("~capture_height", 800))
        self.fps = float(rospy.get_param("~fps", 30))
        self.reduce = int(rospy.get_param("~decode_reduce", 2))
        self.frame_id = rospy.get_param("~frame_id", "arducam")
        self.auto_exposure = bool(rospy.get_param("~auto_exposure", False))
        self.exposure = rospy.get_param("~exposure_absolute", 40)   # 100 us units -> 4 ms
        self.gain = rospy.get_param("~gain", 30)
        self.discard_first = int(rospy.get_param("~discard_first", 10))
        self.stats_period = float(rospy.get_param("~stats_period", 5.0))
        self.max_stamp_skew = float(rospy.get_param("~max_stamp_skew", 0.5))  # s, POS_MSEC vs monotonic
        if self.reduce not in REDUCE_FLAGS:
            rospy.logwarn("decode_reduce must be 1/2/4/8, got %d -> 2", self.reduce)
            self.reduce = 2
        self.pub = rospy.Publisher("~image_raw", Image, queue_size=2, tcp_nodelay=True)
        # optional colour copy (bgr8, same size) for recordings/videos only; VIO uses mono8
        self.publish_color = bool(rospy.get_param("~publish_color", False))
        self.pub_color = rospy.Publisher("~image_color", Image, queue_size=2, tcp_nodelay=True) if self.publish_color else None
        self.last_raw = None
        self.cap = None
        self.raw_mode = False      # CONVERT_RGB=0 honoured -> retrieve() gives raw JPEG
        self.stamp_mode = "probe"  # probe -> v4l2-monotonic | fallback-monotonic
        self.last_stamp = 0.0
        self.probe = []            # (pos_sec, mono_sec) during probing

    # -- device ------------------------------------------------------------------------------
    def open(self):
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError("cannot open %s" % self.dev)
        # Order matters for OpenCV's V4L2 backend: format, size, fps, then buffer count.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.ch)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.raw_mode = bool(cap.set(cv2.CAP_PROP_CONVERT_RGB, 0))
        aw, ah, afps = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_s = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
        rospy.loginfo("%s opened: %s %dx%d @ %.1f fps (requested %dx%d @ %.1f), raw_jpeg_mode=%s",
                      self.dev, fourcc_s, aw, ah, afps, self.cw, self.ch, self.fps, self.raw_mode)
        if int(aw) != self.cw or int(ah) != self.ch:
            rospy.logwarn("camera did not accept %dx%d; running at %dx%d", self.cw, self.ch, aw, ah)
            self.cw, self.ch = int(aw), int(ah)
        self.cap = cap
        # Start streaming (first grab), then apply controls: some UVC cameras reset them on STREAMON.
        if not cap.grab():
            raise RuntimeError("first grab failed on %s" % self.dev)
        self.apply_controls()

    def apply_controls(self):
        if self.auto_exposure:
            v4l2_set(self.dev, [("exposure_auto", 3), ("exposure_auto_priority", 0), ("gain", self.gain)])
        else:
            v4l2_set(self.dev, [("exposure_auto", 1), ("exposure_auto_priority", 0),
                                ("exposure_absolute", self.exposure), ("gain", self.gain)])
        rospy.loginfo("controls now: %s", v4l2_get(self.dev, ["exposure_auto", "exposure_absolute", "gain", "exposure_auto_priority"]))

    # -- timestamps ----------------------------------------------------------------------------
    def stamp_for(self, pos_msec, t_mono, t_wall):
        """Return (stamp_wall_sec). Chooses/validates the V4L2 buffer timestamp mode."""
        pos = pos_msec / 1000.0
        use_pos = False
        if self.stamp_mode == "probe":
            self.probe.append((pos, t_mono))
            if len(self.probe) >= 60:
                skew = np.array([abs(p - m) for p, m in self.probe])
                dts = np.diff([p for p, _ in self.probe])
                ok = (skew.max() < self.max_stamp_skew) and (dts.min() > 0) and \
                     (np.median(np.abs(dts - 1.0 / self.fps)) < 0.005)
                self.stamp_mode = "v4l2-monotonic" if ok else "fallback-monotonic"
                rospy.loginfo("stamp mode: %s (POS_MSEC-vs-monotonic skew max %.1f ms, dt median %.2f ms)",
                              self.stamp_mode, 1e3 * skew.max(), 1e3 * np.median(dts))
            use_pos = abs(pos - t_mono) < self.max_stamp_skew and pos > 0
        elif self.stamp_mode == "v4l2-monotonic":
            use_pos = True
            if abs(pos - t_mono) > self.max_stamp_skew or pos <= 0:
                self.stamp_mode = "fallback-monotonic"
                rospy.logwarn("V4L2 timestamp skew %.3f s -> switching to fallback-monotonic stamps", pos - t_mono)
                use_pos = False
        mono_stamp = pos if use_pos else t_mono
        stamp = mono_stamp + (t_wall - t_mono)
        if stamp <= self.last_stamp:
            # never publish non-monotonic stamps (OpenVINS would drop/complain)
            if self.stamp_mode == "v4l2-monotonic":
                self.stamp_mode = "fallback-monotonic"
                rospy.logwarn("V4L2 timestamp went backwards -> fallback-monotonic stamps")
            stamp = max(t_wall, self.last_stamp + 1e-4)
        self.last_stamp = stamp
        return stamp

    # -- main loop -----------------------------------------------------------------------------
    def decode(self):
        ok, buf = self.cap.retrieve()
        if not ok or buf is None:
            return None
        if self.raw_mode:
            self.last_raw = buf.reshape(-1) if self.publish_color else None
            img = cv2.imdecode(buf.reshape(-1), REDUCE_FLAGS[self.reduce])
            if img is None:
                return None
        else:
            if buf.ndim == 3:
                img = cv2.cvtColor(buf, cv2.COLOR_BGR2GRAY)
            else:
                img = buf
            if self.reduce > 1:
                img = cv2.resize(img, (img.shape[1] // self.reduce, img.shape[0] // self.reduce), interpolation=cv2.INTER_AREA)
        return img

    def run(self):
        self.open()
        msg = Image()
        msg.header.frame_id = self.frame_id
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        n_pub = n_drop = n_fail = 0
        t_stats = time.monotonic()
        last_t = None
        max_gap = 0.0
        dec_ms = []
        discarded = 0
        while not rospy.is_shutdown():
            if not self.cap.grab():
                n_fail += 1
                if n_fail > 30:
                    rospy.logerr("%s: grab failed 30x, exiting for respawn", self.dev)
                    return 3
                time.sleep(0.01)
                continue
            n_fail = 0
            pos_msec = self.cap.get(cv2.CAP_PROP_POS_MSEC)
            t_mono = time.monotonic()
            t_wall = time.time()
            if discarded < self.discard_first:
                discarded += 1
                self.cap.retrieve()
                continue
            stamp = self.stamp_for(pos_msec, t_mono, t_wall)
            t0 = time.monotonic()
            img = self.decode()
            dec_ms.append(1e3 * (time.monotonic() - t0))
            if img is None:
                n_drop += 1
                continue
            if not img.flags["C_CONTIGUOUS"]:
                img = np.ascontiguousarray(img)
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.header.seq += 1
            msg.height, msg.width = img.shape[0], img.shape[1]
            msg.step = msg.width
            msg.data = img.tobytes()
            self.pub.publish(msg)
            n_pub += 1
            if self.pub_color is not None and self.pub_color.get_num_connections() > 0 and self.last_raw is not None:
                col = cv2.imdecode(self.last_raw, {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                                                   4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}[self.reduce])
                if col is not None:
                    cmsg = Image()
                    cmsg.header = msg.header
                    cmsg.encoding = "bgr8"
                    cmsg.height, cmsg.width = col.shape[0], col.shape[1]
                    cmsg.step = cmsg.width * 3
                    cmsg.data = np.ascontiguousarray(col).tobytes()
                    self.pub_color.publish(cmsg)
            if last_t is not None:
                gap = stamp - last_t
                max_gap = max(max_gap, gap)
                if gap > 1.5 / self.fps:
                    n_drop += 1
            last_t = stamp
            now = time.monotonic()
            if now - t_stats >= self.stats_period:
                dt = now - t_stats
                sat = 100.0 * float(np.mean(img >= 250))
                rospy.loginfo("%s: %.1f fps published, %dx%d, max gap %.1f ms, drops %d, decode %.1f ms (p95 %.1f), stamps=%s",
                              self.frame_id, n_pub / dt, msg.width, msg.height, 1e3 * max_gap, n_drop,
                              float(np.mean(dec_ms)) if dec_ms else 0.0,
                              float(np.percentile(dec_ms, 95)) if dec_ms else 0.0, self.stamp_mode)
                rospy.loginfo("%s: exposure check: mean brightness %.0f/255, %.1f%% pixels saturated "
                              "(aim ~90-130 mean, <2%% saturated; exposure_absolute=%s gain=%s)",
                              self.frame_id, float(img.mean()), sat, self.exposure, self.gain)
                n_pub = n_drop = 0
                max_gap = 0.0
                dec_ms = []
                t_stats = now
        return 0

    def close(self):
        if self.cap is not None:
            self.cap.release()


def main():
    rospy.init_node("arducam")
    node = ArducamNode()
    rc = 1
    try:
        rc = node.run()
    except Exception as e:  # noqa: BLE001
        rospy.logerr("arducam_node: %s", e)
        rc = 2
    finally:
        node.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
