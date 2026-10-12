#!/usr/bin/env python3
"""
Render a VIO replay video: main view = OpenVINS tracker image (/ov_msckf/trackhist, features
drawn) or the raw camera image, inset bottom-right = top-down trajectory that grows with time.

  vio_video.py --viz-bag <replay_viz.bag> --bag <original.bag> --out <file.mp4>
               [--traj <pose_to_file.txt>] [--scale 2] [--inset 0.42] [--fps 0 (=source)]

Poses come from /ov_msckf/odomimu in the viz bag (IMU rate), or from --traj if that topic is
missing. Images come from /ov_msckf/trackhist if present, else /arducam/image_raw of --bag.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

try:
    import rosbag
except ImportError:
    print("source /opt/ros/noetic/setup.bash first (rosbag python module missing)")
    sys.exit(2)


def load_poses_from_bag(path):
    if not path or not os.path.exists(path):
        return None
    bag = rosbag.Bag(path)
    topics = bag.get_type_and_topic_info()[1]
    if "/ov_msckf/odomimu" not in topics:
        bag.close()
        return None
    T, R, P, V, Q = [], [], [], [], []
    for _, m, rt in bag.read_messages(topics=["/ov_msckf/odomimu"]):
        T.append(m.header.stamp.to_sec())
        R.append(rt.to_sec())
        p = m.pose.pose.position
        P.append([p.x, p.y, p.z])
        v = m.twist.twist.linear
        V.append([v.x, v.y, v.z])          # OpenVINS odomimu: linear velocity in the body (IMU) frame
        q = m.pose.pose.orientation
        Q.append([q.x, q.y, q.z, q.w])
    bag.close()
    if not T:
        return None
    return np.array(T), np.array(P), np.array(V), np.array(R), np.array(Q)


def load_poses_from_txt(path):
    if not path or not os.path.exists(path) or os.path.getsize(path) < 50:
        return None
    d = np.loadtxt(path, comments="#")
    if d.ndim != 2 or len(d) < 2:
        return None
    T, P = d[:, 0], d[:, 1:4]
    Q = d[:, 4:8] if d.shape[1] >= 8 else None
    # body-frame velocity from smoothed finite differences (0.2 s window), rotated by R^T
    vw = np.gradient(P, axis=0) / np.maximum(np.gradient(T), 1e-3)[:, None]
    w = max(1, int(0.2 / max(np.median(np.diff(T)), 1e-3)))
    k = np.ones(w) / w
    vw = np.stack([np.convolve(vw[:, c], k, mode="same") for c in range(3)], axis=1)
    V = vw
    if Q is not None:
        V = np.array([quat_to_R(q).T @ vw[i] for i, q in enumerate(Q)])
    return T, P, V, None, Q  # no receive times in a text trajectory


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_to_rpy(Q):
    """xyzw quaternions -> roll, pitch, yaw (ZYX, right-handed, radians), vectorised."""
    x, y, z, w = Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def load_attitude(raw_bag):
    """FC attitude from /msp/attitude: (header stamps, receive times, N x 3 roll/pitch/yaw rad)."""
    bag = rosbag.Bag(raw_bag)
    if "/msp/attitude" not in bag.get_type_and_topic_info()[1]:
        bag.close()
        return None
    T, R, A = [], [], []
    for _, m, rt in bag.read_messages(topics=["/msp/attitude"]):
        T.append(m.header.stamp.to_sec())
        R.append(rt.to_sec())
        A.append([m.vector.x, m.vector.y, m.vector.z])
    bag.close()
    if len(T) < 10:
        return None
    return np.array(T), np.array(R), np.array(A)


def fit_vio_to_fc(Tv, rpy_v, Ta, A):
    """Per-axis sign and constant offset so that FC ~= sign * VIO + offset (yaw unwrapped)."""
    t0, t1 = max(Tv[0], Ta[0]), min(Tv[-1], Ta[-1])
    sel = (Ta >= t0) & (Ta <= t1)
    if sel.sum() < 10:
        return np.ones(3), np.zeros(3)
    signs, offs = np.ones(3), np.zeros(3)
    for k in range(3):
        fc = A[sel, k] if k < 2 else np.unwrap(A[sel, k])
        vio = np.interp(Ta[sel], Tv, rpy_v[:, k] if k < 2 else np.unwrap(rpy_v[:, k]))
        c = np.corrcoef(fc - fc.mean(), vio - vio.mean())[0, 1]
        signs[k] = 1.0 if (np.isnan(c) or c >= 0) else -1.0
        offs[k] = float(np.median(fc - signs[k] * vio))
    return signs, offs


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def image_iter(viz_bag, raw_bag, view="camera"):
    """Yield (header stamp, receive time, bgr_image, kind).
    view: camera (colour if recorded, else mono; default), tracker (OpenVINS features drawn), color, mono."""
    if view == "tracker" and viz_bag and os.path.exists(viz_bag):
        bag = rosbag.Bag(viz_bag)
        if "/ov_msckf/trackhist" in bag.get_type_and_topic_info()[1]:
            for _, m, rt in bag.read_messages(topics=["/ov_msckf/trackhist"]):
                arr = np.frombuffer(m.data, np.uint8)
                if m.encoding in ("bgr8", "rgb8"):
                    img = arr.reshape(m.height, m.step // 3, 3)[:, : m.width]
                    if m.encoding == "rgb8":
                        img = img[:, :, ::-1]
                else:
                    img = cv2.cvtColor(arr.reshape(m.height, m.step)[:, : m.width], cv2.COLOR_GRAY2BGR)
                yield m.header.stamp.to_sec(), rt.to_sec(), np.ascontiguousarray(img), "tracker view"
            bag.close()
            return
        bag.close()
    bag = rosbag.Bag(raw_bag)
    topics = bag.get_type_and_topic_info()[1]
    if view in ("camera", "color") and "/arducam/image_color" in topics:
        for _, m, rt in bag.read_messages(topics=["/arducam/image_color"]):
            arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3, 3)[:, : m.width]
            yield m.header.stamp.to_sec(), rt.to_sec(), np.ascontiguousarray(arr), "camera (colour)"
        bag.close()
        return
    for _, m, rt in bag.read_messages(topics=["/arducam/image_raw"]):
        arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width]
        yield m.header.stamp.to_sec(), rt.to_sec(), cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR), "camera"
    bag.close()


class TrajInset:
    """Top-down (x right, y up) incremental trajectory canvas."""

    def __init__(self, P, size):
        self.size = size
        self.canvas = np.full((size, size, 3), 28, np.uint8)
        xmin, ymin = P[:, 0].min(), P[:, 1].min()
        xmax, ymax = P[:, 0].max(), P[:, 1].max()
        span = max(xmax - xmin, ymax - ymin, 1.0) * 1.15
        self.cx, self.cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        self.ppm = (size - 24) / span  # pixels per metre
        self.span = span
        self.last = None
        self._grid()

    def _grid(self):
        step = 1.0 if self.span < 12 else (5.0 if self.span < 60 else 10.0)
        x0 = math.floor((self.cx - self.span / 2) / step) * step
        y0 = math.floor((self.cy - self.span / 2) / step) * step
        n = int(self.span / step) + 3
        for i in range(n):
            u, _ = self.to_px(x0 + i * step, self.cy)
            _, v = self.to_px(self.cx, y0 + i * step)
            cv2.line(self.canvas, (u, 0), (u, self.size), (55, 55, 55), 1)
            cv2.line(self.canvas, (0, v), (self.size, v), (55, 55, 55), 1)
        cv2.putText(self.canvas, "grid %g m" % step, (6, self.size - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1, cv2.LINE_AA)
        cv2.putText(self.canvas, "top view, x = initial heading ->, y left ^", (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1, cv2.LINE_AA)

    def to_px(self, x, y):
        u = int(round(self.size / 2 + (x - self.cx) * self.ppm))
        v = int(round(self.size / 2 - (y - self.cy) * self.ppm))
        return u, v

    def advance(self, pts):
        for p in pts:
            q = self.to_px(p[0], p[1])
            if self.last is None:
                cv2.drawMarker(self.canvas, q, (80, 200, 80), cv2.MARKER_CROSS, 10, 1)
            else:
                cv2.line(self.canvas, self.last, q, (0, 200, 255), 2, cv2.LINE_AA)
            self.last = q

    def render(self, cur):
        out = self.canvas.copy()
        if cur is not None:
            cv2.circle(out, self.to_px(cur[0], cur[1]), 4, (0, 60, 255), -1, cv2.LINE_AA)
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viz-bag", default="")
    ap.add_argument("--bag", required=True, help="original sensor bag (fallback images)")
    ap.add_argument("--traj", default="", help="pose_to_file txt (fallback poses)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=float, default=2.0, help="upscale of the main view")
    ap.add_argument("--inset", type=float, default=0.42, help="inset size as a fraction of frame height")
    ap.add_argument("--fps", type=float, default=0.0, help="output fps (0 = source rate)")
    ap.add_argument("--view", default="camera", choices=["camera", "tracker", "color", "mono"],
                    help="main view: camera (default: colour if recorded, else mono), tracker (OpenVINS features drawn), color, mono")
    args = ap.parse_args()

    poses = load_poses_from_bag(args.viz_bag) or load_poses_from_txt(args.traj)
    if poses is None:
        print("no poses found (no /ov_msckf/odomimu in viz bag and no usable --traj); the inset will say so")
    T, P, V, Trecv, Q = poses if poses else (np.zeros(0), np.zeros((0, 3)), np.zeros((0, 3)), None, None)
    rpy_v = quat_to_rpy(Q) if Q is not None and len(Q) else None
    # express the trajectory in a world frame with origin + heading at the first pose (OpenVINS's
    # global yaw is arbitrary), so the inset's +x is the initial camera direction
    if len(P) > 1 and rpy_v is not None:
        yaw0 = rpy_v[0, 2]
        c, s_ = math.cos(-yaw0), math.sin(-yaw0)
        Rw = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        P = (P - P[0]) @ Rw.T
        rpy_v = rpy_v.copy(); rpy_v[:, 2] = rpy_v[:, 2] - yaw0
    att = load_attitude(args.bag)
    signs, offs = (fit_vio_to_fc(T, rpy_v, att[0], att[2]) if (att is not None and rpy_v is not None) else (np.ones(3), np.zeros(3)))
    if att is not None:
        print("FC attitude found: VIO angles shown as sign*VIO+offset, signs %s, offsets %s deg" % (signs.astype(int).tolist(), np.round(np.degrees(offs), 1).tolist()))

    frames = list(image_iter(args.viz_bag, args.bag, args.view))
    if not frames:
        print("no images found")
        sys.exit(1)
    # time base for matching frames to poses: tracker view -> receive times (OpenVINS stamps the
    # tracker image with wall time); raw camera -> header stamps (sensor time, same as odometry)
    use_recv = frames[0][3] == "tracker view" and Trecv is not None
    if frames[0][3] == "tracker view" and Trecv is None:
        print("note: tracker view without odometry receive times; matching by frame order instead")
    Tm = Trecv if use_recv else T
    stamps = np.array([f[1] if use_recv else f[0] for f in frames])
    if frames[0][3] == "tracker view" and Trecv is None and len(T):
        # fall back: spread the poses uniformly over the frames
        stamps = np.linspace(T[0], T[-1], len(frames))
    src_fps = (len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-3)
    fps = args.fps if args.fps > 0 else src_fps
    h0, w0 = frames[0][2].shape[:2]
    W, H = int(round(w0 * args.scale)), int(round(h0 * args.scale))
    isz = int(H * args.inset)
    inset = TrajInset(P, isz) if len(P) > 1 else None
    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    if not writer.isOpened():
        print("cannot open video writer for", args.out)
        sys.exit(1)
    t0 = stamps[0]
    k = 0  # pose index already drawn
    for i, (_, _, img, kind) in enumerate(frames):
        ts = stamps[i]
        frame = cv2.resize(img, (W, H), interpolation=cv2.INTER_LINEAR)
        cur = None
        if inset is not None:
            k_new = int(np.searchsorted(Tm, ts + 0.02, side="right"))
            if k_new > k:
                inset.advance(P[k:k_new])
                k = k_new
            if k > 0:
                cur = P[k - 1]
            pane = inset.render(cur)
            frame[H - isz - 8: H - 8, W - isz - 8: W - 8] = pane
            cv2.rectangle(frame, (W - isz - 8, H - isz - 8), (W - 8, H - 8), (200, 200, 200), 1)
        # text overlay (one decimal everywhere)
        lines = ["t = %.1f s   %s" % (ts - t0, kind)]
        if cur is not None:
            vb = V[k - 1] if len(V) == len(T) else np.zeros(3)
            sp = float(np.linalg.norm(vb))
            lines.append("pos x %+.1f  y %+.1f  z %+.1f m   speed %.1f m/s   (world: x = initial heading)" % (cur[0], cur[1], cur[2], sp))
            lines.append("vel body vx %+.1f  vy %+.1f  vz %+.1f m/s   (x fwd, y left, z up)" % (vb[0], vb[1], vb[2]))
            if rpy_v is not None:
                r, pch, yw = np.degrees(signs * rpy_v[k - 1] + offs)
                lines.append("VIO roll %+.1f  pitch %+.1f  yaw %+.1f deg" % (r, pch, wrap_deg(yw)))
        elif inset is not None:
            lines.append("waiting for VIO initialization ...")
        else:
            lines.append("no trajectory available")
        if att is not None:
            Tatt = att[1] if use_recv else att[0]
            j = int(np.searchsorted(Tatt, ts, side="right")) - 1
            if j >= 0:
                r, pch, yw = np.degrees(att[2][j])
                lines.append("FC  roll %+.1f  pitch %+.1f  yaw %+.1f deg" % (r, pch, wrap_deg(yw)))
        y = 24
        for s in lines:
            cv2.putText(frame, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            y += 24
        writer.write(frame)
    writer.release()
    if len(P) > 1:
        L = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
        print("video written: %s  (%d frames, %.1f fps, %dx%d)  path %.2f m, start->end %.3f m" % (args.out, len(frames), fps, W, H, L, np.linalg.norm(P[-1] - P[0])))
    else:
        print("video written: %s  (%d frames, %.1f fps, %dx%d), no trajectory" % (args.out, len(frames), fps, W, H))


if __name__ == "__main__":
    main()
