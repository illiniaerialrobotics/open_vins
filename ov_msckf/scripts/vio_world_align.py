#!/usr/bin/env python3
"""
vio_world_align: re-express OpenVINS odometry in a WORLD frame with a defined heading.

OpenVINS's "global" frame is gravity aligned (z up) but its yaw is arbitrary (built from gravity
alone; on the bench it came out 180 deg from the initial camera direction, and it changes with the
tilt at initialisation). This node defines
    world := pose of the IMU at the first message, yaw only (origin there, +x = heading, z up)
and publishes /ov_msckf/odomimu (IMU frame) as ~odom_world (frame "world", child "imu"), plus the
static TF world -> global. Position, velocity (world frame) and covariance are rotated accordingly.
Optional ~yaw_offset_deg rotates the world further (e.g. if the camera is not along the airframe nose).
"""
import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


def q_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_q(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


class WorldAlign:
    def __init__(self):
        self.yaw_off = math.radians(rospy.get_param("~yaw_offset_deg", 0.0))
        self.R_WG = None   # world <- global (yaw only)
        self.t_WG = None
        self.pub = rospy.Publisher("~odom_world", Odometry, queue_size=2, tcp_nodelay=True)
        self.pub_pose = rospy.Publisher("~pose_world", PoseStamped, queue_size=2, tcp_nodelay=True)
        self.tf_pub = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber(rospy.get_param("~odom_topic", "/ov_msckf/odomimu"), Odometry, self.cb, queue_size=5, tcp_nodelay=True)

    def cb(self, m):
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        q = np.array([m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w])
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            return
        R_GI = q_to_R(q)
        if self.R_WG is None:
            yaw0 = math.atan2(R_GI[1, 0], R_GI[0, 0]) - self.yaw_off
            c, s = math.cos(-yaw0), math.sin(-yaw0)
            self.R_WG = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            self.t_WG = -self.R_WG @ p
            tfm = TransformStamped()
            tfm.header.stamp = m.header.stamp
            tfm.header.frame_id = "world"
            tfm.child_frame_id = m.header.frame_id or "global"
            tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z = self.t_WG
            tfm.transform.rotation.x, tfm.transform.rotation.y, tfm.transform.rotation.z, tfm.transform.rotation.w = R_to_q(self.R_WG)
            self.tf_pub.sendTransform(tfm)
            rospy.loginfo("vio_world_align: world defined at first pose: global yaw of heading %.1f deg -> world +x", math.degrees(yaw0 + self.yaw_off))
        pw = self.R_WG @ p + self.t_WG
        R_WI = self.R_WG @ R_GI
        qw = R_to_q(R_WI)
        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id = "world"
        out.child_frame_id = m.child_frame_id or "imu"
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pw
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = qw
        # OpenVINS odomimu twist: linear in the IMU frame, angular in the IMU frame -> keep angular, give linear in world
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        vw = R_WI @ v
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = vw
        out.twist.twist.angular = m.twist.twist.angular
        C = np.array(m.pose.covariance).reshape(6, 6)
        T6 = np.zeros((6, 6)); T6[:3, :3] = self.R_WG; T6[3:, 3:] = self.R_WG
        out.pose.covariance = (T6 @ C @ T6.T).ravel().tolist()
        out.twist.covariance = m.twist.covariance
        self.pub.publish(out)
        ps = PoseStamped(); ps.header = out.header; ps.pose = out.pose.pose
        self.pub_pose.publish(ps)


if __name__ == "__main__":
    rospy.init_node("vio_world_align")
    WorldAlign()
    rospy.spin()
