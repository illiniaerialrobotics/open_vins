#!/usr/bin/env python3
import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry


class OdomFrameTransformer:
    def __init__(self):
        rospy.init_node('odom_frame_transformer')

        # 1. Setup TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 2. Subs and Pubs
        # Input: OpenVINS providing IMU pose in Global frame
        self.sub = rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.callback)
        # Output: Robot Base pose in the Map (Mocap) frame
        self.pub = rospy.Publisher('/ov_msckf/odomimu_corrected_WORLD', Odometry, queue_size=10)
        self.pub_pose = rospy.Publisher(
            '/ov_msckf/poseimu_corrected', PoseWithCovarianceStamped, queue_size=10
        )
        self.pub_pose_cov = rospy.Publisher(
            '/ov_msckf/poseimu_corrected_cov', PoseWithCovarianceStamped, queue_size=10
        )

        # If True: twist.linear/angular are rotated into header.frame_id ("global").
        # nav_msgs/Odometry convention expects twist in child_frame_id ("base_link");
        # enable this only if downstream explicitly wants world-frame velocities.
        self.express_twist_in_global = rospy.get_param("~express_twist_in_global", True)
        if self.express_twist_in_global:
            rospy.logwarn(
                "express_twist_in_global:=true: twist is in 'global', not child_frame_id "
                "(non-standard for nav_msgs/Odometry)."
            )

        rospy.loginfo("Transformer initialized. Mapping [world -> global -> imu -> base_link]")

    def callback(self, msg):
        try:
            # IMPORTANT: Look up the chain from the Ground Truth origin (map) 
            # all the way to the robot center (base_link)
            # This handles both the global yaw shift AND the D435i axis swap.
            transform = self.tf_buffer.lookup_transform(
                "global",                # Target: Where we want the pose (Ground Truth World)
                "base_link",          # Source: The frame we want to track (Robot Center)
                msg.header.stamp,     # Synchronize with the Odom timestamp
                rospy.Duration(0.1)
            )

            # --- TRANSFORM POSE ---
            # We treat the incoming message as a reference. 
            # Note: We use an identity pose because 'transform' already contains 
            # the full Map -> Base_link state.
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose.orientation.w = 1.0 # Identity
            
            ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)

            # --- TRANSFORM VELOCITIES ---
            # Twist is usually expressed in the child_frame (base_link).
            # We just need to rotate the vectors from the IMU frame to the Base_link frame.
            # We look up the local extrinsic only for velocity.
            local_extrinsic = self.tf_buffer.lookup_transform("base_link", "imu", rospy.Time(0))

            v_lin = Vector3Stamped()
            v_lin.vector = msg.twist.twist.linear
            v_lin_transformed = tf2_geometry_msgs.do_transform_vector3(v_lin, local_extrinsic)

            v_ang = Vector3Stamped()
            v_ang.vector = msg.twist.twist.angular
            v_ang_transformed = tf2_geometry_msgs.do_transform_vector3(v_ang, local_extrinsic)

            # --- CONSTRUCT NEW MESSAGE ---
            out_msg = Odometry()
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = "global"
            out_msg.child_frame_id = "base_link"

            out_msg.pose.pose = ps_transformed.pose
            lin_vec = v_lin_transformed.vector
            ang_vec = v_ang_transformed.vector
            twist_cov = np.array(msg.twist.covariance, dtype=float).reshape(6, 6)

            if self.express_twist_in_global:
                # Rotate free vectors from base_link into global using current attitude.
                q = ps_transformed.pose.orientation
                rot_tf = TransformStamped()
                rot_tf.header.stamp = msg.header.stamp
                rot_tf.header.frame_id = "global"
                rot_tf.child_frame_id = "base_link"
                rot_tf.transform.translation.x = 0.0
                rot_tf.transform.translation.y = 0.0
                rot_tf.transform.translation.z = 0.0
                rot_tf.transform.rotation = q

                v_lin_g = Vector3Stamped()
                v_lin_g.header.frame_id = "base_link"
                v_lin_g.header.stamp = msg.header.stamp
                v_lin_g.vector = lin_vec
                lin_vec = tf2_geometry_msgs.do_transform_vector3(v_lin_g, rot_tf).vector

                v_ang_g = Vector3Stamped()
                v_ang_g.header.frame_id = "base_link"
                v_ang_g.header.stamp = msg.header.stamp
                v_ang_g.vector = ang_vec
                ang_vec = tf2_geometry_msgs.do_transform_vector3(v_ang_g, rot_tf).vector

                R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
                T6 = np.zeros((6, 6))
                T6[0:3, 0:3] = R
                T6[3:6, 3:6] = R
                twist_cov = T6 @ twist_cov @ T6.T

            out_msg.twist.twist.linear = lin_vec
            out_msg.twist.twist.angular = ang_vec

            # Preserve pose covariance; twist covariance rotated if express_twist_in_global
            out_msg.pose.covariance = list(msg.pose.covariance)
            out_msg.twist.covariance = twist_cov.reshape(-1).tolist()

            self.pub.publish(out_msg)

            pose_out = PoseWithCovarianceStamped()
            pose_out.header = out_msg.header
            pose_out.pose = out_msg.pose
            self.pub_pose.publish(pose_out)

            pose_cov_out = PoseWithCovarianceStamped()
            pose_cov_out.header = out_msg.header
            pose_cov_out.pose = out_msg.pose
            self.pub_pose_cov.publish(pose_cov_out)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "Waiting for TF chain [map -> global -> imu -> base_link]: %s" % str(e))

if __name__ == '__main__':
    OdomFrameTransformer()
    rospy.spin()
