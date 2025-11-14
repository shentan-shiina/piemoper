#!/usr/bin/env python3

import rospy
import math
import threading
import time
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from pika.sense import Sense
from tf.transformations import (
    quaternion_matrix,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_from_euler,
    euler_from_quaternion,
)

# ------------------------------------------------------
# Quaternion & math helpers
# ------------------------------------------------------

def quat_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0., 0., 0., 1.])
    return q / n

def quat_to_rotmat(q):
    return quaternion_matrix(q)[:3, :3]

def quat_inverse(q):
    return quaternion_inverse(q)

def quat_mul(q1, q2):
    return quaternion_multiply(q1, q2)

def quat_angle_between(q1, q2):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    qrel = quat_mul(quat_inverse(q1), q2)
    qrel = quat_normalize(qrel)
    angle = 2.0 * math.acos(max(-1.0, min(1.0, qrel[3])))
    if angle > math.pi:
        angle = 2 * math.pi - angle
    return abs(angle)

def slerp(q1, q2, t):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = np.dot(q1, q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        res = q1 + t * (q2 - q1)
        return quat_normalize(res)
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta_0 * t
    q3 = quat_normalize(q2 - q1 * dot)
    return quat_normalize(q1 * math.cos(theta) + q3 * math.sin(theta))

# ------------------------------------------------------
# Pose smoother for translation + rotation
# ------------------------------------------------------

class PoseSmoother:
    def __init__(self, pos_alpha=0.3, rot_alpha=0.25):
        self.pos_alpha = pos_alpha
        self.rot_alpha = rot_alpha
        self.last_pos = None
        self.last_quat = None

    def reset(self, pos, quat):
        self.last_pos = np.array(pos, dtype=float)
        self.last_quat = quat_normalize(quat)

    def smooth(self, pos, quat):
        pos = np.array(pos, dtype=float)
        quat = quat_normalize(quat)
        if self.last_pos is None:
            self.reset(pos, quat)
            return pos.tolist(), quat.tolist()
        new_pos = self.pos_alpha * pos + (1.0 - self.pos_alpha) * self.last_pos
        new_quat = slerp(self.last_quat, quat, self.rot_alpha)
        self.last_pos = new_pos
        self.last_quat = new_quat
        return new_pos.tolist(), new_quat.tolist()

# ------------------------------------------------------
# Main ROS teleoperation class
# ------------------------------------------------------

class RosOperator:
    def __init__(self):
        rospy.init_node('pika_localization_py', anonymous=True)

        # Publishers
        self.pub_base = rospy.Publisher("/pika_pose_base", PoseStamped, queue_size=10)
        self.pub_l = rospy.Publisher("/pika_pose_l", PoseStamped, queue_size=10)
        self.pub_r = rospy.Publisher("/pika_pose_r", PoseStamped, queue_size=10)

        # Sense instance
        self.sense = Sense()

        # Tracker names
        self.tracker_name_l = rospy.get_param("~pika_l_name", 'WM0')
        self.tracker_name_r = rospy.get_param("~pika_r_name", 'WM1')
        self.tracker_name_base = rospy.get_param("~pika_base_name", 'WM2')

        # Shared pose data
        self.poses = {
            self.tracker_name_l: None,
            self.tracker_name_r: None,
            self.tracker_name_base: None
        }

        self.lock = threading.Lock()
        self.running = True

        # Smoothers for left/right arms
        self.smoother_l = PoseSmoother(pos_alpha=0.99, rot_alpha=0.99)
        self.smoother_r = PoseSmoother(pos_alpha=0.99, rot_alpha=0.99)

        # Store last relative poses for deadzone computation
        self._last_rel_pose = {}

    def init_tracker(self):
        self.sense.get_vive_tracker()
        time.sleep(8)
        init_pose = self.sense.get_pose(self.tracker_name_l)
        return init_pose is not None

    def publish_pose(self, publisher, position, quat):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        publisher.publish(pose_msg)

    # ------------------------------------------------------
    # Relative pose computation
    # ------------------------------------------------------
    def publish_localization_pose_base_rel(self, publisher, pose, pose_base, smoother=None,
                                           trans_deadzone=0.005, rot_deadzone_rad=0.01):
        """
        Compute and publish the relative pose of `pose` w.r.t. `pose_base`.
        Uses quaternion math (no Euler) and applies per-frame deadzone + smoothing.
        """
        if pose is None or pose_base is None:
            return

        # Extract pose vectors and quaternions
        t_pose = np.array(pose.position, dtype=float)
        t_base = np.array(pose_base.position, dtype=float)
        q_pose = quat_normalize(np.array(pose.rotation, dtype=float))
        q_base = quat_normalize(np.array(pose_base.rotation, dtype=float))

        # Relative rotation: q_rel = q_base^{-1} * q_pose
        q_base_inv = quat_inverse(q_base)
        q_rel = quat_mul(q_base_inv, q_pose)
        q_rel = quat_normalize(q_rel)

        # Relative translation: t_rel = R_base^{-1} * (t_pose - t_base)
        R_base = quat_to_rotmat(q_base)
        t_diff = t_pose - t_base
        t_rel = R_base.T.dot(t_diff)

        # Retrieve last pose for deadzone check
        if publisher not in self._last_rel_pose:
            self._last_rel_pose[publisher] = {"t_rel": t_rel, "q_rel": q_rel}
            self.publish_pose(publisher, t_rel.tolist(), q_rel.tolist())
            return

        last_t_rel = self._last_rel_pose[publisher]["t_rel"]
        last_q_rel = self._last_rel_pose[publisher]["q_rel"]

        # Compute per-frame motion magnitude
        trans_diff = np.linalg.norm(t_rel - last_t_rel)
        rot_diff = quat_angle_between(last_q_rel, q_rel)

        # Apply deadzone to per-frame change
        print('Translation Difference:',trans_diff)
        print('Rotation Difference:',rot_diff)

        if trans_diff < trans_deadzone and rot_diff < rot_deadzone_rad:
            return  # skip negligible jitter

        # Smooth (optional)
        if smoother is not None:
            t_rel_list, q_rel = smoother.smooth(t_rel.tolist(), q_rel.tolist())
            t_rel = np.array(t_rel_list, dtype=float)

        print('Smoothed Translation Difference:', np.linalg.norm(t_rel - last_t_rel))
        print('Smoothed Rotation Difference:', quat_angle_between(last_q_rel, q_rel))

        # Publish updated pose
        self.publish_pose(publisher, t_rel, q_rel)

        # Store current as last
        self._last_rel_pose[publisher]["t_rel"] = t_rel
        self._last_rel_pose[publisher]["q_rel"] = q_rel

    # ------------------------------------------------------
    # Tracker threads and main loop
    # ------------------------------------------------------
    def tracker_thread(self, tracker_name):
        while not rospy.is_shutdown() and self.running:
            pose = self.sense.get_pose(tracker_name)
            with self.lock:
                self.poses[tracker_name] = pose
            time.sleep(0.001)

    def run(self):
        # Launch threads for all trackers
        threads = []
        for tracker_name in self.poses.keys():
            t = threading.Thread(target=self.tracker_thread, args=(tracker_name,))
            t.daemon = True
            t.start()
            threads.append(t)

        rate = rospy.Rate(60)  # reduced rate for stability
        while not rospy.is_shutdown():
            with self.lock:
                pose_l = self.poses[self.tracker_name_l]
                pose_r = self.poses[self.tracker_name_r]
                pose_base = self.poses[self.tracker_name_base]

            # Publish relative poses (left & right) and base
            self.publish_localization_pose_base_rel(self.pub_l, pose_l, pose_base, smoother=self.smoother_l, trans_deadzone=0.006, rot_deadzone_rad=0.015)
            self.publish_localization_pose_base_rel(self.pub_r, pose_r, pose_base, smoother=self.smoother_r, trans_deadzone=0.006, rot_deadzone_rad=0.015)
            if pose_base is not None:
                self.publish_pose(self.pub_base, pose_base.position, pose_base.rotation)
                # self.publish_pose(self.pub_l, pose_l.position, pose_l.rotation)
                # self.publish_pose(self.pub_r, pose_r.position, pose_r.rotation)

            rate.sleep()

        # Stop threads
        self.running = False
        for t in threads:
            t.join(timeout=1.0)

# ------------------------------------------------------
# Entry point
# ------------------------------------------------------

def main():
    ros_operator = RosOperator()
    if ros_operator.init_tracker():
        rospy.loginfo("Localization for trackers started.")
        ros_operator.run()
    else:
        rospy.logerr("Localization error!")

if __name__ == '__main__':
    main()
