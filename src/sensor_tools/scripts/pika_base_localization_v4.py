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
)

# ------------------------------------------------------
# Quaternion & math helpers
# ------------------------------------------------------

def quat_normalize(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
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
    # angle = 2 * acos(w)
    w = max(-1.0, min(1.0, float(qrel[3])))
    angle = 2.0 * math.acos(w)
    if angle > math.pi:
        angle = 2 * math.pi - angle
    return abs(angle)

def slerp(q1, q2, t):
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = float(np.dot(q1, q2))
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
    def __init__(self, pos_alpha=0.25, rot_alpha=0.25):
        """
        pos_alpha: EMA weight for new position (0..1). Higher -> more reactive, less smoothing.
        rot_alpha: SLERP weight for rotation interpolation (0..1). Higher -> more reactive.
        """
        self.pos_alpha = float(pos_alpha)
        self.rot_alpha = float(rot_alpha)
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
        # EMA for translation
        new_pos = self.pos_alpha * pos + (1.0 - self.pos_alpha) * self.last_pos
        # SLERP for rotation
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

        # Reasonable default smoothers (tune to taste)
        self.smoother_l = PoseSmoother(pos_alpha=0.3, rot_alpha=0.25)
        self.smoother_r = PoseSmoother(pos_alpha=0.3, rot_alpha=0.25)
        # Base smoother should be stronger (more smoothing) to prevent amplification
        self.smoother_base = PoseSmoother(pos_alpha=0.1, rot_alpha=0.1)

        # Store last relative/base poses for deadzone computation (dict keyed by publisher)
        self._last_rel_pose = {}

    def init_tracker(self):
        # Initialize trackers (increase wait briefly to ensure trackers are discovered)
        self.sense.get_vive_tracker()
        time.sleep(8.0)
        init_pose = self.sense.get_pose(self.tracker_name_l)
        return init_pose is not None

    def publish_pose(self, publisher, position, quat):
        # Ensure types are python floats/lists
        pos = list(map(float, position))
        q = list(map(float, quat))
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        publisher.publish(pose_msg)

    # ------------------------------------------------------
    # Base pose smoothing & publishing
    # ------------------------------------------------------
    def publish_localization_pose_base(self, publisher, pose_base, smoother=None,
                                       trans_deadzone=0.005, rot_deadzone_rad=0.01):
        """
        Smooth and publish base pose. Returns (t_base_list, q_base_list) the smoothed base pose
        for downstream relative-pose calculations.
        """
        if pose_base is None:
            return None

        # Extract pose vectors and quaternions
        t_base = np.array(pose_base.position, dtype=float)
        q_base = quat_normalize(np.array(pose_base.rotation, dtype=float))

        # Initialize storage if first time for this publisher
        key = ('base', publisher)
        if key not in self._last_rel_pose:
            # publish initial raw (or optionally smoothed)
            t_out = t_base.tolist()
            q_out = q_base.tolist()
            self._last_rel_pose[key] = {"t_base": np.array(t_out, dtype=float), "q_base": np.array(q_out, dtype=float)}
            self.publish_pose(publisher, t_out, q_out)
            return (t_out, q_out)

        last_t_base = self._last_rel_pose[key]["t_base"]
        last_q_base = self._last_rel_pose[key]["q_base"]

        # Compute per-frame change magnitude
        trans_diff = np.linalg.norm(t_base - last_t_base)
        rot_diff = quat_angle_between(last_q_base, q_base)

        #print("Base trans diff: %.6f m, rot diff: %.6f rad", trans_diff, rot_diff)

        # If movement below deadzone, keep last (no new sample)
        if trans_diff < trans_deadzone and rot_diff < rot_deadzone_rad:
            # Publish last stored pose as-is (to keep topic alive)
            self.publish_pose(publisher, last_t_base.tolist(), last_q_base.tolist())
            return (last_t_base.tolist(), last_q_base.tolist())

        # Apply smoother (if provided)
        if smoother is not None:
            t_out_list, q_out_list = smoother.smooth(t_base.tolist(), q_base.tolist())
            t_out = np.array(t_out_list, dtype=float)
            q_out = np.array(q_out_list, dtype=float)
        else:
            t_out = t_base
            q_out = q_base

        # Publish and store
        self.publish_pose(publisher, t_out.tolist(), q_out.tolist())
        self._last_rel_pose[key]["t_base"] = np.array(t_out, dtype=float)
        self._last_rel_pose[key]["q_base"] = np.array(q_out, dtype=float)

        return (t_out.tolist(), q_out.tolist())

    # ------------------------------------------------------
    # Relative pose computation and publishing
    # ------------------------------------------------------
    def publish_localization_pose_base_rel(self, publisher, pose, pose_base_tuple, smoother=None,
                                           trans_deadzone=0.005, rot_deadzone_rad=0.01):
        """
        Compute relative pose of `pose` w.r.t. smoothed base (pose_base_tuple).
        pose_base_tuple: (t_base_list, q_base_list) returned by publish_localization_pose_base.
        """
        if pose is None or pose_base_tuple is None:
            return

        # Unpack smoothed base
        t_base = np.array(pose_base_tuple[0], dtype=float)
        q_base = quat_normalize(np.array(pose_base_tuple[1], dtype=float))

        # Pose to be converted
        t_pose = np.array(pose.position, dtype=float)
        q_pose = quat_normalize(np.array(pose.rotation, dtype=float))

        # Relative rotation: q_rel = q_base^{-1} * q_pose
        q_base_inv = quat_inverse(q_base)
        q_rel = quat_mul(q_base_inv, q_pose)
        q_rel = quat_normalize(q_rel)

        # Relative translation: t_rel = R_base^{-1} * (t_pose - t_base)
        R_base = quat_to_rotmat(q_base)
        t_diff = t_pose - t_base
        t_rel = R_base.T.dot(t_diff)

        # Use publisher-specific last-rel storage
        key = ('rel', publisher)
        if key not in self._last_rel_pose:
            self._last_rel_pose[key] = {"t_rel": np.array(t_rel, dtype=float), "q_rel": np.array(q_rel, dtype=float)}
            self.publish_pose(publisher, t_rel.tolist(), q_rel.tolist())
            return

        last_t_rel = self._last_rel_pose[key]["t_rel"]
        last_q_rel = self._last_rel_pose[key]["q_rel"]

        # Compute per-frame motion magnitude
        trans_diff = np.linalg.norm(t_rel - last_t_rel)
        rot_diff = quat_angle_between(last_q_rel, q_rel)

        #rospy.logdebug("Rel trans diff: %.6f m, rot diff: %.6f rad", trans_diff, rot_diff)

        # Deadzone on *movement*
        if trans_diff < trans_deadzone and rot_diff < rot_deadzone_rad:
            # Re-publish last stable pose to keep topic alive
            self.publish_pose(publisher, last_t_rel.tolist(), last_q_rel.tolist())
            return

        # Smooth (optional)
        if smoother is not None:
            t_rel_list, q_rel_list = smoother.smooth(t_rel.tolist(), q_rel.tolist())
            t_rel = np.array(t_rel_list, dtype=float)
            q_rel = np.array(q_rel_list, dtype=float)

        # Publish updated pose
        self.publish_pose(publisher, t_rel.tolist(), q_rel.tolist())

        # Store current as last
        self._last_rel_pose[key]["t_rel"] = np.array(t_rel, dtype=float)
        self._last_rel_pose[key]["q_rel"] = np.array(q_rel, dtype=float)

    # ------------------------------------------------------
    # Tracker threads and main loop
    # ------------------------------------------------------
    def tracker_thread(self, tracker_name):
        while not rospy.is_shutdown() and self.running:
            pose = self.sense.get_pose(tracker_name)
            with self.lock:
                self.poses[tracker_name] = pose
            # sleep to avoid burning CPU; trackers can be polled faster if needed
            time.sleep(0.001)

    def run(self):
        # Launch threads for all trackers
        threads = []
        for tracker_name in self.poses.keys():
            t = threading.Thread(target=self.tracker_thread, args=(tracker_name,))
            t.daemon = True
            t.start()
            threads.append(t)

        rate = rospy.Rate(60)  # reduced rate for stability; tune to your needs
        while not rospy.is_shutdown():
            with self.lock:
                pose_l = self.poses[self.tracker_name_l]
                pose_r = self.poses[self.tracker_name_r]
                pose_base = self.poses[self.tracker_name_base]

            if pose_base is not None:
                # smooth base first (prevents base noise from contaminating relative transform)
                pose_base_smoothed = self.publish_localization_pose_base(
                    self.pub_base, pose_base,
                    smoother=self.smoother_base,
                    trans_deadzone=0.015, rot_deadzone_rad=0.05
                )
                # compute and publish relative poses using smoothed base
                self.publish_localization_pose_base_rel(
                    self.pub_l, pose_l, pose_base_smoothed,
                    smoother=self.smoother_l, trans_deadzone=0.006, rot_deadzone_rad=0.015
                )
                self.publish_localization_pose_base_rel(
                    self.pub_r, pose_r, pose_base_smoothed,
                    smoother=self.smoother_r, trans_deadzone=0.006, rot_deadzone_rad=0.015
                )
            else:
                # If base missing, optional fallback: publish raw poses
                if pose_l is not None:
                    self.publish_pose(self.pub_l, pose_l.position, pose_l.rotation)
                if pose_r is not None:
                    self.publish_pose(self.pub_r, pose_r.position, pose_r.rotation)

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
