#!/usr/bin/env python3

import rospy
import math
import threading
import time
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from pika.sense import Sense
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

    def init_tracker(self):
        self.sense.get_vive_tracker()
        time.sleep(8)
        init_pose = self.sense.get_pose(self.tracker_name_l)
        return init_pose is not None

    def create_transformation_matrix(self, x, y, z, roll, pitch, yaw):
        # Use NumPy-based TF utilities (clearer)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        R = Rz @ Ry @ Rx
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def matrix_to_xyzrpy(self, matrix):
        x, y, z = matrix[0:3, 3]
        roll, pitch, yaw = euler_from_quaternion(
            quaternion_from_euler(*self.rotation_matrix_to_rpy(matrix[:3, :3]))
        )
        return [x, y, z, roll, pitch, yaw]

    def rotation_matrix_to_rpy(self, R):
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(-R[2, 0])
        yaw = math.atan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw
        
    def pose_difference(self, pose_a, pose_b):
        if pose_a is None or pose_b is None:
            return float('inf')
        dp = np.linalg.norm(np.array(pose_a.position) - np.array(pose_b.position))
        dq = np.linalg.norm(np.array(pose_a.rotation) - np.array(pose_b.rotation))
        return dp + dq


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

    def publish_localization_pose_base_rel(self, publisher, pose, pose_base):
        if pose is None or pose_base is None:
            return

        roll_base, pitch_base, yaw_base = euler_from_quaternion(pose_base.rotation)
        roll, pitch, yaw = euler_from_quaternion(pose.rotation)

        T_base = self.create_transformation_matrix(
            *pose_base.position, roll_base, pitch_base, yaw_base)
        T_pose = self.create_transformation_matrix(
            *pose.position, roll, pitch, yaw)

        T_rel = np.linalg.inv(T_base) @ T_pose
        x, y, z = T_rel[0:3, 3]
        roll_rel, pitch_rel, yaw_rel = self.rotation_matrix_to_rpy(T_rel[:3, :3])
        quat_rel = quaternion_from_euler(roll_rel, pitch_rel, yaw_rel)

        self.publish_pose(publisher, [x, y, z], quat_rel)

    def tracker_thread(self, tracker_name):
        last_pose = None
        while not rospy.is_shutdown() and self.running:
            pose = self.sense.get_pose(tracker_name)
            with self.lock:
                if tracker_name == self.tracker_name_base:
                    if self.pose_difference(pose, last_pose) < 9e-1:  # ~1mm/rotation unit
                        continue  # ignore tiny jitter
                    last_pose = self.poses[tracker_name]
                self.poses[tracker_name] = pose
            time.sleep(0.001)

    def run(self):
        threads = []
        for tracker_name in self.poses.keys():
            t = threading.Thread(target=self.tracker_thread, args=(tracker_name,))
            t.daemon = True
            t.start()
            threads.append(t)

        rate = rospy.Rate(150)
        while not rospy.is_shutdown():
            with self.lock:
                pose_l = self.poses[self.tracker_name_l]
                pose_r = self.poses[self.tracker_name_r]
                pose_base = self.poses[self.tracker_name_base]

            self.publish_localization_pose_base_rel(self.pub_l, pose_l, pose_base)
            self.publish_localization_pose_base_rel(self.pub_r, pose_r, pose_base)
            
            #self.publish_pose(self.pub_l, pose_l.position, pose_base.rotation)
            #self.publish_pose(self.pub_r, pose_r.position, pose_base)
            self.publish_pose(self.pub_base, pose_base.position, pose_base.rotation)

            rate.sleep()

        self.running = False
        for t in threads:
            t.join(timeout=1.0)


def main():
    ros_operator = RosOperator()
    if ros_operator.init_tracker():
        rospy.loginfo("Localization for trackers started.")
        ros_operator.run()
    else:
        rospy.logerr("Localization error!")


if __name__ == '__main__':
    main()
