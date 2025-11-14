#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import threading
import time
from pika.sense import Sense


class RosOperator:
    def __init__(self):
        rospy.init_node('pika_localization_py', anonymous=True)

        # Publishers
        self.pub_base = rospy.Publisher("/pika_pose_base", PoseStamped, queue_size=10)
        self.pub_l = rospy.Publisher("/pika_pose_l", PoseStamped, queue_size=10)
        self.pub_r = rospy.Publisher("/pika_pose_r", PoseStamped, queue_size=10)

        # Sense instance
        self.sense = Sense()

        # Tracker names (from ROS params)
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
        time.sleep(2)
        init_pose = self.sense.get_pose(self.tracker_name_l)
        return init_pose is not None

    def publish_localization_pose(self, publisher, pose):
        if pose is None:
            return
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pose.position[0]
        pose_msg.pose.position.y = pose.position[1]
        pose_msg.pose.position.z = pose.position[2]
        pose_msg.pose.orientation.x = pose.rotation[0]
        pose_msg.pose.orientation.y = pose.rotation[1]
        pose_msg.pose.orientation.z = pose.rotation[2]
        pose_msg.pose.orientation.w = pose.rotation[3]
        publisher.publish(pose_msg)

    def tracker_thread(self, tracker_name):
        """Thread to continuously read pose for one tracker."""
        while not rospy.is_shutdown() and self.running:
            pose = self.sense.get_pose(tracker_name)
            with self.lock:
                self.poses[tracker_name] = pose
            time.sleep(0.001)  # small sleep to reduce CPU load

    def run(self):
        # Start threads for each tracker
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

            # Publish concurrently fetched poses
            self.publish_localization_pose(self.pub_l, pose_l)
            self.publish_localization_pose(self.pub_r, pose_r)
            self.publish_localization_pose(self.pub_base, pose_base)

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
