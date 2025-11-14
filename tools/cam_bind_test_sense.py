#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pika Sense & Pika Gripper Binding
Manually test multiple pika devices/video ports.
"""

import time
import cv2
import numpy as np
from pika import sense


def main():
    camera_param = (640, 480, 15)
    left_name = '/dev/ttyUSB0'
    right_name = '/dev/ttyUSB1'
    left_index = 4
    right_index = 6
    left_rs_serial = '315122272588'
    right_rs_serial = '315122272438'

    # Start Pika Sense/Gripper and connect device
    print("Connecting Pika devices...")
    my_sense_left = sense(left_name)
    my_sense_right = sense(right_name)

    if not my_sense_left.connect():
        print(f"Connect Pika device {left_name} failed.")
        return
    print(f"Connected to Pika device {left_name}")

    if not my_sense_right.connect():
        print(f"Connect Pika device {right_name} failed.")
        return
    print(f"Connected to Pika device {right_name}")

    my_sense_left.set_camera_param(camera_param[0], camera_param[1], camera_param[2])
    my_sense_left.set_fisheye_camera_index(left_index)
    my_sense_left.set_realsense_serial_number(left_rs_serial)

    # Get fisheye camera left
    fisheye_camera_left = my_sense_left.get_fisheye_camera()
    # Get realsense camera left
    realsense_camera_left = my_sense_left.get_realsense_camera()

    my_sense_right.set_camera_param(camera_param[0], camera_param[1], camera_param[2])
    my_sense_right.set_fisheye_camera_index(right_index)
    my_sense_right.set_realsense_serial_number(right_rs_serial)

    # Get fisheye camera right
    fisheye_camera_right = my_sense_right.get_fisheye_camera()
    ## Get realsense camera right
    realsense_camera_right = my_sense_right.get_realsense_camera()

    while True:
        # Left camera
        if fisheye_camera_left:
            # print("\n Trying to get fisheye image left...")
            success_left, frame_left = fisheye_camera_left.get_frame()
            if success_left and frame_left is not None:
                # print("\n Fisheye image left obtained successfully")
                cv2.imshow("Fisheye Camera Left", frame_left)
                cv2.imwrite("fisheye_image_left.jpg", frame_left)
            else:
                print("\n Fisheye image left obtained failed!")
                pass
        else:
            print("\n Fisheye camera left opening failed!")
            pass

        if realsense_camera_left:
            # print("\n Trying to get realsense RGB image left...")
            success_color_left, color_frame_left = realsense_camera_left.get_color_frame()
            if success_color_left and color_frame_left is not None:
                # print("\n Realsense RGB image left obtained successfully")
                cv2.imshow("RealSense Color Left", color_frame_left)
                cv2.imwrite("realsense_color_left.jpg", color_frame_left)
            else:
                print("\n Realsense RGB image left obtained failed!")
                pass

            success_depth_left, depth_frame_left = realsense_camera_left.get_depth_frame()
            if success_depth_left and depth_frame_left is not None:
                # print("\n Realsense depth image left obtained successfully")
                depth_colormap_left = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame_left, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("RealSense Depth Left", depth_colormap_left)
                cv2.imwrite("gripper_realsense_depth_left.jpg", depth_colormap_left)
            else:
                print("\n Realsense depth image left obtained failed!")
                pass
        else:
            print("\n Realsense camera left opening failed!")
            pass

        # Right camera
        if fisheye_camera_right:
             # print("\n Trying to get fisheye image right...")
            success_right, frame_right = fisheye_camera_right.get_frame()
            if success_right and frame_right is not None:
                # print("\n Fisheye image right obtained successfully")
                cv2.imshow("Fisheye Camera Right", frame_right)
                cv2.imwrite("fisheye_image_right.jpg", frame_right)
            else:
                print("\n Fisheye image right obtained failed!")
                pass
        else:
            print("\n Fisheye camera right opening failed!")
            pass

        if realsense_camera_right:
            # print("\n Trying to get realsense RGB image right...")
            success_color_right, color_frame_right = realsense_camera_right.get_color_frame()
            if success_color_right and color_frame_right is not None:
                # print("\n Realsense RGB image right obtained successfully")
                cv2.imshow("RealSense Color  Right", color_frame_right)
                cv2.imwrite("realsense_color_right.jpg", color_frame_right)
            else:
                print("\n Realsense RGB image left obtained failed!")
                pass

            success_depth_right, depth_frame_right = realsense_camera_right.get_depth_frame()
            if success_depth_right and depth_frame_right is not None:
                # print("\n Realsense depth image right obtained successfully")
                depth_colormap_right = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame_right, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("RealSense Depth Right", depth_colormap_right)
                cv2.imwrite("gripper_realsense_depth_right.jpg", depth_colormap_right)
            else:
                print("\n Realsense depth image right obtained failed!")
                pass
        else:
            print("\n Realsense camera right opening failed!")
            pass

        cv2.waitKey(1)


if __name__ == "__main__":
    main()
