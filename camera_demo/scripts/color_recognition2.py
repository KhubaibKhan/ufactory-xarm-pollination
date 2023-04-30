#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import rospy
import time
import yaml
import threading
import rospy
from tf2_geometry_msgs import *
import cv2
import threading
from tuts.gripper_ctrl import GripperCtrl
from tuts.xarm_ctrl import XArmCtrl
from utilsm.utils import *
from utilsm.motion_thread import MotionThread
import warnings

warnings.filterwarnings("ignore")

if sys.version_info < (3, 0):
    PY3 = False
    import Queue as queue
else:
    PY3 = True
    import queue

if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)

    # Initialize OpenCV bridge
    dof = rospy.get_param('/xarm/DOF')

    rate = rospy.Rate(10.0)
    motion_que = queue.Queue(1)
    motion = MotionThread(motion_que, dof=dof, grab_z=195, safe_z=300, iden_z=200)
    motion.start()
    # motion_que.put([318, 50, 461, -2.96, -0.314, -0.27])

    # Initialize TF buffer and listener
    global tf_buffer
    global tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    realsense = Realsense()
    cnts = 0
    while not rospy.is_shutdown():
        rate.sleep()
        # frame = cam.get_frame()
        frame = realsense.image
        if frame is None:
            print("No frame")
            continue
        # xc, yc = detect_color_block(realsense.image, color)
        # if xc == -1 and yc == -1:
        #     continue
        centroids = detect_color_block(realsense.image, color)
        if len(centroids) == 0:
            continue
        xc, yc, label = centroids[0]


        # Convert ROS image to OpenCV image
        cv_image = cv_bridge.imgmsg_to_cv2(realsense.image, 'bgr8')

        # Get depth
        depth = realsense.get_depth(realsense.depth, xc, yc)
        # print("Image coordinate point", xc, yc, depth)
        # Get the 3D position of the block
        x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)
        # print("3D position of the block", x, y, z)

        # Transform 3D position from camera frame to base frame
        x, y, z = transform_3d_position(x, y, z, 'camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)
        # print("Transformed 3D position of the block", x, y, z)

        cnts += 1
        if cnts < 50:
            continue
        # if len(rects) == 0:
        #     continue
        if motion.in_motion or motion_que.qsize() != 0:
            continue
        motion_que.put([x, y, z])
        