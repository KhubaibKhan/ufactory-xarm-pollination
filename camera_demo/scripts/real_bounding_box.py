#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import yaml
import time
import rospy
from tf2_geometry_msgs import *
import cv2
import numpy as np
from tuts.gripper_ctrl import GripperCtrl
from tuts.xarm_ctrl import XArmCtrl
from utilsm.utils import *
from utilsm.utils import AzureKinect
from utilsm.motion_thread import MotionThread
import warnings
from numpy.linalg import norm
from scipy.linalg import logm
import geometry_msgs.msg
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import moveit_commander
from scipy.spatial.transform import Rotation
import tf
import transforms3d as tf3d
import csv

warnings.filterwarnings("ignore")

if sys.version_info < (3, 0):
    PY3 = False
    import Queue as queue
else:
    PY3 = True
    import queue

# Image size
image_size = (640, 480)

if __name__ == '__main__':
    print("Inside main")
    rospy.init_node('color_recognition_node', anonymous=False)

    rate = rospy.Rate(10.0)
    # motion_que.put([318, 50, 461, -2.96, -0.314, -0.27])

    # Initialize TF buffer and listener
    global tf_buffer
    global tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    azure = AzureKinect()
    cnts = 0

    while not rospy.is_shutdown():
        rate.sleep()

        # detect boxes
        centroids = detect_color_block(azure.image, color)
        
        camera_intrinsics = azure.camera_info

        # get the center and hw of the bounding box
        if len(centroids) > 0:
            xc, yc, label, hw, id = centroids[0]
        else:
            continue

        # get the depth of the center of the bounding box
        depth = azure.get_depth(xc, yc)

        # convert depth to cv image
        depth_cv = azure.convert_depth_to_cv(depth)

        # plot using cv
        cv2.imshow("Depth", depth_cv)
        cv2.waitKey(1)


    
    # end the program
    rospy.signal_shutdown("Done")