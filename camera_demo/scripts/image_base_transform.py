#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from tf2_geometry_msgs import *
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import moveit_commander
import numpy as np
import cv2

# Global image
IMAGE = None


# Initialize ROS node
rospy.init_node('color_block_recognition')

# Initialize TF buffer and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Initialize OpenCV bridge
cv_bridge = CvBridge()

# Moveit arm commander
xarm_commander = moveit_commander.move_group.MoveGroupCommander('xarm6')

# Moveit gripper commander
gripper_commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')

# Function that detects color block in image
def detect_color_block(image, color):
    # Convert ROS image to OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')

    # Convert image to HSV format
    cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define color range
    color_lower = (color[0] - 10, 100, 100)
    color_upper = (color[0] + 10, 255, 255)

    # Create mask for color range
    mask = cv2.inRange(cv_image_hsv, color_lower, color_upper)

    # Find contours in mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find largest contour in mask
    contour = max(contours, key=cv2.contourArea)

    # find the xmin, ymin, xmax, ymax of the contour
    x, y, w, h = cv2.boundingRect(contour)

    # Find centroid of largest contour
    M = cv2.moments(contour)
    xc = int(M['m10'] / M['m00'])
    yc = int(M['m01'] / M['m00'])

    # Draw bounding box around color block
    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    # Draw centroid of color block
    cv2.circle(cv_image, (xc, yc), 5, (0, 0, 255), -1)

    global IMAGE
    IMAGE = cv_image

    # Return image coordinates of color block
    return (x, y, w, h)

# Define image callback function
def image_callback(image):

    # Color block red
    color = (0, 0, 255)

    # Detect color block in image
    x, y, w, h = detect_color_block(image, color)

    # find the center from xmin, ymin, xmax, ymax
    x = x + w/2
    y = y + h/2

    # Convert the pixel coordinates to the camera frame
    K = np.array([904.2245483398438, 0.0, 631.6015014648438, 0.0, 904.0277099609375, 362.0469665527344, 0.0, 0.0, 1.0])
    D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    K = K.reshape(3, 3)
    D = D.reshape(5, 1)
    R = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    R = R.reshape(3, 3)
    P = np.array([904.2245483398438, 0.0, 631.6015014648438, 0.0, 0.0, 904.0277099609375, 362.0469665527344, 0.0, 0.0, 0.0, 1.0, 0.0])
    P = P.reshape(3, 4)

    # Get the point in the camera frame
    point_camera = cv2.undistortPoints(np.array([[[x, y]]]), K, D, R=R, P=P)

    rospy.logerr(point_camera)
    # show the detected 
    cv2.imshow('Color block detection', IMAGE)
    cv2.waitKey(1)

    rospy.loginfo('Color block coordinates in image: ({}, {})'.format(x, y))

    # Create PointStamped object in camera frame
    point_camera = PointStamped()
    point_camera.header = image.header
    point_camera.point.x = x
    point_camera.point.y = y
    point_camera.point.z = 0

    # Transform point from camera frame to robot base frame
    try:
        # wait for transform to become available
        wait = tf_buffer.can_transform('link_base', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
        transform = tf_buffer.lookup_transform('link_base', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
        point_robot = tf2_geometry_msgs.do_transform_point(point_camera, transform)
        rospy.loginfo('Point coordinates in robot base frame: ({}, {})'.format(point_robot.point.x*1000, point_robot.point.y*1000))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        # log the detailed error details
        rospy.logerr("Exception: %s", e.message if e.message else "Failed to transform point from camera frame to robot base frame")
        return

    # Use transformed point to control robot arm
    # Get the current pose target
    # pose_target = xarm_commander.get_current_pose().pose
    # pose_target.position.x = point_robot.point.x
    # pose_target.position.y = point_robot.point.y
    # # Set the new pose target
    # xarm_commander.set_pose_target(pose_target)
    # # Plan the motion
    # # plan = xarm_commander.plan()
    # # Execute the motion
    # xarm_commander.go(wait=True)

# Subscribe to image topic
image_sub = rospy.Subscriber('/camera/align_depth_to_color/image_raw', Image, image_callback)
# Spin ROS node
rospy.spin()
