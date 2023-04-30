#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from tf2_geometry_msgs import *
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import moveit_commander
import numpy as np
import cv2
import threading

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

# Realsense class to get aligned image and depth image
class Realsense:
    def __init__(self):
        # Subscribe to aligned image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Subscribe to aligned depth image topic
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

        # Subscribe to camera info topic
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # camera info
        self.camera_info = None

        # Global image
        self.image = None
        self.depth = None

    # Function that gets depth from depth image
    def get_depth(self, depth, xc, yc):
        # Convert ROS image to OpenCV image
        cv_depth = cv_bridge.imgmsg_to_cv2(depth, 'passthrough')

        # Return depth
        return cv_depth[yc, xc]
    
    # Function that gets image from image topic
    def image_callback(self, image):
        self.image = image

    # Function that gets depth from depth topic
    def depth_callback(self, depth):
        self.depth = depth

    # Function that gets camera info from camera info topic
    def camera_info_callback(self, camera_info):
        self.camera_info = camera_info

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

    # Return centroid
    return xc, yc

# Using image and depth find the 3D position of the block
def get_3d_position(depth, xc, yc, camera_info=None):
    # Get camera intrinsics
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    print("Camera info", camera_info.K)

    # Get depth
    z = depth

    # Get 3D position
    x = (xc - cx) * z / fx
    y = (yc - cy) * z / fy

    # Return 3D position
    return x, y, z

# Transform 3d position from camera frame to base frame
def transform_3d_position(x, y, z, camera_frame, base_frame):
    # Initialize transform
    transform = None

    # Wait for transform
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform(base_frame, camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # Create point
    point = PointStamped()
    point.header.frame_id = camera_frame
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = z

    # Transform point
    transformed_point = do_transform_point(point, transform)

    # Return transformed point
    return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z

# thread to move arm to block
def move_arm_to_block(x, y, z):
    # Set target position
    xarm_commander.set_position_target([x, y, z, 0, 0, 0])

    # Plan motion
    plan = xarm_commander.plan()

    # Execute motion
    xarm_commander.execute(plan)

# Main function
if __name__ == '__main__':
    # Initialize Realsense class
    realsense = Realsense()

    # Define color
    color = (0, 0, 255)

    # Define rate
    rate = rospy.Rate(10)

    # Loop until ROS is shutdown
    while not rospy.is_shutdown():
        # If image is not None
        if realsense.image is not None:
            # Detect color block
            xc, yc = detect_color_block(realsense.image, color)

            # Convert ROS image to OpenCV image
            cv_image = cv_bridge.imgmsg_to_cv2(realsense.image, 'bgr8')

            # Convert ROS depth image to OpenCV image
            cv_depth = cv_bridge.imgmsg_to_cv2(realsense.depth, 'passthrough')

            # Draw circle on image
            cv2.circle(cv_image, (xc, yc), 5, (0, 0, 255), -1)

            # Show image
            cv2.imshow('Image', cv_image)


            # Get depth
            depth = realsense.get_depth(realsense.depth, xc, yc)
            print("Image coordinate point", xc, yc, depth)
            # Get the 3D position of the block
            x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)
            print("3D position of the block", x, y, z)

            # Transform 3D position from camera frame to base frame
            x, y, z = transform_3d_position(x, y, z, 'camera_color_optical_frame', 'link_base')
            print("Transformed 3D position of the block", x, y, z)
            
            # Run the moving command in a different thread
            t = threading.Thread(target=move_arm_to_block, args=(x, y, z))
            t.start()


            # Use transformed point to control robot arm
            # Get the current pose target
            pose_target = xarm_commander.get_current_pose().pose
            pose_target.position.x = x
            pose_target.position.y = y
            # Set the new pose target
            xarm_commander.set_pose_target(pose_target)
            # Plan the motion
            # plan = xarm_commander.plan()
            # Execute the motion
            xarm_commander.go(wait=True)

            # Wait for key press
            cv2.waitKey(1)
            # Print depth
            print(depth)

        # Sleep for 1/rate seconds
        rate.sleep()