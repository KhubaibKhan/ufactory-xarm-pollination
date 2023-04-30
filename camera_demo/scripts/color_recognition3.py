#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import cv2
import rospy
import rospy
from tf2_geometry_msgs import *
import cv2
import numpy as np
from tuts.gripper_ctrl import GripperCtrl
from tuts.xarm_ctrl import XArmCtrl
from utilsm.utils import *
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

warnings.filterwarnings("ignore")

if sys.version_info < (3, 0):
    PY3 = False
    import Queue as queue
else:
    PY3 = True
    import queue

# All the possible labels
labels = {0: '(0, 0)', 1: '(1, 0)', 2: '(-1, 0)', 3: '(-1, -1)', 4: '(0, 1)', 5: '(0, -1)', 6: '(1, 1)', 7: '(1, -1)', 8: '(-1, 1)'}
labels_reverse = {v:k for k,v in labels.items()}

# Desired orientation in quaternions corresponding to the labels
d_orientations = {0: [0, 0, 0], 1: [1.57, 0, 0], 2: [-1.57, 0, 0], 3: [-1.57, 0.78, 0], 4: [0, -1.57, 0], 5: [0, 1.57, 0], 6: [1.57, -0.78, 0], 7: [0.78, 1.57, 0], 8: [-0.78, -1.57, 0]}

# Create the publisher
twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10) # Create publisher object

# Get the desired end orientation
def desired_orientation(orientation, arm):
    current_pose = arm._commander.get_current_pose().pose
    current_orientation = current_pose.orientation

    # Rotate the end effector using orientation
    q = tf.transformations.quaternion_multiply(
        [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w],
        orientation)
    new_orientation = geometry_msgs.msg.Quaternion()
    new_orientation.x = q[0]
    new_orientation.y = q[1]
    new_orientation.z = q[2]
    new_orientation.w = q[3]

    orientation_tuple = (new_orientation.x, new_orientation.y, new_orientation.z, new_orientation.w)
    return orientation_tuple

# Compute the quaternion error
def quaternion_error(q_current, q_desired):
    """
    Computes the quaternion error between two orientations.
    :param q_current: current orientation quaternion (4D vector)
    :param q_desired: desired orientation quaternion (4D vector)
    :return: quaternion error as a 3D vector
    """
    # normalize the quaternions
    q_current = q_current / np.linalg.norm(q_current)
    q_desired = q_desired / np.linalg.norm(q_desired)
    
    # compute the difference quaternion
    q_error = np.quaternion(*q_current) * np.quaternion(*q_desired).conjugate()
    
    # convert the difference quaternion to a 3D vector
    return np.array([q_error.x, q_error.y, q_error.z])


if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)

    # Initialize OpenCV bridge
    dof = rospy.get_param('/xarm/DOF')

    rate = rospy.Rate(10.0)
    # motion_que.put([318, 50, 461, -2.96, -0.314, -0.27])

    # Initialize TF buffer and listener
    global tf_buffer
    global tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Define the arm control and gripper control
    arm = XArmCtrl(6)
    gripper = GripperCtrl()

    # Define an initial pose
    starting_joints = [-0.00025873768026940525, -0.31601497530937195, -1.3658411502838135, 0.00034579072962515056, 1.6826152801513672, -0.0012134681455790997]
    # initial_pose = [0.370, 0.00, 0.400, 0.707, 0, 0, 0.707]


    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    realsense = Realsense()
    cnts = 0
    while not rospy.is_shutdown():
        rate.sleep()

        frame = realsense.image
        if frame is None:
            print("No frame")
            continue

        # Go to the initial pose
        ret = arm.set_joints(starting_joints, wait=True)
        # ret = arm.moveto(x=initial_pose[0], y=initial_pose[1], z=initial_pose[2], ox=initial_pose[3], oy=initial_pose[4], oz=initial_pose[5], ow=initial_pose[6], wait=True, relative=False)
        if ret:
            print("Go to the initial pose successfully!")
        else:
            print("Failed to go to the initial pose!")
            continue
        # Open the gripper
        ret = gripper.open()
        if ret:
            print("Open the gripper successfully!")
        else:
            print("Failed to open the gripper!")
            continue

        # Current pose of the end effector
        current_pose = arm._commander.get_current_pose().pose
        print("Current pose of the robot: ", current_pose)

        # Get current joint values
        current_joints = arm._commander.get_current_joint_values()
        print("Current joint values: ", current_joints)

        # centroids = detect_color_block(realsense.image, color)
        # if len(centroids) == 0:
        #     continue
        # print(centroids)
        # xc, yc, label = centroids[0]
        # # label_ind = labels.keys()[labels.values().index(label)]
        # print("Centroid: ", xc, yc, label)
        # key = [key for key, value in labels.items() if value == label]
        # print("Index: ", key)

        # # Convert ROS image to OpenCV image
        # cv_image = cv_bridge.imgmsg_to_cv2(realsense.image, 'bgr8')

        # # Get depth
        # depth = realsense.get_depth(realsense.depth, xc, yc)
        # # Get the 3D position of the block
        # x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)

        # # Transform 3D position from camera frame to base frame
        # x, y, z = transform_3d_position(x, y, z, 'camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)

        # # Target pose of the end effector in pose stamped
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = 'link_base'
        # target_pose.pose.position.x = x
        # target_pose.pose.position.y = y
        # target_pose.pose.position.z = z
        # target_pose.pose.orientation.x = 1
        # target_pose.pose.orientation.y = 0
        # target_pose.pose.orientation.z = 0
        # target_pose.pose.orientation.w = 1

        # Change the orientation of the end-effector by 90 degrees
        # target_orientation = tf.transformations.quaternion_from_euler(-1.57, 0, 0)
        # target_orientation = [target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w]
        # target_orientation = [-1.57, 0, 0]
        # orientation = [0.707, 0, 0, 0.707]
        # print("Orientation: ", target_orientation)

        # # Get the desired orientation
        # d_orient = desired_orientation(orientation, arm=arm)
        # # convert this to quaternion
        # d_orient = geometry_msgs.msg.Quaternion(*d_orient)
        # print("Desired orientation: ", d_orient)
        
        # current_pose = arm._commander.get_current_pose().pose
        # current_orientation = current_pose.orientation
        # q = tf.transformations.quaternion_multiply(
        # [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w],
        # orientation)

        # for i in range(10):
        #     ret = arm.change_orientation(orientation, wait=True)
        #     if ret:
        #         print("Change orientation successfully!")
        #     else:
        #         print("Failed to change orientation!")

        kp = 0.3
        error = [-1, -1, -1]
        last_error = [0, 0, 0]
        label_list = []
        # abs(error[0]) < 3 and abs(error[1]) < 3 and abs(error[2]) < 3
        while(True):
            print("start")
            centroids = detect_color_block(realsense.image, color)
            if len(centroids) == 0:
                continue
            print(centroids)
            xc, yc, label = centroids[0]
            label_reverse = labels_reverse[label]

            # label_ind = labels.keys()[labels.values().index(label)]
            print("Centroid: ", xc, yc, label)
            # if label == 0:
            #     break
            # Moving maximum of the last 30 labels
            label_list.append(label_reverse)
            if len(label_list) > 10:
                label_list.pop(0)
            else:
                continue
            label_reverse = max(set(label_list), key=label_list.count)

            if label_reverse == 0:
                break
            print("Label: ", label_reverse)

            # Get the current pose of the end effector
            current_pose = arm._commander.get_current_pose().pose
            # current_pose = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
            # Convert quaternion to euler angles
            current_euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])

            # target_orientation = tf.transformations.quaternion_from_euler(*d_orientations[label_reverse])
            rotation_euler_angles = d_orientations[label_reverse]

            # Create rotation objects from the Euler angles
            initial_rotation = Rotation.from_euler('xyz', current_euler)
            rotation_to_apply = Rotation.from_euler('xyz', rotation_euler_angles)

            # Calculate the resulting rotation by multiplying the two rotation matrices
            resulting_rotation = rotation_to_apply.as_matrix().dot(initial_rotation.as_matrix())
            
            # Convert the resulting rotation back to Euler angles
            target_euler = Rotation.from_matrix(resulting_rotation).as_euler('xyz')

            print("Current euler: ", current_euler)
            print("target euler: ", target_euler)
            # Calculate quaternion difference
            # quat_diff = Rotation.from_quat(current_pose) * Rotation.from_quat(target_orientation).inv()

            # # Convert quaternion difference to Euler angles
            # error = quat_diff.as_euler('xyz')
            # last_error = error

            # # Get depth
            # depth = realsense.get_depth(realsense.depth, xc, yc)
            # # Get the 3D position of the block
            # x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)

            # # Transform 3D position from camera frame to base frame
            # x, y, z = transform_3d_position(x, y, z, 'camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)

            # current_position = arm._commander.get_current_pose().pose
            # target_position = [x, y, z]

            # # Find the error between current position and desired position
            # pos_error = [target_position[0] - current_position.position.x, target_position[1] - current_position.position.y, target_position[2] - current_position.position.z]

            # # find the velocity from pos_error
            # vel = [kp * pos_error[0], kp * pos_error[1], kp * pos_error[2]]

            # print("Error: ", error)

            # find the error between current orientation and desired orientation
            # error = [target_orientation[0] - current_euler[0], target_orientation[1] - current_euler[1], target_orientation[2] - current_euler[2]]
            # error = [current_euler[0] - target_orientation[0], current_euler[1] - target_orientation[1], current_euler[2] - target_orientation[2]]

            # Convert the Euler angles to rotation matrices
            current_rotation = Rotation.from_euler('xyz', current_euler).as_matrix()
            target_rotation = Rotation.from_euler('xyz', target_euler).as_matrix()

            # Calculate the matrix difference between the target and current rotations
            rotation_difference = target_rotation.dot(current_rotation.T)

            # Convert the matrix difference back to Euler angles
            error = Rotation.from_matrix(rotation_difference).as_euler('xyz')


            # Convert the error to angular velocity
            angular_velocity = [error[0] * kp, error[1] * kp, error[2] * kp]
            print("Angular velocity: ", angular_velocity)

            # Publish the angular velocity to /servo_server/delta_twist_cmds topic
            twist_stamped = TwistStamped() # Create TwistStamped message object
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            twist_stamped.twist.linear.x = 0
            twist_stamped.twist.linear.y = 0
            twist_stamped.twist.linear.z = 0
            twist_stamped.twist.angular.x = angular_velocity[0]
            twist_stamped.twist.angular.y = angular_velocity[1]
            twist_stamped.twist.angular.z = angular_velocity[2]

            twist_pub.publish(twist_stamped)

            

            # # Get the current pose of the end effector
            # current_pose = arm._commander.get_current_pose().pose
            # # convert the quaternion to euler angles
            # current_euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
            # # compute the error between current and desired euler angles
            # error = [target_orientation[i] - current_euler[i] for i in range(3)]

            # # compute error integral and derivative
            # error_sum += error
            # error_diff = error - last_error
            # last_error = error

            # # compute the control signal using PID formula
            # p_signal = [Kp * error[i] for i in range(3)]
            # i_signal = [Ki * error_sum[i] for i in range(3)]
            # d_signal = [Kd * error_diff[i] for i in range(3)]

            # control_signals = [p_signal[i] + i_signal[i] + d_signal[i] for i in range(3)]

            # # Publish the angular velocity to /servo_server/delta_twist_cmds topic
            # twist_stamped = TwistStamped() # Create TwistStamped message object
            # twist_stamped.header.stamp = rospy.Time.now()
            # twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            # twist_stamped.twist.linear.x = 0
            # twist_stamped.twist.linear.y = 0
            # twist_stamped.twist.linear.z = 0
            # twist_stamped.twist.angular.x = control_signals[0]
            # twist_stamped.twist.angular.y = control_signals[1]
            # twist_stamped.twist.angular.z = control_signals[2]

            # twist_pub.publish(twist_stamped)
            # rate.sleep()
            
            
        
        # Publish the angular velocity to /servo_server/delta_twist_cmds topic
        twist_stamped = TwistStamped() # Create TwistStamped message object
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

        twist_stamped.twist.linear.x = 0
        twist_stamped.twist.linear.y = 0
        twist_stamped.twist.linear.z = 0
        twist_stamped.twist.angular.x = 0
        twist_stamped.twist.angular.y = 0
        twist_stamped.twist.angular.z = 0

        twist_pub.publish(twist_stamped)

        # # Convert quaternion to euler angles
        # current_euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])

        # # find the error between current orientation and desired orientation
        # error = [target_orientation[0] - current_euler[0], target_orientation[1] - current_euler[1], target_orientation[2] - current_euler[2]]
        
        # # Convert the error to angular velocity
        # kp = 0.5
        # angular_velocity = [error[0] * kp, error[1] * kp, error[2] * kp]

        # # Publish the angular velocity to /servo_server/delta_twist_cmds topic
        # twist = Twist()
        # twist.linear.x = 0
        # twist.linear.y = 0
        # twist.linear.z = 0
        # twist.angular.x = angular_velocity[0]
        # twist.angular.y = angular_velocity[1]
        # twist.angular.z = angular_velocity[2]

        # twist_pub.publish(twist)

        # Move the end effector using moveit
        # arm.move_velocity(desired_velocity[0], desired_velocity[1], desired_velocity[2], desired_velocity[3], desired_velocity[4], desired_velocity[5], wait=False)
        
        # sleep for 10 second
        time.sleep(2)

        # Move the end effector using moveit
        # arm.move_velocity(desired_velocity[0], desired_velocity[1], desired_velocity[2], desired_velocity[3], desired_velocity[4], desired_velocity[5], wait=False)
        
        cnts += 1
        
        if cnts < 50:
            continue
        