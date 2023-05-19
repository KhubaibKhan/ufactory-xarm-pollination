#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1) # Create publisher object

# Image dimensions
image_width = 640
image_height = 480

def position_error(centroid):
    """
    calculate the position error between the centroid and the center of the image
    """
    # Get the centroid
    xc, yc, _ = centroid

    # Get the error
    x_error = xc - image_width / 2
    y_error = yc - image_height / 2

    # Return the error
    return x_error, y_error


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
    current_pose = arm._commander.get_current_pose().pose
    print("Current pose of the robot: ", current_pose)

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

        # pid for translational velocity
        kp_t = 0.2
        ki_t = 0.01
        kd_t = 0.01

        error_tra = np.array([0, 0, 0])
        prev_error_tra = np.array([0, 0, 0])
        integral_tra = np.array([0, 0, 0])

        # pid for rotational velocity
        kp_r = 0.1
        ki_r = 0.01
        kd_r = 0.001

        error_rot = np.array([0, 0, 0])
        prev_error_rot = np.array([0, 0, 0])
        integral_rot = np.array([0, 0, 0])

        # # Whole pid control
        # kp = 0.1 # proportional
        # ki = 0.01 # integral
        # kd = 0.001 # derivative

        # time step
        dt = 0.01

        # # initialize the error and integral
        # error = np.array([0, 0, 0, 0, 0, 0])
        # prev_error = np.array([0, 0, 0, 0, 0, 0])
        # integral = np.array([0, 0, 0, 0, 0, 0])

        # tolerance for translational error
        tolerance = 0.04

        label_list = []
        error_list = []
        # abs(error[0]) < 3 and abs(error[1]) < 3 and abs(error[2]) < 3
        while(True):
            print("start")
            start = time.time()
            centroids, hw_list = detect_color_block(realsense.image, color)
            print("Time taken: ", time.time() - start)
            if len(centroids) == 0:
                continue
            print(centroids)
            xc, yc, label = centroids[0]
            hw = hw_list[0]
            label_reverse = labels_reverse[label]

            # label_ind = labels.keys()[labels.values().index(label)]
            # print("Centroid: ", xc, yc, label)
            # if label == 0:
            #     break
            # Moving maximum of the last 30 labels
            label_list.append(label_reverse)
            if len(label_list) > 10:
                label_list.pop(0)
            else:
                continue
            label_reverse = max(set(label_list), key=label_list.count)

            # if label_reverse == 0:
            #     break

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

            try:
                depth = realsense.get_depth(realsense.depth, xc, yc)
            except Exception as e:
                print(e)
                exit(0)

            print("Get the 3d position.")

            # Current position of the end effector
            current_position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            print("current position: ", current_position)
            # get the current position of the camera
            current_position = find_camera_location(current_position, tf_buffer)

            print("Current position: ", current_position)
            # # Find the error between current position and desired position
            pos_error = position_error(centroids[0])

            # pos_error = np.array(target_position) - np.array(current_position)
            print("Position error: ", pos_error)

            if np.linalg.norm(pos_error) < tolerance:
                pos_error = [0, 0, 0]

            # convert the error to a numpy array
            pos_error = np.array(pos_error)

            # Convert the Euler angles to rotation matrices
            current_rotation = Rotation.from_euler('xyz', current_euler).as_matrix()
            target_rotation = Rotation.from_euler('xyz', target_euler).as_matrix()

            # Calculate the matrix difference between the target and current rotations
            rotation_difference = target_rotation.dot(current_rotation.T)

            # Convert the matrix difference back to Euler angles
            error = Rotation.from_matrix(rotation_difference).as_euler('xyz')
            print("Orientation error: ", error)
            if label_reverse == 0:
                error = np.array([0, 0, 0])

            # PID for positional control
            # compute the integral of the error using the trapezoidal rule
            integral_tra = integral_tra + 0.5 * (pos_error + prev_error_tra) * dt

            # compute the derivative of the error using the backward difference method
            derivative = (pos_error - prev_error_tra) / dt

            # compute the PID control signal
            control_signal_tra = kp_t * pos_error + ki_t * integral_tra + kd_t * derivative

            # PID for orientation control
            # compute the integral of the error using the trapezoidal rule
            integral_rot = integral_rot + 0.5 * (error + prev_error_rot) * dt

            # compute the derivative of the error using the backward difference method
            derivative = (error - prev_error_rot) / dt

            # compute the PID control signal
            control_signal_rot = kp_r * error + ki_r * integral_rot + kd_r * derivative


            # # combine pos error and orientation error in a list
            # error = [pos_error[0], pos_error[1], pos_error[2], error[0], error[1], error[2]]

            # # convert error into numpy array
            # error = np.array(error)

            # # compute the integral of the error using the trapezoidal rule
            # integral = integral + 0.5 * (error + prev_error) * dt

            # # compute the derivative of the error using the backward difference method
            # derivative = (error - prev_error) / dt

            # # compute the PID control signal
            # control_signal = kp * error + ki * integral + kd * derivative
            
            # control velocity
            vel = control_signal_tra
            angular_velocity = control_signal_rot

            # print("Translational velocity: ", vel)

            # Publish the angular velocity to /servo_server/delta_twist_cmds topic
            twist_stamped = TwistStamped() # Create TwistStamped message object
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            twist_stamped.twist.linear.x = vel[0]
            twist_stamped.twist.linear.y = vel[1]
            twist_stamped.twist.linear.z = vel[2]
            # twist_stamped.twist.angular.x = 0
            # twist_stamped.twist.angular.y = 0
            # twist_stamped.twist.angular.z = 0
            twist_stamped.twist.angular.x = angular_velocity[0]
            twist_stamped.twist.angular.y = angular_velocity[1]
            twist_stamped.twist.angular.z = angular_velocity[2]

            twist_pub.publish(twist_stamped)
            
            
        
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

        time.sleep(2)

        cnts += 1
        
        if cnts < 50:
            continue
        