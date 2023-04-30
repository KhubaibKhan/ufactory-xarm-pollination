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
from gripper_ctrl import GripperCtrl
from xarm_ctrl import XArmCtrl
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

# A set of different motions for the robot arm servo
# angular [x, y, z, xo, yo, zo]

# Move forward direction and then reduce the speed no rotation of the end effector
motion1 = [[0.2, 0.0, 0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

# Move forward and then move in the opposite direction with the same speed
motion2 = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

# Move forward, then move in the y direction and then move in the z direction
motion3 = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]]

# Move in a random direction and random orientaion three times
motion4 = [[]]

motions_all = [motion1, motion2, motion3, motion4]

# Create the publisher
twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10) # Create publisher object

if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)

    # Initialize OpenCV bridge
    dof = rospy.get_param('/xarm/DOF')

    rate = rospy.Rate(10.0)
    # motion_que.put([318, 50, 461, -2.96, -0.314, -0.27])

    # Define the arm control and gripper control
    arm = XArmCtrl(6)
    gripper = GripperCtrl()

    # Define an initial pose
    starting_joints = [-0.00025873768026940525, -0.31601497530937195, -1.3658411502838135, 0.00034579072962515056, 1.6826152801513672, -0.0012134681455790997]
    # initial_pose = [0.370, 0.00, 0.400, 0.707, 0, 0, 0.707]

    while not rospy.is_shutdown():
        rate.sleep()

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

        # Start a timer
        start_time = time.time()

        i = 1
        motions = motions_all[i]
        while(True):
            if time.time() - start_time < 2:
                print(time.time()-start_time)
                print(motions[0])
                # Publish the angular velocity to /servo_server/delta_twist_cmds topic
                twist_stamped = TwistStamped() # Create TwistStamped message object
                twist_stamped.header.stamp = rospy.Time.now()
                twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

                twist_stamped.twist.linear.x = motions[0][0]
                twist_stamped.twist.linear.y = motions[0][1]
                twist_stamped.twist.linear.z = motions[0][2]
                twist_stamped.twist.angular.x = motions[0][3]
                twist_stamped.twist.angular.y = motions[0][4]
                twist_stamped.twist.angular.z = motions[0][5]

                twist_pub.publish(twist_stamped)
            
            elif(time.time() - start_time < 6 and time.time() - start_time > 2):
                print(time.time()-start_time)
                print(motions[1])
                # Publish the angular velocity to /servo_server/delta_twist_cmds topic
                twist_stamped = TwistStamped() # Create TwistStamped message object
                twist_stamped.header.stamp = rospy.Time.now()
                twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

                twist_stamped.twist.linear.x = motions[1][0]
                twist_stamped.twist.linear.y = motions[1][1]
                twist_stamped.twist.linear.z = motions[1][2]
                twist_stamped.twist.angular.x = motions[1][3]
                twist_stamped.twist.angular.y = motions[1][4]
                twist_stamped.twist.angular.z = motions[1][5]

                twist_pub.publish(twist_stamped)
            
            elif(time.time() - start_time < 10 and time.time() - start_time > 6):
                print(time.time()-start_time)
                print(motions[2])
                # Publish the angular velocity to /servo_server/delta_twist_cmds topic
                twist_stamped = TwistStamped() # Create TwistStamped message object
                twist_stamped.header.stamp = rospy.Time.now()
                twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

                twist_stamped.twist.linear.x = motions[2][0]
                twist_stamped.twist.linear.y = motions[2][1]
                twist_stamped.twist.linear.z = motions[2][2]
                twist_stamped.twist.angular.x = motions[2][3]
                twist_stamped.twist.angular.y = motions[2][4]
                twist_stamped.twist.angular.z = motions[2][5]

                twist_pub.publish(twist_stamped)
            
            else:
                break



            rate.sleep()

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

        # exit the program after this
        exit()

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
        time.sleep(10)

        # Move the end effector using moveit
        # arm.move_velocity(desired_velocity[0], desired_velocity[1], desired_velocity[2], desired_velocity[3], desired_velocity[4], desired_velocity[5], wait=False)
        
        cnts += 1
        
        if cnts < 50:
            continue
        