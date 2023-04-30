#!/usr/bin/env python
# -*- coding: utf-8 -*-

# A demo to control xarm6 to find the current position

import rospy
# from xarm_api import xarm_api
from xarm_msgs.srv import *
from xarm_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import moveit_commander


# Get position using moveit commander
def moveit_position():
    # Initialize moveit commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create a xarm6 commander
    xarm6 = moveit_commander.MoveGroupCommander("xarm6")

    # Get current position
    current_position = xarm6.get_current_pose().pose
    print("Current position: ", current_position)

    # Get current joint states
    current_joint_states = xarm6.get_current_joint_values()
    print("Current joint states: ", current_joint_states)

    # Get current pose
    current_pose = xarm6.get_current_pose()
    print("Current pose: ", current_pose)

    # Get current end effector force
    current_force = xarm6.get_end_effector_force()
    print("Current end effector force: ", current_force)

    # Get current end effector link
    current_link = xarm6.get_end_effector_link()
    print("Current end effector link: ", current_link)

    # Get current end effector pose
    current_end_effector_pose = xarm6.get_end_effector_pose()
    print("Current end effector pose: ", current_end_effector_pose)

    # Get current end effector twist
    current_end_effector_twist = xarm6.get_end_effector_twist()
    print("Current end effector twist: ", current_end_effector_twist)

    # Get current end effector wrench
    current_end_effector_wrench = xarm6.get_end_effector_wrench()
    print("Current end effector wrench: ", current_end_effector_wrench)

    # Get current end effector velocity
    current_end_effector_velocity = xarm6.get_end_effector_velocity()
    print("Current end effector velocity: ", current_end_effector_velocity)

    # Get current joint value target
    current_joint_value_target = xarm6.get_joint_value_target()
    print("Current joint value target: ", current_joint_value_target)

    # Get current pose target
    current_pose_target = xarm6.get_pose_target()
    print("Current pose target: ", current_pose_target)

    # Get current robot state
    current_robot_state = xarm6.get_current_state()
    print("Current robot state: ", current_robot_state)


# Go to a specific position using moveit commander
def moveit_move(x, y, z, ox, oy, oz, w):
    # Initialize moveit commander
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    xarm6 = moveit_commander.MoveGroupCommander("xarm6")

    # Set planning time to 10 seconds
    xarm6.set_planning_time(1000.0)
    # xarm6.set_goal_orientation_tolerance(0.1)
    # # Create a xarm6 commander
    # xarm6 = moveit_commander.MoveGroupCommander("xarm6")

    # Get current position
    current_position = xarm6.get_current_pose().pose
    print("Current position: ", current_position)

    current_position.position.x = x
    current_position.position.y = y
    current_position.position.z =z
    current_position.orientation.x = 1
    current_position.orientation.y = 0
    current_position.orientation.z = 0
    current_position.orientation.w = 0

    # Move to a specific position
    xarm6.set_pose_target(current_position)
    xarm6.go(wait=True)

    # Go to a specific position
    # xarm6.set_position_target([x, y, z])
    # xarm6.go(wait=True)

    # # Go to a specific joint state
    # xarm6.set_joint_value_target([0, 0, 0, 0, 0, 0])
    # xarm6.go(wait=True)

    # # Go to a specific pose
    # xarm6.set_pose_target([0.3, 0.2, 0.3, 0, 0, 0])
    # xarm6.go(wait=True)


# Position of the xarm in the world frame
def xarm_position():
    xarm_api = xarm_api.XArmAPI('/xarm6')
    xarm_api.motion_enable(enable=True)
    xarm_api.set_mode(0)
    xarm_api.set_state(state=0)

    # Get current position
    x, y, z, roll, pitch, yaw = xarm_api.get_position()
    print("Current position: ", x, y, z, roll, pitch, yaw)

    # Get current joint states
    joint_states = xarm_api.get_joint_states()
    print("Current joint states: ", joint_states)

    # Get current pose
    pose = xarm_api.get_pose()
    print("Current pose: ", pose)

    # Get current end effector force
    force = xarm_api.get_force()

# Move xarm to a specific position
def xarm_move(x, y, z):
    xarm_api = xarm_api.XArmAPI('/xarm6')
    xarm_api.motion_enable(enable=True)
    xarm_api.set_mode(0)
    xarm_api.set_state(state=0)

        # Get current position
    x, y, z, roll, pitch, yaw = xarm_api.get_position()
    print("Current position: ", x, y, z, roll, pitch, yaw)

    # Move to a specific position
    xarm_api.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, wait=True)

    # # Move to a specific joint state
    # xarm_api.set_joint(joint=[0, 0, 0, 0, 0, 0], wait=True)

    # # Move to a specific pose
    # xarm_api.set_pose(pose=[0.3, 0.2, 0.3, 0, 0, 0], wait=True)
    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('xarm_position')


    try:
        # moveit_move(0.37, 0.00, 0.60, 1, 0, 0, 0)
        moveit_position()
    except rospy.ROSInterruptException:
        pass

# Path: xarm_vision/camera_demo/scripts/xarm_position.py