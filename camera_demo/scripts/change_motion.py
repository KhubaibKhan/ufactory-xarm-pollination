#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
# import moveit_commander
# from moveit_msgs.msg import RobotTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint

# # Initialize MoveIt
# rospy.init_node('moveit_demo')
# robot = moveit_commander.RobotCommander()
# group = moveit_commander.MoveGroupCommander('xarm6')

# # Plan and execute first trajectory
# group.set_named_target('home')
# plan1 = group.plan()
# group.execute(plan1)

# # Send new motion command during execution of first trajectory
# joint_names = plan1.joint_trajectory.joint_names
# point = JointTrajectoryPoint()
# point.positions = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
# point.time_from_start = rospy.Duration.from_sec(2.0)
# trajectory = RobotTrajectory()
# trajectory.joint_trajectory.joint_names = joint_names
# trajectory.joint_trajectory.points.append(point)
# group.execute(trajectory)

# # Plan and execute second trajectory
# group.set_named_target('up')
# plan2 = group.plan()
# group.execute(plan2)



import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Initialize MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_example')

# Set up MoveGroupCommander for the arm
group = moveit_commander.MoveGroupCommander('arm')

# Set the target pose for the arm
group.set_pose_target([x, y, z, qx, qy, qz, qw])

# Plan and execute the trajectory to the target pose
plan = group.go(wait=True)

# Create a new trajectory to move the arm to a different pose
new_trajectory = RobotTrajectory()
new_trajectory.joint_trajectory.joint_names = plan.joint_trajectory.joint_names

# Set the starting point of the new trajectory to the current joint positions
new_point = JointTrajectoryPoint()
new_point.positions = group.get_current_joint_values()
new_trajectory.joint_trajectory.points.append(new_point)

# Set the target joint positions for the new trajectory
new_point = JointTrajectoryPoint()
new_point.positions = [j1, j2, j3, j4, j5, j6]
new_point.time_from_start = rospy.Duration(5.0)
new_trajectory.joint_trajectory.points.append(new_point)

# Send the new trajectory to the robot controller to interrupt the current trajectory
group.execute(new_trajectory, wait=True)

# Clean up
moveit_commander.roscpp_shutdown()


# import rospy
# import moveit_commander
# from moveit_msgs.msg import RobotTrajectory

# # Initialize MoveIt
# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('moveit_example')

# # Set up MoveGroupCommander for the arm
# group = moveit_commander.MoveGroupCommander('arm')

# # Set the target pose for the arm
# group.set_pose_target([x, y, z, qx, qy, qz, qw])

# # Plan and execute the trajectory to the target pose
# plan1 = group.go(wait=True)

# # Set the target pose for the arm again
# group.set_pose_target([x2, y2, z2, qx2, qy2, qz2, qw2])

# # Plan a new trajectory to the new target pose
# plan2 = group.plan()

# # Execute the new trajectory
# if plan2:
#     group.execute(plan2)

# # Clean up
# moveit_commander.roscpp_shutdown()
