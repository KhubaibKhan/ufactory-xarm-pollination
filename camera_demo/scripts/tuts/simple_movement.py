#! /usr/bin/env_python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_turotial", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("xarm6")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


current_joint_values = group.get_current_joint_values()
print(current_joint_values)
target_joint_values = current_joint_values.copy()
target_joint_values[1] = -0.5
group.set_joint_value_target(target_joint_values)
plan1 = group.plan()
print(plan1)
rospy.sleep(5)
group.go(wait=True)
moveit_commander.roscpp_shutdown()