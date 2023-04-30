#! /usr/bin/env python

# Moving robot using moveit_python

import rospy
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes

rospy.init_node("moveit_python_test", anonymous=True)
move_group = MoveGroupInterface("xarm6", "world_joint")

joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# current_joint_values = move_group.get_current_joint_values()

# current_joint_values[1] = -0.9

current_joint_values = [0.12718342244625092, -0.9992650747299194, -0.15316841006278992, 0.06295859813690186, -0.8201352953910828, 0.0034742257557809353]

while not rospy.is_shutdown():
    result = move_group.moveToJointPosition(joints, current_joint_values, 0.02)
    if result:

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Hooray, success!")
            break
        else:
            rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
            rospy.sleep(1.0)
    else:
        rospy.logerr("MoveIt! failure no result returned.")
        rospy.sleep(1.0)

rospy.loginfo("Done.")

move_group.get_move_action().cancel_all_goals() 