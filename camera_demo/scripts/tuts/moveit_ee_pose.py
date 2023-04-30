#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


rospy.init_node("moveit_python_tutorial", anonymous=True)

move_group = MoveGroupInterface("xarm6", "world")
gripper_frame = 'xarm_gripper_base_link'

gripper_poses = [Pose(Point(0.809, 0.0, 1.358), Quaternion(0.01, 0.0, 0.0, 1.0)),
                 Pose(Point(0.809, 0.0, 1.136), Quaternion(0.01, 0.0, 0.0, 1.0))]

gripper_pose_stamped = PoseStamped()
gripper_pose_stamped.header.frame_id = 'world'

while not rospy.is_shutdown():
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()

        gripper_pose_stamped.pose = pose

        result = move_group.moveToPose(gripper_pose_stamped, gripper_frame)

        if result:

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Trajectory successfully executed!")
            else:
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()