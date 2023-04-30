#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

# Initialize node
rospy.init_node('publish_transform')

# Get transformation parameters from the parameter server
qw = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/qw')
qx = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/qx')
qy = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/qy')
qz = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/qz')
x = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/x')
y = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/y')
z = rospy.get_param('/xarm_realsense_handeyecalibration_eye_on_hand/transformation/z')

# Create transformation
transform = geometry_msgs.msg.TransformStamped()
transform.header.frame_id = 'tracking_base_frame'
transform.child_frame_id = 'robot_base_frame'
transform.transform.translation.x = x
transform.transform.translation.y = y
transform.transform.translation.z = z
transform.transform.rotation.w = qw
transform.transform.rotation.x = qx
transform.transform.rotation.y = qy
transform.transform.rotation.z = qz

# Initialize tf2 broadcaster
tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

# Broadcast transformation
tf_broadcaster.sendTransform(transform)
