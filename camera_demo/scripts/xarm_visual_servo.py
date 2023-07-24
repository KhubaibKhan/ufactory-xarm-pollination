#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import yaml
import time
import rospy
from tf2_geometry_msgs import *
from geometry_msgs.msg import TwistStamped


twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1) # Create publisher object

if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)
    i = 0
    rate = rospy.Rate(10.0)

    start = time.time()
    while not rospy.is_shutdown():
        if i < 10:
            print("published")
            # Publish the angular velocity to /servo_server/delta_twist_cmds topic
            twist_stamped = TwistStamped() # Create TwistStamped message object
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            twist_stamped.twist.linear.x = 0
            twist_stamped.twist.linear.y = 0
            twist_stamped.twist.linear.z = -0.3
            twist_stamped.twist.angular.x = 0
            twist_stamped.twist.angular.y = 0
            twist_stamped.twist.angular.z = 0

            # publish the twist
            twist_pub.publish(twist_stamped)
        else:
            print("Total time: ", start - time.time())
        i+=1
        rate.sleep()
    
    # end the program
    rospy.signal_shutdown("Done")