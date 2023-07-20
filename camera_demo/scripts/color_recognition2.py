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
import transforms3d as tf3d
import csv

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

# Image size
image_size = (640, 480)


# get the calibration data from the yaml file
def get_calibration_data(calibration_file):
    '''
    Get the calibration data from the yaml file
    Yaml file has the following format:
        qw: 0.7013088518485089
        qx: 0.0039751934245023735
        qy: -0.003477682492098677
        qz: 0.7128379885223908
        x: 0.0629845508606165
        y: -0.03221690881661964
        z: 0.019403200790438897
    '''

    with open(calibration_file) as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        calibration_data = yaml.load(file, Loader=yaml.FullLoader)

    # Get the camera matrix and distortion coefficients


def position_error(current_position, target_position):
    """
    Computes the position error between two positions.
    :param current_position: current position (3D vector)
    :param target_position: target position (3D vector)
    :return: position error as a 3D vector
    """
    
    # If the target position x, y and z are in a certain range, then the error is 0
    # if np.linalg.norm(np.array(target_position) - np.array(current_position)) < 0.3:
    #     return np.array([0, 0, 0])
    # else:
    #     return np.array(target_position) - np.array(current_position)

    # target_position[-2] = target_position[-2]*-1
    error = np.array([target_position[0] - current_position[0], target_position[1] - current_position[1], target_position[2] - current_position[2]])
    # error[1] = error[1] * -1
    return error

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
    initial_position = [0.12740904092788696, -0.5061454772949219, -0.16231562197208405, 0.06283185631036758, -0.8237953782081604, 0.003490658476948738]
    initial_position2 = [0.17664214968681335, -0.17604023218154907, -1.2110868692398071, -1.0321168899536133, 1.580592155456543, 0.24543769657611847]
    initial_position3 = [-0.004821084439754486, -0.1753533035516739, -1.3093904256820679, 0.049373093992471695, 1.4882516860961914, -1.5750622749328613]

    # initial_pose = [0.370, 0.00, 0.400, 0.707, 0, 0, 0.707]


    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    realsense = Realsense()
    cnts = 0

    # Store depth values for every successfull loop
    depth_success = []
    # x, y, z with real depth
    xyz_depth = []
    # x, y, z with size depth
    xyz_size = []
    # Store error values
    error_or = []
    error_pos = []
    # camera position change
    camera_pos = []
    # time
    time_list = []
    # translational and rotational velocities
    vel_tra = []
    vel_rot = []
    # translational and rotational error separate
    error_tra_sep = []
    error_rot_sep = []

    # store time
    time_1_success = []

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
        # # Open the gripper
        # ret = gripper.open()

        # # wait for input key to start
        # input("Press Enter to start the program...") 
        # # close the gripper
        # ret = gripper.close()
        # # wait for gripper to close
        # time.sleep(1)

        # if ret:
        #     print("Open the gripper successfully!")
        # else:
        #     print("Failed to open the gripper!")
        #     continue

        # Current pose of the end effector
        current_pose = arm._commander.get_current_pose().pose
        print("Current pose of the robot: ", current_pose)

        # Get current joint values
        current_joints = arm._commander.get_current_joint_values()
        print("Current joint values: ", current_joints)

        # pid for translational velocity
        kp_t = 0.3
        ki_t = 0.01
        kd_t = 0.001

        error_tra = np.array([0, 0, 0])
        prev_error_tra = np.array([0, 0, 0])
        integral_tra = np.array([0, 0, 0])

        # pid for rotational velocity
        kp_r = 0.3
        ki_r = 0.01
        kd_r = 0.001

        error_rot = np.array([0, 0, 0])
        prev_error_rot = np.array([0, 0, 0])
        integral_rot = np.array([0, 0, 0])

        # time step
        dt = 0.01
        # time step for translational
        dt_tr = 0.001

        # tolerance for translational error
        tolerance = 0.002

        label_list = []
        error_list = []
        # abs(error[0]) < 3 and abs(error[1]) < 3 and abs(error[2]) < 3

        # x, y, z with real depth
        xyz_depth_1 = []
        # x, y, z with size depth
        xyz_size_1 = []
        # Store error values
        error_or_1 = []
        error_pos_1 = []
        # camera position change
        camera_pos_1 = []
        # record time
        time_1 = []
        # translational and rotational velocities
        vel_tra_1 = []
        vel_rot_1 = []
        # translational and rotational error separate
        error_tra_sep_1 = []
        error_rot_sep_1 = []

        start_of_time = time.time()
        while(True):
            print("start")
            start = time.time()
            centroids = detect_color_block(realsense.image, color)
            # Time taken by deep learning model in seconds
            print("Time taken by deep learning model: ", time.time() - start, "seconds")
            if len(centroids) == 0:
                continue
            print(centroids)
            xc, yc, label, hw = centroids[0]
            label_reverse = labels_reverse[label]

            # get the ratio between the size of the object and the image
            ratio = find_ratio(image_size, hw)
            ratio *= 4

            # label_ind = labels.keys()[labels.values().index(label)]
            # print("Centroid: ", xc, yc, label)
            # if label == 0:
            #     break
            # Moving maximum of the last 30 labels
            label_list.append(label_reverse)
            # if label_reverse == 0:
            #     time.sleep(30)
            #     continue
            if len(label_list) > 4:
                label_list.pop(0)
            else:
                continue

            label_reverse = max(set(label_list), key=label_list.count)

            # if label_reverse == 0:
            #     break

            # Get the current pose of the end effector
            current_pose = arm._commander.get_current_pose().pose

            try:
                depth = realsense.get_depth(realsense.depth, realsense.image, xc, yc, hw)
            except Exception as e:
                print(e)
                exit(0)

            print("Ratio of the object and depth of the object: ", ratio, depth)

            print("x, y, z before 3d position: ", xc, yc, depth)
            # Get the 3D position of the block
            x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)
            xyz_depth_1.append([x, y, z])
            print("x, y, z after 3d position for depth: ", x, y, z)
            x, y, z = get_3d_position(ratio, xc, yc, realsense.camera_info)
            print("x, y, z after 3d position for ratio: ", x, y, z)
            # append the values to the list
            xyz_size_1.append([x, y, z])

            # Get the transformation from link eef to camera frame
            current_position = [x, y, z]

            # target position calculation
            target_position = [0, 0, 0]

            # # Find the error between current position and desired position
            pos_error = position_error(target_position, current_position)
            # transform the error to the link base
            eef_R = get_transform('link_eef', 'camera_color_frame', tf_buffer=tf_buffer)
            eef_R = np.array([eef_R.transform.rotation.x, eef_R.transform.rotation.y, eef_R.transform.rotation.z, eef_R.transform.rotation.w])
            eef_R = Rotation.from_quat(eef_R).as_matrix()
            pos_error = eef_R.dot(pos_error)
            # Error between current and target position
            target_pose = [0, 0, 0]
            pos_error[-1] -= 0.06
            print("Translational error: ", pos_error)

            # Convert the Euler angles to rotation matrices
            current_pose = d_orientations[label_reverse]
            current_rotation = Rotation.from_euler('xyz', current_pose).as_matrix()
            target_rotation = Rotation.from_euler('xyz', target_pose).as_matrix()

            # Calculate the matrix difference between the target and current rotations
            rotation_difference = current_rotation.dot(target_rotation.T)

            # Convert the matrix difference back to Euler angles
            error = Rotation.from_matrix(rotation_difference).as_euler('xyz')
            # Adjust the distance from the flower to 0.06
            print("Rotational error: ", error)
            

            
            # append seprate errors to the list
            error_tra_sep_1.append(pos_error)
            error_rot_sep_1.append(error)
            # append the pos error and orientation error to the list
            error_pos_1.append(np.linalg.norm(pos_error))
            error_or_1.append(np.linalg.norm(error))
            if np.linalg.norm(pos_error) + np.linalg.norm(error) < tolerance:
                depth_success.append([depth])
                time_1_success.append([time.time() - start_of_time])
                break
            # PID for positional control
            # compute the integral of the error using the trapezoidal rule
            integral_tra = integral_tra + 0.5 * (pos_error + prev_error_tra) * dt_tr

            # compute the derivative of the error using the backward difference method
            derivative = (pos_error - prev_error_tra) / dt_tr

            # compute the PID control signal
            control_signal_tra = kp_t * pos_error + ki_t * integral_tra + kd_t * derivative

            # PID for orientation control
            # compute the integral of the error using the trapezoidal rule
            integral_rot = integral_rot + 0.5 * (error + prev_error_rot) * dt

            # compute the derivative of the error using the backward difference method
            derivative = (error - prev_error_rot) / dt

            # compute the PID control signal
            control_signal_rot = kp_r * error + ki_r * integral_rot + kd_r * derivative

            
            # control velocity
            vel = control_signal_tra
            angular_velocity = control_signal_rot

            # appnend to the list
            vel_tra.append(vel)
            vel_rot.append(angular_velocity)

            # Publish the angular velocity to /servo_server/delta_twist_cmds topic
            twist_stamped = TwistStamped() # Create TwistStamped message object
            twist_stamped.header.stamp = rospy.Time.now()
            twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            twist_stamped.twist.linear.x = 0.07
            twist_stamped.twist.linear.y = 0
            twist_stamped.twist.linear.z = 0
            twist_stamped.twist.angular.x = 0
            twist_stamped.twist.angular.y = 0
            twist_stamped.twist.angular.z = 0
            # twist_stamped.twist.angular.x = angular_velocity[0]
            # twist_stamped.twist.angular.y = angular_velocity[1]
            # twist_stamped.twist.angular.z = angular_velocity[2]

            twist_pub.publish(twist_stamped)
            break


            # twist_stamped = TwistStamped() # Create TwistStamped message object
            # twist_stamped.header.stamp = rospy.Time.now()
            # twist_stamped.header.frame_id = "link_eef"  # set to the desired frame ID

            # twist_stamped.twist.linear.x = vel[0]
            # twist_stamped.twist.linear.y = vel[1]
            # twist_stamped.twist.linear.z = vel[2]
            # # twist_stamped.twist.angular.x = 0
            # # twist_stamped.twist.angular.y = 0
            # # twist_stamped.twist.angular.z = 0
            # twist_stamped.twist.angular.x = angular_velocity[0]
            # twist_stamped.twist.angular.y = angular_velocity[1]
            # twist_stamped.twist.angular.z = angular_velocity[2]

            # twist_pub.publish(twist_stamped)
            
        time.sleep(5)
        exit(0)
        
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

        # append all the values to the list
        error_pos.append(error_pos_1)
        error_or.append(error_or_1)

        xyz_depth.append(xyz_depth_1)
        xyz_size.append(xyz_size_1)

        camera_pos.append(camera_pos_1)
        time_list.append(time_1)
        vel_tra.append(vel_tra_1)
        vel_rot.append(vel_rot_1)
        error_tra_sep.append(error_tra_sep_1)
        error_rot_sep.append(error_rot_sep_1)


        # publish the twist
        twist_pub.publish(twist_stamped)
        time.sleep(3)

        cnts += 1
        
        if cnts < 0:
            continue
        # save all the data to the csv file
        with open('error_pos.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(error_pos)

        with open('error_or.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(error_or)

        with open('xyz_depth.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(xyz_depth)

        with open('xyz_size.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(xyz_size)

        with open('depth_success.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(depth_success)
        
        with open('time_success.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(time_1_success)

        with open('camera_position.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(camera_pos)
        
        with open('time.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(time_list)
        
        with open('vel_tra.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(vel_tra)

        with open('vel_rot.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(vel_rot)

        with open('error_tra_sep.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(error_tra_sep)

        with open('error_rot_sep.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(error_rot_sep)


        print("Done")
        break

    
    # end the program
    rospy.signal_shutdown("Done")