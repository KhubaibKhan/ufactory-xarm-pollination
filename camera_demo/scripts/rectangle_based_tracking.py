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
from utilsm.utils import AzureKinect
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

# create a csv file
data_file = open('/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/data.csv', mode='w')
data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# write column names as depth, size
data_writer.writerow(['size', 'depth', 'depth_error', 'actual_depth'])


# =======================================================================
# ============================ ARDUINO CODE =============================
# from pyfirmata import Arduino, util
# import time
# import pyfirmata


# # define pin connections
# dirPin = 9
# stepPin = 8
# switchPin = 2
# Enpin = 7
# stepsPerRevolution = 250  # Modify this according to your actuator's specifications

# eepromAddress = 0  # Address to store the switch state
# pulseWidth = 600
# NumOfRot = 10

# # connect to the arduino board
# board = Arduino('/dev/ttyACM0')

# print("Connection to the board established...")
# # Set the pin modes
# board.digital[dirPin].mode = pyfirmata.OUTPUT
# board.digital[stepPin].mode = pyfirmata.OUTPUT
# board.digital[switchPin].mode = pyfirmata.INPUT
# board.digital[Enpin].mode = pyfirmata.OUTPUT
# board.digital[Enpin].write(0)  # Set Enable pin LOW initially

# # function to move the motor in the specified direction
# def move_motor(direction):
#     # set motor direction
#     board.digital[dirPin].write(direction)

#     for _ in range(NumOfRot * stepsPerRevolution):
#         print("Step number: ", _)
#         board.digital[stepPin].write(1)
#         # time.sleep(pulseWidth / 2000000.0)
#         board.digital[stepPin].write(0)
#         # time.sleep(pulseWidth / 2000000.0)
#         board.digital[stepPin].write(1)

#     time.sleep(1)
# ============================================================================
# ============================== ARDUINO SETUP END ===========================

# All the possible labels
# labels = {0: '(0, 0)', 1: '(1, 0)', 2: '(-1, 0)', 3: '(-1, -1)', 4: '(0, 1)', 5: '(0, -1)', 6: '(1, 1)', 7: '(1, -1)', 8: '(-1, 1)'}
# labels_reverse = {v:k for k,v in labels.items()}

# Desired orientation in quaternions corresponding to the labels
d_orientations = {'(0, 0)': [0, 0, 0], '(1, 0)': [1.57, 0, 0], '(-1, 0)': [-1.57, 0, 0], '(-1, -1)': [-1.57, 0.78, 0], '(0, 1)': [0, -1.57, 0], '(0, -1)': [0, 1.57, 0], '(1, 1)': [1.57, -0.78, 0], '(1, -1)': [0.78, 1.57, 0], '(-1, 1)': [-0.78, -1.57, 0]}

# Create the publisher
twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1) # Create publisher object

# Image size
image_size = (640, 480)

class BoundingBoxAssociator:
    def __init__(self, distance_threshold=50):
        self.distance_threshold = distance_threshold

    def calculate_distance(self, box1, box2):
        # Calculate Euclidean distance between the center points of two bounding boxes
        center1 = np.array([box1[0], box1[1]])
        center2 = np.array([box2[0], box2[1]])
        distance = np.linalg.norm(center1 - center2)
        # print("===========================================")
        # print(center1, center2)
        # print("===================================================")
        # print(distance)
        # print("=================================================")
        return distance

    def associate_bounding_boxes(self, current_bboxes, previous_bboxes):
        num_current_boxes = len(current_bboxes)
        num_previous_boxes = len(previous_bboxes)
        associations = {}  # Dictionary to store the associations between IDs
        print("\n"*4)

        for i in range(num_current_boxes):
            min_distance = float('inf')
            best_match_id = None

            for j in range(num_previous_boxes):
                distance = self.calculate_distance(current_bboxes[i], previous_bboxes[j])
                if distance < min_distance and distance < self.distance_threshold:
                    min_distance = distance
                    best_match_id = j

            if best_match_id is not None:
                # Associate the current box (i) with the best match from previous_bboxes (best_match_id)
                associations[i] = best_match_id

        return associations



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
    print("Inside main")
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
    starting_joints = [-0.0002942924329545349, -0.2688990533351898, -0.6428003907203674, 0.00041848589899018407, 0.9124296307563782, -0.0014197833370417356]
    initial_position = [0.12740904092788696, -0.5061454772949219, -0.16231562197208405, 0.06283185631036758, -0.8237953782081604, 0.003490658476948738]
    initial_position2 = [0.17664214968681335, -0.17604023218154907, -1.2110868692398071, -1.0321168899536133, 1.580592155456543, 0.24543769657611847]
    initial_position3 = [-0.004821084439754486, -0.1753533035516739, -1.3093904256820679, 0.049373093992471695, 1.4882516860961914, -1.5750622749328613]

    # initial_pose = [0.370, 0.00, 0.400, 0.707, 0, 0, 0.707]


    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    azure = AzureKinect()
    cnts = 0

    while not rospy.is_shutdown():
        rate.sleep()

        # frame = realsense.image
        # if frame is None:
        #     print("No frame")
        #     continue

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

        # wait for input key to start
        input("Press Enter to start the program...") 
        # # close the gripper
        # ret = gripper.close()
        # # wait for gripper to close
        # input("press to open it")
        # ret = gripper.open()
        # # wait for gripper to close
        # time.sleep(1)

        # if ret:
        #     print("Open the gripper successfully!")
        # else:
        #     print("Failed to open the gripper!")
        #     continue

        # continue

        # pid for translational velocity
        kp_t = 0.5
        ki_t = 0.01
        kd_t = 0.001

        error_tra = np.array([0, 0, 0])
        prev_error_tra = np.array([0, 0, 0])
        integral_tra = np.array([0, 0, 0])

        # pid for rotational velocity
        kp_r = 0.2
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
        tolerance = 0.004

        label_list = []
        # previous angle
        previous_angle = [0, 0, 0]
        previous_label = [0, 0, 0]

        # detect boxes
        centroids = detect_color_block(azure.image, color)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if len(centroids) == 0:
            continue

        print("Number of flowers:", len(centroids))

        # Example usage:
        previous_bboxes = centroids.copy()  # List of bounding boxes in the previous frame

        # Create an instance of the BoundingBoxAssociator class
        bbox_associator = BoundingBoxAssociator(distance_threshold=50)

        choose_flower = 0

        start_of_time = time.time()
        while(True):
            print("start")
            start = time.time()
            centroids = detect_color_block(azure.image, color)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Time taken by deep learning model in seconds
            # print("Time taken by deep learning model: ", time.time() - start, "seconds")
            if len(centroids) == 0:
                continue

            current_bboxes = centroids.copy()  # List of bounding boxes in the current frame
            previous_bboxes = centroids.copy()
            # continue
            # Perform bounding box association
            associations = bbox_associator.associate_bounding_boxes(current_bboxes, previous_bboxes)
    
            # Output the associations and update your tracking results accordingly
            print(associations)
            print(associations.values())
            if len(associations) == 0:
                continue
            # assign the previous ids to the current ids
            centroids = [current_bboxes[i] for i in associations.values()]
            
            xc, yc, label, hw, id = centroids[choose_flower]
            # label_reverse = labels_reverse[label]

            # get the ratio between the size of the object and the image
            ratio, size_bbox, ratio_delta = find_ratio(image_size, hw, method='chessboard')
            ratio *= 40
            ratio_delta *= 40

            # label_ind = labels.keys()[labels.values().index(label)]
            # print("Centroid: ", xc, yc, label)
            # if label == 0:
            #     break
            # Moving maximum of the last 30 labels
            label_list.append(label)
            # if label_reverse == 0:
            #     time.sleep(30)
            #     continue
            if len(label_list) > 4:
                label_list.pop(0)
            else:
                continue

            label = max(set(label_list), key=label_list.count)

            # if label_reverse == 0:
            #     break

            # Get the current pose of the end effector
            current_pose = arm._commander.get_current_pose().pose
            # current_pose = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
            # Convert quaternion to euler angles
            current_euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])

            # target_orientation = tf.transformations.quaternion_from_euler(*d_orientations[label_reverse])
            rotation_euler_angles = d_orientations[label]

            # Create rotation objects from the Euler angles
            initial_rotation = Rotation.from_euler('xyz', current_euler)
            rotation_to_apply = Rotation.from_euler('xyz', rotation_euler_angles)

            # Calculate the resulting rotation by multiplying the two rotation matrices
            resulting_rotation = rotation_to_apply.as_matrix().dot(initial_rotation.as_matrix())
            
            # Convert the resulting rotation back to Euler angles
            target_euler = Rotation.from_matrix(resulting_rotation).as_euler('xyz')

            try:
                depth = azure.get_depth(azure.depth, azure.image, xc, yc, hw)
            except Exception as e:
                print(e)
                exit(0)

            data_writer.writerow([size_bbox, ratio, ratio - ratio_delta, depth])
            print("Ratio of the object and depth of the object: ", ratio, depth)
            # continue
            # print("x, y, z before 3d position: ", xc, yc, depth)
            # Get the 3D position of the block
            x, y, z = get_3d_position(depth, xc, yc, azure.camera_info)
            # print("x, y, z after 3d position for depth: ", x, y, z)
            x, y, z = get_3d_position(ratio, xc, yc, azure.camera_info)
            # print("x, y, z after 3d position for ratio: ", x, y, z)

            # Transform 3D position from camera frame to base frame
            # start = time.time()
            x, y, z = transform_3d_position(x, y, z, 'camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)

            # print("time taken: ", time.time() - start)

            # find the camera location in the base frame
            current_position = get_camera_position('camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)

            target_position = [x, y, z]

            # # Find the error between current position and desired position
            pos_error = position_error(current_position, target_position)
            start = time.time()

            # get the transform from base to link_eef
            eef_R = get_transform('link_eef', 'link_base', tf_buffer=tf_buffer)
            eef_R = np.array([eef_R.transform.rotation.x, eef_R.transform.rotation.y, eef_R.transform.rotation.z, eef_R.transform.rotation.w])
            # print("Rotation quaternion eef: ", eef_R)
            # the orientation of the link_base
            base_R = np.array([0, 0, 0, 1])
            # print("Rotation quaternion base: ", base_R)
            # convert the orientation to a rotation matrix
            base_R = Rotation.from_quat(base_R).as_matrix()
            eef_R = Rotation.from_quat(eef_R).as_matrix()
            # get the rotation matrix from link_base to link_eef
            R = eef_R.dot(base_R.T)
            # print("Rotation matrix: ", R)
            # transform the error from base frame to end effector frame
            pos_error = R.dot(pos_error)
            # print("Pose error after transforming: ", pos_error)
            # print("Time taken by translation error: ", time.time() - start, "seconds")

            # if np.linalg.norm(pos_error[-1]) < 0.1: 
            #     pos_error[-1] = 0

            # if np.linalg.norm(pos_error) < tolerance:
            #     print(np.linalg.norm(pos_error) < tolerance)
            #     pos_error = [0, 0, 0]
            
            # convert the error to a numpy array
            pos_error = np.array(pos_error)

            # Convert the Euler angles to rotation matrices
            current_rotation = Rotation.from_euler('xyz', current_euler).as_matrix()
            if previous_label == rotation_euler_angles:
                target_euler = previous_angle
            previous_label = rotation_euler_angles
            previous_angle = target_euler
            target_rotation = Rotation.from_euler('xyz', target_euler).as_matrix()

            # Calculate the matrix difference between the target and current rotations
            rotation_difference = target_rotation.dot(current_rotation.T)

            # Convert the matrix difference back to Euler angles
            error = Rotation.from_matrix(rotation_difference).as_euler('xyz')


            if label == '(0, 0)':
                error = np.array([0, 0, 0])
            
            # Adjust the distance from the flower to 0.06
            pos_error[-1] -= 0.08
            pos_error[0] -= 0.007
            pos_error[1] -= 0.004
            
            print("Position error: ", pos_error)
            print("Total error: ", np.linalg.norm(pos_error) + np.linalg.norm(error))
            if np.linalg.norm(pos_error) + np.linalg.norm(error) < tolerance:
                # time.sleep(20)
                break
            # print("Orientation error: ", error)
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

            # continue

            
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
            twist_stamped.twist.angular.x = 0
            twist_stamped.twist.angular.y = 0
            twist_stamped.twist.angular.z = 0
            # twist_stamped.twist.angular.x = angular_velocity[0]
            # twist_stamped.twist.angular.y = angular_velocity[1]
            # twist_stamped.twist.angular.z = angular_velocity[2]

            twist_pub.publish(twist_stamped)


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


        # publish the twist
        twist_pub.publish(twist_stamped)
        time.sleep(1)

        cnts += 1
        
        if cnts < 0:
            continue

        print("Done")
        # # move the motor forward
        # move_motor(1)
        # # move the motor backward
        # move_motor(0)
        break

    
    # end the program
    rospy.signal_shutdown("Done")