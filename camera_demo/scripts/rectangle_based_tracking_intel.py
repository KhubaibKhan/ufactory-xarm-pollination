#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
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
from utilsm.utils import Realsense
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


name = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
# create a csv file
data_file = open(f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/{name}.csv', mode='w')
data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
# write column names as depth, size
data_writer.writerow(['size', 'depth', 'depth_error', 'actual_depth', 'tr_velocity', 'rot_velocity', '3dPositions', 'current_orientation', 'target_position', 'target_orientation', 'pos_error', 'or_error', 'total_error', 'time', 'hw'])


# make a dir
os.mkdir(f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/success_image/{name}')
# create a video writer
create_videoWriter(name)

# =======================================================================
# ============================ ARDUINO CODE =============================
# from pyfirmata import Arduino, util
# import time
# import pyfirmata


# # # define pin connections
# dirPin = 9
# stepPin = 8
# switchPin = 2
# Enpin = 7
# stepsPerRevolution = 250  # Modify this according to your actuator's specifications

# eepromAddress = 0  # Address to store the switch statesuccessful
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

# class PIDController:
#     def __init__(self, Kp, Ki, Kd, dt):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.dt = dt
#         self.prev_error = 0
#         self.integral = 0

#     def compute(self, setpoint, current_value):
#         error = setpoint - current_value
        
#         # Proportional term
#         P = self.Kp * error
        
#         # Integral term
#         self.integral += error * self.dt
#         I = self.Ki * self.integral
        
#         # Derivative term
#         D = self.Kd * (error - self.prev_error) / self.dt
#         self.prev_error = error
        
#         # Calculate control output
#         control_output = P + I + D
        
#         return control_output



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
     # Get current joint values
    current_joints = arm._commander.get_current_joint_values()
    print("Current joint values: ", current_joints)
    # exit(0)

    # Define an initial pose
    starting_joints = [0.19913798570632935, -0.18164342641830444, -0.23626525700092316, -0.5351921916007996, 0.24083471298217773, 0.6678767800331116]
    starting_joints4 = [0.21440628170967102, -0.3760954439640045, -0.5886629819869995, -0.024925874546170235, 0.8663324117660522, 0.2064579576253891]
    starting_joints2 = [0.5147331953048706, -0.6630587577819824, -1.4484703540802002, -0.19422030448913574, 1.9430080652236938, 0.3878381848335266]
    starting_joints3 = [0.5118288397789001, -0.6988098621368408, -1.360672116279602, -0.19130946695804596, 1.9157021045684814, 0.3925255537033081]
    initial_position = [0.12740904092788696, -0.5061454772949219, -0.16231562197208405, 0.06283185631036758, -0.8237953782081604, 0.003490658476948738]
    initial_position2 = [0.17664214968681335, -0.17604023218154907, -1.2110868692398071, -1.0321168899536133, 1.580592155456543, 0.24543769657611847]
    initial_position3 = [-0.004821084439754486, -0.1753533035516739, -1.3093904256820679, 0.049373093992471695, 1.4882516860961914, -1.5750622749328613]

    # initial_pose = [0.370, 0.00, 0.400, 0.707, 0, 0, 0.707]


    # color yellow
    color = (0, 255, 255)
    color = (0, 0, 255)
    realsense = Realsense()
    cnts = 0

    while not rospy.is_shutdown():
        rate.sleep()

        data_writer.writerow([f'size{cnts}', f'depth{cnts}', f'depth_error{cnts}', f'actual_depth{cnts}', f'tr_velocity{cnts}', f'rot_velocity{cnts}', f'3dPositions{cnts}', f'current_orientation{cnts}', f'target_position{cnts}', f'target_orientation{cnts}', f'pos_error{cnts}', f'or_error{cnts}', f'total_error{cnts}', f'time{cnts}', f'hw{cnts}'])

        # frame = realsense.image
        # if frame is None:
        #     print("No frame")
        #     continue

        # Go to the initial pose
        ret = arm.set_joints(starting_joints3, wait=True)
        # ret = arm.moveto(x=initial_pose[0], y=initial_pose[1], z=initial_pose[2], ox=initial_pose[3], oy=initial_pose[4], oz=initial_pose[5], ow=initial_pose[6], wait=True, relative=False)
        if ret:
            print("Go to the initial pose successfully!")
        else:
            print("Failed to go to the initial pose!")
            continue
        # # Open the gripper
        # ret = gripper.open()

        # wait for input key to start
        # input("Press Enter to start the program...") 
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
        kp_t = 1.4
        ki_t = 0.01
        kd_t = 0.001
        dt = 0.01

        # pid_tr = PIDController(kp_t, ki_t, kd_t, dt)

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
        dt_tr = 0.01

        # tolerance for translational error
        tolerance = 0.006

        label_list = []
        # previous angle
        previous_angle = [0, 0, 0]
        previous_label = [0, 0, 0]
        previous_target_rotation = Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
        # detect boxes
        centroids = detect_color_block(realsense.image, realsense.depth, color, realsense.camera_info)
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
            rgb_image = realsense.image
            depth_image = realsense.depth
            centroids = detect_color_block(rgb_image, depth_image, color, realsense.camera_info)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # Time taken by deep learning model in seconds
            # print("Time taken by deep learning model: ", time.time() - start, "seconds")
            if len(centroids) == 0:
                continue
            # continue


            current_bboxes = centroids.copy()  # List of bounding boxes in the current frame
            previous_bboxes = centroids.copy()
            # continue
            # Perform bounding box association
            associations = bbox_associator.associate_bounding_boxes(current_bboxes, previous_bboxes)
    
            # Output the associations and update your tracking results accordingly
            # print(associations)
            # print(associations.values())
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
            if len(label_list) > 10:
                label_list.pop(0)
            else:
                continue

            label = max(set(label_list), key=label_list.count)

            # if label_reverse == 0: 
            #     break

            # current camera pose
            # camera_pose = get_camera_pose('camera_color_optical_frame', 'link_base', tf_buffer=tf_buffer)
            # print("camera pose:", camera_pose)
            # Get the current pose of the end effector
            current_pose = arm._commander.get_current_pose().pose
            
            # current_pose = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
            # Convert quaternion to euler angles
            current_euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
            # print("robot pose:", current_euler)
            # continue

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
                depth = realsense.get_depth(depth_image, rgb_image, xc, yc, hw)
            except Exception as e:
                print(e)
                exit(0)

            print("Ratio of the object and depth of the object: ", ratio, depth)
            # continue
            # print("x, y, z before 3d position: ", xc, yc, depth)
            
            # print("x, y, z after 3d position for depth: ", x, y, z)
            x, y, z = get_3d_position(ratio, xc, yc, realsense.camera_info)
            # Get the 3D position of the block
            x, y, z = get_3d_position(depth, xc, yc, realsense.camera_info)
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
                target_rotation = previous_target_rotation
            else:
                prev_error_rot = [0, 0, 0]
                target_rotation = Rotation.from_euler('xyz', target_euler).as_matrix()
            previous_label = rotation_euler_angles
            previous_angle = target_euler
            previous_target_rotation = target_rotation
            
            # target_rotation = Rotation.from_euler('xyz', target_euler).as_matrix()

            # Calculate the matrix difference between the target and current rotations
            rotation_difference = target_rotation.dot(current_rotation.T)

            # Convert the matrix difference back to Euler angles
            error = Rotation.from_matrix(rotation_difference).as_euler('xyz')


            if label == '(0, 0)':
                error = np.array([0, 0, 0])
            
            # Adjust the distance from the flower to 0.08
            pos_error[-1] -= 0.07
            # pos_error[0] -= 0.009
            # pos_error[1] -= 0.02
            
            print("Position error: ", pos_error)
            print("Rotational error: ", error)
            # print("Total error: ", np.linalg.norm(pos_error) + np.linalg.norm(error))
            if np.linalg.norm(pos_error) + np.linalg.norm(error) < tolerance:
                data_writer.writerow([size_bbox, ratio, ratio - ratio_delta, depth, vel, angular_velocity, current_position, current_euler, target_position, target_euler, pos_error, error, np.linalg.norm(pos_error) + np.linalg.norm(error), time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())), hw])
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

            prev_error_tra = pos_error

            # PID for orientation control
            # compute the integral of the error using the trapezoidal rule
            integral_rot = integral_rot + 0.5 * (error + prev_error_rot) * dt

            # compute the derivative of the error using the backward difference method
            derivative = (error - prev_error_rot) / dt

            # compute the PID control signal
            control_signal_rot = kp_r * error + ki_r * integral_rot + kd_r * derivative

            prev_error_rot = error
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

            print("translational velocity:", vel)
            print("Angular velocity:", angular_velocity)

            data_writer.writerow([size_bbox, ratio, ratio - ratio_delta, depth, vel, angular_velocity, current_position, current_euler, target_position, target_euler, pos_error, error, np.linalg.norm(pos_error) + np.linalg.norm(error), time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())), hw])

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

            print("One loop time: ", str(time.time() - start))
            
            
        
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

        print("Done")
        # move the motor forward
        # move_motor(1)

        # save success_image
        save_success_image(realsense.image, f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/success_image/{name}/' + str(time.time()) + '.png')
        # success_image = azure.image
        # # convert to cv2_image
        # success_image = azure_image_to_cv2(success_image)
        # # save this image in the folder
        # cv2.imwrite(f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/success_image/{name}/' + str(time.time()) + '.png', success_image)
        # input("Press enter to start.....")
        # move the motor backward
        # move_motor(0)

        # if cnts < 5:
        # input("Press enter to start....................")
        
        if cnts < 10:
            continue

        ret = arm.set_joints(starting_joints3, wait=True)
        # ret = arm.moveto(x=initial_pose[0], y=initial_pose[1], z=initial_pose[2], ox=initial_pose[3], oy=initial_pose[4], oz=initial_pose[5], ow=initial_pose[6], wait=True, relative=False)
        if ret:
            print("Go to the initial pose successfully!")
        else:
            print("Failed to go to the initial pose!")
            continue

        break

    
    # end the program
    rospy.signal_shutdown("Done")