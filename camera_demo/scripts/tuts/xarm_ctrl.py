import math
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf.transformations

import numpy as np
from math import acos, sin


def mid_quartnion(q1, q3, t):
    dot = q1[0]*q3[0] + q1[1]*q3[1] + q1[2]*q3[2] + q1[3]*q3[3]
    if dot < 0:
        q3 = (-q3[0], -q3[1], -q3[2], -q3[3])
        dot = -dot
    angle = acos(dot)
    sin_angle = np.sqrt(1 - dot**2)
    if sin_angle < 1e-5:
        
        q2 = ((1-t)*q1[0]+t*q3[0], (1-t)*q1[1]+t*q3[1],
              (1-t)*q1[2]+t*q3[2], (1-t)*q1[3]+t*q3[3])
    else:
        q2 = (sin((1-t)*angle)*q1[0]/sin_angle + sin(t*angle)*q3[0]/sin_angle,
              sin((1-t)*angle)*q1[1]/sin_angle + sin(t*angle)*q3[1]/sin_angle,
              sin((1-t)*angle)*q1[2]/sin_angle + sin(t*angle)*q3[2]/sin_angle,
              sin((1-t)*angle)*q1[3]/sin_angle + sin(t*angle)*q3[3]/sin_angle)
   
    return np.asarray(q2) / np.sqrt(q2[0]**2 + q2[1]**2 + q2[2]**2 + q2[3]**2)

class XArmCtrl(object):
    def __init__(self, dof):
        self._commander = moveit_commander.move_group.MoveGroupCommander('xarm{}'.format(dof))
        self._init()
    
    def _init(self):
        # self._commander.set_max_acceleration_scaling_factor(0.5)
        # self._commander.set_max_velocity_scaling_factor(0.5)
        pass

    def change_orientation(self, orientation, wait=True):
        try:
            # Set the end effector orientation to the current orientation
            self._commander.set_start_state_to_current_state()
            current_pose = self._commander.get_current_pose().pose
            current_orientation = current_pose.orientation

            # Rotate the end effector using orientation

            # for i in range(3):
            #     q[i] /= 1
            # q[1] /= 2
            q = orientation
            q = mid_quartnion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w], q, 0.5)
            new_orientation = geometry_msgs.msg.Quaternion()
            new_orientation.x = q[0]
            new_orientation.y = q[1]
            new_orientation.z = q[2]
            new_orientation.w = q[3]
            print(new_orientation)
            orientation_tuple = (new_orientation.x, new_orientation.y, new_orientation.z, new_orientation.w)
            # self._commander.set_orientation_target(orientation_tuple)
            current_pose.orientation = new_orientation
            self._commander.set_pose_target(current_pose)
            # Plan and execute the motion
            ret = self._commander.go(wait=wait)
            # plan = self._commander.plan(wait=wait)
            # ret = self._commander.execute(plan)

            return ret
        except Exception as e:
            print('[Ex] change_orientation exception, ex={}'.format(e))
        return False


    def set_joints(self, angles, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            for i in range(len(joint_target)):
                if i >= len(angles):
                    break
                if angles[i] is not None:
                    # joint_target[i] = math.radians(angles[i])
                    joint_target[i] = angles[i]
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joints exception, ex={}'.format(e))
    
    def set_joint(self, angle, inx=-1, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            joint_target[inx] = math.radians(angle)
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joint exception, ex={}'.format(e))
        return False

    def moveto(self, x=None, y=None, z=None, ox=None, oy=None, oz=None, ow=None, relative=False, wait=True):
        if x == 0 and y == 0 and z == 0 and ox == 0 and oy == 0 and oz == 0 and relative:
            return True
        try:
            pose_target = self._commander.get_current_pose().pose
            if relative:
                pose_target.position.x += x / 1000.0 if x is not None else 0
                pose_target.position.y += y / 1000.0 if y is not None else 0
                pose_target.position.z += z / 1000.0 if z is not None else 0
                pose_target.orientation.x += ox if ox is not None else 0
                pose_target.orientation.y += oy if oy is not None else 0
                pose_target.orientation.z += oz if oz is not None else 0
            else:
                pose_target.position.x = x if x is not None else pose_target.position.x
                pose_target.position.y = y if y is not None else pose_target.position.y
                pose_target.position.z = z if z is not None else pose_target.position.z
                pose_target.orientation.x = ox if ox is not None else pose_target.orientation.x
                pose_target.orientation.y = oy if oy is not None else pose_target.orientation.y
                pose_target.orientation.z = oz if oz is not None else pose_target.orientation.z
                pose_target.orientation.w = ow if ow is not None else pose_target.orientation.w
            print('move to position=[{:.2f}, {:.2f}, {:.2f}], orientation=[{:.6f}, {:.6f}, {:.6f}]'.format(
                pose_target.position.x, pose_target.position.y, pose_target.position.z,
                pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z
            ))
            self._commander.set_pose_target(pose_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] moveto exception: {}'.format(e))
        return False