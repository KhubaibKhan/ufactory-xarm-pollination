import moveit_commander

class GripperCtrl(object):
    def __init__(self):
        self._commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
        self._init()

    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    
    def open(self, wait=True):
        try:
            self._commander.set_named_target('open')
            ret = self._commander.go(wait=wait)
            print('gripper_open, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper open exception, {}'.format(e))
        return False

    def close(self, wait=True):
        try:
            self._commander.set_named_target('close')
            ret = self._commander.go(wait=wait)
            print('gripper_close, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper close exception, {}'.format(e))
        return False
