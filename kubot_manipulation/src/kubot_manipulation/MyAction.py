from utils import array_to_pose
from copy import deepcopy

class MyAction(object):

    def __init__(self,name,robot):
        self.robot = robot
        self.name = name

class Push(MyAction):

    def __init__(self,robot):
        MyAction.__init__(self,'push',robot)

    def prepare(self,pose_arr):
        return self.robot.arm.go_to_pose(array_to_pose(pose_arr))

    def execute(self):
        start_pose = self.robot.arm.get_current_pose()
        object_pushed_pose = deepcopy(start_pose)
        object_pushed_pose.position.x += 0.1
        object_pushed_pose.position.y += 0.1
        self.robot.arm.go_to_pose_cartesian([object_pushed_pose])
