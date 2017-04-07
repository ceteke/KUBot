from utils import array_to_pose
from copy import deepcopy
import pickle
import numpy as np
from sklearn.preprocessing import minmax_scale


class MyAction(object):

    def __init__(self,name,robot, base_path='/home/cem/learning/models/'):
        self.robot = robot
        self.name = name
        self.dimensions = 70 # 69 + 1 (bias)
        self.base_path = base_path

    def load_prefitted_model(self):
        self.model = pickle.load(open('%s%s_linear_regression'%(self.base_path, self.name), 'rb'))
        self.effect_cluster = pickle.load(open('%s%s_effect_cluster'%(self.base_path, self.name), 'rb'))

    def prepare(self,pose_arr):
        return self.robot.arm.go_to_pose(array_to_pose(pose_arr))

    def __str__(self):
        return self.name

class Push(MyAction):

    def __init__(self,robot):
        MyAction.__init__(self,'push',robot)

    def execute(self):
        start_pose = self.robot.arm.get_current_pose()
        object_pushed_pose = deepcopy(start_pose)
        object_pushed_pose.position.x += 0.1
        object_pushed_pose.position.y += 0.1
        self.robot.arm.go_to_pose_cartesian([object_pushed_pose])
