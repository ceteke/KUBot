from utils import array_to_pose
from copy import deepcopy
import pickle
import numpy as np
from transformer import Transformer
from geometry_msgs.msg import Quaternion

class MyAction(object):

    def __init__(self,name,robot, orientation, base_path='/home/cem/learning/models/'):
        self.robot = robot
        self.name = name
        self.dimensions = 70 # 69 + 1 (bias)
        self.base_path = base_path
        self.orientation = orientation
        self.transformer = Transformer(self.robot)

    def load_batch_models(self):
        self.model = pickle.load(open('%s%s_linear_regression'%(self.base_path, self.name), 'rb'))
        self.effect_cluster = pickle.load(open('%s%s_effect_cluster'%(self.base_path, self.name), 'rb'))

    def __str__(self):
        return self.name

class Push(MyAction):

    def __init__(self,robot):
        orientation = Quaternion()
        orientation.x = -0.636984559527
        orientation.y = -0.249563909878
        orientation.z = 0.245820513626
        orientation.w = 0.686688285098
        MyAction.__init__(self,'push',robot,orientation)

    def prepare(self,before_feats,obj_name):
        pose = self.transformer.transform(self, before_feats)
        pose_clone = deepcopy(pose)
        pose.position.x -= 0.125
        pose.position.y -= 0.075
        obj_size = (before_feats[4]+0.5, before_feats[5]+0.5, before_feats[6]+0.5)
        return self.robot.arm.go_to_object(pose, obj_name, pose_clone, obj_size)

    def execute(self):
        start_pose = self.robot.arm.get_current_pose()
        object_pushed_pose = deepcopy(start_pose)
        object_pushed_pose.position.x += 0.1
        object_pushed_pose.position.y += 0.1
        return self.robot.arm.go_to_pose_cartesian([object_pushed_pose])
