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

    def init_online_learning(self):
        self.W = np.random.rand(self.dimensions, self.dimensions)

    def load_prefitted_model(self):
        self.model = pickle.load(open('%s%s_linear_regression'%(self.base_path, self.name), 'rb'))
        self.effect_cluster = pickle.load(open('%s%s_effect_cluster'%(self.base_path, self.name), 'rb'))
        self.before_scaler = pickle.load(open('%s%s_before_scaler'%(self.base_path, self.name), 'rb'))
        self.effect_scaler = pickle.load(open('%s%s_effect_scaler'%(self.base_path, self.name), 'rb'))
        # self.W = pickle.load(open('%s%s_weights'%(self.base_path, self.name), 'rb'))

    def update_models(self, X, y, epsilon):
        X_pre = minmax_scale(X)
        y_pre = minmax_scale(y)
        X_pre = X_pre[np.newaxis].T
        y_pre = y_pre[np.newaxis].T
        X_pre = np.vstack([X_pre, [1.0]])
        y_pre = np.vstack([y_pre, [0]])
        return self.update_weights(X_pre, y_pre, 0.05, epsilon)

    def update_weights(self, X, y, alpha, epsilon):
        J = self.get_square_error(X, y) / 2
        if J <= epsilon:
            return False
        dJdW = np.matmul(self.W, np.matmul(X, X.T)) - np.matmul(y, X.T)
        self.W -= alpha * dJdW
        return True

    def save_weights(self):
        path = '%s%s_weights' % (self.base_path, self.name)
        pickle.dump(self.W, open(path, 'wb'))

    def get_square_error(self, x, y):
        a = y - np.matmul(self.W, x)
        return np.matmul(a.T, a)[0][0]

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
