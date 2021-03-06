import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from random import randint
from kubot_manipulation.MyAction import Push
import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
from pc_segmentation.msg import PcFeatures
from utils import pc_features_to_array
import csv
import os
import pickle
import numpy as np
from models import ActionModel
from sklearn.preprocessing import minmax_scale
from errors import IterationError

class AffordanceCore:

    def __init__(self, feature_size = 51, run_id = None, models_path='/home/cem/learning/models/'):
        self.robot = Robot()
        if run_id is None:
            self.run_id = self.get_run_id()
        else:
            self.run_id = run_id
        self.push = Push(self.robot)
        self.actions = [self.push]
        self.action_models = [ActionModel(self.push, self.run_id)]
        self.load_scalers()

        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/media/cem/ROSDATA/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/media/cem/ROSDATA/ros_data/features'

        self.iteration_num = 0

        self.models_path = models_path

        self.feature_size = feature_size

        self.clusters = {0: "Stay", 1:"Roll", 6: "Roll", 5:"Drop", 9:"Drop", 14:"Drop"}

    def get_cluster_label(self, cid):
        return self.clusters[cid]

    def load_scalers(self):
        for am in self.action_models:
            am.load_scalers()

    def move_to_object(self, action_model):
        try:
            before_feats = self.get_features()
        except rospy.exceptions.ROSException as e:
            raise IterationError("Waiting feature topic timed out.")
        if before_feats is None:
            raise IterationError("Object out of scope.")
        if action_model.action.prepare(before_feats) == -1:
            raise IterationError("No plan found.")
        return before_feats

    def execute_action(self, action_model):
        is_gone = False
        after_feats = []
        if action_model.action.execute() == -1:
            raise IterationError("Cartesian path is not good enough")
        rospy.sleep(5)
        try:
            after_feats = self.get_features()
            if after_feats is not None: #When object dropped from table but still visible
                if after_feats[2] > 1.0:
                    print "Object dropped"
                    is_gone = True
            else: # segmented blue part 5 times
                print "Missing object"
                is_gone = True
        except rospy.exceptions.ROSException as e: # timeout
            print "Missing object"
            is_gone = True
        rospy.sleep(0.5)
        self.iteration_num += 1
        if is_gone:
            after_feats = np.array([0.0]*self.feature_size)
        return after_feats

    def save(self, before_feats, after_feats, obj, action_model):
        self.save_data(before_feats,obj,action_model.action,0)
        self.save_data(after_feats,obj,action_model.action,1)

    def get_run_id(self):
        with open('/home/cem/run_id.txt', 'r+') as f:
            lines = f.readlines()
            run_id = int(lines[0])
            new_run_id = run_id + 1
            f.seek(0)
            f.truncate()
            f.write(str(new_run_id))
            f.close()
            return run_id

    def save_data(self,features,obj,action,status):
        #Status 0:Before, 1:After
        #self.save_pcd(obj,action,iteration_num,status)
        self.save_features(features,obj,action,status)

    def save_pcd(self,obj,action,status):
        pcd_path = '%s%d/%s/%s/%d/%d.pcd' % (self.features_base_path, self.run_id, action.name, obj.name, self.iteration_num, status)
        msg = rospy.wait_for_message(self.pcd_topic, PointCloud2)
        python_pcd.write_pcd(pcd_path, msg)

    def get_features(self):
        msg = rospy.wait_for_message(self.features_topic, PcFeatures,timeout=10)
        hue_counter = 0
        while msg.hue != 0.0:
            if hue_counter > 5:
                return None
            msg = rospy.wait_for_message(self.features_topic, PcFeatures,timeout=10)
            hue_counter += 1
        return np.array(pc_features_to_array(msg)[1])

    def save_features(self,features,obj,action,status): #obj_name/csv/iteration_num/
        csv_path = '%s/new6/%d/%s/%s/%d/' % (self.features_base_path, self.run_id, action.name, obj.name, self.iteration_num)
        if not os.path.exists(csv_path):
            os.makedirs(csv_path)
        csv_path += '%d.csv' % (status)
        with open(csv_path, "wb") as f:
            writer = csv.writer(f)
            writer.writerow(features)
            rospy.loginfo("Saved preprocessed features to: %s" % (csv_path))

    def get_random_action(self):
        i = randint(0,len(self.actions)-1)
        return self.actions[i]

    def get_random_action_model(self):
        i = randint(0,len(self.action_models)-1)
        return self.action_models[i]

    def save_models(self):
        for am in self.action_models:
            am.save()

    def init_models(self):
        for am in self.action_models:
            am.init_models()

    def load_models(self):
        for am in self.action_models:
            am.load()

    def go_initial(self):
        self.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
