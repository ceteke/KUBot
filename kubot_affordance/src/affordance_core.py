import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
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

class IterationError(Exception):
    pass

class AffordanceCore:

    def __init__(self, models_path='/home/cem/learning/models/'):
        self.robot = Robot()

        self.push = Push(self.robot)
        self.actions = [self.push]
        self.action_models = [ActionModel(self.push)]

        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/media/cem/ROSDATA/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/media/cem/ROSDATA/ros_data/features/'

        self.run_id = self.get_run_id()

        self.iteration_num = 0
        self.object_handler = ObjectHandler()

        self.models_path = models_path


    def prepare_action(self, obj, action_model):
        self.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
        rospy.sleep(5)
        obj.set_position(self.object_handler.get_random_object_pose())
        obj.place_on_table()
        try:
            before_feats = self.get_features()
        except rospy.exceptions.ROSException as e:
            obj.remove()
            raise IterationError("Waiting feature topic timed out.")
        if before_feats is None:
            obj.remove()
            raise IterationError("Object out of scope.")
        if action_model.action.prepare(before_feats,obj.name) == -1:
            obj.remove()
            raise IterationError("No plan found.")
        return before_feats

    def execute_action(self, before_feats, action_model, obj, should_save, should_update):
        if action_model.action.execute() == -1:
            obj.remove()
            raise IterationError("Cartesian path is not good enough")
        rospy.sleep(5)
        try:
            after_feats = self.get_features()
        except rospy.exceptions.ROSException as e:
            obj.remove()
            raise IterationError("Waiting feature topic timed out.")
        if after_feats is None:
            print "Missing object"
            after_feats = np.array([0.0] * 69)
        if should_update:
            action_model.update(before_feats, after_feats)
        if should_save:
            self.save_data(before_feats,obj,action_model.action,0)
            self.save_data(after_feats,obj,action_model.action,1)
        rospy.sleep(0.5)
        obj.remove()
        self.iteration_num += 1

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
        csv_path = '%s%d/%s/%s/%d/%d.csv' % (self.features_base_path, self.run_id, action.name, obj.name, self.iteration_num, status)

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

    def load_models(self):
        for am in self.action_models:
            am.load()
