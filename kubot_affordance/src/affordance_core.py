import roslib; roslib.load_manifest('kubot_manipulation');
from kubot_manipulation.robot import Robot
from random import randint
from kubot_manipulation.MyAction import Push
import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
import datetime
import rosbag
from pc_segmentation.msg import PcFeatures
from utils import pc_features_to_array
import csv
import os
import pickle
import numpy as np
from models import GradientDescent
from sklearn.preprocessing import minmax_scale
from pair import Pair

class AffordanceCore:

    def __init__(self):
        self.robot = Robot()
        self.action_poses = \
            {'push':[
                [0.122082807535,0.253172402032,0.0547882234144,
                    -0.675679130692,-0.282736229211,0.24811287633,0.634001528104],
                #0.7,-1.5,2.7,5.0599,-4.7834,1.4994
                [-0.28412935179,0.458596878826,0.0587319921008,
                    -0.651242299506,-0.26303761784,0.245163873518,0.668273412651]
                ]
                #2.0714,-0.9666,1.7952,-0.9666,2.905,1.45
            }

        self.actions = [Push(self.robot)]

        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/media/cem/ROSDATA/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/media/cem/ROSDATA/ros_data/features/'

        self.run_id = self.get_run_id()

        self.cluster_labels = {1: 'Stay', 0: 'Roll'}

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

    def save_data(self,obj,action,iteration_num,status):
        #Status 0:Before, 1:After
        #self.save_pcd(obj,action,iteration_num,status)
        self.save_features(obj,action,iteration_num,status)

    def save_pcd(self,obj_name,action_name,iteration_num,status):
        msg = rospy.wait_for_message(self.pcd_topic, PointCloud2)
        python_pcd.write_pcd(self.pcd_base_path+self.generate_file_name(obj_name,iteration_num,status,0), msg)

    def get_features(self):
        msg = rospy.wait_for_message(self.features_topic, PcFeatures)
        while msg.hue != 0.0:
            print msg.hue
            msg = rospy.wait_for_message(self.features_topic, PcFeatures)
        return np.array(pc_features_to_array(msg)[1])

    def save_features(self,obj,action,iteration_num,status):
        bag_path = self.features_base_path+'bag/'+self.generate_file_name(obj,action,iteration_num,status,1)
        csv_directory = self.features_base_path+'csv/'+self.generate_file_name(obj,action,iteration_num,status,2)
        msg = rospy.wait_for_message(self.features_topic, PcFeatures)
        while msg.hue != 0.0:
            print msg.hue
            msg = rospy.wait_for_message(self.features_topic, PcFeatures)
        print msg.hue
        try:
            bag = rosbag.Bag(bag_path, 'w')
            bag.write(self.features_topic,msg)
        finally:
            bag.close()

        feature_tuple = pc_features_to_array(msg)
        if not os.path.exists(csv_directory):
            os.makedirs(csv_directory)

        csv_path_features = csv_directory + str(status) + '.csv'
        csv_path_preproc_features = csv_directory + str(status) + '_preproc.csv'

        with open(csv_path_features, "wb") as f:
            writer = csv.writer(f)
            writer.writerow(feature_tuple[0])
            rospy.loginfo("Saved features to: %s" % (csv_path_features))

        with open(csv_path_preproc_features, "wb") as f:
            writer = csv.writer(f)
            writer.writerow(feature_tuple[1])
            rospy.loginfo("Saved preprocessed features to: %s" % (csv_path_preproc_features))

    def generate_file_name(self, obj, action,iteration_num, status, type):
        file_name = '%d_%d_%s_%s_%d' % (self.run_id, iteration_num, obj.name, action.name,status)
        folder_name = '%d_%d_%s_%s/' % (self.run_id, iteration_num, obj.id,action.name)
        if type == 0:
            return file_name + '.pcd'
        elif type == 1:
            return file_name + '.bag'
        elif type == 2:
            return folder_name

    def get_action_initial_point(self,action,obj):
        return self.action_poses[action.name][obj.pose_num]

    def get_random_action(self):
        i = randint(0,len(self.actions)-1)
        return self.actions[i]

    def predict_effect(self,action,before_features):
        before_features = minmax_scale(np.array(before_features)).reshape(1,-1)
        predicted_effect = action.model.predict(before_features)
        predicted_cluster = action.effect_cluster.predict(predicted_effect)[0]
        return self.cluster_labels[predicted_cluster]

    def predict_effect(self,action,obj,before_features):
        pair = next((x for x in self.pairs if x.action.name == action.name and x.obj.id == obj.id), None)
        y_predicted = pair.model.predict(before_features)
        predicted_cluster = action.effect_cluster.predict(y_predicted.T)[0]
        return self.cluster_labels[predicted_cluster]

    def form_pairs(self, object_handler):
        self.pairs = []
        for a in self.actions:
            for o in object_handler.objects:
                self.pairs.append(Pair(a, o, GradientDescent(minmax_scale)))

    def load_pairs(self,object_handler):
        self.pairs = []
        for a in self.actions:
            for o in object_handler.objects:
                self.pairs.append(Pair(a,o,pickle.load(open('%s%s_%s_gradient_descent'%(a.base_path, a.name, o.id), 'rb'))))

    def get_random_pair(self):
        i = randint(0,len(self.pairs)-1)
        return self.pairs[i]

    def save_pairs(self):
        for p in self.pairs:
            p.save()

    # Is interesting?
    # Predict effect
    # Fit new data
    # Reduce feature size
