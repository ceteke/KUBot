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
from kubot_manipulation.utils import pc_features_to_array
import csv
import os


class AffordanceCore:

    def __init__(self):
        self.robot = Robot()
        self.action_poses = \
            {'push':[
                [0.122082807535,0.253172402032,0.0547882234144,
                    -0.675679130692,-0.282736229211,0.24811287633,0.634001528104],
                #0.7,-1.5,2.7,5.0599,-4.7834,1.4994
                [-0.283016464947,0.457409320807,0.0585147204124,
                    -0.636984559527,-0.249563909878,0.245820513626,0.686688285098]
                ]
                #2.0714,-0.9666,1.7952,-0.9666,2.9249,1.5
            }

        self.actions = [Push(self.robot)]

        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/media/cem/ROSDATA/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/media/cem/ROSDATA/ros_data/features/'

        self.run_id = self.get_run_id()

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

        csv_array = pc_features_to_array(msg)
        if not os.path.exists(csv_directory):
            os.makedirs(csv_directory)

        csv_path = csv_directory + str(status) + '.csv'

        with open(csv_path, "wb") as f:
            writer = csv.writer(f)
            writer.writerow(csv_array)
            rospy.loginfo("Saved features to: %s" % (csv_path))

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

    # Is interesting?
    # Predict effect
    # Fit new data
    # Reduce feature size
