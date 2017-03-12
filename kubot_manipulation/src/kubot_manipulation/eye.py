import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
import datetime
import rosbag
from pc_segmentation.msg import PcFeatures
from utils import pc_features_to_array
import csv
import os

class Eye:
    def __init__(self, run_id):
        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/media/cem/ROSDATA/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/media/cem/ROSDATA/ros_data/features/'

        self.run_id = run_id

    def save_data(self,obj_name,action_name,iteration_num,status):
        #Status 0:Before, 1:After
        self.save_pcd(obj_name,action_name,iteration_num,status)
        self.save_features(obj_name,action_name,iteration_num,status)

    def save_pcd(self,obj_name,action_name,iteration_num,status):
        msg = rospy.wait_for_message(self.pcd_topic, PointCloud2)
        #python_pcd.write_pcd(self.pcd_base_path+self.generate_file_name(obj_name,iteration_num,status,0), msg)

    def save_features(self,obj_name,action_name,iteration_num,status):
        bag_path = self.features_base_path+'bag/'+self.generate_file_name(obj_name,action_name,iteration_num,status,1)
        csv_directory = self.features_base_path+'csv/'+self.generate_file_name(obj_name,action_name,iteration_num,status,2)
        msg = rospy.wait_for_message(self.features_topic, PcFeatures)
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

    def generate_file_name(self, obj_name, action_name,iteration_num, status, type):
        file_name = '%d_%d_%s_%s_%d' % (self.run_id, iteration_num, obj_name, action_name,status)
        folder_name = '%d_%d_%s_%s/' % (self.run_id, iteration_num, obj_name,action_name)
        if type == 0:
            return file_name + '.pcd'
        elif type == 1:
            return file_name + '.bag'
        elif type == 2:
            return folder_name
