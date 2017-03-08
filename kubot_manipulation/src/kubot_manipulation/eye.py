import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
import datetime
import rosbag
from pc_segmentation.msg import PcFeatures

class Eye:
    def __init__(self, run_id):
        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/home/cem/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_base_path = '/home/cem/ros_data/features/'

        self.run_id = run_id

    def save_data(self,obj_name,iteration_num,status):
        self.save_pcd(obj_name,iteration_num,status)
        self.save_features(obj_name,iteration_num,status)

    def save_pcd(self,obj_name,iteration_num,status):
        msg = rospy.wait_for_message(self.pcd_topic, PointCloud2)
        python_pcd.write_pcd(self.pcd_base_path+self.generate_file_name(obj_name,iteration_num,status,True), msg)

    def save_features(self,obj_name,iteration_num,status):
        msg = rospy.wait_for_message(self.features_topic, PcFeatures)
        try:
            bag = rosbag.Bag(self.features_base_path+self.generate_file_name(obj_name,iteration_num,status,False), 'w')
            bag.write(self.features_topic,msg)
        finally:
            bag.close()

    def generate_file_name(self, obj_name, iteration_num, status, is_pcd):
        file_name = '%d_%d_%s_%s' % (self.run_id, iteration_num, obj_name, status)
        if is_pcd:
            return file_name + '.pcd'
        return file_name + '.bag'
