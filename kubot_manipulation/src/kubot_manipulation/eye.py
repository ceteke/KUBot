import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
import datetime
import rosbag

class Eye:
    def __init__(self, run_id):
        self.pcd_topic = '/camera/points'
        self.pcd_base_path = '/home/cem/ros_data/pcd/'

        self.features_topic = '/baris/features'
        self.features_path = '/home/cem/ros_data/features/'

        self.run_id = run_id

    def save_pcd(self,obj_name,iteration_num,status):
        msg = rospy.wait_for_message(self.pcd_topic, PointCloud2)
        python_pcd.write_pcd(self.pcd_base_path+self.generate_file_name(obj_name,iteration_num,status,True), msg)

#    def save_features():
    def generate_file_name(self, obj_name, iteration_num, status, is_pcd):
        file_name = '%d_%d_%s_%s' % (self.run_id, iteration_num, obj_name, status)
        if is_pcd:
            return file_name + '.pcd'
        return file_name
