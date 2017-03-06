import rospy
from sensor_msgs.msg import PointCloud2
import python_pcd
import datetime

class Eye:
    def __init__(self):
        self.topic = '/camera/points'
        self.path = '/home/cem/pcd_data/'

    def save_pcd(self):
        msg = rospy.wait_for_message(self.topic, PointCloud2)
        python_pcd.write_pcd(self.path+self.generate_pcd_name(), msg)

    def generate_pcd_name(self):
        date = datetime.datetime.now().strftime("%m-%d_%H:%M:%S")
        return date+'.pcd'
