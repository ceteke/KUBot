#!/usr/bin/env python
import rospy
from arm import Arm
import rosbag
from utils import pc_features_to_array

def main():

    rospy.init_node('testing_stuff', anonymous=True)

    bag = rosbag.Bag('/media/cem/ROSDATA/ros_data/features/bag/35_1_sphere1_after.bag')
    for topic, msg, t in bag.read_messages(topics=['/baris/features']):
        print len(pc_features_to_array(msg))
    bag.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
