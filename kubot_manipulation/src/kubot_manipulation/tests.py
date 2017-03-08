#!/usr/bin/env python
import rospy
from arm import Arm
import rosbag

def main():

    rospy.init_node('testing_stuff', anonymous=True)

    bag = rosbag.Bag('/home/cem/ros_data/features/14_1_box2_before.bag')
    for topic, msg, t in bag.read_messages(topics=['/baris/features']):
        print msg
    bag.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
