#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
import numpy as np
from kubot_manipulation.robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface
from random import randint
from affordance_core import AffordanceCore

def main():
    rospy.init_node('kubot_online_learning', anonymous=True)
    object_handler = ObjectHandler()
    gazebo_interface = GazeboInterface()
    epoch = 1
    epsilon = 0.015
    learned_pairs = []
    i = 1
    # Epochs
    while True:
        obj = object_handler.pick_random_object()
        obj.set_position(object_handler.get_random_object_pose())
        obj.place_on_table()
        rospy.sleep(1)
        obj.remove()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
