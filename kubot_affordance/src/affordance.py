#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.arm import Arm
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface

def main():
    a = Arm()
    object_handler = ObjectHandler()
    a.go_next_to_object('1')
    object_handler.spawn_sphere_on_table('sphere1')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
