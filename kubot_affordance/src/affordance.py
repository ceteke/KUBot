#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface

def main():
    rospy.init_node('kubot_affordance', anonymous=True)
    robot = Robot()
    object_handler = ObjectHandler()
    gazebo_interface = GazeboInterface()

    while True:
        picked_object = object_handler.pick_random_object()
        rospy.loginfo('Picked object: ' + picked_object[1].name)
        robot.arm.go_next_to_object(picked_object[0])
        picked_object[1].place_on_table()
        robot.eye.save_pcd()
        robot.arm.push()
        rospy.sleep(8)
        robot.eye.save_pcd()
        gazebo_interface.delete_object(picked_object[1].name)
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
