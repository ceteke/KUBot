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

    robot.arm.go_next_to_object('1')
    object_handler.spawn_sphere_on_table('sphere1')
    robot.eye.save_pcd()
    robot.arm.push()
    rospy.sleep(8)
    robot.eye.save_pcd()
    gazebo_interface.delete_object('sphere1')

    robot.arm.go_next_to_object('1')
    object_handler.spawn_box_on_table('box1')
    robot.eye.save_pcd()
    robot.arm.push()
    rospy.sleep(8)
    robot.eye.save_pcd()
    gazebo_interface.delete_object('box1')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
