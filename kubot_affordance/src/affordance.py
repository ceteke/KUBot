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

    iteration_num = 0
    while True:
        iteration_num += 1
        picked_object = object_handler.pick_random_object()
        rospy.loginfo('Picked object: ' + picked_object.name)
        if robot.arm.go_next_to_object(picked_object.pose_num) == -1:
            rospy.loginfo("Faild to go next to %s passing..." %(picked_object.name))
            continue
        picked_object.place_on_table()
        robot.eye.save_data(picked_object.name, iteration_num, 0)
        robot.arm.push()
        rospy.sleep(3)
        robot.eye.save_data(picked_object.name, iteration_num, 1)
        gazebo_interface.delete_object(picked_object.name)
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
