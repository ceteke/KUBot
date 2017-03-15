#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface
from random import randint
from affordance_core import AffordanceCore

def main():
    rospy.init_node('kubot_affordance', anonymous=True)
    object_handler = ObjectHandler()
    gazebo_interface = GazeboInterface()
    affordance_core = AffordanceCore()

    iteration_num = 0
    while True:
        iteration_num += 1

        action = affordance_core.get_random_action()
        picked_object = object_handler.pick_random_object()
        rospy.loginfo('Picked object: %s in pose %d' %(picked_object.name, picked_object.pose_num))
        if action.prepare(affordance_core.get_action_initial_point(action,picked_object)) == -1:
            rospy.loginfo("Faild to go next to %s in pose %d passing..." %(picked_object.name, picked_object.pose_num))
            continue
        picked_object.place_on_table()
        affordance_core.save_data(picked_object, action, iteration_num, 0)
        rospy.loginfo("Performing action: %s"%(action.name))
        action.execute()
        rospy.sleep(3)
        affordance_core.save_data(picked_object, action, iteration_num, 1)
        gazebo_interface.delete_object(picked_object.name)
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
