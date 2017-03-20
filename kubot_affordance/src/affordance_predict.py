#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface
from random import randint
from affordance_core import AffordanceCore

def main():
    rospy.init_node('kubot_predictor', anonymous=True)
    object_handler = ObjectHandler()
    gazebo_interface = GazeboInterface()
    affordance_core = AffordanceCore()
    iteration_num = 1
    while True:
        action = affordance_core.get_random_action()
        picked_object = object_handler.pick_random_object()
        if action.prepare(affordance_core.get_action_initial_point(action,picked_object)) == -1:
            rospy.loginfo("Faild to go next to %s in pose %d passing..." %(picked_object.name, picked_object.pose_num))
            continue
        picked_object.place_on_table()
        before_features = affordance_core.get_features()
        rospy.loginfo("I predict this object will %s" % (affordance_core.predict_effect(action,'linear_regression',before_features)))
        action.execute()
        rospy.sleep(5)
        picked_object.remove()
        rospy.sleep(0.5)
        iteration_num += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
