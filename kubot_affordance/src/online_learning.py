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
    affordance_core = AffordanceCore()
    while True:
        affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
        rospy.sleep(5)
        action_model = affordance_core.get_random_action_model()
        obj = object_handler.pick_random_object()
        obj.set_position(object_handler.get_random_object_pose())
        obj.place_on_table()
        before_feats = affordance_core.get_features()
        if before_feats is None:
            continue
        if action_model.action.prepare(before_feats,obj.name) == -1:
            obj.remove()
            continue
        if action_model.action.execute() == -1:
            obj.remove()
            continue
        rospy.sleep(5)
        after_feats = affordance_core.get_features()
        if after_feats is None:
            print "Missing object"
            after_feats = np.array([0.0] * 69)
        action_model.update(before_feats, after_feats)
        rospy.sleep(0.5)
        obj.remove()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
