#!/usr/bin/env python
import roslib;roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore, IterationError

def main():
    rospy.init_node('kubot_data_collector', anonymous=True)
    affordance_core = AffordanceCore()
    object_handler = ObjectHandler()
    while True:
        try:
            obj = object_handler.pick_random_object()
            action_model = affordance_core.get_random_action_model()
            before_feats = affordance_core.prepare_action_random(obj, action_model)
            affordance_core.execute_action(before_feats,action_model, obj, True, False)
        except IterationError as e:
            rospy.loginfo(e.message)
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
