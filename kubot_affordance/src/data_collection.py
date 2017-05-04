#!/usr/bin/env python
import roslib;roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore
from errors import IterationError

def main():
    rospy.init_node('kubot_data_collector', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler)
    while True:
        try:
            obj = object_handler.pick_random_object()
            action_model = affordance_core.get_random_action_model()
            before_feats = affordance_core.prepare_action_random(obj, action_model)
            after_feats = affordance_core.execute_action(action_model)
            affordance_core.save(before_feats, after_feats, obj, action_model, is_gone)
        except IterationError as e:
            rospy.loginfo(e.message)
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
