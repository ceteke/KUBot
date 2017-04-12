#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_gazebo')
from kubot_gazebo.object_handler import ObjectHandler
import rospy
import sys
from affordance_core import AffordanceCore, IterationError

def main():
    rospy.init_node('kubot_online_learning', anonymous=True)
    affordance_core = AffordanceCore()
    object_handler = ObjectHandler()
    try:
        while True:
            try:
                obj = object_handler.pick_random_object()
                action_model = affordance_core.get_random_action_model()
                before_feats = affordance_core.prepare_action(obj, action_model)
                affordance_core.execute_action(before_feats,action_model, obj, False, True)
                affordance_core.save_models()
            except IterationError as e:
                rospy.loginfo(e.message)
                continue
    except KeyboardInterrupt:
        affordance_core.save_models()
        sys.exit()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
