#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_gazebo')
from kubot_gazebo.object_handler import ObjectHandler
import rospy
import sys
from affordance_core import AffordanceCore
from errors import IterationError

def main():
    rospy.init_node('kubot_online_learning', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler)
    #affordance_core.load_models()
    try:
        while True:
            affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
            rospy.sleep(3)
            try:
                obj = object_handler.pick_random_object()
                obj_pose = object_handler.get_random_object_pose()
                action_model = affordance_core.get_random_action_model()
                is_first = True
                while True:
                    before_feats = affordance_core.prepare_action(obj, action_model, obj_pose, is_first)
                    is_learned, after_feats = affordance_core.execute_action(before_feats,action_model, obj, True, True)
                    affordance_core.save_models()
                    if is_learned:
                        rospy.loginfo("=================Boring=============")
                        break
                    else:
                        is_first = False
                        affordance_core.robot.arm.go_prev_pose()
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
