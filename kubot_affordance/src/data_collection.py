#!/usr/bin/env python
import roslib;roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore
from errors import IterationError

def main():
    rospy.init_node('kubot_data_collector', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore()
    while True:
        try:
            affordance_core.go_initial()
            rospy.sleep(3)
            obj = object_handler.pick_random_object()
            obj_pos = object_handler.get_random_object_pose()
            obj.set_position(obj_pos)
            obj.place_on_table()
            action_model = affordance_core.get_random_action_model()
            before_feats = affordance_core.move_to_object(action_model)
            after_feats = affordance_core.execute_action(action_model)
            affordance_core.save(before_feats, after_feats, obj, action_model)
            obj.remove()
        except IterationError as e:
            obj.remove()
            rospy.loginfo(e.message)
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
