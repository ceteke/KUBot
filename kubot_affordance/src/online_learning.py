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
    action_index = 0
    epoch = 3
    # Iterate over actions
    while True:
        if action_index == len(affordance_core.actions):
            rospy.loginfo('Nothing left to learn')
            break
        action = affordance_core.actions[action_index]
        action.init_online_learning()
        # Iterate over objects
        for i in range(epoch):
            object_index = 0
            while True:
                if object_index == len(object_handler.objects):
                    break
                picked_object = object_handler.objects[object_index]
                rospy.loginfo('%d Picked object: %s in pose %d' %(i, picked_object.name, picked_object.pose_num))
                if action.prepare(affordance_core.get_action_initial_point(action,picked_object)) == -1:
                    rospy.loginfo("Faild to go next to %s in pose %d passing..." %(picked_object.name, picked_object.pose_num))
                    continue
                picked_object.place_on_table()
                before_features = affordance_core.get_features()
                rospy.loginfo("Performing action: %s"%(action.name))
                action.execute()
                rospy.sleep(5)
                after_features = affordance_core.get_features()
                effect_features = np.subtract(after_features, before_features)
                is_updated = action.update_models(before_features, effect_features, 0.05)
                rospy.sleep(0.5)
                picked_object.remove()
                rospy.sleep(0.5)
                if not is_updated:
                    object_index += 1
                    rospy.loginfo('This object is not interesting when I %s' % (action.name))
        action_index += 1
        action.save_weights()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
