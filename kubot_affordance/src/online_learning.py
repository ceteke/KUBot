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
    epoch = 1
    epsilon = 0.015
    learned_pairs = []
    i = 1
    affordance_core.form_pairs(object_handler)
    # Epochs
    while True:
        if len(affordance_core.pairs) == len(learned_pairs):
            print "Epoch %d: Nothing left to learn" % (i)
            learned_pairs = []
            if i == epoch:
                break
            i += 1

        pair = affordance_core.get_random_pair()

        rospy.loginfo('%d Picked object: %s in pose %d' %(i, pair.obj.name, pair.obj.pose_num))
        if pair.action.prepare(affordance_core.get_action_initial_point(pair.action,pair.obj)) == -1:
            rospy.loginfo("Faild to go next to %s in pose %d passing..." %(pair.obj.name, pair.obj.pose_num))
            continue

        pair.obj.place_on_table()
        before_features = affordance_core.get_features()
        rospy.loginfo("Performing action: %s"%(pair.action.name))
        pair.action.execute()
        rospy.sleep(5)
        after_features = affordance_core.get_features()
        effect_features = np.subtract(after_features, before_features)
        is_updated = pair.model.update(before_features, effect_features, epsilon)
        rospy.sleep(0.5)
        pair.obj.remove()
        rospy.sleep(0.5)
        if not is_updated:
            rospy.loginfo('%s is not interesting when I %s' % (pair.obj.id, pair.action.name))
            learned_pairs.append(pair)

    affordance_core.save_pairs()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
