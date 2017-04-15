#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore, IterationError
from voice import VoiceAssistant
import numpy as np

def main():
    rospy.init_node('kubot_predictor', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler)
    affordance_core.load_models()
    va = VoiceAssistant()
    va.start()
    while True:
        try:
            obj = object_handler.pick_random_object()
            action_model = affordance_core.get_random_action_model()
            before_feats = affordance_core.prepare_action_random(obj, action_model)
            e, y_predicted = action_model.predict(before_feats)
            print "Effect cid:", e
            is_learned, y_actual = affordance_core.execute_action(before_feats,action_model, obj, False, False)
            a = np.subtract(y_actual, y_predicted)
            err = np.matmul(a.T, a)[0][0]
            print err / 2
            affordance_core.save_models()
        except IterationError as e:
            rospy.loginfo(e.message)
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
