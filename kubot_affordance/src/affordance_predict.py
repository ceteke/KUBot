#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore
from errors import IterationError
from voice import VoiceAssistant
import numpy as np

def main():
    rospy.init_node('kubot_predictor', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler,run_id=648)
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
            is_learned, after_feats = affordance_core.execute_action(before_feats,action_model, obj, False, False)
            if after_feats is not None:
                y_actual = np.absolute(np.subtract(after_feats, before_feats))
            else:
                y_actual = before_feats
            y_actual = action_model.effect_scaler.transform(y_actual.reshape(1, -1)).flatten()[0:3]
            print y_predicted
            print y_actual
        except IterationError as e:
            rospy.loginfo(e.message)
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
