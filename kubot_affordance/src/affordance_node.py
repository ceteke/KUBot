#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore
from errors import IterationError
from voice import VoiceAssistant
import numpy as np
from kubot_affordance.srv import predict, execute_action, goto_object

class AffordanceNode():
    def __init__(self):
        self.object_handler = ObjectHandler()
        self.affordance_core = AffordanceCore(self.object_handler,run_id=0)
        self.affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
        self.affordance_core.load_models()
        self.affordance_core.load_scalers()

    def run(self):
        s_p = rospy.Service('affordance_node/predict', predict, self.predict)
        s_e = rospy.Service('affordance_node/execute_action', execute_action, self.execute_action)
        s_g = rospy.Service('affordance_node/goto_object', goto_object, self.goto_object)

    def execute_action(self, req):
        action_name = req.action
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        try:
            after_feats = self.affordance_core.execute_action(action_model)
        except IterationError as e:
            after_feats = None
        response = execute_actionResponse()
        response.after_feats = after_feats
        return response

    def goto_object(self, req):
        action_name = req.action
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        try:
            before_feats = self.affordance_core.move_to_object(action_model)
        except IterationError as e:
            before_feats = None
            rospy.loginfo(e.message)
        response = goto_objectResponse()
        response.before_feats = before_feats
        return response

    def predict(self, req):
        action_name = req.action
        before_feats = req.x
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        e, y_predicted = action_model.predict(before_feats)
        response = predictResponse()
        response.effect_cid = e
        response.y_predicted = y_predicted
        return response

def main():
    rospy.init_node('affordance_node', anonymous=True)
    an = AffordanceNode()
    an.run()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
