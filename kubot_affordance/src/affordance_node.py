#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from affordance_core import AffordanceCore
from errors import IterationError
from voice import VoiceAssistant
import numpy as np
from kubot_affordance.srv import *
import tensorflow as tf

class AffordanceNode():
    def __init__(self):
        self.affordance_core = AffordanceCore(run_id=0)
        self.affordance_core.go_initial()
        self.affordance_core.load_models()
        self.graph = tf.get_default_graph()
        self.affordance_core.load_scalers()

    def run(self):
        s_p = rospy.Service('affordance_node/predict', predict, self.predict)
        s_e = rospy.Service('affordance_node/execute_action', execute_action, self.execute_action)
        s_g = rospy.Service('affordance_node/goto_object', goto_object, self.goto_object)
        s_g = rospy.Service('affordance_node/get_error', GetError, self.get_error)

    def execute_action(self, req):
        action_name = req.action
        success = True
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        try:
            after_feats = self.affordance_core.execute_action(action_model)
        except IterationError as e:
            rospy.loginfo(e.message)
            after_feats = []
            success = False
        response = execute_actionResponse()
        response.after_feats = after_feats
        response.success = success
        self.affordance_core.go_initial()
        return response

    def goto_object(self, req):
        action_name = req.action
        success = True
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        try:
            before_feats = self.affordance_core.move_to_object(action_model)
        except IterationError as e:
            before_feats = []
            success = False
            rospy.loginfo(e.message)
        response = goto_objectResponse()
        response.before_feats = before_feats
        response.success = success
        return response

    def predict(self, req):
        action_name = req.action
        before_feats = np.array(req.x)
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        with self.graph.as_default():
            e, y_predicted = action_model.predict(before_feats)
        response = predictResponse()
        response.effect_id = e
        response.y_predicted = y_predicted
        return response

    def get_error(self, req):
        action_name = req.action
        y_actual = np.array(req.y_actual)
        y_predicted = np.array(req.y_predicted)
        action_model = next((x for x in self.affordance_core.action_models if x.action.name == action_name), None)
        y_actual_s = action_model.effect_scaler.transform(y_actual.reshape(1,-1)).flatten()[0:3]
        err = np.linalg.norm(y_predicted-y_actual_s)
        response = GetErrorResponse()
        response.err = err
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
