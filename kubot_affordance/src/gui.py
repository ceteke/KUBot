#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
import numpy as np
import Tkinter
import tkMessageBox
from kubot_affordance.srv import *

class GUI():
    def __init__(self):
        self.object_handler = ObjectHandler()

        self.top = Tkinter.Tk()
        self.B = Tkinter.Button(self.top, text ="Predict Random", command = lambda: self.randomCallback())
        self.B.pack()

        self.var = Tkinter.StringVar(self.top)
        # initial value
        self.var.set('box')
        self.choices = ['box', 'sphere', 'hcylinder', 'vcylinder', 'duck', 'bunny']
        self.option = Tkinter.OptionMenu(self.top, self.var, *self.choices)
        self.option.pack(side='left', padx=10, pady=10)

        self.s_button = Tkinter.Button(self.top, text="Spawn", command=lambda: self.pickObjectCallback(self.var))
        self.s_button.pack(side='left', padx=20, pady=10)
        self.r_button = Tkinter.Button(self.top, text="Remove", command=lambda: self.removeObjectCallback(self.var))
        self.r_button.pack(side='left', padx=20, pady=10)
        self.g_button = Tkinter.Button(self.top, text="Go!", command=lambda: self.goObjectCallback(self.var))
        self.g_button.pack(side='left', padx=20, pady=10)

        self.top.mainloop()

    def prediction(self, obj, is_random=False):
        action = 'push'
        if is_random:
            obj.set_position(self.object_handler.get_random_object_pose())
            obj.place_on_table()

        goto_sp = rospy.ServiceProxy('affordance_node/goto_object', goto_object)
        goto_rq = goto_objectRequest()
        goto_rq.action = action
        goto_rsp = goto_sp(goto_rq)
        if not goto_rsp.success:
            obj.remove()
            return
        before_feats = goto_rsp.before_feats

        p_sp = rospy.ServiceProxy('affordance_node/predict', predict)
        p_rq = predictRequest()
        p_rq.action = action
        p_rq.x = before_feats
        p_rsp = p_sp(p_rq)
        effect_cid = p_rsp.effect_id
        y_predicted = p_rsp.y_predicted

        print effect_cid

        ex_sp = rospy.ServiceProxy('affordance_node/execute_action', execute_action)
        ex_rq = execute_actionRequest()
        ex_rq.action = action
        ex_rsp = ex_sp(ex_rq)
        if not ex_rsp.success:
            obj.remove()
            return
        after_feats = ex_rsp.after_feats
        obj.remove()

    def randomCallback(self):
        obj = self.object_handler.pick_random_object()
        self.prediction(obj, is_random=True)

    def pickObjectCallback(self, var):
        oid = var.get()
        obj = self.object_handler.get_object(oid)
        obj.set_position(self.object_handler.default_pose)
        obj.place_on_table()

    def removeObjectCallback(self, var):
        oid = var.get()
        obj = self.object_handler.get_object(oid)
        obj.remove()

    def goObjectCallback(self, var):
        oid = var.get()
        obj = self.object_handler.get_object(oid)
        self.prediction(obj)

def main():
    rospy.init_node('kubot_gui')
    gui = GUI()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
