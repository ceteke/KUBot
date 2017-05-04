#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
import numpy as np
import Tkinter
import tkMessageBox
from kubot_affordance.srv import *
import threading

class GUI():
    def __init__(self):
        self.object_handler = ObjectHandler()

        self.top = Tkinter.Tk()
        self.frame = Tkinter.Frame(self.top)
        self.frame.pack()

        self.B = Tkinter.Button(self.frame, text ="Predict Random", command=self.randomCallback)
        self.B.pack()

        self.var = Tkinter.StringVar(self.top)
        # initial value
        self.var.set('box')
        self.choices = ['box', 'sphere', 'hcylinder', 'vcylinder', 'duck']
        self.option = Tkinter.OptionMenu(self.frame, self.var, *self.choices)
        self.option.pack(side='left', padx=10, pady=10)

        self.s_button = Tkinter.Button(self.frame, text="Spawn", command=self.pickObjectCallback)
        self.s_button.pack(side='left', padx=20, pady=10)
        self.r_button = Tkinter.Button(self.frame, text="Remove", command=self.removeObjectCallback)
        self.r_button.pack(side='left', padx=20, pady=10)
        self.g_button = Tkinter.Button(self.frame, text="Go!", command=self.goObjectCallback)
        self.g_button.pack(side='left', padx=20, pady=10)

        self.status = Tkinter.Label(self.top, text="Hello Everyone", font=("Helvetica", 16))
        self.status.pack(side="bottom", anchor="center")

        self.t = None

        self.labels = {0:"Stay", 1:"Roll", 43:"Stay", 5:"Drop", 9:"Drop", 11:"Drop"}

        self.top.mainloop()

    def prediction(self, obj, is_random):
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
            self.status["text"] = "I can't go there."
            return
        before_feats = goto_rsp.before_feats

        p_sp = rospy.ServiceProxy('affordance_node/predict', predict)
        p_rq = predictRequest()
        p_rq.action = action
        p_rq.x = before_feats
        p_rsp = p_sp(p_rq)
        effect_cid = p_rsp.effect_id
        y_predicted = p_rsp.y_predicted

        print "Effect:", effect_cid
        try:
            label = self.labels[effect_cid]
            self.status["text"] = "I think this object will %s" % (label)
        except KeyError:
            self.status["text"] = "I don't know this effect in human language."

        ex_sp = rospy.ServiceProxy('affordance_node/execute_action', execute_action)
        ex_rq = execute_actionRequest()
        ex_rq.action = action
        ex_rsp = ex_sp(ex_rq)
        if not ex_rsp.success:
            obj.remove()
            self.status["text"] = "I can't %s." % (action)
            return
        after_feats = ex_rsp.after_feats
        y_actual = np.absolute(np.array(after_feats)-np.array(before_feats))

        err_sp = rospy.ServiceProxy('affordance_node/get_error', GetError)
        err_rq = GetErrorRequest()
        err_rq.action = action
        err_rq.y_actual = y_actual
        err_rq.y_predicted = y_predicted
        err_rsp = err_sp(err_rq)
        err = err_rsp.err

        print "Error:", err

        obj.remove()

        if err > 0.1:
            self.status["text"] = "Oh! This is new. I can't wait to learn this after the demo!"
        else:
            self.status["text"] = "Come on let me show you my skills.."

    def randomCallback(self):
        obj = self.object_handler.pick_random_object()
        self.t = threading.Thread(target=self.prediction, args=(obj, True))
        self.t.start()

    def pickObjectCallback(self):
        oid = self.var.get()
        obj = self.object_handler.get_object(oid)
        obj.set_position(self.object_handler.default_pose)
        obj.place_on_table()

    def removeObjectCallback(self):
        oid = self.var.get()
        obj = self.object_handler.get_object(oid)
        obj.remove()

    def goObjectCallback(self):
        oid = self.var.get()
        obj = self.object_handler.get_object(oid)
        self.t = threading.Thread(target=self.prediction, args=(obj, False))
        self.t.start()


def main():
    rospy.init_node('kubot_gui')
    gui = GUI()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
