#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
from affordance_core import AffordanceCore
from errors import IterationError
from voice import VoiceAssistant
import numpy as np
import Tkinter
import tkMessageBox

def prediction(obj, object_handler, affordance_core, va, is_random=False):
    try:
        action_model = affordance_core.get_random_action_model()
        if is_random:
            before_feats = affordance_core.prepare_action_random(obj, action_model, is_gui=True)
        else:
            before_feats = affordance_core.move_to_object(obj, action_model, True)
        e, y_predicted = action_model.predict(before_feats)
        print "Effect cid:", e
        try:
            va.add_say("I predict this object will, %s" % (affordance_core.get_cluster_label(e)))
        except KeyError:
            va.add_say("Sorry, you humans can not understand.")
        is_learned, after_feats = affordance_core.execute_action(before_feats,action_model, obj, False, False)
        if after_feats is not None:
            y_actual = np.absolute(np.subtract(after_feats, before_feats))
        else:
            y_actual = before_feats
        y_actual = action_model.effect_scaler.transform(y_actual.reshape(1, -1)).flatten()[0:3]
        print y_predicted
        print y_actual
        dist = np.linalg.norm(y_predicted-y_actual)
        if dist > 0.07:
            va.add_say("Fuck!")
        print dist
        affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
    except IterationError:
        va.add_say("Sorry :(")

def randomCallback(object_handler, affordance_core, va):
    obj = object_handler.pick_random_object()
    prediction(obj, object_handler, affordance_core, va, is_random=True)

def pickObjectCallback(var, object_handler, affordance_core, va):
    oid = var.get()
    obj = object_handler.get_object(oid)
    obj.set_position(object_handler.default_pose)
    obj.place_on_table()

def removeObjectCallback(var, object_handler, affordance_core, va):
    oid = var.get()
    obj = object_handler.get_object(oid)
    obj.remove()

def goObjectCallback(var, object_handler, affordance_core, va):
    oid = var.get()
    obj = object_handler.get_object(oid)
    prediction(obj, object_handler, affordance_core, va)

def main():
    rospy.init_node('kubot_gui', anonymous=True)
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler,run_id=0)
    affordance_core.load_models()
    va = VoiceAssistant()
    va.start()
    affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])

    top = Tkinter.Tk()
    B = Tkinter.Button(top, text ="Predict Random", command = lambda: randomCallback(object_handler, affordance_core, va))
    B.pack()

    var = Tkinter.StringVar(top)
    # initial value
    var.set('box')
    choices = ['box', 'sphere', 'hcylinder', 'vcylinder']
    option = Tkinter.OptionMenu(top, var, *choices)
    option.pack(side='left', padx=10, pady=10)

    s_button = Tkinter.Button(top, text="Spawn", command=lambda: pickObjectCallback(var, object_handler, affordance_core, va))
    s_button.pack(side='left', padx=20, pady=10)
    r_button = Tkinter.Button(top, text="Remove", command=lambda: removeObjectCallback(var, object_handler, affordance_core, va))
    r_button.pack(side='left', padx=20, pady=10)
    g_button = Tkinter.Button(top, text="Go!", command=lambda: goObjectCallback(var, object_handler, affordance_core, va))
    g_button.pack(side='left', padx=20, pady=10)

    top.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
