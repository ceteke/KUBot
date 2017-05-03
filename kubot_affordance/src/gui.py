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
from threading import Thread

def prediction(obj, object_handler, affordance_core, robot_label, is_random=False):
    try:
        action_model = affordance_core.get_random_action_model()
        if is_random:
            before_feats = affordance_core.prepare_action_random(obj, action_model, is_gui=True)
        else:
            before_feats = affordance_core.prepare_action(obj, action_model, obj.get_position(), True)
        e, y_predicted = action_model.predict(before_feats)
        print "Effect cid:", e
        try:
            label = affordance_core.get_cluster_label(e)
            robot_label.config(text="I predict this object will, %s" % (label))
        except KeyError:
            robot_label.config(text="I do not know '%d' in human language." % (e))
        is_learned, after_feats = affordance_core.execute_action(before_feats,action_model, obj, False, False)
        if after_feats is not None:
            y_actual = np.absolute(np.subtract(after_feats, before_feats))
        else:
            y_actual = before_feats
        y_actual = action_model.effect_scaler.transform(y_actual.reshape(1, -1)).flatten()[0:3]
        print y_predicted
        print y_actual
        dist = np.linalg.norm(y_predicted-y_actual)
        print dist
        is_learned = dist < 0.07
        if not is_learned:
            robot_label.config(text= "WOW! This looks interesting")
        while not is_learned:
            is_learned = intrinsic_update(affordance_core, obj, action_model)
            if is_learned:
                robot_label.config(text="Boooring..")
        affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])
    except IterationError as e:
        rospy.loginfo(e.message)
        robot_label.set(e.message)
        affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])

def intrinsic_update(affordance_core, obj, action_model):
    is_moved = -1
    while is_moved == -1:
        is_moved = affordance_core.robot.arm.go_prev_pose()
    obj.place_on_table()
    before_feats = affordance_core.move_to_object(obj, action_model, False)
    is_learned, after_feats = affordance_core.execute_action(before_feats,action_model,obj, False, True)
    return is_learned

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
    rospy.init_node('kubot_gui')
    object_handler = ObjectHandler()
    affordance_core = AffordanceCore(object_handler,run_id=0)
    affordance_core.load_models()
    affordance_core.load_scalers()

    affordance_core.robot.arm.ang_cmd([2.0714,-1.5,2.2,-0.9666,2.905,1.45])

    top = Tkinter.Tk()
    B = Tkinter.Button(top, text ="Predict Random", command = lambda: Thread(target=randomCallback, args=(object_handler, affordance_core, robot_label)))
    B.pack()

    var = Tkinter.StringVar(top)
    # initial value
    var.set('box')
    choices = ['box', 'sphere', 'hcylinder', 'vcylinder', 'duck', 'bunny']
    option = Tkinter.OptionMenu(top, var, *choices)
    option.pack(side='left', padx=10, pady=10)

    robot_label = Tkinter.Label(top, text="Hello everyone!")
    robot_label.pack()
    s_button = Tkinter.Button(top, text="Spawn", command=lambda: Thread(target=pickObjectCallback, args=(var, object_handler, affordance_core, robot_label)))
    s_button.pack(side='left', padx=20, pady=10)
    r_button = Tkinter.Button(top, text="Remove", command=lambda: Thread(target=removeObjectCallback, args=(var, object_handler, affordance_core, robot_label)))
    r_button.pack(side='left', padx=20, pady=10)
    g_button = Tkinter.Button(top, text="Go!", command=lambda: Thread(target=goObjectCallback, args=(var, object_handler, affordance_core, robot_label)))
    g_button.pack(side='left', padx=20, pady=10)

    top.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
