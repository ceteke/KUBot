#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_gazebo.object_handler import ObjectHandler
import numpy as np
import Tkinter
import tkMessageBox
from kubot_affordance.srv import *

def prediction(obj, object_handler, is_random=False):
    action = 'push'
    if is_random:
        obj.set_position(object_handler.get_random_object_pose())
        obj.place_on_table()

    goto_sp = rospy.ServiceProxy('affordance_node/goto_object', goto_object)
    goto_rq = goto_objectRequest()
    goto_rq.action = action
    goto_rsp = goto_sp(goto_rq)
    print goto_rsp

def randomCallback(object_handler):
    obj = object_handler.pick_random_object()
    prediction(obj, object_handler, is_random=True)

def pickObjectCallback(var, object_handler):
    oid = var.get()
    obj = object_handler.get_object(oid)
    obj.set_position(object_handler.default_pose)
    obj.place_on_table()

def removeObjectCallback(var, object_handler):
    oid = var.get()
    obj = object_handler.get_object(oid)
    obj.remove()

def goObjectCallback(var, object_handler):
    oid = var.get()
    obj = object_handler.get_object(oid)
    prediction(obj, object_handler, va)

def main():
    rospy.init_node('kubot_gui')
    object_handler = ObjectHandler()

    top = Tkinter.Tk()
    B = Tkinter.Button(top, text ="Predict Random", command = lambda: randomCallback(object_handler))
    B.pack()

    var = Tkinter.StringVar(top)
    # initial value
    var.set('box')
    choices = ['box', 'sphere', 'hcylinder', 'vcylinder', 'duck', 'bunny']
    option = Tkinter.OptionMenu(top, var, *choices)
    option.pack(side='left', padx=10, pady=10)

    s_button = Tkinter.Button(top, text="Spawn", command=lambda: pickObjectCallback(var, object_handler))
    s_button.pack(side='left', padx=20, pady=10)
    r_button = Tkinter.Button(top, text="Remove", command=lambda: removeObjectCallback(var, object_handler))
    r_button.pack(side='left', padx=20, pady=10)
    g_button = Tkinter.Button(top, text="Go!", command=lambda: goObjectCallback(var, object_handler))
    g_button.pack(side='left', padx=20, pady=10)

    top.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
