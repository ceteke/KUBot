#!/usr/bin/env python
from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose
from random import randint
from MyObject import Sphere, Box

class ObjectHandler:
    def __init__(self):
        self.sphere_pos1 = Pose()
        self.sphere_pos1.position.x = 0.045729
        self.sphere_pos1.position.y = 0.018710
        self.sphere_pos1.position.z = 0.781567
        self.sphere1 = Sphere('sphere1', self.sphere_pos1,0)

        self.sphere_pos2 = Pose()
        self.sphere_pos2.position.x = 0.195137
        self.sphere_pos2.position.y = -0.399931
        self.sphere_pos2.position.z = 0.781575
        self.sphere2 = Sphere('sphere2', self.sphere_pos2,1)

        self.box_pos1 = Pose()
        self.box_pos1.position.x = 0.027310
        self.box_pos1.position.y = 0.027579
        self.box_pos1.position.z = 0.781567
        self.box1 = Box('box1',self.box_pos1,0)

        self.box_pos2 = Pose()
        self.box_pos2.position.x = 0.178885
        self.box_pos2.position.y = -0.396604
        self.box_pos2.position.z = 0.780575
        self.box2 = Box('box2',self.box_pos2,1)

        self.objects = [self.sphere1, self.sphere2, self.box1, self.box2]

    def pick_random_object(self):
        i = randint(0,len(self.objects)-1)
        obj = self.objects[i]
        return obj
