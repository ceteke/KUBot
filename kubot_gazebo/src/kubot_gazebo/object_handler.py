#!/usr/bin/env python
from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose
from random import randint, uniform
from MyObject import Sphere, Box, Cylinder, Duck, Bunny

class ObjectHandler:
    def __init__(self):
        self.box = Box()
        self.sphere = Sphere()
        self.hcylinder = Cylinder('h')
        self.vcylinder = Cylinder('v')
        self.duck = Duck()
        #self.bunny = Bunny()
        self.default_pose = Pose()
        self.default_pose.position.x = 0.0
        self.default_pose.position.y = 0.0
        self.default_pose.position.z = 0.8
        self.objects = [self.box, self.sphere, self.vcylinder, self.hcylinder, self.duck]

    def pick_random_object(self):
        i = randint(0,len(self.objects)-1)
        obj = self.objects[i]
        return obj

    def get_object(self, oid):
        return next((x for x in self.objects if x.name == oid), None)

    def get_random_object_pose(self):
        pos = Pose()
        pos.position.x = uniform(-0.45,0.2)
        pos.position.y = uniform(-0.5, 0.15)
        pos.position.z = 0.8
        return pos
