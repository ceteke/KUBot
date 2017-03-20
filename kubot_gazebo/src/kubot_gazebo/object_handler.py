#!/usr/bin/env python
from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose
from random import randint
from MyObject import Sphere, Box, Cylinder

class ObjectHandler:
    def __init__(self):
        self.sphere_pos1 = Pose()
        self.sphere_pos1.position.x = 0.045729
        self.sphere_pos1.position.y = 0.018710
        self.sphere_pos1.position.z = 0.781567
        self.sphere1 = Sphere(self.sphere_pos1,0)

        self.sphere_pos2 = Pose()
        self.sphere_pos2.position.x = 0.195137
        self.sphere_pos2.position.y = -0.399931
        self.sphere_pos2.position.z = 0.781575
        self.sphere2 = Sphere(self.sphere_pos2,1)

        self.box_pos1 = Pose()
        self.box_pos1.position.x = 0.027310
        self.box_pos1.position.y = 0.027579
        self.box_pos1.position.z = 0.781567
        self.box1 = Box(self.box_pos1,0)

        self.box_pos2 = Pose()
        self.box_pos2.position.x = 0.178885
        self.box_pos2.position.y = -0.396604
        self.box_pos2.position.z = 0.781575
        self.box2 = Box(self.box_pos2,1)

        self.vcylinder_pos1 = Pose()
        self.vcylinder_pos1.position.x = 0.045729
        self.vcylinder_pos1.position.y = 0.018710
        self.vcylinder_pos1.position.z = 0.781575
        self.vcylinder1 = Cylinder(self.vcylinder_pos1,0,'v')

        self.vcylinder_pos2 = Pose()
        self.vcylinder_pos2.position.x = 0.195137
        self.vcylinder_pos2.position.y = -0.399931
        self.vcylinder_pos2.position.z = 0.781567
        self.vcylinder2 = Cylinder(self.vcylinder_pos2,1,'v')

        self.hcylinder_pos1 = Pose()
        self.hcylinder_pos1.position.x = 0.027310
        self.hcylinder_pos1.position.y = 0.027579
        self.hcylinder_pos1.position.z = 0.781575
        self.hcylinder1 = Cylinder(self.hcylinder_pos1,0,'h')

        self.hcylinder_pos2 = Pose()
        self.hcylinder_pos2.position.x = 0.178885
        self.hcylinder_pos2.position.y = -0.396604
        self.hcylinder_pos2.position.z = 0.781575
        self.hcylinder2 = Cylinder(self.hcylinder_pos2,1,'h')

        self.objects = [self.hcylinder2, self.hcylinder1, self.vcylinder1, self.vcylinder2,
                        self.box1, self.box2, self.sphere1, self.sphere2]

    def pick_random_object(self):
        i = randint(0,len(self.objects)-1)
        obj = self.objects[i]
        return obj
