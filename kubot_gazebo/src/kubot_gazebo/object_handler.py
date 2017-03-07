#!/usr/bin/env python
from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose
from random import randint
from MyObject import Sphere, Box

class ObjectHandler:
    def __init__(self):
        self.sphere_pos1 = Pose()
        self.sphere_pos1.position.x = -0.108716
        self.sphere_pos1.position.y = 0.037888
        self.sphere_pos1.position.z = 0.806575
        self.sphere1 = Sphere('sphere1', self.sphere_pos1,1)

        self.sphere_pos2 = Pose()
        self.sphere_pos2.position.x = -0.281780
        self.sphere_pos2.position.y = -0.173955
        self.sphere_pos2.position.z = 0.806576
        self.sphere2 = Sphere('sphere2', self.sphere_pos2,2)

        self.box_pos1 = Pose()
        self.box_pos1.position.x = -0.138391
        self.box_pos1.position.y = 0.032788
        self.box_pos1.position.z = 0.806568
        self.box1 = Box('box1',self.box_pos1,1)

        self.box_pos2 = Pose()
        self.box_pos2.position.x = -0.294147
        self.box_pos2.position.y = -0.170558
        self.box_pos2.position.z = 0.805576
        self.box2 = Box('box2',self.box_pos2,2)

        self.objects = [self.sphere1, self.sphere2, self.box1, self.box2]

    def pick_random_object(self):
        i = randint(0,len(self.objects)-1)
        obj = self.objects[i]
        return obj.pose_num, obj
