#!/usr/bin/env python
from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose
import random

class ObjectHandler:
    def __init__(self):
        self.sphere_pos1 = Pose()
        self.sphere_pos1.position.x = -0.108716
        self.sphere_pos1.position.y = 0.037888
        self.sphere_pos1.position.z = 0.806575

        self.box_pos1 = Pose()
        self.box_pos1.position.x = -0.138391
        self.box_pos1.position.y = 0.032788
        self.box_pos1.position.z = 0.806568

        self.rospack = rospkg.RosPack()
        self.objects_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        self.gazebo_interface = GazeboInterface()

        self.sphere_poses = [self.sphere_pos1,]
        self.box_poses = [self.box_pos1]

    def get_object_file(self,urdf_file_name):
        return self.objects_path+urdf_file_name

    def spawn_object(self,file_name,name,pose):
        file_path = self.get_object_file(file_name)
        return self.gazebo_interface.spawn_object(file_path,name,pose)

    def spawn_box_on_table(self,box_name):
        box_pose = random.choice(self.box_poses)
        return self.spawn_object('basic_cube.urdf',box_name,box_pose)

    def spawn_sphere_on_table(self,sphere_name):
        sphere_pose = random.choice(self.sphere_poses)
        return self.spawn_object('basic_sphere.urdf',sphere_name,sphere_pose)
