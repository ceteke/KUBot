from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose

class ObjectHandler:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.objects_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        self.gazebo_interface = GazeboInterface()

    def get_object_file(self,urdf_file_name):
        return self.objects_path+urdf_file_name

    def spawn_object(self,file_name,name,pose):
        file_path = self.get_object_file(file_name)
        return self.gazebo_interface.spawn_object(file_path,name,pose)

    def spawn_cube_on_table(self,cube_name):
        cube_pose = Pose()
        cube_pose.position.x = 0.95
        cube_pose.position.y = -0.35
        cube_pose.position.z = 1.2
        return self.spawn_object('basic_cube.urdf',cube_name,cube_pose)

    def spawn_sphere_on_table(self,sphere_name):
        sphere_pose = Pose()
        sphere_pose.position.x = 0.0
        sphere_pose.position.y = 0.0
        sphere_pose.position.z = 0.0
        return self.spawn_object('basic_sphere.urdf',sphere_name,sphere_pose)
