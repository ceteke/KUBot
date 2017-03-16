from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose

class MyObject(object):
    def __init__(self,name,pose,pose_num):
        self. pose = pose
        self.name = name
        self.pose_num = pose_num
        self.gazebo_interface = GazeboInterface()
        self.rospack = rospkg.RosPack()
        self.object_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        self.urdf_file = ''

    def place_on_table(self):
        return self.gazebo_interface.spawn_object(self.object_path+self.urdf_file,
                                            self.name, self.pose)

class Sphere(MyObject):
    def __init__(self,pose,pose_num):
        MyObject.__init__(self,'sphere',pose,pose_num)
        self.urdf_file = 'basic_sphere.urdf'

class Box(MyObject):
    def __init__(self,pose,pose_num):
        MyObject.__init__(self,'box',pose,pose_num)
        self.urdf_file = 'basic_cube.urdf'
