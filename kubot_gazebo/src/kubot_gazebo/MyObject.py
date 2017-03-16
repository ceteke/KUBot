from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose

class MyObject(object):
    def __init__(self,name,pose,pose_num,urdf_file):
        self. pose = pose
        self.name = name
        self.pose_num = pose_num
        self.id = '%s_%d' % (self.name, self.pose_num)
        self.gazebo_interface = GazeboInterface()
        self.rospack = rospkg.RosPack()
        self.object_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        self.urdf_file = urdf_file
        self.invisible_pose = Pose()
        self.invisible_pose.position.x = self.pose.position.x - 100
        self.invisible_pose.position.y = self.pose.position.y - 100
        self.invisible_pose.position.z = self.pose.position.z - 100
        self.gazebo_interface.spawn_object(self.object_path+self.urdf_file,
                                            self.id, self.invisible_pose)

    def place_on_table(self):
        return self.gazebo_interface.set_object_pose(self.id,self.pose)

    def remove(self):
        return self.gazebo_interface.set_object_pose(self.id,self.invisible_pose)

class Sphere(MyObject):
    def __init__(self,pose,pose_num):
        MyObject.__init__(self,'sphere',pose,pose_num,'basic_sphere.urdf')

class Box(MyObject):
    def __init__(self,pose,pose_num):
        MyObject.__init__(self,'box',pose,pose_num,'basic_cube.urdf')
