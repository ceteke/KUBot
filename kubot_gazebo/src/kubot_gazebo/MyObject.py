from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose, Twist, Quaternion

class MyObject(object):
    def __init__(self,name,urdf_file,orientation=None):
        self.name = name
        self.gazebo_interface = GazeboInterface()
        self.rospack = rospkg.RosPack()
        self.object_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        self.urdf_file = urdf_file
        self.invisible_pose = Pose()
        self.invisible_pose.position.x = -100
        self.invisible_pose.position.y = -100
        self.invisible_pose.position.z = -100
        self.orientation = orientation
        if orientation is not None:
            self.invisible_pose.orientation = self.orientation
        self.gazebo_interface.spawn_object(self.object_path+self.urdf_file,
                                            self.name, self.invisible_pose)

    def place_on_table(self):
        return self.gazebo_interface.set_object_pose(self.name,self.pose,twist=self.twist)

    def remove(self):
        return self.gazebo_interface.set_object_pose(self.name,self.invisible_pose)

    def set_position(self, position, twist=None):
        self.pose = position
        if self.orientation is not None:
            self.pose.orientation = self.orientation
        self.twist = twist

class Sphere(MyObject):
    def __init__(self):
        MyObject.__init__(self,'sphere','basic_sphere.urdf')

class Box(MyObject):
    def __init__(self):
        MyObject.__init__(self,'box','basic_cube.urdf')

class Cylinder(MyObject):
    def __init__(self,type):
        if type == 'v':
            MyObject.__init__(self,'vcylinder','basic_cylinder.urdf')
        if type == 'h':
            h_or = Quaternion()
            h_or.x = 0.706939881073
            h_or.y = 0.0156166025414
            h_or.z = 0.0156778071406
            h_or.w = 0.706927388518
            MyObject.__init__(self,'hcylinder','basic_cylinder.urdf', orientation=h_or)
