from gazebo_interface import GazeboInterface
import rospkg
from geometry_msgs.msg import Pose, Twist, Quaternion

class MyObject(object):
    def __init__(self,name,urdf_file,should_spawn=True,orientation=None,is_sdf=False):
        self.name = name
        self.gazebo_interface = GazeboInterface()
        self.rospack = rospkg.RosPack()
        self.object_path = self.rospack.get_path('kubot_gazebo')+"/objects/"
        if is_sdf:
            self.model_path = self.rospack.get_path('kubot_gazebo')+"/models/"+self.name+"/model.sdf"
        self.urdf_file = urdf_file
        self.invisible_pose = Pose()
        self.invisible_pose.position.x = -100
        self.invisible_pose.position.y = -100
        self.invisible_pose.position.z = -100
        self.orientation = orientation
        self.is_sdf = is_sdf
        if orientation is not None:
            self.invisible_pose.orientation = self.orientation
        if should_spawn:
            if not is_sdf:
                self.gazebo_interface.spawn_object(self.object_path+self.urdf_file,
                                                self.name, self.invisible_pose,self.is_sdf)
            else:
                self.gazebo_interface.spawn_object(self.model_path,
                                                self.name, self.invisible_pose,self.is_sdf)

    def place_on_table(self):
        return self.gazebo_interface.set_object_pose(self.name,self.pose,twist=self.twist)

    def remove(self):
        return self.gazebo_interface.set_object_pose(self.name,self.invisible_pose)

    def set_position(self, position, twist=None):
        self.pose = position
        if self.orientation is not None:
            self.pose.orientation = self.orientation
        self.twist = twist

    def get_position(self):
        return self.gazebo_interface.get_object_pose(self.name)

class Sphere(MyObject):
    def __init__(self):
        MyObject.__init__(self,'sphere','basic_sphere.urdf')

class Box(MyObject):
    def __init__(self):
        MyObject.__init__(self,'box','basic_cube.urdf')

class Duck(MyObject):
    def __init__(self):
        MyObject.__init__(self,'duck',None, should_spawn=False, is_sdf=True)

class Bunny(MyObject):
    def __init__(self):
        MyObject.__init__(self,'bunny',None, is_sdf=True)

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
