import tf2_ros
import tf2_geometry_msgs
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose

class Transformer():
    def __init__(self,robot, source_frame='camera_depth_optical_frame', target_frame='arm_base_link'):
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot = robot

    def transform(self, action, before_feats):
        transform = self.tf_buffer.lookup_transform(self.target_frame,
                                   self.source_frame,
                                   rospy.Time(0),
                                   rospy.Duration(1.0))

        kinect_object = PointStamped()
        kinect_object.point.x = before_feats[0]
        kinect_object.point.y = before_feats[1]
        kinect_object.point.z = before_feats[2]
        kinect_object.header = transform.header

        robot_object = tf2_geometry_msgs.do_transform_point(kinect_object, transform)

        pos = Pose()
        pos.position.x = robot_object.point.x
        pos.position.y = robot_object.point.y
        pos.position.z = robot_object.point.z
        pos.orientation = action.orientation

        return pos
