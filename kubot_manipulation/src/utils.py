import rospy
import random
import tf
import geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import PositionIKRequest

def trans_rot_to_pose(trans,rot):
    p = geometry_msgs.msg.Pose()
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = trans[2]

    p.orientation.x = rot[0]
    p.orientation.y = rot[1]
    p.orientation.z = rot[2]
    p.orientation.w = rot[3]

    return p

def array_to_pose(arr):
    p = geometry_msgs.msg.Pose()
    p.position.x = arr[0]
    p.position.y = arr[1]
    p.position.z = arr[2]

    p.orientation.x = arr[3]
    p.orientation.y = arr[4]
    p.orientation.z = arr[5]
    p.orientation.w = arr[6]

    return p

def pose_to_array(p):
    return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]

def random_joint_angle():
    return random.uniform(0.5, 5.0)
