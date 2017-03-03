import rospy
import random
import tf
import geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import PositionIKRequest
from tf.transformations import euler_from_quaternion

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

def are_poses_equal(p1, p2):
    pos1 = p1.position
    pos2 = p2.position
    if isclose(pos1.x, pos2.x) and isclose(pos1.y, pos2.y) and isclose(pos1.z, pos2.z):
        o1 = p1.orientation
        o2 = p2.orientation
        r1, p1, y1 = euler_from_quaternion([o1.x, o1.y, o1.z, o1.w])
        r2, p2, y2 = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
        if isclose(r1, r2) and isclose(p1, p2) and isclose(y1, y2):
            return True
    return False


def isclose(n1, n2, tol=0.001):
    return abs(n1 - n2) <= tol
