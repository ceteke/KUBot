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

def pc_features_to_array(pc_feats):
    transform = pc_feats.transform
    translation = transform.translation
    rotation = transform.rotation
    pc_centroid = pc_feats.points_centroid
    pc_min = pc_feats.points_min
    pc_max = pc_feats.points_max
    rgba_color = pc_feats.rgba_color
    bb_center = pc_feats.bb_center
    bb_dims = pc_feats.bb_dims
    pc_size = pc_feats.num_points #???

    bb_volume = bb_dims.x * bb_dims.y * bb_dims.z
    bb_area = 2*(bb_dims.x*bb_dims.y + bb_dims.x*bb_dims.z + bb_dims.y*bb_dims.z)
    bb_aspect_ratio = bb_dims.y / bb_dims.x
    bb_area_over_volume = bb_area / bb_volume
    compactness = bb_volume / pc_size

    result = [335, bb_center.x, bb_center.y, bb_center.z, pc_feats.bb_angle,
                pc_centroid.x, pc_centroid.y, pc_centroid.z, pc_min.x, pc_min.y,
                pc_min.z, pc_max.x, pc_max.y, pc_max.z, rgba_color.r, rgba_color.g,
                rgba_color.b, pc_feats.hue, pc_size, bb_dims.x, bb_dims.y, bb_dims.z, bb_volume,
                bb_area, bb_aspect_ratio, bb_area_over_volume, compactness]

    return result + list(pc_feats.data)

def isclose(n1, n2, tol=0.001):
    return abs(n1 - n2) <= tol
