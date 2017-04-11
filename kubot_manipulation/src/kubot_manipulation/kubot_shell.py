#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_gazebo')
import rospy
from arm import Arm
from hand import Hand
from robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface
from MyAction import Push
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PointStamped, Pose


def main():
    rospy.init_node('kubot_shell', anonymous=True)
    robot = Robot()
    oh = ObjectHandler()
    push_action = Push(robot)
    gi = GazeboInterface()
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    action_poses = \
        {'push':[
            [0.122082807535,0.253172402032,0.0547882234144,
                -0.675679130692,-0.282736229211,0.24811287633,0.634001528104],
            #0.7,-1.5,2.7,5.0599,-4.7834,1.4994
            [-0.283016464947,0.457409320807,0.0585147204124,
                -0.636984559527,-0.249563909878,0.245820513626,0.686688285098]
            ]
            #2.0714,-0.9666,1.7952,-0.9666,2.9249,1.5
        }

    while True:
        raw = raw_input('$')
        if len(raw) == 0:
            continue
        cmd_input = raw.split(' ')
        cmd = cmd_input[0]
        if cmd == 'exit':
            break
        elif cmd == 'get_arm_position':
            print robot.arm.get_current_pose()
        elif cmd == 'change_arm_joint_angles':
            args = cmd_input[1]
            angles = [float(num) for num in args.split(',')]
            robot.arm.change_joint_angles(angles)
        elif cmd == 'ang_cmd':
            args = cmd_input[1]
            angles = [float(num) for num in args.split(',')]
            robot.arm.ang_cmd(angles)
        elif cmd == 'get_arm_joint_angles':
            print arm.get_joint_state()
        elif cmd == 'push':
            push_action.execute()
        elif cmd == 'go_initial':
            robot.arm.go_initial()
        elif cmd == 'close_gripper':
            robot.hand.close_gripper()
        elif cmd == 'open_gripper':
            robot.hand.open_gripper()
        elif cmd == 'spawn_sphere1':
            oh.sphere1.place_on_table()
        elif cmd == 'spawn_sphere2':
            oh.sphere2.place_on_table()
        elif cmd == 'spawn_box1':
            oh.box1.place_on_table()
        elif cmd == 'spawn_box2':
            oh.box2.place_on_table()
        elif cmd == 'go_to':
            action_name = cmd_input[1]
            pose_num = cmd_input[2]
            robot.arm.go_to_pose(action_poses[action_name][int(pose_num)])
        elif cmd == 'get_object_pose':
            o_name = cmd_input[1]
            print gi.get_object_pose(o_name)
        elif cmd == 'tf':
            transform = tf_buffer.lookup_transform('arm_base_link',
                                       'camera_depth_optical_frame', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            res = PointStamped()
            res.point.x = -0.037543
            res.point.y = -0.364611
            res.point.z = 1.00199
            res.header = transform.header
            point_transformed = tf2_geometry_msgs.do_transform_point(res, transform)

            pos = Pose()
            pos.position.x = point_transformed.point.x
            pos.position.y = point_transformed.point.y
            pos.position.z = point_transformed.point.z
            pos.orientation.x = -0.636984559527
            pos.orientation.y = -0.249563909878
            pos.orientation.z = 0.245820513626
            pos.orientation.w = 0.686688285098
            print pos

            robot.arm.go_to_pose(pos)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
