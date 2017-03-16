#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_gazebo')
import rospy
from arm import Arm
from hand import Hand
from robot import Robot
from kubot_gazebo.object_handler import ObjectHandler
from MyAction import Push

def main():
    rospy.init_node('kubot_shell', anonymous=True)
    robot = Robot()
    oh = ObjectHandler()
    push_action = Push(robot)

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

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
