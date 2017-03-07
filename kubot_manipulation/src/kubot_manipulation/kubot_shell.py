#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_gazebo')
import rospy
from arm import Arm
from hand import Hand
from eye import Eye
from kubot_gazebo.object_handler import ObjectHandler

def main():
    rospy.init_node('kubot_shell', anonymous=True)
    arm = Arm()
    hand = Hand()
    eye = Eye()
    oh = ObjectHandler()
    while True:
        raw = raw_input('$')
        if len(raw) == 0:
            continue
        cmd_input = raw.split(' ', 1)
        cmd = cmd_input[0]
        if cmd == 'exit':
            break
        elif cmd == 'get_arm_position':
            print arm.get_current_pose()
        elif cmd == 'change_arm_joint_angles':
            args = cmd_input[1]
            angles = [float(num) for num in args.split(',')]
            arm.change_joint_angles(angles)
        elif cmd == 'ang_cmd':
            args = cmd_input[1]
            angles = [float(num) for num in args.split(',')]
            arm.ang_cmd(angles)
        elif cmd == 'get_arm_joint_angles':
            print arm.get_joint_state()
        elif cmd == 'push':
            arm.push()
        elif cmd == 'go_initial':
            arm.go_initial()
        elif cmd == 'close_gripper':
            hand.close_gripper()
        elif cmd == 'open_gripper':
            hand.open_gripper()
        elif cmd == 'spawn_sphere':
            oh.sphere1.place_on_table()
        elif cmd == 'go_next_to_object':
            way = cmd_input[1]
            arm.go_next_to_object(way)
        elif cmd == 'save_pcd':
            eye.save_pcd()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass