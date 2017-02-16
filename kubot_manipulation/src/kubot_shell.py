#!/usr/bin/env python
import rospy
from arm_manipulator import ArmMoveIt

def main():
    rospy.init_node('kubot_shell', anonymous=True)
    arm_manipulator = ArmMoveIt()
    while True:
        raw = raw_input('$')
        if len(raw) == 0:
            continue
        cmd_input = raw.split(' ', 1)
        cmd = cmd_input[0]
        if cmd == 'exit':
            break
        elif cmd == 'get_arm_position':
            print arm_manipulator.get_FK()
        elif cmd == 'change_arm_joint_angles':
            args = cmd_input[1]
            angles = [float(num) for num in args.split(',')]
            arm_manipulator.plan_jointTargetInput(angles)
        elif cmd == 'get_arm_joint_angles':
            print arm_manipulator.get_IK(arm_manipulator.get_FK()[0].pose)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
