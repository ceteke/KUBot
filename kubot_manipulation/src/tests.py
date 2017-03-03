#!/usr/bin/env python
import rospy
from arm import Arm

def main():

    rospy.init_node('testing_stuff', anonymous=True)

    arm = Arm()
    arm.ang_cmd([0.7,-1.5,2.1,-0.5,1.5,1.5])
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
