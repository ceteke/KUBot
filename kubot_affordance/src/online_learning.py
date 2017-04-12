#!/usr/bin/env python
import rospy
from affordance_core import AffordanceCore, IterationError

def main():
    rospy.init_node('kubot_online_learning', anonymous=True)
    affordance_core = AffordanceCore()
    while True:
        try:
            affordance_core.iterate(False, True)
        except IterationError as e:
            rospy.loginfo(e.message)
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
