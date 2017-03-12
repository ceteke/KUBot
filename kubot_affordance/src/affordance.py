#!/usr/bin/env python
import roslib; roslib.load_manifest('kubot_manipulation'); roslib.load_manifest('kubot_gazebo')
import rospy
from kubot_manipulation.robot import Robot
from kubot_manipulation.MyAction import Push
from kubot_gazebo.object_handler import ObjectHandler
from kubot_gazebo.gazebo_interface import GazeboInterface
from random import randint

def main():
    rospy.init_node('kubot_affordance', anonymous=True)
    robot = Robot()
    object_handler = ObjectHandler()
    gazebo_interface = GazeboInterface()

    action_poses = \
        {'push':[[0.122082807535,0.253172402032,0.0547882234144,
            -0.675679130692,-0.282736229211,0.24811287633,0.634001528104],
        #0.7,-1.5,2.7,5.0599,-4.7834,1.4994
        [-0.283016464947,0.457409320807,0.0585147204124,
            -0.636984559527,-0.249563909878,0.245820513626,0.686688285098]]}
        #2.0714,-0.9666,1.7952,-0.9666,2.9249,1.5

    actions = [Push('push',robot)]

    iteration_num = 0
    while True:
        i = randint(0,len(actions)-1)
        action = actions[i]

        iteration_num += 1
        picked_object = object_handler.pick_random_object()
        rospy.loginfo('Picked object: ' + picked_object.name)
        if action.prepare(action_poses[action.name][picked_object.pose_num]) == -1:
            rospy.loginfo("Faild to go next to %s passing..." %(picked_object.name))
            continue
        picked_object.place_on_table()
        robot.eye.save_data(picked_object.name, action.name, iteration_num, 0)
        rospy.loginfo("Performing action: %s"%(action.name))
        action.execute()
        rospy.sleep(3)
        robot.eye.save_data(picked_object.name, action.name, iteration_num, 1)
        gazebo_interface.delete_object(picked_object.name)
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
