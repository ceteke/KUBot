import rospy
from math import pi
from std_msgs.msg import Float64

class Hand:

    def __init__(self):

        self.finger_position_controllers = ['/hand_j12_position_controller',
                                     '/hand_j22_position_controller',
                                     '/hand_j32_position_controller']

        self.finger_position_pub = [rospy.Publisher(pc+'/command', Float64, queue_size=10) for pc in self.finger_position_controllers]

        self.orientation_controller = '/hand_j11_position_controller'
        self.orientation_pub = rospy.Publisher(self.orientation_controller+'command', Float64, queue_size=10)

    def set_orientation(self, angle):
        self.orientation_pub.publish(angle)

    def open_gripper(self):
        for pub in self.finger_position_pub:
            pub.publish(0.0)

    def close_gripper(self):
        for pub in self.finger_position_pub:
            pub.publish(2*pi/3)
