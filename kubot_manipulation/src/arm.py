import roslib
import utils
import rospy
from arm_manipulator import ArmMoveIt
from moveit_msgs.srv import GetPositionIK
from copy import deepcopy

class Arm:
    def __init__(self):
        self.initial_pose_arr = [0.152154051453,0.191454891608,0.897687109724,
            0.517374498524,0.517348885096,0.482040788803,0.481985930601]
        self.initial_pose = utils.array_to_pose(self.initial_pose_arr)
        self.arm_planner = ArmMoveIt()

    def go_initial(self):
        return self.go_to_pose(self.initial_pose)

    def push(self):
        start_pose = self.get_current_pose()
        object_pushed_pose = deepcopy(start_pose)
        object_pushed_pose.position.y -= 0.3
        self.go_to_pose_cartesian([object_pushed_pose])

    def get_current_pose(self):
        return self.arm_planner.get_FK()[0].pose

    def change_joint_angles(self,angles):
        plan = self.arm_planner.plan_jointTargetInput(angles)
        self.arm_planner.group[0].execute(plan)

    def get_joint_state(self):
        return self.arm_planner.get_IK(self.get_current_pose())

    def go_to_pose(self,pose):
        plan = self.arm_planner.plan_poseTargetInput(pose)
        self.arm_planner.group[0].execute(plan)

    def go_to_pose_cartesian(self,waypoints):
        (plan, fraction) = self.arm_planner.group[0].compute_cartesian_path (waypoints, 0.01, 0.0, False)
        rospy.loginfo("Path computed successfully with %f fraction. Moving the arm." % fraction)
        self.arm_planner.group[0].execute(plan)

    def go_random(self):
        waypoints = [[utils.random_joint_angle() for i in range(6)] for j in range(3)]
        return self.change_joint_angles(waypoints)
