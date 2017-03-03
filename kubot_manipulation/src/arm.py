import roslib
import utils
import rospy
from arm_manipulator import ArmMoveIt
from moveit_msgs.srv import GetPositionIK
from copy import deepcopy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Arm:
    def __init__(self):
        self.initial_pose_arr = [0.152154051453,0.191454891608,0.897687109724,
            0.517374498524,0.517348885096,0.482040788803,0.481985930601]
        self.object_poses = {'south': [0.225551892984,0.340317621398,0.151336879897,
            -0.695678673187,-0.209043631254,0.324403767098,0.605882942323]}
        self.dumb_pose_arr = [0.81725000034,0.19145,-0.00549099853312,
            0.707106780552,0.707106781821,1.26918356229e-09,1.26918351361e-09]
        self.dumb_pose = utils.array_to_pose(self.dumb_pose_arr)
        #south angles sould be [0.7000298097253443,-1.4999774338590477,2.1999635583551616,-0.5001153552691662,1.4999666894284447,1.4999763591757844]
        self.initial_pose = utils.array_to_pose(self.initial_pose_arr)
        self.arm_planner = ArmMoveIt()
        self.pub_ang_cmd = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10, latch=True)
        self.joint_names = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint',
                            'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint'
                            , 'arm_wrist_3_joint']

    def go_initial(self):
        return self.go_to_pose(self.initial_pose)

    def go_next_to_object(self,way):
        pose = self.object_poses[way]
        plan = self.arm_planner.plan_poseTargetInput(utils.array_to_pose(pose))
        self.arm_planner.group[0].execute(plan)

    def push(self):
        start_pose = self.get_current_pose()
        print start_pose
        if utils.are_poses_equal(start_pose, self.dumb_pose):
            rospy.loginfo("Wrong calculated FK, try again")
            return
        object_pushed_pose = deepcopy(start_pose)
        object_pushed_pose.position.x += 0.1
        object_pushed_pose.position.y += 0.1
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

    def ang_cmd(self, angles):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        jp = JointTrajectoryPoint()
        jp.positions = angles
        jp.time_from_start = rospy.Duration.from_sec(2.0)
        trajectory.points = [jp]
        self.pub_ang_cmd.publish(trajectory)
