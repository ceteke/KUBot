#!/usr/bin/env python
import roslib
import utils
import rospy
import moveit_msgs.msg
import moveit_msgs.srv
import moveit_commander
import std_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Arm:
    def __init__(self):
        self.root = 'arm_base_link'

        self.pub_ang_cmd = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10, latch=True)
        self.joint_names = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint',
                            'arm_elbow_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint'
                            , 'arm_wrist_3_joint']

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_pose_reference_frame(self.root)
        self.prev_pose = self.get_current_pose()

    def get_current_pose(self):
        return self.get_FK()[0].pose

    def get_FK(self):
        rospy.wait_for_service('compute_fk')
        compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

        header = std_msgs.msg.Header()
        header.frame_id = self.root
        header.stamp = rospy.Time.now()
        fk_link_names = ['arm_ee_link']
        robot_state = self.robot.get_current_state()
        try:
          reply=compute_fk(header,fk_link_names,robot_state)
          return reply.pose_stamped

        except rospy.ServiceException, e:
          print "Service call failed: %s"%e

    def get_IK(self, pose):
        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)
        robot_state = self.robot.get_current_state()

        wkPose = geometry_msgs.msg.PoseStamped()
        wkPose.header.frame_id = self.root
        wkPose.header.stamp=rospy.Time.now()
        wkPose.pose=pose

        msgs_request = moveit_msgs.msg.PositionIKRequest()
        msgs_request.group_name = self.group.get_name()
        msgs_request.pose_stamped = wkPose
        msgs_request.timeout.secs = 2
        msgs_request.avoid_collisions = False

        try:
          jointAngle=compute_ik(msgs_request)
          ans=list(jointAngle.solution.joint_state.position[0:6])
          if jointAngle.error_code.val == -31:
            print 'No IK solution'
            return None
          if all(x==0.0 for x in ans):
              print "Wrong IK"
              return None
          return ans

        except rospy.ServiceException, e:
          print "Service call failed: %s"%e

    def change_joint_angles(self,angles):
        plan = self.arm_planner.plan_jointTargetInput(angles)
        self.arm_planner.group[0].execute(plan)

    def get_joint_state(self):
        return self.get_IK(self.get_current_pose())

    def go_to_pose(self,pose):
        self.group.set_pose_target(pose)
        plan = self.group.plan()
        if len(plan.joint_trajectory.points) == 0:
            return -1
        self.prev_pose = self.get_current_pose()
        self.group.execute(plan)
        self.group.clear_pose_targets()
        return 1

    def go_prev_pose(self):
        return self.go_to_pose(self.prev_pose)

    def go_to_object(self,pose,obj_name,obj_pose,obj_size):
        self.scene.remove_world_object()
        obj_pose_stamped = geometry_msgs.msg.PoseStamped()
        obj_pose_stamped.header.frame_id = self.robot.get_planning_frame()
        obj_pose_stamped.pose = obj_pose
        self.scene.add_box(obj_name, obj_pose_stamped, obj_size)
        return self.go_to_pose(pose)

    def go_to_pose_cartesian(self,waypoints):
        trial = 0
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, False)
        while fraction != 1.0:
            if trial > 5:
                return -1
            rospy.loginfo("Path computed with %f fraction. Retrying..." % fraction)
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, False)
            trial += 1
        self.prev_pose = self.get_current_pose()
        rospy.loginfo("Path computed successfully with %f fraction. Moving the arm." % fraction)
        self.group.execute(plan)
        return 1

    def ang_cmd(self, angles):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        jp = JointTrajectoryPoint()
        jp.positions = angles
        jp.time_from_start = rospy.Duration.from_sec(2.0)
        trajectory.points = [jp]
        self.pub_ang_cmd.publish(trajectory)
