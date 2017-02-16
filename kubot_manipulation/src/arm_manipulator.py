#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
from math import pi, floor, ceil, fabs

class ArmMoveIt:

  def __init__(self, planning_frame='arm_base_link', default_planner="RRTConnectkConfigDefault"):

    # Make sure the moveit service is up and running
    rospy.logwarn("Waiting for MoveIt! to load")
    try:
      rospy.wait_for_service('compute_ik')
    except rospy.ROSExecption, e:
      rospy.logerr("No moveit service detected. Exiting")
      exit()
    else:
      rospy.loginfo("MoveIt detected: arm planner loading")

    self.robot = moveit_commander.RobotCommander()

    self.scene = moveit_commander.PlanningSceneInterface()

    self.group = [moveit_commander.MoveGroupCommander("arm")]

    self.planner = default_planner

    self.group[0].set_pose_reference_frame(planning_frame)

    self.continuous_joints = ['shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    self.continuous_joints_list = [0,3,4,5]

  def get_IK(self, newPose, root = None):
    ## from a defined newPose (geometry_msgs.msg.Pose()), retunr its correspondent joint angle(list)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)

    wkPose = geometry_msgs.msg.PoseStamped()
    if root is None:
      wkPose.header.frame_id = self.group[0].get_planning_frame() # name:odom
    else:
      wkPose.header.frame_id = root

    wkPose.header.stamp=rospy.Time.now()
    wkPose.pose=newPose

    msgs_request = moveit_msgs.msg.PositionIKRequest()
    msgs_request.group_name = self.group[0].get_name() # name: arm
    # msgs_request.robot_state = robot.get_current_state()
    msgs_request.pose_stamped = wkPose
    msgs_request.timeout.secs = 2
    msgs_request.avoid_collisions = False

    try:
      jointAngle=compute_ik(msgs_request)
      ans=list(jointAngle.solution.joint_state.position[1:7])
      if jointAngle.error_code.val == -31:
        print 'No IK solution'
        return None
      return ans

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def get_FK(self, root = 'arm_base_link'):

    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv.GetPositionFK)

    header = std_msgs.msg.Header()
    header.frame_id = root
    header.stamp = rospy.Time.now()
    fk_link_names = ['arm_ee_link']
    robot_state = self.robot.get_current_state()
    try:
      reply=compute_fk(header,fk_link_names,robot_state)
      return reply.pose_stamped

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def plan_targetInput(self, target, joint_flag):
    '''Generic target planner that what type is specified'''
    try:
      if (joint_flag):
        self.group[0].set_joint_value_target(self._simplify_joints(target))
      else:
        self.group[0].set_pose_target(target)

      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None


  def plan_targetInputWaypoint(self, targets, joint_flag, merged=False, current_joints=None):
    '''Generic target planner that what type is specified'''
    ## input: list of pose (geometry_msgs.msg.Pose())
    ## output: plan from current  pose all of the target poses
    ## If merge true - then a single large plan is returned
    ## If merge is false - then several plans in an array are returned

    # Plan each pose individually and stitch together
    try:
      full_plan = []
      points = []
      current_state = self.robot.get_current_state()
      if current_joints is not None:
          current_state = self.set_robot_state_joint_dict(current_joints)
      for target in targets:
        self.group[0].set_start_state(current_state)
        plan = self.plan_targetInput(target, joint_flag)
        if plan is not None:
          full_plan.append(plan)
          if merged:
            points = self.merge_points(points, plan.joint_trajectory.points)
          traj = plan.joint_trajectory
          current_state = self.set_robot_state_pose(traj)
        else:
          print 'No full plan found, see the moveit terminal for the error'
          return None

      if merged:
        plan = full_plan[0]
        plan.joint_trajectory.points = points
        return plan
      else:
        return full_plan

    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  def set_robot_state_pose(self, traj):
    '''Gets the current robot state pose and sets it to the joint pose'''
    cur_robot_state = self.robot.get_current_state()
    last_point = traj.points[-1].positions
    # convert the joints to array
    joints = [x for x in cur_robot_state.joint_state.position]
    for i in xrange(len(traj.joint_names)):
      # Find index of joint
      joint_name = traj.joint_names[i]
      idx = cur_robot_state.joint_state.name.index(joint_name)
      joints[idx] = last_point[i]

    # Set full joint tuple now
    cur_robot_state.joint_state.position = joints

    return cur_robot_state

  def set_robot_state_joint_dict(self, joint_dict):
    cur_robot_state = self.robot.get_current_state()
    joints = [x for x in cur_robot_state.joint_state.position]
    for joint_name in joint_dict:
        idx = cur_robot_state.joint_state.name.index(joint_name)
        joints[idx] = joint_dict[joint_name]

    cur_robot_state.joint_state.position = joints
    return cur_robot_state

  def merge_points(self, points, new_points):
    '''Merge two sets of points and taking into account time'''
    # Check if this is the first set
    if len(points) < 1:
      return new_points

    all_points = points
    # Pull out the last time from current points
    last_point_time = points[-1].time_from_start+rospy.Duration(0.1)
    for point in new_points:
      point.time_from_start = point.time_from_start+last_point_time
      all_points = all_points + [point]
    return all_points

  def _simplify_angle(self, angle):
    # Very simple function that makes sure the angles are between -pi and pi
    if angle > pi:
      while angle > pi:
        angle -= 2*pi
    elif angle < -pi:
      while angle < -pi:
        angle += 2*pi

    return angle

  def _simplify_joints(self, joint_dict):
    # Helper function to convert a dictionary of joint values
    if isinstance(joint_dict, dict):
      simplified_joints = dict()
      for joint in joint_dict:
        # Pull out the name of the joint
        joint_name = '_'.join(joint.split('_')[1::])
        if joint_name in self.continuous_joints:
          simplified_joints[joint] = self._simplify_angle(joint_dict[joint])
        else:
          simplified_joints[joint] = joint_dict[joint]
    elif isinstance(joint_dict, list):
      simplified_joints = []
      for i in xrange(len(joint_dict)):
        a = joint_dict[i]
        if i in self.continuous_joints_list:
          simplified_joints.append(self._simplify_angle(a))
        else:
          simplified_joints.append(a)
    return simplified_joints

  '''Older functions - left for backwards compatibility'''

  def plan_jointTargetInput(self,target_joint):
    ## input: target joint angles (list) of the robot
    ## output: plan from current joint angles to the target one
    try:
      self.group[0].set_joint_value_target(self._simplify_joints(target_joint))
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None

  def plan_poseTargetInput(self,target_pose):
    ## input: tart pose (geometry_msgs.msg.Pose())
    ## output: plan from current  pose to the target one
    try:
      self.group[0].set_pose_target(target_pose)
      self.group[0].set_planner_id(self.planner)
      planAns=self.group[0].plan()
      return planAns
    except:
      print 'No plan found, see the moveit terminal for the error'
      print("Unexpected error:", sys.exc_info()[0])
      return None
