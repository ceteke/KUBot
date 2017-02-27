import rospy
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

class GazeboInterface:
    def __init__(self, sim_context = '/gazebo'):
        self.model_state_service = sim_context+'/get_model_state'
        self.delete_object_service = sim_context+'/delete_model'
        self.spawn_object_service = sim_context+'/spawn_urdf_model'
        self.set_object_state_service = sim_context + '/set_model_state'

    def get_object_pose(self,object_name,return_twist = False):
        rospy.wait_for_service(self.model_state_service)
        try:
            gp = rospy.ServiceProxy(self.model_state_service, GetModelState)
            gpReq = GetModelStateRequest()
            gpReq.model_name = object_name;
            gpResp = gp(gpReq)
            if not gpResp.success:
                rospy.logerr('Failed to get object pose. %s'%gpResp.status_message)
                return None
            else:
                if return_twist:
                    return gpResp.pose, gpResp.twist
                else:
                    return gpResp.pose
        except rospy.ServiceException, e:
            rospy.logerr("Service called failed: %s"%e)
            return None

    def delete_object(self,object_name):
        rospy.wait_for_service(self.delete_object_service)
        try:
            dobj = rospy.ServiceProxy(self.delete_object_service, DeleteModel)
            dobjReq = DeleteModelRequest()
            dobjReq.model_name = object_name
            dobjResp = dobj(dobjReq)
            if not dobjResp.success:
                rospy.logerr('Failed to delete object. %s'%dobjResp.status_message)
                return False
        except rospy.ServiceException, e:
            rospy.logerr("Service called failed: %s"%e)
            return -1
        return 1

    def spawn_object(self,file_name, object_name, desired_pose):
        rospy.wait_for_service(self.spawn_object_service)
        try:
            so = rospy.ServiceProxy(self.spawn_object_service, SpawnModel)
            soReq = SpawnModelRequest()
            soReq.model_name = object_name
            f = open(file_name,'r')
            soReq.model_xml = f.read()
            soReq.initial_pose = desired_pose
            soResp = so(soReq)
            if not soResp.success:
                rospy.logerr('Failed to spawn object. %s'%soResp.status_message)
                return False
        except rospy.ServiceException, e:
            rospy.logerr("Service called failed: %s"%e)
            return False
        return True

    def set_object_pose(self,object_name,pose,twist=None):
        rospy.wait_for_service(self.set_object_state_service)
        try:
            sp = rospy.ServiceProxy(self.set_object_state_service, SetModelState)
            spReq = SetModelStateRequest()
            spReq.model_state.model_name = object_name;
            spReq.model_state.pose = pose
            if twist is not None:
                spReq.model_state.twist = desired_twist
            spResp = sp(spReq)
            if not spResp.success:
                rospy.logerr('Failed to set object pose. %s'%spResp.status_message)
                return -1
        except rospy.ServiceException, e:
            rospy.logerr("Service called failed: %s"%e)
            return -1
        return 1
