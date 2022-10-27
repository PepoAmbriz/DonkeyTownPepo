from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest
from gazebo_msgs.srv import GetLinkStateResponse
import rospy

class ObstaclePose: 
	def __init__(self,name):
		self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.get_link_req   = GetLinkStateRequest()
		self.get_link_req.link_name = str(name)+'::base_link'
	def get_pose(self): 
		link_state = self.get_link_state(self.get_link_req)
		return link_state.link_state.pose

class CameraPose:
	def __init__(self):
		self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.get_link_req   = GetLinkStateRequest()
		self.get_link_req.link_name = 'AutoModelMini::camera_link'
	def get_pose(self): 
		link_state = self.get_link_state(self.get_link_req)
		return link_state.link_state.pose