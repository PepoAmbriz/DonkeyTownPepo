#!/usr/bin/env python2 
import rospy 
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo  
import sys
import numpy as np

#Camera pose from gazebo
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest
from gazebo_msgs.srv import GetLinkStateResponse

import geometry_msgs.msg
from tf.transformations import quaternion_matrix, euler_matrix, euler_from_quaternion
import tf2_ros

from sensor_msgs.msg import PointCloud

def rpy_from_quaternion(q): 
	x = q.x
	y = q.y
	z = q.z
	w = q.w
	return euler_from_quaternion([x,y,z,w])

def rospose2homo(rospose): 
	q = rospose.orientation
	p = rospose.position
	dv = [p.x,p.y,p.z]
	quat = [q.x, q.y, q.z, q.w]
	homo = quaternion_matrix(quat)
	homo[0:3,3] = dv
	return homo

class Obstacle_Pose: 
	def __init__(self,name):
		self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.get_link_req   = GetLinkStateRequest()
		self.get_link_req.link_name = str(name)+'::base_link'
	def get_pose(self): 
		link_state = self.get_link_state(self.get_link_req)
		return link_state.link_state.pose

class Camera_Pose:
	def __init__(self):
		self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.get_link_req   = GetLinkStateRequest()
		self.get_link_req.link_name = 'AutoModelMini::camera_link'
	def get_pose(self): 
		link_state = self.get_link_state(self.get_link_req)
		return link_state.link_state.pose

class Bbbox_Basis:
	def __init__(self,car_id,detection): 
		self.id = str(car_id)+"."+str(detection.results[0].id)
		self.xc = detection.bbox.center.x
		self.yc = detection.bbox.center.y 
		self.sx = detection.bbox.size_x
		self.sy = detection.bbox.size_y
	def get_lowcenter(self):
		return [self.xc,self.yc+self.sy/2.0]
	def get_corners(self):
		cx1 = self.xc-self.sx/2.0
		cx2 = self.xc+self.sx/2.0
		cy1 = self.yc-self.sy/2.0
		cy2 = self.yc+self.sy/2.0
		return [[cx1,cy1],[cx2,cy1],[cx2,cy2],[cx1,cy2]]


class Pedestrian_Detection: 
	def __init__(self, car_id): 
		self.ped_r = 0.07
		self.car_id = car_id

		self.mat = [[]]
		self.dist = []
		self.caminfo_sub = rospy.Subscriber("/app/camera/rgb/camera_info",CameraInfo,self.on_caminfo)

		self.br = tf2_ros.TransformBroadcaster()
		self.t = geometry_msgs.msg.TransformStamped()
		self.t.header.frame_id = "map"

		self.img_pose = euler_matrix(0.0,np.pi/2.0,-np.pi/2.0,'rxyz')
		self.camera_pose = Camera_Pose()
		#self.obs_pose = Obstacle_Pose('osito_blanco_1') #get obstacle pose from gazebo
		self.detectpts_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray,self.on_pts)

		self.publisher = rospy.Publisher("/sensors/obstacles/",PointCloud,queue_size=1)

	def on_pts(self,msg): 
		#print("gazebo pose: ")
		#print(self.obs_pose.get_pose())

		#msg init
		pcmsg = PointCloud()
		pcmsg.header.stamp = msg.header.stamp
		pts = []
		
		cam_rosmsg_pose = self.camera_pose.get_pose()
		_,_,yaw = rpy_from_quaternion(cam_rosmsg_pose.orientation)
		#Fixed descentralized point. 
		#pose correction (camera object and camera rays pose are not the same)
		hc = np.matmul(rospose2homo(cam_rosmsg_pose),self.img_pose)
		hc_inv = np.linalg.inv(hc)
		K2 = np.zeros((3,4))
		K2[0:3,0:3] = self.mat
		C = np.matmul(K2,hc_inv)

		det_count = 0
		for det in msg.detections: 
			#position estimation
			bbox = Bbbox_Basis(self.car_id,det)
			if bbox.sx > 500: 
				break
			u,v = bbox.get_lowcenter()

			#Future work: Correct distortion 

			#Solving Mx=b
			eqs_M = np.array([	[C[0,0]-C[2,0]*u, C[0,1]-C[2,1]*u], 
								[C[1,0]-C[2,0]*v, C[1,1]-C[2,1]*v]])
			eqs_b = np.array(	[C[2,3]*u-C[0,3], C[2,3]*v-C[1,3]])
			x,y = np.linalg.solve(eqs_M,eqs_b)

			#transform broadcast (for rviz)
			
			self.t.transform.translation.x = x
			self.t.transform.translation.y = y 
			self.t.transform.translation.z = 0
			self.t.transform.rotation.x = 0
			self.t.transform.rotation.y = 0
			self.t.transform.rotation.z = 0
			self.t.transform.rotation.w = 1
			self.t.child_frame_id = str(self.car_id)+"_obstacle_"+str(det_count) 
			self.br.sendTransform(self.t)
			
			det_count = det_count+1

			#publish msg
			point = geometry_msgs.msg.Point32()
			point.x = x+self.ped_r*np.cos(yaw)
			point.y = y+self.ped_r*np.sin(yaw)
			point.z = 0
			pts.append(point)
		if det_count>0: 
			pcmsg.points = pts
			self.publisher.publish(pcmsg)

	def on_caminfo(self,msg): 
		self.mat = np.reshape(msg.K,(3,3))
		self.dist = np.asarray(msg.D)
		self.caminfo_sub.unregister()
		#get camera intrinsic matrix and distortion coefficients.

def main(args): 
	rospy.init_node('PedestrianDetection', anonymous=True)
	node = Pedestrian_Detection(4)
	try:
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down")
	

if __name__ == '__main__': 
	main(sys.argv)