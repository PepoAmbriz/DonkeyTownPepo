import rospy 
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo  
import sys
import numpy as np

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

class Bbbox_Basis:
	def __init__(self,detection): 
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

class PixelLocator:
	def __init__(self,mat,dist):
		self.ped_r = 0.07
		self.cam_mat = mat
		self.cam_dist = dist
		#self.img_pose = euler_matrix(0.0,np.pi/2.0,-np.pi/2.0,'rxyz')
		self.img_pose = np.eye(4)
	def set_cam_model(self, mat, dist):
		self.cam_mat = mat
		self.cam_dist = dist
	def compute_localization(self,camera_pose, detections):
		hc = np.matmul(rospose2homo(camera_pose),self.img_pose)
		hc_inv = np.linalg.inv(hc)
		K2 = np.zeros((3,4))
		K2[0:3,0:3] = self.cam_mat
		C = np.matmul(K2,hc_inv)
		pts = []
		for det in detections: 
			#position estimation
			bbox = Bbbox_Basis(det)
			if bbox.sx > 500: 
				continue
			u,v = bbox.get_lowcenter()

			#Future work: Correct distortion 

			#Solving Mx=b
			eqs_M = np.array([	[C[0,0]-C[2,0]*u, C[0,1]-C[2,1]*u], 
								[C[1,0]-C[2,0]*v, C[1,1]-C[2,1]*v]])
			eqs_b = np.array(	[C[2,3]*u-C[0,3], C[2,3]*v-C[1,3]])
			x,y = np.linalg.solve(eqs_M,eqs_b)
			pts.append([x,y])
		return pts