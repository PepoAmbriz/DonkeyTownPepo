import rospy 
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose as PoseMsg

import yaml 
import os
from collections import deque
import numpy as np

from tf.transformations import euler_matrix, euler_from_matrix

def subs_ang(th1,th2):
	dth = (th1-th2)%(2*np.pi)
	if dth>np.pi:
		dth %= -np.pi
	return dth

class SensorQ:
	def __init__(self,width,depth,timeout=1,stamp=0):
		self.qs = [deque(maxlen=depth) for i in range(width)]
		self.width = width
		self.depth = depth
		self.stamp = stamp
		self.timeout = timeout
		self.mean = [0 for i in range(width)]
		self.var = [0 for i in range(width)]
	def push(self,data,stamp):
		if stamp-self.stamp > self.timeout:
			self.flush()
			print("Flushing")
		self.stamp = stamp
		for i in range(self.width):
			self.qs[i].append(data[i])
			self.mean[i] = np.mean(self.qs[i])
			self.var[i] = np.var(self.qs[i])
	def pop(self):
		for q in self.qs:
			q.popleft()
	def flush(self):
		for q in self.qs:
			q.clear()
	def get_depth(self):
		return len(self.qs[0])

def yaml2model(paramfile='calibration.yaml'): 
	path = os.path.dirname(os.path.abspath(__file__))
	paramfile = path+'/'+paramfile
	with open(paramfile,'r') as f:
		params = yaml.load(f)
	mat = np.array(params['camera_matrix']) #Camera parameters matrix
	dist = np.array(params["dist_coeff"])
	dist = np.reshape(dist,(-1,1)) #Camera distortion coefficients
	return(mat,dist)

def pose2homo(pose):
	(x,y,x,roll,pitch,yaw) = pose
	R = euler_matrix(roll, pitch, yaw, 'rxyz')
	H = np.zeros((4,4))
	H[0:3,0:3] = R
	H[0,3] = x
	H[1,3] = y
	H[2,3] = z
	return H

def homo2pose(homo):
	(roll, pitch, yaw) = euler_from_matrix(H[0:3,0:3], 'rxyz')
	(x, y, z, _) = H[:,3]
	return(x,y,z,roll,pitch,yaw)

def yaml2pose(paramfile='pose.yaml'):
	path = os.path.dirname(os.path.abspath(__file__))
	paramfile = path+'/'+paramfile
	with open(paramfile,'r') as f:
		params = yaml.load(f)
	roll = params('roll')
	pitch = params('pitch')
	yaw = params('yaw')
	dx = params('dx')
	dy = params('dy')
	dz = params('dz')
	return (dx,dy,dz,roll,pitch,yaw)

class CameraModelPublisher: #Uncomplete, but costume-made for my needs so far
	def __init__(self,paramfile,topic="/camera/camera_info"): 
		self.publisher = rospy.Publisher(topic,CameraInfo,queue_size=1)
		(mat,dist) = yaml2model(paramfile)
		self.msg = CameraInfo()
		self.msg.D = dist.flatten() 
		self.msg.K = mat.flatten()
	def publish(self, stamp):
		self.msg.header.stamp = stamp 
		self.publisher.publish(self.msg)

class AttachedLinkPose:
	def __init__(self,pose_file):
		pose = yaml2pose(pose_file)
		self.relTransform = pose2homo(pose)
	def getAbsPose(self, baseLinkPose):
		H = np.matmul(baseLinkPose,self.relTransform)
		return homo2pose(H)