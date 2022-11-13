import rospy 
from sensor_msgs.msg import CameraInfo 
import yaml 
import os
import numpy as np

resolutions = {
	"480p": (640,480),
	"600p": (800,600),
	"720p": (1280,720),
	"1080p": (1920,1080)
}

def yaml2model(paramfile='calibration.yaml'): 
	path = os.path.dirname(os.path.abspath(__file__))
	paramfile = path+'/'+paramfile
	with open(paramfile,'r') as f:
		params = yaml.load(f)
	mat = np.array(params['camera_matrix']) #Camera parameters matrix
	dist = np.array(params["dist_coeff"])
	dist = np.reshape(dist,(-1,1)) #Camera distortion coefficients
	return(mat,dist)


class CameraModel(object): 
	def __init__(self,topic,kind='publisher',paramfile='calibration.yaml'):
		self.mat = [[]]
		self.dist = []
		if(kind=='subscriber'): 						
			self.subscriber = rospy.Subscriber(topic,CameraInfo,self.callback)
		else: 
			(self.mat,self.dist) = yaml2model(paramfile)
			self.msg = CameraInfo()
			self.msg.D = self.dist.flatten() 
			self.msg.K = self.mat.flatten()
			self.publisher = rospy.Publisher(topic,CameraInfo,queue_size=1)
	def publish(self): 
		self.publisher.publish(self.msg)
	def callback(self,msg): 
		self.mat = np.reshape(msg.K,(3,3))
		self.dist = np.asarray(msg.D)
		self.subscriber.unregister()

class CameraModelPublisher(object): #Uncomplete, but costume-made for my needs so far
	def __init__(self,paramfile,topic="/camera/camera_info"): 
		self.publisher = rospy.Publisher(topic,CameraInfo,queue_size=1)
		(self.mat,self.dist) = yaml2model(paramfile)
		self.msg = CameraInfo()
	def publish(self): 
		self.msg.D = self.dist.flatten() 
		self.msg.K = self.mat.flatten()
		self.publisher.publish(self.msg)