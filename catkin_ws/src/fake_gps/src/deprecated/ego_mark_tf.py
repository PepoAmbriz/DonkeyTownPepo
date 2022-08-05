#!/usr/bin/env python
#Deprecated
import rospy 
import yaml
import numpy as np 
import sys
import os 
import cv2
from donkietown_msgs.msg import MarkerEdge, MarkerEdgeArray, Square
from nav_msgs.msg import Odometry
import tf
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_figure import order_corners
import filterpy.common as kalman
import yaml 
from framework import FrameBr as fw
class ego_gps:
	def __init__(self,egoid=3,refids=[2],markerLen=0.1,arucoDict=cv2.aruco.DICT_4X4_50,paramfile='calibration.yaml'): 
		self.egoid = egoid
		self.refids = refids
		self.markerLen = markerLen
		self.arucoDict = cv2.aruco.Dictionary_get(arucoDict)
		self.ego_pose_pub = rospy.Publisher('/fake_gps/ego_mark_tf/'+str(egoid),Odometry,queue_size=1)
		self.global_gps_sub = rospy.Subscriber('/fake_gps/marks_corners',MarkerEdgeArray, self.on_globalgps)
		self.mark_corners = np.zeros((4,2))
		self.mark_corners_estimation = np.zeros((4,2))
		self.ltime = rospy.get_rostime()
		self.corners_kf = [kalman.kinematic_kf(2,1) for i in range(4)]
		self.filters_init()
		path = os.path.dirname(os.path.abspath(__file__))
		paramfile = path+'/'+paramfile
		with open(paramfile,'r') as f:
			params = yaml.load(f)
		self.mat = np.array(params['camera_matrix'])
		dist = np.array(params["dist_coeff"])
		self.dist = np.reshape(dist,(-1,1))
		self.markerLen = markerLen
		self.physFrame = fw("map","rel_car_link_"+str(self.egoid))
	#Debug only:
		#self.global_img_sub = rospy.Subscriber('/sensors/global_camera/image', Image, self.on_globalview)
		#self.egomark_img_pub = rospy.Publisher('/ego_gps/mark/image', Image, queue_size=1)
		#self.bridge = CvBridge()
	def on_globalgps(self, msgdata):
		print("Hello from callback")
		time = msgdata.header.stamp
		dt = time.to_sec()-self.ltime.to_sec()
		self.corner_estimate(dt)
		self.ltime = time
		markers = msgdata.MarkerEdges
		ismarkid = False 
		for m in markers:
			if m.id == self.egoid: 
				ismarkid = True
				break 
		if ismarkid: 
			self.mark_corners = np.reshape(m.corners,(4,2)).astype(int)
			self.update_kf()
		elif msgdata.rejected is not None: 
			min_norm = 32
			befstfit_corner = np.zeros((4,2))
			mark_corners = order_corners(self.mark_corners_estimation)
			for r in msgdata.rejected: 
				square = np.reshape(r.corners,(4,2))
				square = order_corners(square)
				#Change to mark estimation
				norm = np.linalg.norm(square-mark_corners)
				if norm < min_norm: 
					min_norm = norm
					bestfit_corner = square
				else: 
					continue
			if min_norm<32:
				costs = [np.linalg.norm(bestfit_corner-p, axis=1) for p in self.mark_corners]
				row_ind, col_ind = linear_sum_assignment(np.reshape(costs, (4,4)))
				bestfit_corner = np.array([bestfit_corner[i] for i in row_ind], dtype="int32")
				self.mark_corners = np.reshape(bestfit_corner,(4,2)).astype(int)
				self.update_kf()
			else:
				#self.mark_corners = self.mark_corners_estimation	
				return
		else:
			#self.mark_corners = self.mark_corners_estimation	
			return
		#get ego_mark frame pose 
		corner = np.reshape(self.mark_corners,(1,4,2)).astype("float32")
		corners = [corner]
		rvec,tvec = cv2.aruco.estimatePoseSingleMarkers(corners,self.markerLen,self.mat,self.dist)
		self.physFrame.update_from_rotvec(rvec[0,0],tvec[0,0])
		self.physFrame.broadcast()
		
		
	def on_globalview(self,img):
		try: 
			frame = self.bridge.imgmsg_to_cv2(img,'bgr8')
		except CvBridgeError as e:
			print(e)
		corners = [np.reshape(self.mark_corners, (-1,1,2)).astype(int)]
		frame = cv2.polylines(frame,corners,True,(0,255,0),1)
		self.egomark_img_pub.publish(self.bridge.cv2_to_imgmsg(frame,'bgr8'))

	def corner_estimate(self,dt=1):
		for kf in self.corners_kf: 
			kf.F = np.array([[1,dt,0,0],
							 [0,1,0,0],
							 [0,0,1,dt],
							 [0,0,0,1]])
			kf.predict()
		self.mark_corners_estimation = self.kf2markCorners()

	def filters_init(self,P0=100,R0=1,Q0=10):
		for kf in self.corners_kf:
			kf.P *= P0
			kf.R *= R0
			kf.Q *= Q0

	def kf2markCorners(self): 
		corners = []
		for i in range(4): 
			kfx = self.corners_kf[i].x
			corners.append([kfx[0,0],kfx[2,0]])
		return np.asarray(corners)

	def update_kf(self):
		for i in range(4): 
			z = self.mark_corners[i]
			self.corners_kf[i].update(z)
		self.mark_corners_estimation = self.kf2markCorners()

def main():
	rospy.init_node("egomark_tfbr", anonymous=True)
	robot_id = rospy.get_param("~car_id") #Unico para cada robot
	node =  ego_gps(robot_id) 
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
if __name__=='__main__':
	main()


