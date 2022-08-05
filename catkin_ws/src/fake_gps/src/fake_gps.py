#!/usr/bin/env python2
#Works only for all-same-oriented-fiducial-marks scenarios
#Always taking paramters.yaml as camera model, even when gazebo model is the one. To be fixed. Still pending
import rospy
import cv2
import sys
import yaml 
from sensor_msgs.msg import Image
from donkietown_msgs.msg import MarkerEdge, MarkerEdgeArray, Square
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_figure import *
import tf
from framework import FrameBr as fw
from node_cfg import marks_offset
from camera_misc import CameraModel

class fake_gps: 
	def __init__(self, cam=0, arucoDict=cv2.aruco.DICT_4X4_50, camera_src='internal', refids={'0'}, markerLen=0.1, paramfile='calibration.yaml'): 
		self.bridge = CvBridge()
		self.arucoDict = cv2.aruco.Dictionary_get(arucoDict)
		self.markerLen = markerLen #square mark lenght 
		self.refids = refids
		self.time = rospy.Time.now() 
		self.markerCornersPub = rospy.Publisher('/fake_gps/marks_corners',MarkerEdgeArray,queue_size=1)
		self.max_var = np.zeros(2) #Max variance on detected marks sides.
		self.rej = []
		self.ref_frame = {mark_id:fw("map","ref_mark_"+mark_id) for mark_id in refids}  #Defining every
																#frame for each reference mark.
		self.cam_src = camera_src
		self.cam_model = CameraModel(paramfile=paramfile,topic='/sensors/global_camera/info')
		if self.cam_src == 'internal': 
			self.cam = cv2.VideoCapture(cam)
		else:
			self.view_sub = rospy.Subscriber('/sensors/global_camera/image', Image, self.callback)
			if self.cam_src == 'gazebo': 
				del self.cam_model
				self.cam_model = CameraModel(kind="subscriber",topic='/sensors/global_camera/info')

	def talker(self):
		while not rospy.is_shutdown():
			foo,frame = self.cam.read()
			self.detectMarks(frame)
	def callback(self,img): #Only executed alongside rosbag.
		try: 
			frame = self.bridge.imgmsg_to_cv2(img,'bgr8')
		except CvBridgeError as e:
			print(e)
		corners,ids,rejected = self.detectMarks(frame)
	def detectMarks(self,frame): 
		corners,ids,rejected = cv2.aruco.detectMarkers(frame,self.arucoDict) #Aruco basis function. 
																			#Get detected marks corners
																			#And possible marks corners
		markerArray = MarkerEdgeArray() #Instantiate marks corners message. 
		markerArray.header.frame_id = "base_link"
		markerArray.header.stamp = self.time
		if ids is not None: 
			for i in range(len(ids)): 
				marker = MarkerEdge()
				marker.id = ids[i]
				marker.corners = list(corners[i].flatten())
				self.max_var = np.max([self.max_var,square_var(corners[i])],axis=0) #Compute max variance of
																					#latest detected marks sides.	
				markerArray.MarkerEdges.append(marker)
				id_dict = str(ids[i,0])
				if id_dict in self.refids: #Compute and broadcast each detected reference marks' pose
					ref_corners = [corners[i]]
					rvec,tvec = cv2.aruco.estimatePoseSingleMarkers(ref_corners,
						self.markerLen,self.cam_model.mat,self.cam_model.dist)
					rvec = np.reshape(rvec,(3))
					tvec = np.reshape(tvec,(3))
					self.ref_frame[id_dict].update_from_rotvec(rvec,tvec)
					self.ref_frame[id_dict].broadcast()
			rej = []
			#Discard possible marks corners according to a sides variance threshold. 
			for r in rejected:
				squareRej = Square()
				squareRej.corners = list(r.flatten())
				r_var = square_var(r)
				if r_var[0]<self.max_var[0] and r_var[1]<self.max_var[1]: 
					markerArray.rejected.append(squareRej)
					rej.append(r)
			self.rej = rej
		else: #If there's no possible mark corner which meet variance criteria, set 
				# "-1". as the only possible mark. 
			marker = MarkerEdge()
			marker.id = -1
			marker.corners = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
			markerArray.MarkerEdges.append(marker)
		timej = rospy.Time.now()
		self.time = timej
		#Publish camera model parameters. 
		if not self.cam_src == 'gazebo':
			self.cam_model.publish()
		#Publish possible marks. 
		try: 
			self.markerCornersPub.publish(markerArray)
		except:
			pass
		return corners,ids,rejected

def main(args):
	rospy.init_node('fakegps',anonymous=True)
	cam_src = rospy.get_param("~cam_src", "internal") #Unique for each vehicle
	ref_marks = marks_offset.keys()
	node = fake_gps(2,camera_src=cam_src,refids=ref_marks)
	if not cam_src=='internal':	
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
	else:
		try: 
			node.talker()
		except rospy.ROSInterruptException:
			pass

if __name__=='__main__':
	main(sys.argv)