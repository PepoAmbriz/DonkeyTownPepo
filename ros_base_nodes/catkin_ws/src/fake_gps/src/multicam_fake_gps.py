#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
from camera_misc import CameraModel, resolutions
from donkietown_msgs.msg import MarkerEdge, MarkerEdgeArray, Square
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix
from framework import FrameBr as fw
from framework import axis_matrix
from node_cfg import marks_offset
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
import threading
import Queue as queue


class Marker(object):
	def __init__(self,mid,upcam_id,stamp,corners):
		self.mid = mid
		self.tf_name = str(upcam_id)+"_marker_"+mid
		self.frame = fw("map",self.tf_name)
		self.update(stamp,corners)
		self.upcam_id = upcam_id
		self.relH = np.zeros((4,4))
	def update(self,stamp,corners):
		self.stamp = stamp
		self.corners = corners
	def broadcast_tf(self,rvec,tvec):
		self.frame.update_from_rotvec(rvec,tvec)
		self.frame.broadcast()
	def set_relative_pose(self,rvec,tvec):
		self.relH = axis_matrix(rvec,tvec)
	def get_corners_center(self):
		return np.mean(self.corners[0], axis=0)

class MobileMarker(Marker):
	def __init__(self,robot_id,upcam_id,stamp,corners):
		super(MobileMarker,self).__init__(robot_id,upcam_id,stamp,corners)
		self.posePub = rospy.Publisher("/fake_gps/ego_pose_raw/"+str(robot_id),PCS,queue_size=1)
		self.init_message()
	def init_message(self):
		self.pose_msg = PCS()
		self.pose_msg.header.frame_id = "map"
		self.pose_msg.pose.covariance = [0.01,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,1e-3,0,0,
										 0,0,0,0,1e-3,0,
										 0,0,0,0,0,0.03]
		self.pose_msg.pose.pose.position.z = 0.0
	def update_posemsg_tf(self,refmark_id,lookup_transform):
		father_tf = str(self.upcam_id)+"_marker_"+str(refmark_id)
		trans = lookup_transform(father_tf,self.tf_name,rospy.Time(0))
		(xo,yo,tho) = marks_offset[str(refmark_id)] #Add reference mark pose offset. 
		self.pose_msg.header.stamp = trans.header.stamp 
		self.pose_msg.pose.pose.position.x = trans.transform.translation.x+xo
		self.pose_msg.pose.pose.position.y = trans.transform.translation.y+yo
		#Assume ego mark is inside x-y plane. 
		qx = trans.transform.rotation.x
		qy = trans.transform.rotation.y
		qz = trans.transform.rotation.z
		qw = trans.transform.rotation.w
		(roll,pitch,yaw) = euler_from_quaternion([qx,qy,qz,qw])
		quat = quaternion_from_euler(0.0,0.0,yaw+tho)
		self.pose_msg.pose.pose.orientation.x = quat[0]
		self.pose_msg.pose.pose.orientation.y = quat[1]
		self.pose_msg.pose.pose.orientation.z = quat[2]
		self.pose_msg.pose.pose.orientation.w = quat[3]
	def update_posemsg_h(self,refmark):
		h_abs = np.matmul(np.linalg.inv(refmark.relH),self.relH)
		(roll,pitch,yaw) = euler_from_matrix(h_abs[0:3,0:3])
		(xo,yo,tho) = marks_offset[str(refmark.mid)] #Add reference mark pose offset.
		quat = quaternion_from_euler(0.0,0.0,yaw+tho)

		self.pose_msg.header.stamp = self.stamp
		self.pose_msg.pose.pose.position.x = h_abs[0,3]+xo
		self.pose_msg.pose.pose.position.y = h_abs[1,3]+yo
		self.pose_msg.pose.pose.orientation.x = quat[0]
		self.pose_msg.pose.pose.orientation.y = quat[1]
		self.pose_msg.pose.pose.orientation.z = quat[2]
		self.pose_msg.pose.pose.orientation.w = quat[3]

	def publish(self):
		self.posePub.publish(self.pose_msg) #Publish ego mark pose. 

class fake_gps: 
	def __init__(self, cam=0, resolution="720p", cam_id=0, arucoDict=cv2.aruco.DICT_4X4_50, 
					markerLen=0.1, refids={'0'}, calib_f="calibration.yaml", debug_lvl=0): 
		self.arucoDict = cv2.aruco.Dictionary_get(arucoDict) 
		self.markerLen = markerLen #square mark lenght 
		self.cam_id = cam_id #To allow multiple cameras running
		self.refids = refids #Set of known reference markers' ids
		self.frame_q = queue.Queue(maxsize=1) #To share video frames into different threads 
		self.time = rospy.Time.now()
		self.deb_lvl = int(debug_lvl)
		if self.deb_lvl>1:
			self.tfBuffer = tf2_ros.Buffer() #This and the following are used for tf transformations
			self.listener = tf2_ros.TransformListener(self.tfBuffer) 
			#This is recquired to import all TF transformations	
			#tf broadcast will be only used for debugging purpopses
		topic_bn = '/sensors/global_camera_'+str(cam_id)
		self.markerCornersPub = rospy.Publisher(topic_bn+'/marks_corners',MarkerEdgeArray,queue_size=1)
		
		self.cam_model = CameraModel(paramfile=calib_f,topic=topic_bn+'/info')

		self.cam = cv2.VideoCapture(cam)
		res_w, res_h = resolutions[resolution]
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
	def talker(self):
		proc_thread = threading.Thread(target=self.img_proc_threaded)
		proc_thread.daemon = True
		proc_thread.start()
		while not rospy.is_shutdown():
			foo,frame = self.cam.read()
			try:
				self.frame_q.put_nowait(frame)
			except:
				pass

	def img_proc_threaded(self):
		while True:
			frame = self.frame_q.get()
			self.get_raw_poses(frame)

	def get_raw_poses(self,frame):
		corners,ids,rejected = cv2.aruco.detectMarkers(frame,self.arucoDict) #Aruco basis function. 
																			#Get detected marks corners
																			#And possible marks corners
		if ids is None:
			return
		stamp = rospy.Time.now()
		
		markerArray = MarkerEdgeArray() #Instantiate marks corners message. 
		markerArray.header.frame_id = "base_link"
		markerArray.header.stamp = stamp

		#Classify them as reference markers and mobile markers.
		mob_marks = {}
		ref_marks = {}
		for i in range(len(ids)):
			marker_edges = MarkerEdge() #Creating markers msg item
			marker_edges.id = ids[i]
			marker_edges.corners = list(corners[i].flatten())
			markerArray.MarkerEdges.append(marker_edges)

			id_dict = str(ids[i,0])
			#Get marker to camera transformation in (rotational vector, translation vector)
			rvec,tvec = cv2.aruco.estimatePoseSingleMarkers([corners[i]],
						self.markerLen,self.cam_model.mat,self.cam_model.dist)
			if id_dict in self.refids:
				marker = Marker(id_dict,self.cam_id,stamp,corners[i]) 
				marker.set_relative_pose(rvec,tvec)
				if self.deb_lvl>1:
					marker.broadcast_tf(rvec,tvec)
				ref_marks[id_dict] = marker
				continue #To next detected marker.
			else:
				marker = MobileMarker(id_dict,self.cam_id,stamp,corners[i]) 
				marker.set_relative_pose(rvec,tvec)
				if self.deb_lvl>1:
					marker.broadcast_tf(rvec,tvec)
				mob_marks[id_dict] = marker
			#Pass if its a mobile marker.
		if self.deb_lvl>0:
			self.markerCornersPub.publish(markerArray)
		#TODO: check current ref_markers sanity.
		acc_ref_marks = {}
		for test_rm in ref_marks.values():
			test_rm_mid = test_rm.mid
			for good_rm in ref_marks.values():
				if test_rm_mid == good_rm.mid:
					continue
				h_test_to_good = np.matmul(np.linalg.inv(test_rm.relH),good_rm.relH)
				(g_r,g_p,g_y) = euler_from_matrix(h_test_to_good[0:3,0:3])
				(t_xo,t_yo,t_tho) = marks_offset[str(test_rm.mid)]
				(g_xo,g_yo,g_tho) = marks_offset[str(good_rm.mid)]

				x_dist = g_xo-t_xo-h_test_to_good[0,3]
				y_dist = g_yo-t_yo-h_test_to_good[1,3]
				th_dist = g_tho-t_tho-g_y
				if((x_dist**2+y_dist**2)**0.5 < 0.05):
					acc_ref_marks[test_rm_mid] = test_rm
					break
		for mm in mob_marks.values():
			#Find nearest reference marker
			min_dist = 1E9
			for rm in acc_ref_marks.values():
				dist = np.linalg.norm(mm.get_corners_center()-rm.get_corners_center())
				if dist < min_dist: 
					min_dist = dist
					near_rm = rm
			if min_dist == 1E9:
				return #No reference markers found, absolute pose cannot be calculated.
			#Publish position.
			try:
				#mm.update_posemsg_tf(near_rm.mid,self.tfBuffer.lookup_transform)
				mm.update_posemsg_h(near_rm)
				mm.publish()
			except:
				print("Couldn't compute absolute pose of mobile marker"+str(mm.mid))
def main():
	rospy.init_node('fakegps',anonymous=True)
	upper_cam_id = rospy.get_param("~upcam_id",0)
	cam_port = rospy.get_param("~cam_port", 0)
	deb_lvl = rospy.get_param("~debug_lvl", 0)
	cam_calib_file = rospy.get_param("~calib_file")
	resolution = rospy.get_param("~resolution", "720p")
	ref_marks = marks_offset.keys()
	node = fake_gps(cam_port,resolution=resolution,cam_id=upper_cam_id,refids=ref_marks,
						calib_f=cam_calib_file,debug_lvl=deb_lvl) 
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	main()