#!/usr/bin/env python 
#Deprecated
import rospy 
import tf2_ros 
from geometry_msgs.msg import PoseStamped
import sys 
from nav_msgs.msg import Odometry
from node_cfg import marks_offset

class ego_pose:
	def __init__(self,robot_id,t_frame="map",rate=50): 
		self.robot_id = robot_id
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.t_frame = t_frame
		self.s_frame = "rel_car_link_"+str(robot_id)
		self.rate = rospy.Rate(rate)
		self.posePub = rospy.Publisher("/fake_gps/ego_pose_raw/"+str(robot_id),Odometry,queue_size=1)
		self.odom_msg = Odometry()
		self.odom_msg.header.frame_id = t_frame
		self.odom_msg.child_frame_id = self.s_frame
		ref_id = marks_offset.keys()
		self.ref_id = ref_id[0]
	def talker(self): 
		#To be changed to 3D Pose with Odometry message for Kalman Filter
		while not rospy.is_shutdown():
			try:  
				trans = self.tfBuffer.lookup_transform("ref_mark_"+self.ref_id,self.s_frame,rospy.Time())
			except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException): 
				self.rate.sleep()
				continue
			self.publish(trans)
	def publish(self,trans):
		self.odom_msg.header.stamp = trans.header.stamp 
		self.odom_msg.pose.pose.position.x = trans.transform.translation.x
		self.odom_msg.pose.pose.position.y = trans.transform.translation.y
		self.odom_msg.pose.pose.position.z = 0.0
		self.odom_msg.pose.pose.orientation = trans.transform.rotation
		self.odom_msg.pose.covariance = [0.01,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,1e-3,0,0,
										 0,0,0,0,1e-3,0,
										 0,0,0,0,0,0.03]
		self.odom_msg.twist.covariance = [1e6,0,0,0,0,0,
										  0,1e-3,0,0,0,0,
										  0,0,0,0,0,0, 
										  0,0,0,1e-3,0,0,
										  0,0,0,0,1e-3,0,
										  0,0,0,0,0,1e6]
		self.posePub.publish(self.odom_msg)

def main(args):
	rospy.init_node("ego_pose_raw") 
	robot_id = rospy.get_param("~car_id") #Unico para cada robot
	node = ego_pose(robot_id)
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		print("Shutting down")

if __name__=='__main__': 
	main(sys.argv) 

"""
import rospy 
import tf2_ros 
import geometry_msgs.msg 
import sys
from framework import FrameWork as fw
import filterpy.common as kalman
import numpy as np

class ego_pose: 
	def __init__(self,robot_id,t_frame="map"): 
		rospy.init_node("ego_pose_raw"+str(robot_id))
		self.robot_id = robot_id
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.t_frame = t_frame
		self.s_frame = "base_link_mark"+str(robot_id)
		self.rate = rospy.Rate(30.0)
		self.kf = kalman.kinematic_kf(2,1)
		self.last_update = rospy.get_rostime()
		self.last_estimate = self.last_update
		self.filters_init(R0=0.0025,Q0=0.01)

	def get_current_pose(self): 
		trans = self.tfBuffer.lookup_transform(self.t_frame,self.s_frame,rospy.Time())
		return trans 
	def talker(self): 
		while not rospy.is_shutdown(): 
			try: 
				trans = self.get_current_pose()
				tf_time = trans.header.stamp
			except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException): 
				self.rate.sleep()
				continue
			c_time = rospy.get_rostime() 
			dt = c_time.to_sec()-self.last_estimate.to_sec()
			self.kf_estimate(dt)
			self.last_estimate = c_time
			if tf_time!=self.last_update: 
				self.kf_update(trans.transform.translation)
				self.last_update = c_time
			print(self.kf.x)
	def filters_init(self,P0=100,R0=1,Q0=10):
		self.kf.P *= P0
		self.kf.R *= R0
		self.kf.Q *= Q0
	def kf_estimate(self,dt):
			self.kf.F = np.array([[1,dt,0,0],
							 	  [0,1,0,0],
							 	  [0,0,1,dt],
							 	  [0,0,0,1]])
			self.kf.predict()
	def kf_update(self,translation):	
		x = translation.x
		y = translation.y
		z = [x,y]
		self.kf.update(z)

def main(args): 
	robot_id = 3 #unico para cada robot
	node = ego_pose(robot_id)
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		print("Shutting down")

if __name__=='__main__': 
	main(sys.argv)
"""