#!/usr/bin/env python2
from obstacle_localization.gazebo_utils import CameraPose, ObstaclePose
from obstacle_localization.obstacle_localization import PixelLocator, rpy_from_quaternion
from sensor_msgs.msg import PointCloud, CameraInfo
from geometry_msgs.msg import Point32, PoseStamped
from vision_msgs.msg import Detection2DArray
import numpy as np
import sys
import rospy

class CameraPoseTopic:
	def __init__(self, topic):
		self.msg = PoseStamped()
		self.pose_sub = rospy.Subscriber(topic, PoseStamped, self.on_pose)
	def on_pose(self, cam_pose_msg):
		self.msg = cam_pose_msg
	def get_pose(self):
		return self.msg.pose

class obstacle_localization_node: 
	def __init__(self): 
		self.ped_r = 0.07
		self.ready = False

		self.pl = PixelLocator([[]], [])
		
		pcmsg = PointCloud()
		pcmsg.header.frame_id = "map"
		self.pcmsg = pcmsg

		self.publisher = rospy.Publisher("/sensors/obstacles/",PointCloud,queue_size=1)

		#self.camera_pose = CameraPose() #Gazebo
		self.camera_pose = CameraPoseTopic("/camera/pose")
		#self.obs_pose = ObstaclePose('osito_blanco_1') #get obstacle pose from gazebo
		self.caminfo_sub = rospy.Subscriber("/app/camera/rgb/camera_info",CameraInfo,self.on_caminfo)
		self.detectpts_sub = rospy.Subscriber("/detectnet/detections", Detection2DArray,self.on_pts)

	def on_pts(self,msg): 
		if not self.ready:
			return
		#print("gazebo pose: ")
		#print(self.obs_pose.get_pose())
		#msg init
		self.pcmsg.header.stamp = msg.header.stamp
		cam_rosmsg_pose = self.camera_pose.get_pose() #Add sanity check from variance.
		_,_,yaw = rpy_from_quaternion(cam_rosmsg_pose.orientation)
		pts = self.pl.compute_localization(cam_rosmsg_pose, msg.detections)
		if len(pts) == 0:
			return	 
		rospts = []
		for pt in pts:
			#Fixed descentralized point. 
			#pose correction (camera object and camera rays pose are not the same)
			rospt = Point32()
			rospt.x = pt[0]+self.ped_r*np.cos(yaw)
			rospt.y = pt[1]+self.ped_r*np.sin(yaw)
			rospts.append(rospt)
		self.pcmsg.points = rospts
		self.publisher.publish(self.pcmsg)

	def on_caminfo(self,msg): 
		mat = np.reshape(msg.K,(3,3))
		dist = np.asarray(msg.D)
		rospy.logwarn(dist)
		self.pl.set_cam_model(mat, dist)
		self.ready = True
		self.caminfo_sub.unregister()
		#gets camera intrinsic matrix and distortion coefficients.

def main(args): 
	rospy.init_node('pedestrian_detection', anonymous=True)
	node = obstacle_localization_node()
	try:
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down")
	

if __name__ == '__main__': 
	main(sys.argv)