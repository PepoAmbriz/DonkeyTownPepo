#!/usr/bin/ python3
import rospy
from sensor_msgs.msg import PointCloud
import sensor_sharing

class SensorSharingNode:
	def __init__(self,time_window,rate,q_size):
		self.time_window = time_window
		self.rate = rate
		self.pts_sub = rospy.Subscriber("/v2x/sensors/obstacles/raw", PointCloud, queue_size=q_size)
		self.obs_pub = rospy.Publisher("/v2x/sensors/obstacles/filtered", PointCloud, self.on_ptcloud, queue_size=1)
	def on_ptcloud(self,pcmsg):

def main():
	rospy.init_node("sensor_sharing")
	time_window =  rospy.get_param("~time_window",1.0)
	rate = rospy.get_param("~rate",10.0)
	q_size = rospy.get_param("~queue_size", 5)
	node = SensorSharingNode(time_window,rate,q_size)
	try:
		node.spin()
	except ROSInterruptException:
		print("Shutting down")