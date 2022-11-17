#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_sharing import SensorSharing

def pts2ptmsg(pts):
	msg_pts = []
	for pt in pts:
		ptmsg = Point32()
		ptmsg.x = pt[0]
		ptmsg.y = pt[1]
		msg_pts.append(ptmsg)
	return msg_pts

class SensorSharingNode(SensorSharing):
	def __init__(self,time_window,eps,min_samples,rate,q_size,debug):
		super().__init__(time_window,eps,min_samples)
		self.time_window = time_window
		self.rate = rate
		self.debug_en = debug

		self.pcmsg = PointCloud()
		self.pcmsg.header.frame_id = "map"

		self.pts_sub = rospy.Subscriber("/v2x/sensors/obstacles/raw", PointCloud, self.on_ptcloud, queue_size=q_size)

		self.obs_pub = rospy.Publisher("/v2x/sensors/obstacles/filtered", PointCloud, queue_size=1)
		if debug:
			self.acc_pc_pub = rospy.Publisher("/v2x/sensors/obstacles/pool", PointCloud, queue_size=1)

	def on_ptcloud(self,pcmsg):
		pts = [[pt.x,pt.y] for pt in pcmsg.points]
		now = pcmsg.header.stamp.to_sec()
		self.append(pts)
	def talker(self):
		rate = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.clean()
			self.debug()
			_, filtered_pts = self.cluster()
			if not filtered_pts:
				continue
			self.pcmsg.header.stamp = rospy.Time.now()
			self.pcmsg.points = pts2ptmsg(filtered_pts)
			self.obs_pub.publish(self.pcmsg)
				
			rate.sleep()
	def debug(self):
		if self.debug_en:
			acc_pc_msg = PointCloud()
			acc_pc_msg.points = pts2ptmsg(self.get_raw_data())
			acc_pc_msg.header.frame_id = "map"
			self.acc_pc_pub.publish(acc_pc_msg)


def main():
	rospy.init_node("sensor_sharing")
	time_window =  rospy.get_param("~time_window",2.0)
	cluster_eps = rospy.get_param("~cluster_eps",0.1)
	cluster_min = rospy.get_param("~cluster_min",5)
	rate = rospy.get_param("~rate",2.0)
	q_size = rospy.get_param("~queue_size", 5)
	debug = rospy.get_param("~debug", True)
	node = SensorSharingNode(time_window,cluster_eps,cluster_min,rate,q_size,debug)
	try:
		node.talker()
	except rospy.ROSInterruptException:
		print("Shutting down")

if __name__=='__main__':
	main()