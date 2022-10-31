#!/usr/bin/env python2
import rospy
from donkietown_msgs.msg import CooperativeAwarenessMessage as CAM
import sys

def matches(camA, camB, rate=10, max_dist=3.0):
	ax = camA.reference_pose.x
	ay = camA.reference_pose.y
	bx = camB.reference_pose.x
	by = camB.reference_pose.y
	dist = ((ax-bx)**2+(ay-by)**2)**0.5
	dt = abs((camA.header.stamp-camB.header.stamp).to_sec())
	if (dist<=max_dist) and (dt<=2/rate):
		return True
	else:
		return False

def bucketize(cams, rate=10, max_dist=3.0):
	connections = []
	while len(cams)>0:
		cam_i = cams.pop()
		cam_cols = cams[:]
		while len(cam_cols)>0:
			cam_j = cam_cols.pop()
			if matches(cam_i,cam_j,rate,max_dist):
				connections.append([int(cam_i.header.frame_id),
									int(cam_j.header.frame_id)])
	return connections

class CAMTarget:
	def __init__(self,target_id, tx_q_size=3):
		self.publisher = rospy.Publisher('/v2x/'+str(target_id)+'/rx',CAM,queue_size=tx_q_size)
	def publish(self,cam):
		self.publisher.publish(cam)
		
class VehicularNetwork:
	def __init__(self,rx_q_size=5):
		self.msg_cache = {}
		self.receiver = rospy.Subscriber('/v2x/pull', CAM, self.onRx, queue_size=rx_q_size)
		self.tx_buffer = {}
	def onRx(self,cam):
		car_id = int(cam.header.frame_id)
		if not car_id in self.tx_buffer:
			self.tx_buffer[car_id] = CAMTarget(car_id)
		self.msg_cache[car_id] = cam
	def talker(self,rate):
		ros_rate = rospy.Rate(rate)
		while not rospy.is_shutdown():
			connections = bucketize(self.msg_cache.values(),rate)
			print(connections)
			for connection in connections:
				self.tx_buffer[connection[0]].publish(self.msg_cache[connection[1]])
				self.tx_buffer[connection[1]].publish(self.msg_cache[connection[0]])
			ros_rate.sleep()

def main(args):
	rospy.init_node('vehicular_network', anonymous=True)
	node = VehicularNetwork()
	try:
		node.talker(1)
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	main(sys.argv)