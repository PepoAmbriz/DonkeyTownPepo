#!/usr/bin/env python2
import rospy
import shared_mem.shared_msg as shmsg
from donkietown_msgs.msg import CooperativeAwarenessMessage as CAM
import sys
from time import time

class CAMSubscriber(object):
	def __init__(self, car_id, queue_size=2):
		self.car_id = int(car_id)
		self.msg_cache = {}
		self.cam_sub = rospy.Subscriber('/v2x/'+str(car_id)+'/rx', CAM, self.onCAM, queue_size=queue_size)
	def onCAM(self,cam):
		car_id = int(cam.header.frame_id)
		if car_id == self.car_id:
			return
		self.msg_cache[car_id] = cam
	def clean(self, time_th=2.0):
		last_time = rospy.Time.now()
		for key in self.msg_cache.keys():
			if abs((last_time-self.msg_cache[key].header.stamp).to_sec()) > time_th:
				del self.msg_cache[key]

class V2X_node:
	def __init__(self, car_id):
		self.car_id = car_id
		self.mem_CAM = shmsg.SharedCAM('selfCAM')
		self.CAM_pub = rospy.Publisher('/v2x/pull',CAM,queue_size=1)	
		self.CAM_sub = CAMSubscriber(car_id,queue_size=5)
	def talker(self,rate):
		rate = rospy.Rate(rate)
		while not rospy.is_shutdown():
			start = time()
			CAM_msg = self.mem_CAM.get()
			CAM_msg.header.frame_id = str(self.car_id)
			CAM_msg.header.stamp = rospy.Time.now()
			self.CAM_sub.clean()
			print(self.CAM_sub.msg_cache.values())
			self.CAM_pub.publish(CAM_msg)
			rate.sleep()

def main(args):
	rospy.init_node('vehicular_communication', anonymous=True)
	car_id = rospy.get_param("~car_id",10)
	node = V2X_node(int(car_id))
	try:
		node.talker(10)
	except rospy.ROSInterruptException:
		pass
if __name__=='__main__':
	main(sys.argv)