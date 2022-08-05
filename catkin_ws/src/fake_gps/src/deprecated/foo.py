#!/usr/bin/env python 
#dummy node for concept test
import rospy
import sys 
import tf
from framework import FrameBr as fw
import numpy as np


class foo_node:
	def __init__(self,father_f='odom'):
		self.frame = fw(father_f,"base_link") 
		self.rate = rospy.Rate(1)
	def talker(self): 
		while not rospy.is_shutdown():
			self.frame.update_from_quaternion(np.array([0,0,0,1]),np.array([0,0,0]))
			self.frame.broadcast()
			self.rate.sleep()
if __name__=='__main__':
	rospy.init_node('foo_tf_broadcaster')
	node = foo_node()
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		pass	

	

