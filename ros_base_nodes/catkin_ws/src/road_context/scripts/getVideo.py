#!/usr/bin/env python2
#Converting ros video to mpg4

import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class image_converter:
	def __init__(self,img_topic): 
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(img_topic,Image,self.on_img)
	def on_img(self,img_msg):
		try: 
			cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		h,w,l = cv_img.shape
		print(h)
def main(args): 
	node = image_converter("/app/camera/rgb/image_raw")
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down")
	print(os.getcwd())
	

if __name__ == '__main__': 
	main(sys.argv)