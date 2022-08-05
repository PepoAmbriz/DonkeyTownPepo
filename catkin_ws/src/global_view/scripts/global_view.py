#!/usr/bin/env python2
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from camera import webcam
import sys
#Added camera_info topic, and publish into it
class global_view: 
	def __init__(self, cam_port=0):
		self.image_pub = rospy.Publisher('/sensors/global_camera/image_raw', Image, queue_size=1)
		self.bridge = CvBridge()
		self.cam = webcam(cam_port)
	def talker(self):
		rate = rospy.Rate(20) #fps
		while not rospy.is_shutdown():
			frame = self.cam.getImg()
			try: 
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
			except CvBridgeError as e:
				print(e)
			rate.sleep()

def main(args):
	rospy.init_node('globalview', anonymous=True)
	port = rospy.get_param("~cam_port",0) #Unique for each vehicle  
	node = global_view(port)
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	main(sys.argv)
