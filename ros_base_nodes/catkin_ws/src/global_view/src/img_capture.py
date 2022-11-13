#!/usr/bin/env python2
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import os

def get_time():
	now = time.localtime()
	return str(now.tm_year)+'_'+str(now.tm_mon)+'_'+str(now.tm_mday)+'_'+\
			str(now.tm_hour)+'_'+str(now.tm_min)+'_'+str(now.tm_sec)
class ImgCaptureNode:
	def __init__(self, debug):
		self.debug = debug
		self.video_subs = rospy.Subscriber('/image_in',Image,self.onImage,queue_size=1)
		self.bridge = CvBridge()
		rospack = rospkg.RosPack()
		self.path = rospack.get_path('global_view')+'/photos/'
		dirname = os.path.dirname(self.path)
		if not os.path.exists(dirname):
			os.makedirs(dirname)
	def onImage(self,img_msg):
		rospy.loginfo("Hello from onImage callback")
		try:
			frame = self.bridge.imgmsg_to_cv2(img_msg,'bgr8')
		except CvBridgeError as e:
			print(e)
			return
		if self.debug:
			width = int(frame.shape[1] * 0.5)
			height = int(frame.shape[0] * 0.5)
			resized = cv2.resize(frame, (width,height), interpolation = cv2.INTER_AREA)
			cv2.imshow("img",resized)
		if cv2.waitKey(20)&0xFF == ord('c'):
			name = get_time()+'.jpg'
			path = self.path+name
			cv2.imwrite(path,frame)
			if self.debug:
				cv2.imshow("img",resized)
				cv2.waitKey(0)

def main():
	rospy.init_node("image_capture")
	debug = rospy.get_param("~debug",True)
	rospy.loginfo(str(debug))
	node = ImgCaptureNode(debug)
	try:
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("image_capture node terminated.")

if __name__=='__main__':
	main()