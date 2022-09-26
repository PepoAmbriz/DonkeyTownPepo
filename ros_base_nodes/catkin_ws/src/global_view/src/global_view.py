#!/usr/bin/env python2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import sys
from dynamic_reconfigure.server import Server

class webcam:
    def __init__(self,cam=0):
        self.cam = cv2.VideoCapture(cam)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        #self.cam.set(cv2.CAP_PROP_BRIGHTNESS,0.5)
        #self.cam.set(cv2.CAP_PROP_CONTRAST,0.5)
        #self.cam.set(cv2.CAP_PROP_SATURATION,0.5)
        #https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python
    def getImg(self):
        ret,frame = self.cam.read()
        print(frame.shape)
        if ret==True:
            return frame

#Added camera_info topic, and publish into it
class global_view: 
	def __init__(self, cam_port=0):
		self.image_pub = rospy.Publisher('/sensors/global_camera/image_raw', Image, queue_size=1)
		self.bridge = CvBridge()
		self.cam = cv2.VideoCapture(cam_port)
	def talker(self):
		rate = rospy.Rate(20) #fps
		while not rospy.is_shutdown():
			ret,frame = self.cam.read()
			try: 
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
			except CvBridgeError as e:
				print(e)
			rate.sleep()
	def reconfigure_callback(self,config,level):
		rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},
						{str_param}, {bool_param}, {size}""".format(**config))		
		return config

def main(args):
	rospy.init_node('global_view', anonymous=True)
	port = rospy.get_param("~cam_port",2) #Unique for each vehicle  
	node = global_view(port)
	try: 
		node.talker()
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	main(sys.argv)
