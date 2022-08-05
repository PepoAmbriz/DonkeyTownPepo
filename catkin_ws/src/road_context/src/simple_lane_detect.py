#!/usr/bin/env python2

#Solo detecta las lineas de un carril recto. 
#como mejorar? Modificar la transformada de Hough.
#Mejorar el calculo de lineas de carril.
import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import sys
from lane_detection import *
 

class simple_lanedet_node:
	def __init__(self,img_topic): 
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(img_topic,Image,self.on_img)
		self.image_pub = rospy.Publisher("/AutoModelMini/lane/img",Image,queue_size=1)
		self.lqueue = []
	def on_img(self,img_msg):
		try: 
			cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		#color_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
		color_image = cv_img
		img_h,img_w,img_l = color_image.shape
		#get lane lines
		lane_lines = get_lane_lines(color_image)
		#smooth over a time window
		self.lqueue.append(lane_lines)
		if(len(self.lqueue)>10):
			self.lqueue.pop(0)
		lane_lines = smoothen_over_time(self.lqueue)

		# prepare empty mask on which lines are drawn
		#lane_lines = candidate_lines
		line_img = np.zeros(shape=(img_h, img_w))
		for lane in lane_lines: 
			lane.draw(line_img)

		#region of interest mask 
		vertices = np.array([[(0,int(0.8*img_h)),
							(0,int(0.3*img_h)),
							(img_w,int(0.3*img_h)), 
							(img_w,int(0.8*img_h))]])
		img_masked,_ = region_of_interest(line_img,vertices)

		#make blend on color image
		img_blend = weighted_img(img_masked,color_image,alpha=0.8,beta=1.0,gamma=0.0)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_blend,"bgr8"))
		except CvBridgeError as e: 
			print(e)
		
def main(args): 
	node = simple_lanedet_node("/app/camera/rgb/image_raw")
	rospy.init_node('SimpleLaneDetector', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt: 
		print("Shutting down")
	

if __name__ == '__main__': 
	main(sys.argv)