#!/usr/bin/env python2
import rospy
from auto_model import AsinusCar, pcs2pose2d
from fub_controller import VectorfieldController
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from sensor_msgs.msg import PointCloud
import v2x

class CooperativeDrivingNode:
	def __init__(self,car_id,map_name,lane,look_ahead,speed_value):
		self.car_id = car_id
		self.car = AsinusCar(car_id)
		self.speed_value = speed_value
		self.controller = VectorfieldController(map_name,lane,look_ahead)

		self.obs_msg = None
		self.sub_obs = rospy.Subscriber("/sensors/obstacles",PointCloud,self.on_obstacle, queue_size=1)

		self.myCAM = v2x.CAMPublisher(car_id)
		self.extCAM = v2x.CAMSubscriber(car_id)

		rospy.on_shutdown(self.shutdown)
		self.shutdown_ = False

	def on_obstacle(self,obs_msg): 
		self.obs_msg = obs_msg
	def on_shutdown(self):
		print("shutdown!")
		self.shutdown_ = True
		self.car.stop()
		rospy.sleep(1)
  	def talker(self,rate):
  		rate = rospy.Rate(rate)
  		heading = None
  		while not rospy.is_shutdown():
  			#get car state variables
  			pcs = self.car.getPCS()
  			twist = self.car.getTwist()
  			if pcs is None or twist is None:
  				continue
  			s = twist.linear.x
  			w = twist.angular.z
  			pose2D = pcs2pose2d(pcs)
  			#send CAM message
  			self.myCAM.set(pose2D,s,self.controller.lane,True,heading,
  							(rospy.Time.now()-self.controller.last_lane_change).to_sec()<10.0)
  			self.myCAM.publish()

  			#fub control
  			ctrl_s,ctrl_w,aux_heading = self.controller.steering_control(pcs,self.speed_value)
  			if aux_heading is not None:
  				heading = aux_heading
  			if ctrl_s is not None:
  				self.car.drive(ctrl_s,ctrl_w)
  			rate.sleep()
  	def shutdown(self):
  		print("shutdown!")
  		self.shutdown_ = True
  		self.car.stop()
  		rospy.sleep(1)

def main():
	rospy.init_node("copperative_driving")
	car_id = rospy.get_param("~car_id")
	map_name = rospy.get_param("~map_name","cimat_reduced")
	lane = rospy.get_param("~lane",1)
	look_ahead = rospy.get_param("look_ahead","30cm")
	speed = rospy.get_param("~speed",0.1)
	print(speed)
	rate = rospy.get_param("rate",10)
	
	node = CooperativeDrivingNode(car_id,map_name,lane,look_ahead,speed)
	try:
		node.talker(rate)
	except rospy.ROSInterruptException:
		pass
if __name__=='__main__':
	main()