#!/usr/bin/env python2
import rospy
from auto_model import AsinusCar, pcs2pose2d
from fub_controller import VectorfieldController, PID
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from sensor_msgs.msg import PointCloud
import numpy as np
import v2x

class CooperativeDrivingNode(object):
	def __init__(self,car_id,map_name,lane,look_ahead,speed_value):
		self.car_id = car_id
		self.car = AsinusCar(car_id)
		self.car_s = 0.0
		self.car_w = 0.0 
		self.speed_value = speed_value
		self.vectorField = VectorfieldController(map_name,lane,look_ahead)
		self.lanes_blocked = [False,False]

		self.driving_state = v2x.driving_states['Stop'] 

		self.myCAM = v2x.CAMPublisher(car_id)
		self.extCAM = v2x.CAMSubscriber(car_id)
		self.initCAM()
		self.myCAM.publish()

		self.sub_obs = rospy.Subscriber("/sensors/obstacles",PointCloud,self.on_obstacle, queue_size=1) #Need to remap topic while using v2x

		rospy.on_shutdown(self.shutdown)
		self.shutdown_ = False
	def getCAMs(self):
		return self.extCAM.msg_cache
	def on_obstacle(self,msg):
		print("Hello from on_obstacle")
		pts = msg.points
		blocked = [False,False] 
		for pt in pts: 
			pt_car = self.vectorField.get_coords_from_car(pt)
			pt_lanes = self.vectorField.get_coords_from_lanes(pt)
			dist_car = np.linalg.norm(pt_car)
			if dist_car > 0.75:
				continue
			dist_lane = np.linalg.norm(pt_lanes, axis=1)
			rospy.logwarn(dist_car)
			rospy.logwarn(dist_lane)
			blocked |= dist_lane<0.2
		rospy.logwarn(blocked)
		self.lanes_blocked = blocked
	def run(self): #Model car state variables estimation
		pcs = self.car.getPCS()
		twist = self.car.getTwist()
		if pcs is None or twist is None:
			return
		self.s = twist.linear.x
		self.w = twist.angular.z
		if self.driving_state==v2x.driving_states['LaneChangeGrant']:
			rospy.logwarn("LaneChange")
			self.vectorField.lane_change()
		ctrl_s,ctrl_w,heading = self.vectorField.pd_control(pcs,self.speed_value)
		if (ctrl_s is not None) and (not self.shutdown_) and (self.driving_state!=v2x.driving_states['Stop']):
			self.car.drive(ctrl_s,ctrl_w)
		elif self.driving_state==v2x.driving_states['Stop']:
			self.car.stop()
		pose2D = pcs2pose2d(pcs)
		self.myCAM.set(pose2D,self.s,self.vectorField.lane,True,heading,self.driving_state)
		self.myCAM.publish()
  	def talker(self,rate):
  		rate = rospy.Rate(rate)
  		self.driving_state = v2x.driving_states['Drive']
  		while not rospy.is_shutdown():
  			self.run()
  			rate.sleep()
  	def stop(self):
  		self.car.stop()
  	def shutdown(self):
  		print("shutdown!")
  		self.shutdown_ = True
  		self.car.stop()
  		rospy.sleep(1)
  	def initCAM(self):
  		pcs = None
  		while pcs is None:
  			pcs = self.car.getPCS()
		myPose2d = pcs2pose2d(pcs)
		self.myCAM.set(myPose2d,0.0,self.vectorField.lane,True,0.0,self.driving_state)

roles = {	'Unknown': 0,
			'Leader': 1,
			'Follower': 2}

class CooperativeConvoyNode(CooperativeDrivingNode):
	def __init__(self,car_id,map_name,lane,look_ahead,speed_value,distance):
		super(CooperativeConvoyNode,self).__init__(car_id,map_name,lane,look_ahead,speed_value)
		self.distance = distance
		self.dist_ctrler = PID(0.4,0.0,0.05) #PD
		self.role = roles['Unknown']
		while self.role==roles['Unknown'] and not rospy.is_shutdown():
			self.role,self.leading = self.getRole()
			print(self.role,self.leading)
			rospy.sleep(1)

	def getRole(self):
		leading = None
		pcs = self.car.getPCS()
		if pcs is None:
			return (roles['Unknown'],leading)
		myPose2d = pcs2pose2d(pcs)
		min_dist = 3.0
		CAMs = self.getCAMs()
		while len(CAMs)!=1 and not rospy.is_shutdown(): #[FIXME]: Find a smarter way
			self.myCAM.publish()
			CAMs = self.getCAMs()
			if len(CAMs)==0: #==1, ==2. #vehicles-1
				break
			rospy.sleep(0.1)
		for CAM in CAMs:
			pose2D = CAMs[CAM].reference_pose
			dist = self.vectorField.getPathDistance([myPose2d.x,myPose2d.y],[pose2D.x,pose2D.y])
			print(dist)
			if dist<=min_dist and dist>0:
				min_dist = dist
				leading = CAM
		if leading is None:
			return (roles['Leader'],leading)
		return (roles['Follower'],leading)

	def run(self):
		super(CooperativeConvoyNode,self).run()
		rospy.logwarn(self.driving_state)
		#FSM propagation
		CAMs = self.getCAMs()
		if self.role == roles['Follower']:
			leadingCAM = CAMs[self.leading]
			self.driving_state = leadingCAM.driving_state
			#Distance control
			myPose2d = pcs2pose2d(self.car.getPCS())
			leadingPose = leadingCAM.reference_pose
			distance = self.vectorField.getPathDistance([myPose2d.x,myPose2d.y],
														[leadingPose.x,leadingPose.y])
			print(self.distance,distance)
			#self.speed_value = ... #PID?
			self.speed_value = max(0,min(0.25,0.1-self.dist_ctrler.control(self.distance,distance)))
			print(self.speed_value)
			return
		#FSM Handling (Just for Leader)
		print(self.speed_value)
		if self.driving_state==v2x.driving_states['LaneChangeReq']:
			if self.vectorField.lane_change_req():
				self.driving_state = v2x.driving_states['LaneChangeGrant']
			return
		if not self.lanes_blocked[0]:
			self.driving_state = v2x.driving_states['Drive']
			return
		if not self.lanes_blocked[1]:
			self.driving_state = v2x.driving_states['LaneChangeReq']
			return
		self.driving_state = v2x.driving_states['Stop']

	def talker(self,rate):
		rate = rospy.Rate(rate)
		self.driving_state = v2x.driving_states['Drive']
		while not rospy.is_shutdown():
  			self.run()
  			rate.sleep() 
def main():
	rospy.init_node("cooperative_driving")
	car_id = rospy.get_param("~car_id")
	map_name = rospy.get_param("~map_name","cimat_reduced")
	lane = rospy.get_param("~lane",1)
	look_ahead = rospy.get_param("~look_ahead","30cm")
	speed = rospy.get_param("~speed",0.1)
	rate = rospy.get_param("~rate",10)
	distance = rospy.get_param("~distance",0.5)
	
	#node = CooperativeDrivingNode(car_id,map_name,lane,look_ahead,speed)
	node = CooperativeConvoyNode(car_id,map_name,lane,look_ahead,speed,distance)
	try:
		node.talker(rate)
	except rospy.ROSInterruptException:
		pass
if __name__=='__main__':
	main()