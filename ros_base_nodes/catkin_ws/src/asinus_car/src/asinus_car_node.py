#!/usr/bin/python

import rospy 
from asinus import AsinusMotors as motors 
from donkietown_msgs.msg import MotorsSpeed, MotorsState
from time import time,sleep
import numpy as np

class DDR:
	def __init__(self,q0):
		self.L = 0.10 #Please, measure asinus car main axis
		self.r = 0.03
		self.x = q0[0]
		self.y = q0[1]
		self.th = q0[2]
	def update(self,ds,dth):
		self.x += ds*cos(self.th+0.5*dth)
		self.y += ds*sin(self.th+0.5*dth)
		self.th += dth

#Kalman filter
#https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture17-EKFLocalization.pdf


class AsinusCar:
	def __init__(self,L,r,gear_corr=1):
		self.L = L
		self.r = r
		self.gear_corr = gear_corr #How much is the asinus car actually faster?
		self.motors = motors()
		self.motors.stop()
		self.cur_rpm = np.zeros(2)
		self.req_rpm = np.zeros(2)
		self.volt = 5.0
		self.last_time = time()
	def setSpeeds(self,rpm_l,rpm_r):
		self.req_rpm = np.array([rpm_l,rpm_r])
	def getMeasures(self):
		return (self.cur_rpm[0],self.cur_rpm[1],self.volt)
	def update(self):
		self.motors.setSpeeds(self.req_rpm)
		sleep(0.01) #Increase time gap between i2c transactions.
		cur_time = time()
		(dth_l,dth_r,volt) = self.motors.getMeasures()
		sleep(0.01)
		self.volt = volt
		dths = np.array([dth_l,dth_r])/self.gear_corr
		dt = cur_time-self.last_time
		self.cur_rpm = 60.0*dths/dt
		print(self.cur_rpm)
		self.last_time = cur_time
	
#Periodically get displacements.
#Periodically publish speed (twist?, angular rpm is useful for debugging thou)
#Kalman Filter

class asinus_car_node: 
	def __init__(self,car_id): 
		self.car_id = car_id
		self.asinus_car = AsinusCar(0.1,0.03)
		rospy.on_shutdown(self.shutdown)
		self.measures_pub = rospy.Publisher('/asinus_cars/'+str(car_id)+'/motors_raw_data',MotorsState,queue_size=1)
		self.driver_sub = rospy.Subscriber('/asinus_cars/'+str(car_id)+'/motors_driver',MotorsSpeed, self.on_drive)
		self.motor_st = MotorsState()
	def on_drive(self,speed_msg):
		self.asinus_car.setSpeeds(speed_msg.leftMotor,speed_msg.rightMotor)
	def talker(self,rate):
		rate = rospy.Rate(rate)
		while not rospy.is_shutdown():
			stamp = rospy.Time.now()
			self.asinus_car.update()
			(rpm_l,rpm_r,volt) = self.asinus_car.getMeasures()
			self.motors_publish(rpm_l,rpm_r,volt,stamp)
			#publish estimation
			#Kalman FIlter (Another speed...)
			rate.sleep()
	def motors_publish(self,rpm_l,rpm_r,volt,stamp):
		self.motor_st.speed.leftMotor = rpm_l
		self.motor_st.speed.rightMotor = rpm_r
		self.motor_st.voltage = volt
		self.motor_st.header.stamp = stamp
		self.measures_pub.publish(self.motor_st)
	def shutdown(self):
		print("shutdown!")
		rospy.sleep(1)
		self.asinus_car.motors.stop()
		

def main():
	rospy.init_node('asinus_car_node',anonymous=True)
	robot_id = rospy.get_param("~car_id") #Unique for each vehicle  
	node = asinus_car_node(robot_id)
	try:
		node.talker(100)
	except rospy.ROSInterruptException:
		rospy.loginfo("asinucar node terminated.")

if __name__ == '__main__':
    main()
