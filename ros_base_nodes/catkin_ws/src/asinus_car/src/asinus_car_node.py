#!/usr/bin/python

import rospy 
from asinus import AsinusMotors as motors 
from donkietown_msgs.msg import MotorsSpeed, MotorsState
from time import time,sleep
import numpy as np
from math import cos,sin,pi
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
#from tf.transformations import quaternion_from_euler

class DDR(object):
	def __init__(self,q0):
		self.x = q0[0]
		self.y = q0[1]
		self.th = q0[2]
	def update(self,ds,dth):
		self.x += ds*cos(self.th+0.5*dth)
		self.y += ds*sin(self.th+0.5*dth)
		self.th += dth
		print("----")
		print(self.x,self.y,self.th)
class DDR_KF(DDR):
	def __init__(self,q0,p0=1000,R=np.eye(3),kQ=1):
		super(DDR_KF,self).__init__(q0)
		self.P = p0*np.eye(3)
		self.R = R
		self.kQ = kQ
		self.H = np.eye(3) #As it is.
		self.enabled = True #Shall be False in prod.
	def predict(self,dl,dr,axis_L):
		#dl: How much (in meters) left wheel has been displaced since last prediction
		#dr: dl but for right wheel.
		#axis_L distance between both wheels.
		if not self.enabled:
			return
		ds = 0.5*(dl+dr)
		dth = (dr-dl)/axis_L
		c = self.th+0.5*dth
		Fx = np.eye(3)
		Fx[0,2] = -ds*sin(c)
		Fx[1,2] = ds*cos(c)
		cc = cos(c)
		sc = sin(c)
		kfuds = ds/axis_L
		kfudth = 2/axis_L
		Fu = 0.5*np.array([	[cc-kfuds*sc, 	cc+kfuds*sc],
							[sc+kfuds*cc, 	sc-kfuds*cc],
							[kfudth, 		-kfudth]])
		Q = self.kQ*np.array([[dr,0],[0,dl]])
		self.update(ds,dth)
		self.P = np.matmul(Fx,np.matmul(self.P,np.transpose(Fx)))+\
					np.matmul(Fu,np.matmul(Q,np.transpose(Fu)))
	def correct(self,x,y,th):
		if not self.enabled:
			return
		#Remember that the following applies only when H=I.
		K = np.matmul(self.P,np.linalg.inv(self.P+self.R))
		z = np.array([x,y,th])
		q = np.array([self.x,self.y,self.th])
		q = q+np.matmul(K,z-q)
		self.x = q[0]
		self.y = q[1]
		self.th = q[2]
		aux_M = np.eye(3)-K
		self.P = np.matmul(aux_M,np.matmul(self.P,np.transpose(aux_M)))+\
					np.matmul(K,np.matmul(self.R,np.transpose(K)))
	def reinitialize(self,q0):
		self.x = q0[0]
		self.y = q0[1]
		self.th = q0[2]
		self.enabled = True
	def disable(self):
		self.enabled = False

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
		self.KF = DDR_KF([0,0,0])
	def setSpeeds(self,rpm_l,rpm_r):
		self.req_rpm = np.array([rpm_l,rpm_r])
	def getMeasures(self):
		return (self.cur_rpm[0],self.cur_rpm[1],self.volt)
	def update(self):
		#Start of I2C transaction
		self.motors.setSpeeds(self.req_rpm)
		sleep(0.01) #Increase time gap between i2c transactions.
		cur_time = time()
		(dth_l,dth_r,volt) = self.motors.getMeasures()
		sleep(0.01)
		#End of I2C transaction
		self.volt = volt
		dths = np.array([dth_l,dth_r])/self.gear_corr
		dt = cur_time-self.last_time
		self.cur_rpm = 60.0*dths/dt
		#KF update
		#Convert dths to distance displacement... degs -> rad -> m
		du = self.r*(pi/180)*dths
		self.KF.predict(dl=du[0],dr=du[1],axis_L=self.L)
		self.last_time = cur_time


	
#Periodically publish speed (twist?, angular rpm is useful for debugging thou)
#Kalman Filter
class asinus_car_node: 
	def __init__(self,car_id,publish_rate=10): 
		self.car_id = car_id
		self.asinus_car = AsinusCar(0.1,0.03)
		rospy.on_shutdown(self.shutdown)
		self.publish_rate = publish_rate
		msg_stamp = rospy.Time.now()
		self.init_msgs(msg_stamp)
		self.measures_pub = rospy.Publisher('/asinus_cars/'+str(car_id)+'/motors_raw_data',MotorsState,queue_size=1)
		self.posePub = rospy.Publisher("/asinus_cars/"+str(car_id)+"/filtered_pose",PCS,queue_size=1)
		self.driver_sub = rospy.Subscriber('/asinus_cars/'+str(car_id)+'/motors_driver',MotorsSpeed, self.on_drive)

	def on_drive(self,speed_msg):
		self.asinus_car.setSpeeds(speed_msg.leftMotor,speed_msg.rightMotor)
	def talker(self,rate):
		rate = rospy.Rate(rate)
		#Reduce publishing frequency.
		while not rospy.is_shutdown():
			stamp = rospy.Time.now()
			self.asinus_car.update()
			(rpm_l,rpm_r,volt) = self.asinus_car.getMeasures()
			self.motors_publish(rpm_l,rpm_r,volt,stamp)
			#publish estimation
			self.pose_publish(stamp)
			#Kalman FIlter (Another speed...)
			rate.sleep()
	def motors_publish(self,rpm_l,rpm_r,volt,stamp):
		if (stamp-self.motor_st.header.stamp).to_sec() < (1/self.publish_rate):
            		return
		self.motor_st.speed.leftMotor = rpm_l
		self.motor_st.speed.rightMotor = rpm_r
		self.motor_st.voltage = volt
		self.motor_st.header.stamp = stamp
		self.measures_pub.publish(self.motor_st)
	def pose_publish(self,stamp):
		if(stamp-self.pose_msg.header.stamp).to_sec() < (1/self.publish_rate):
			return
		self.pose_msg.pose.pose.position.x = self.asinus_car.KF.x
		self.pose_msg.pose.pose.position.y = self.asinus_car.KF.y
		yaw = self.asinus_car.KF.th
		#quat = quaternion_from_euler(0.0,0.0,yaw)
		#self.pose_msg.pose.pose.orientation.x = quat[0]
		#self.pose_msg.pose.pose.orientation.y = quat[1]
		#self.pose_msg.pose.pose.orientation.z = quat[2]
		#self.pose_msg.pose.pose.orientation.w = quat[3]
		self.posePub.publish(self.pose_msg)
	def shutdown(self):
		print("shutdown!")
		rospy.sleep(1)
		self.asinus_car.motors.stop()
	def init_msgs(self,msg_stamp):
		self.motor_st = MotorsState()
		self.motor_st.header.stamp = msg_stamp
		self.pose_msg = PCS()
		self.pose_msg.header.frame_id = "map"
		self.pose_msg.header.stamp = msg_stamp
		self.pose_msg.pose.covariance = [0.1,0.01,0,0,0,0.01,
										 0.01,0.1,0,0,0,0.01,
										 0,0,0,0,0,0, 
										 0,0,0,0,0,0,
										 0,0,0,0,0,0,
										 0.01,0.01,0,0,0,0.1]

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
