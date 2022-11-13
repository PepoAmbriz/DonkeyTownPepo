#!/usr/bin/python

import rospy 
from asinus import AsinusMotors as motors 
from donkietown_msgs.msg import MotorsSpeed, MotorsState

from time import time,sleep
import numpy as np
from math import cos,sin,pi
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from misc import SensorQ, subs_ang, CameraModelPublisher, AttachedLinkPose
#TODO: 
#	[ ]: Add CAM messages

class DDR(object):
	def __init__(self,q0):
		self.x = q0[0]
		self.y = q0[1]
		self.th = q0[2]
	def update(self,ds,dth):
		self.x += ds*cos(self.th+0.5*dth)
		self.y += ds*sin(self.th+0.5*dth)
		self.th += dth

#Kalman filter
#https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture17-EKFLocalization.pdf
class DDR_KF(DDR):
	def __init__(self,q0,p0=1.0,R=np.eye(3),kQ=2.0):
		super(DDR_KF,self).__init__(q0)
		self.p0 = p0
		self.P = p0*np.eye(3)
		self.R = R
		self.kQ = kQ #Constant gain for moel's covariance matrix
		self.H = np.eye(3) #As it is.
		self.enabled = False #Shall be False in prod.
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
		#Q = self.kQ*np.array([abs(dr),0],[0,abs(dl)]])
		Q = self.kQ*np.array([[dr**2,0],[0,dl**2]])
		self.update(ds,dth)
		self.P = np.matmul(Fx,np.matmul(self.P,np.transpose(Fx)))+\
					np.matmul(Fu,np.matmul(Q,np.transpose(Fu)))
	def correct(self,x,y,th):
		if not self.enabled:
			return
		#Solve th modulus. 

		#Remember that the following applies only when H=I.
		K = np.matmul(self.P,np.linalg.inv(self.P+self.R))
		z = np.array([x,y,th])
		q = np.array([self.x,self.y,self.th])
		vec_dif = z-q
		vec_dif[2] = subs_ang(th,self.th) #Clamping
		q = q+np.matmul(K,vec_dif)
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
		self.P = self.p0*np.eye(3)
		self.enabled = True
	def disable(self):
		self.enabled = False
	def sanity_check(self):
		return np.trace(self.P) < 6.0*self.p0

class AsinusCar:
	def __init__(self,L,r,gear_corr=1.0):
		self.L = L
		self.r = r
		self.gear_corr = gear_corr #How much is the asinus car actually faster?
		self.motors = motors()
		self.motors.stop()
		self.motors.setGains([5.0,0.8])
		self.cur_rpm = np.zeros(2)
		self.req_rpm = np.zeros(2)
		self.volt = 5.0
		self.last_time = time()
		R = np.array([[0.005,0.0,0.0],[0.0,0.005,0.0],[0.0,0.0,1.0]])
		self.KF = DDR_KF([0.0,0.0,0.0], R=R) #Init at (x=0.0, y=-0.0, th=0.0) Just to give it an initial value.
		self.gpsQ =  SensorQ(width=3,depth=5,timeout=2) #data sensor queue for gps data filtering.
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
		dths = np.array([dth_l,dth_r])*self.gear_corr
		dt = cur_time-self.last_time
		self.cur_rpm = 60.0*dths/dt
		du = self.r*2*pi*dths #Convert dths to distance displacement... % -> rad -> m
		self.KF.predict(dl=du[0],dr=du[1],axis_L=self.L) #KF update
		self.last_time = cur_time
	def gps_correct(self,x,y,th,stamp):
		sense_data = [x,y,th]
		self.gpsQ.push(sense_data,stamp)
		#gps data sanity check
		if self.gpsQ.get_depth() < 3:
			print("Returning due lack of data")
			return
		cnt = 0
		for mean,std,data in zip(self.gpsQ.mean,np.sqrt(self.gpsQ.var),sense_data): 
			if abs(data-mean) > 1.8*std:
				print("Returning due to noise")
				return
			cnt += 1
		#If passes gps data sanity check
		if (not self.KF.enabled) or (not self.KF.sanity_check()):
			print("Reinitializing")
			self.KF.reinitialize(sense_data)
			return
		#self.KF.R = np.diag(self.gpsQ.var)
		print("Correcting")
		self.KF.correct(x,y,th)

def pose2msg(pose):
	(x,y,z,roll,pitch,yaw) = pose
	pose_msg = Pose()
	pose_msg.position.x = x
	pose_msg.position.y = y
	pose_msg.position.z = z
	quat = quaternion_from_euler(roll,pitch,yaw)
	pose_msg.orientation.x = quat[0]
	pose_msg.orientation.y = quat[1]
	pose_msg.orientation.z = quat[2]
	pose_msg.orientation.w = quat[3]
	return pose_msg

class AsinusCarCamPosePublisher:
	def __init__(self, pose_file, topic):
		self.rel_pose = AttachedLinkPose(pose_file)
		self.pose_publisher = rospy.Publisher(topic,PoseStamped,queue_size=1)
		self.pose_msg = PoseStamped()
		self.pose_msg.header.frame_id = "map"
	def update_pose(self,base_pose):
		abs_pose = self.rel_pose.getAbsPose(base_pose)
		self.pose_msg.pose = pose2msg(abs_pose)
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_publisher.publish(self.pose_msg)
	#TODO: add car's covariance for sanity checking purposes

#Periodically publish speed (twist?, angular rpm is useful for debugging thou)
#Kalman Filter
class asinus_car_node: 
	def __init__(self,car_id,publish_rate=10): 
		self.car_id = car_id
		self.asinus_car = AsinusCar(0.125,0.03)
		rospy.on_shutdown(self.shutdown)
		self.publish_rate = publish_rate
		msg_stamp = rospy.Time.now()
		self.init_msgs(msg_stamp)
		topic_bn = '/asinus_cars/'+str(car_id)
		self.cam_pose_pub = AsinusCarCamPosePublisher('camera_properties/rel_cam_pose.yaml',topic_bn+'/camera/pose')
		self.cam_info_pub = CameraModelPublisher('camera_properties/calibration.yaml',topic_bn+'/camera/camera_info')
		self.measures_pub = rospy.Publisher(topic_bn+'/motors_raw_data',MotorsState,queue_size=1)
		self.posePub = rospy.Publisher(topic_bn+"/filtered_pose",PCS,queue_size=1)
		self.driver_sub = rospy.Subscriber(topic_bn+'/motors_driver',MotorsSpeed, self.on_drive)
		self.gps_sub = rospy.Subscriber("/fake_gps/ego_pose_raw/"+str(car_id), PCS, self.on_gps)
	def on_drive(self,speed_msg):
		self.asinus_car.setSpeeds(speed_msg.leftMotor,speed_msg.rightMotor)
	def on_gps(self,pcs_msg):
		x = pcs_msg.pose.pose.position.x
		y = pcs_msg.pose.pose.position.y
		qx = pcs_msg.pose.pose.orientation.x
		qy = pcs_msg.pose.pose.orientation.y
		qz = pcs_msg.pose.pose.orientation.z
		qw = pcs_msg.pose.pose.orientation.w
		(roll,pitch,yaw) = euler_from_quaternion([qx,qy,qz,qw])
		stamp = pcs_msg.header.stamp.to_sec()
		self.asinus_car.gps_correct(x,y,yaw,stamp)
	def talker(self,rate):
		rate = rospy.Rate(rate)
		while not rospy.is_shutdown():
			stamp = rospy.Time.now()
			self.asinus_car.update()
			self.motors_publish(stamp)
			self.pose_publish(stamp)
			self.cam_info_publish(stamp)
			rate.sleep()
	def motors_publish(self,stamp):
		if (stamp-self.motor_st.header.stamp).to_sec() < (1/self.publish_rate):
			return
		(rpm_l,rpm_r,volt) = self.asinus_car.getMeasures()
		self.motor_st.speed.leftMotor = rpm_l
		self.motor_st.speed.rightMotor = rpm_r
		self.motor_st.voltage = volt
		self.motor_st.header.stamp = stamp
		self.measures_pub.publish(self.motor_st)
	def pose_publish(self,stamp):
		if(stamp-self.pose_msg.header.stamp).to_sec() < (1/self.publish_rate):
			return
		pose = (self.asinus_car.KF.x, self.asinus_car.KF.y, 0.0,
				0.0, 0.0, self.asinus_car.KF.th)

		self.cam_pose_pub.update_pose(pose)

		self.pose_msg.pose.pose = pose2msg(pose)
		"""
		self.pose_msg.pose.pose.position.x = self.asinus_car.KF.x
		self.pose_msg.pose.pose.position.y = self.asinus_car.KF.y
		yaw = self.asinus_car.KF.th
		quat = quaternion_from_euler(0.0,0.0,yaw) #euler_to_quaternion([0.0,0.0,yaw])
		self.pose_msg.pose.pose.orientation.x = quat[0]
		self.pose_msg.pose.pose.orientation.y = quat[1]
		self.pose_msg.pose.pose.orientation.z = quat[2]
		self.pose_msg.pose.pose.orientation.w = quat[3]
		"""

		#converting Pose2D cov matrix to full 6dof cov matrix.
		self.pose_msg.pose.covariance[0] = self.asinus_car.KF.P[0,0]
		self.pose_msg.pose.covariance[1] = self.asinus_car.KF.P[0,1]
		self.pose_msg.pose.covariance[5] = self.asinus_car.KF.P[0,2]
		self.pose_msg.pose.covariance[6] = self.asinus_car.KF.P[1,0]
		self.pose_msg.pose.covariance[7] = self.asinus_car.KF.P[1,1]
		self.pose_msg.pose.covariance[11] = self.asinus_car.KF.P[1,2]
		self.pose_msg.pose.covariance[30] = self.asinus_car.KF.P[2,0]
		self.pose_msg.pose.covariance[31] = self.asinus_car.KF.P[2,1]
		self.pose_msg.pose.covariance[35] = self.asinus_car.KF.P[2,2]
		self.pose_msg.header.stamp = stamp
		self.posePub.publish(self.pose_msg)
	def cam_info_publish(self,stamp):
		if(stamp-self.cam_info_pub.msg.header.stamp).to_sec() < 10.0:
			return
		self.cam_info_pub.publish(stamp)
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
	rate = rospy.get_param("~rate", 100)
	node = asinus_car_node(robot_id)
	try:
		node.talker(int(rate))
	except rospy.ROSInterruptException:
		rospy.loginfo("asinucar node terminated.")

if __name__ == '__main__':
    main()
