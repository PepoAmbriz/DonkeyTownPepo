#!/usr/bin/python 
import rospy 
from donkietown_msgs.msg import MotorsState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math 
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class wheel_odometry_node: 
	def __init__(self,car_id,r=0.022,l=0.124): 
		#Car parameters
		self.car_id = car_id
		self.r = r 
		self.l = l
		#variables (init pending)
		self.last_time = 0
		self.th = 0 
		self.x = 0
		self.y = 0
		self.are_var_init = False
		#pub_msg init
		self.odom_msg = Odometry()
		self.odom_msg.header.frame_id = "map"
		self.odom_msg.child_frame_id = "odom_"+str(car_id)

		self.odometry_pub = rospy.Publisher('/asinus_cars/'+str(car_id)+'/odometry',Odometry,queue_size=1)
		self.wheels_sub = rospy.Subscriber('/asinus_cars/'+str(car_id)+'/motors_raw_data',MotorsState,self.on_sensor)
		self.pose0_sub = rospy.Subscriber('/fake_gps/ego_pose_raw/'+str(car_id),Odometry,self.on_init)
		rospy.on_shutdown(self.shutdown)

	def on_sensor(self,state_msg): 
		if not self.are_var_init: 
			return 
		time = state_msg.header.stamp
		dt = time.to_sec()-self.last_time
		self.last_time = time.to_sec()
		if(dt>10): 
			return
		s = (state_msg.speed.rightMotor+state_msg.speed.leftMotor)*math.pi*self.r/60
		w = (state_msg.speed.rightMotor-state_msg.speed.leftMotor)*(self.r/self.l)*(2*math.pi/60) 
		self.th = self.th+w*dt
		self.x = self.x+s*math.cos(self.th)*dt
		self.y = self.y+s*math.sin(self.th)*dt
		self.odom_msg.header.stamp = time 
		self.odom_msg.pose.pose.position.x = self.x
		self.odom_msg.pose.pose.position.y = self.y
		self.odom_msg.pose.pose.position.z = 0.0
		quat = quaternion_from_euler(0.0,0.0,self.th)
		self.odom_msg.pose.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
		self.odom_msg.twist.twist.linear.y = s
		self.odom_msg.twist.twist.angular.z = w
		self.odom_msg.pose.covariance = [0.01,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,0,0,0,
										 0,0,0,0,0,0,
										 0,0,0,0,0,0.03]
		self.odom_msg.twist.covariance = [1e-4,0,0,0,0,0,
										 0,0.01,0,0,0,0,
										 0,0,0,0,0,0, 
										 0,0,0,0,0,0,
										 0,0,0,0,0,0,
										 0,0,0,0,0,0.03]
 		print(self.odom_msg)
 		self.odometry_pub.publish(self.odom_msg)
		
	def on_init(self,gps_msg):
		if self.are_var_init: 
			return 
		self.x = gps_msg.pose.pose.position.x 
		self.y = gps_msg.pose.pose.position.y
		quat = gps_msg.pose.pose.orientation
		q_list = [quat.x,quat.y,quat.z,quat.w]
		(roll,pitch,yaw) = euler_from_quaternion(q_list)
		self.th = yaw 
		self.are_var_init = True 
		print(self.x)
		print(self.y)
		print(self.th)


	def shutdown(self):
		print("shutdown!")
		rospy.sleep(1)



def main():
	rospy.init_node('wheel_odometry',anonymous=True)
	robot_id = rospy.get_param("~car_id") #Unico para cada robot
	node = wheel_odometry_node(robot_id)
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("wheelOdometry node terminated.")

if __name__ == '__main__':
    main()

