#!/usr/bin/python 
import rospy 
from threading import Thread
import random
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def euler_from_qmsg(qmsg): 
	qx = qmsg.x
	qy = qmsg.y
	qz = qmsg.z
	qw = qmsg.w
	return (euler_from_quaternion([qx,qy,qz,qw]))

class MarkMovDriver(object):
	def __init__(self,mark_id,pose=[0,0,0],work_space=[[0,2],[0,1]]):
		self.mid = mark_id  
		self.pose = pose #(x,y,yaw) only 
		self.ws = work_space #[[x_inf,x_sup],[y_inf,y_sup]]
	def setRandPose(self): 
		x = random.uniform(self.ws[0][0],self.ws[0][1])
		y = random.uniform(self.ws[1][0],self.ws[1][1])
		yaw = random.uniform(-np.pi, np.pi)
		self.pose = [x,y,yaw]
	def walk(self,traj,t):
		(x_i,y_i,yaw_i) = self.pose
		(x_j,y_j,yaw_j) = traj(t)
		yaw_j = np.arctan2(y_j-y_i,x_j-x_i)
		self.pose = [x_j,y_j,yaw_j]
	def randWalk(self): 
		#print(self.pose)
		x = self.pose[0]+np.random.normal(0,0.025)
		y = self.pose[1]+np.random.normal(0,0.025)
		yaw = self.pose[2]+np.random.normal(0,0.025)
		self.pose = [x,y,yaw]
	def get_pose(self):
		ros_pose = Pose()
		ros_pose.position.x = self.pose[0]
		ros_pose.position.y = self.pose[1]
		ros_pose.position.z = 0.01
		quat = quaternion_from_euler(0.0,0.0,self.pose[2])
		ros_pose.orientation.x = quat[0]
		ros_pose.orientation.y = quat[1]
		ros_pose.orientation.z = quat[2]
		ros_pose.orientation.w = quat[3]
		return ros_pose
	def respawn(self,pose):
		self.pose = pose

class SingleMarkNode(MarkMovDriver): 
	def __init__(self,mark_id,rate=10,pose=[0,0,0],work_space=[[0,2],[0,1]]): 
		super(SingleMarkNode,self).__init__(mark_id,pose,work_space)
		super(SingleMarkNode,self).setRandPose()
		self.publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
		self.msg = ModelState()
		self.msg.model_name = "aruco_marker_"+str(mark_id)
		self.msg.reference_frame = "world"
		self.done = False
		self.subscriber = rospy.Subscriber("/fake_gps/ego_pose_raw/"+str(mark_id),PCS,self.pose_err)
		self.thread = Thread(target=self.run, args=(rate,))
		self.err = [[]]
		print("Marca movil: "+str(mark_id))
	def publish(self): 
		self.publisher.publish(self.msg)
	def run(self, rate): 
		rate = rospy.Rate(rate)
		while not self.done: 
			self.respawn([0.25,0.0,2.0])
			self.msg.pose = self.get_pose()		
			self.publish()
			rate.sleep()
	def start(self): 
		self.thread.start()
	def stop(self): 
		self.done = True
		self.thread.join()
		np.save(self.msg.model_name+"_err",self.err)
	def pose_err(self,pose_msg): 
		x_m = pose_msg.pose.pose.position.x
		y_m = pose_msg.pose.pose.position.y
		(roll_m,pitch_m,yaw_m) = euler_from_qmsg(pose_msg.pose.pose.orientation)
		pose_m = np.array([x_m,y_m,yaw_m])
		x_s = self.msg.pose.position.x 
		y_s = self.msg.pose.position.y
		(roll_s,pitch_s,yaw_s) = euler_from_qmsg(self.msg.pose.orientation)
		pose_s = np.array([x_s,y_s,yaw_s-0.5*np.pi])
		print(pose_s-pose_m)
		self.err.append(pose_s-pose_m)

def main(): 
	rospy.init_node('marks_moving_node',anonymous=True)
	nodes = [SingleMarkNode(mid) for mid in [5]]
	try: 
		for node in nodes: 
			node.start()
		while not rospy.is_shutdown():
			pass
	except Exception as e:
		print(e)
	finally: 
		for node in nodes:
			node.stop()

if __name__ == '__main__':
    main()



