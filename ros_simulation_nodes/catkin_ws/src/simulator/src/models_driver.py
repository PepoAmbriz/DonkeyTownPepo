#!/usr/bin/python 
import rospy 
from threading import Thread
import random
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from node_cfg import marks_offset

def euler_from_qmsg(qmsg): 
	qx = qmsg.x
	qy = qmsg.y
	qz = qmsg.z
	qw = qmsg.w
	return (euler_from_quaternion([qx,qy,qz,qw]))

class ModelNode(object): 
	def __init__(self,model_name,pose=[0,0,0,0,0,0],rate=10): #pose: (x,y,z,roll,pitch,yaw)
		self.model_name = model_name
		self.pose = pose
		self.publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
		self.msg = self.init_msg()
		self.done = False
		self.thread = Thread(target=self.run, args=(rate,))
	def respawn(self, pose): 
		self.pose = pose 
	def get_ros_pose(self): 
		ros_pose = Pose()
		ros_pose.position.x = self.pose[0]
		ros_pose.position.y = self.pose[1]
		ros_pose.position.z = self.pose[2]
		quat = quaternion_from_euler(self.pose[3],self.pose[4],self.pose[5])
		ros_pose.orientation.x = quat[0]
		ros_pose.orientation.y = quat[1]
		ros_pose.orientation.z = quat[2]
		ros_pose.orientation.w = quat[3]
		return ros_pose
	def publish(self): 
		self.msg.pose = self.get_ros_pose()
		self.publisher.publish(self.msg)
	def init_msg(self): 
		msg = ModelState()
		msg.model_name = self.model_name
		msg.reference_frame = "world"
		return msg
	def run(self,rate): 
		rate = rospy.Rate(rate)
		while not self.done: 
			#self.respawn(self.pose)
			self.publish()
			rate.sleep()
	def start(self): 
		self.thread.start()
	def stop(self): 
		self.done = True
		self.thread.join()

class MarkerNode(ModelNode): 
	def __init__(self, mark_id, pose=[0.0,0.0,1.5708]): # work_space: (p_min,p_max)
		self.mid = mark_id
		super(MarkerNode,self).__init__("aruco_marker_"+str(mark_id),[pose[0],pose[1],0.001,0.0,0.0,pose[2]])
		#self.setRandPose(work_space)
		self.fixed = True
		self.cont = 0
	def setRandPose(self,ws=[[-1.5,-1.0],[1.5,1.0]]): 
		x = random.uniform(ws[0][0],ws[1][0])
		y = random.uniform(ws[0][1],ws[1][1])
		z = 0.01
		roll = 0.0
		pitch = 0.0
		yaw = random.uniform(-np.pi, np.pi)
		self.pose = [x,y,z,roll,pitch,yaw]
		

	def detach(self): 
		self.fixed = False
		self.subscriber = rospy.Subscriber("/fake_gps/ego_pose_raw/"+str(self.mid),PCS,self.pose_err)
		self.err = []
	def pose_err(self,pose_msg): 
		x_m = pose_msg.pose.pose.position.x
		y_m = pose_msg.pose.pose.position.y
		(roll_m,pitch_m,yaw_m) = euler_from_qmsg(pose_msg.pose.pose.orientation)
		pose_m = [x_m,y_m,yaw_m]
		x_s = self.msg.pose.position.x 
		y_s = self.msg.pose.position.y
		(roll_s,pitch_s,yaw_s) = euler_from_qmsg(self.msg.pose.orientation)
		pose_s = [x_s,y_s,yaw_s-0.5*np.pi]
		print(pose_msg.pose)
		self.err.append([pose_s[i]-pose_m[i] for i in range(3)])
		self.cont = self.cont+1
		print(self.cont)
	def stop(self): 
		super(MarkerNode,self).stop()
		if not self.fixed:
			np.save("./results/"+self.msg.model_name+"_err",self.err)
			print(self.err)
			print(np.mean(self.err[-100:], axis=0))

class CameraNode(ModelNode): 
	def __init__(self,height): 
		super(CameraNode,self).__init__("camera1", pose=[0.0,0.0,height,0.0,1.5708,1.5708])
	def respawn(self,height): 
		super(CameraNode,self).respawn([0.0,0.0,height,0.0,1.5708,1.5708])

def main(): 
	rospy.init_node('gazebo_models_node',anonymous=True)
	#marks_nodes = [MarkerNode(mid) for mid in [15]]
	sm_nodes = []
	for smark in marks_offset.keys(): 
		(x,y,yaw) = marks_offset[smark]
		sm_nodes.append(MarkerNode(int(smark),[x,y,1.5708]))
	mmark = MarkerNode(4)
	mmark.respawn([1.0,1.0,0.005,0.0,0.0,1.5708])
	mmark.setRandPose()
	mmark.detach()
	#mnodes = sm_nodes.append(mmark)
	mnodes = [mmark]
	cam_node = CameraNode(4.0)
	try: 
		cam_node.start()
		for node in mnodes: 
			node.start()
		for node in sm_nodes: 
			node.start()
		while not rospy.is_shutdown():
			pass
	except Exception as e:
		print(e)
	finally: 
		for node in mnodes:
			node.stop()
		for node in sm_nodes:
			node.stop()
		cam_node.stop()
if __name__ == '__main__':
    main()



