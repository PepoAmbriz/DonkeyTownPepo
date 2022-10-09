from collections import deque
import numpy as np



class SensorQ:
	def __init__(self,width,depth,timeout=1,stamp=0):
		self.qs = [deque(maxlen=depth) for i in range(width)]
		self.width = width
		self.depth = depth
		self.stamp = stamp
		self.timeout = timeout
		self.mean = [0 for i in range(width)]
		self.std = [0 for i in range(width)]
	def push(self,data,stamp):
		if stamp-self.stamp > self.timeout:
			self.flush()
		self.stamp = stamp
		for i in range(self.width):
			self.qs[i].append(data[i])
			self.mean[i] = np.mean(self.qs[i])
			self.std[i] = np.std(self.qs[i])
	def pop(self):
		for q in self.qs:
			q.popleft()
	def flush(self):
		for q in self.qs:
			q.clear()
	def get_len(self):
		return len(self.qs[0])



#Unnecesary. You have previously installed tf on jetson nano
def euler_to_quaternion(r):
    (roll,pitch,yaw) = (r[0],r[1],r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]