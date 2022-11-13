from sklearn.cluster import DBSCAN
import numpy

class TimedBoundedQueue:
	def __init__(self,time_window):
		self.time_window = time_window
		self.data = np.array([])
		self.time_record = np.array([])
	def append(self,new_data, dt)
		np.append(self.data,new_data)
		self.time_record += dt
		np.append(self.data,np.zeros(len(new_data)))
	def clean(self):
		pop_cnt = 0
		for t in time_record:
			if t<self.time_window:
				return
			pop_cnt += 1
		self.data = self.data[pop_cnt:]
		self.time_record = self.time_record[pop_cnt:]
	def get_data(self):
		return self.data

class SensorSharing:
	def __init__(self,time_window,eps,min_samples):
		self.cl_eps = eps
		self.cl_min = min_samples
		self.sensorQ = TimedBoundedQueue(time_window)
	def append(self,data):
		self.sensorQ.append(data)
	def cluster(self):
		self.sensorQ.clean()
		clustering = DBSCAN(eps=self.cl_eps,min_samples=self.cl_min).fit(self.sensorQ.data)
		clusters_sum = {}
		clusters_size = {}
		for i in len(clustering):
			label = clustering[i]
			if label == -1:
				continue
			if not label in clusters_size:
				clusters_size[label] = 1
				clusters_sum[label] = self.sensorQ.data[i]
				continue
			clusters_size[label] += 1
			clusters_sum[label] += sensorQ.data[i]
		centers = []
		for label in clusters_size.keys():
			centers.append(clusters_sum[label]/clusters_size[label])
		return centers