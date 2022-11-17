from sklearn.cluster import DBSCAN
import numpy as np
import time

class TimedBoundedQueue:
	def __init__(self,time_window):
		self.time_window = time_window
		self.data = []
		self.time_record = np.array([])
		self.last_time = time.time()
	def append(self,new_data):
		self.update()
		if len(new_data) == 0:
			return
		self.time_record = np.append(self.time_record,np.zeros(len(new_data)))
		for data in new_data:
			self.data.append(data)
	def update(self):
		now = time.time()
		self.time_record += now-self.last_time
		self.last_time = now
	def clean(self):
		self.update()
		pop_cnt = 0
		for t in self.time_record:
			if t<self.time_window:
				continue
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
		data = self.sensorQ.get_data()
		if len(data) == 0:
			return (None,None)
		clustering = DBSCAN(eps=self.cl_eps,min_samples=self.cl_min).fit(data)
		clusters_sum = {}
		clusters_size = {}
		clusters = {}
		for i in range(len(clustering.labels_)):
			label = clustering.labels_[i]
			if label == -1:
				continue
			if not label in clusters:
				clusters[label] = [data[i]]
				clusters_size[label] = 1
				clusters_sum[label] = np.array(data[i])
				continue
			clusters[label].append(data[i])
			clusters_size[label] += 1
			clusters_sum[label] += data[i]
		
		centers = []
		for label in clusters_size.keys():
			centers.append(clusters_sum[label]/clusters_size[label])
		return (clusters.values(),centers)
	def get_raw_data(self):
		return self.sensorQ.get_data()
	def clean(self):
		self.sensorQ.clean()