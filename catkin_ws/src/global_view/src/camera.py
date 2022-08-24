#!/usr/bin/env python3 
import requests
import cv2
import numpy as np

class ipcamera:
	def __init__(self,ip,port=8080):
		self.url = "http://192.168.1.{}:8080/shot.jpg".format(str(ip))
		self.port = port
	def getImg(self): 
		img_resp = requests.get(self.url)
		img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
		frame = cv2.imdecode(img_arr, -1)	
		return frame
class webcam:
    def __init__(self,cam=0):
        self.cam = cv2.VideoCapture(cam)
    def getImg(self):
        ret,frame = self.cam.read()
        if ret==True:
            return frame