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
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        #self.cam.set(cv2.CAP_PROP_EXPOSURE, 0.1)
        self.cam.set(cv2.CAP_PROP_FPS,30)
        self.cam.set(cv2.CAP_PROP_BRIGHTNESS,0.5)
        #https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python
    def getImg(self):
        ret,frame = self.cam.read()
        if ret==True:
            return frame