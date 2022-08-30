import cv2 
import numpy as np
import constants as const
import os

class ArucoImage:
	def __init__(self,mark_id,aruco_dict=cv2.aruco.DICT_4X4_50): 
		self.mark_id = mark_id 
		self.dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
		self.draw_mark()
	def draw_mark(self): 
		canvas = 255*np.ones((700,700),np.uint8)
		aruco_img = cv2.aruco.drawMarker(self.dict,self.mark_id,500,200)
		canvas[100:600,100:600] = aruco_img
		canvas = self.put_id(canvas)
		self.img = canvas
	def put_id(self,img):
		new_image = cv2.putText(
		  img = img,
		  text = str(self.mark_id),
		  org = (5, 20),
		  fontFace = cv2.FONT_HERSHEY_DUPLEX,
		  fontScale = 0.6,
		  color = (0, 0, 0),
		  thickness = 1
		)	
		return new_image
	def show_img(self): 
		try:
		    cv2.imshow("",self.img)
		    cv2.waitKey(0)
		except NameError:
		    print("No image has been init")
	def save_img(self,path): 
		cv2.imwrite(path,self.img)


class GazeboMarkModel():
	def __init__(self,mark_id): 
		self.mid = mark_id
		self.sdf = const.base_sdf.format(mid=mark_id)
		self.cfg = const.base_config.format(mid=mark_id)
		self.mat = const.mscript.format(mid=mark_id)
		self.texture = ArucoImage(mark_id)
	def gen_files(self,path): 
		aux_path = os.getcwd()
		path = path+'/aruco_visual_marker_'+str(self.mid)
		if not os.path.exists(path):
  			os.makedirs(path)
		os.chdir(path)
		with open("model.sdf",'w',encoding = 'utf-8') as f:
		   f.write(self.sdf)
		with open("model.config",'w',encoding = 'utf-8') as f:
		   f.write(self.cfg)
		if not os.path.exists('./materials'):
  			os.makedirs('./materials')
		os.chdir('./materials')
		if not os.path.exists('./scripts'):
			os.makedirs('./scripts')
		if not os.path.exists('./textures'):
			os.makedirs('./textures')	
		with open('./scripts/aruco_visual_marker_{}_marker.material'.format(self.mid),
  					'w',encoding = 'utf-8') as f:
			f.write(self.mat)
		self.texture.save_img('./textures/aruco_mark_{}.png'.format(self.mid))
		os.chdir(aux_path)

class ArucoWorld: 
	def __init__(self,name="aruco",mrefs={0:[0,0]}):
		self.name = name
		marks = ""
		for m in mrefs: 
			pose = mrefs[m]
			marks = marks+const.world_mark_parser.format(mid=m,cord=pose)
		self.world = const.base_world.format(models=marks)
	def gen_file(self,path):
		aux_path = os.getcwd()
		os.chdir(path)
		with open(self.name+".world",'w',encoding = 'utf-8') as f:
			f.write(self.world)
		os.chdir(aux_path)