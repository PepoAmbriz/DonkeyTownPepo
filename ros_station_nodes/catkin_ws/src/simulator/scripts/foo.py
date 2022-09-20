#!/home/emmanuel/anaconda3/bin/python
from aruco_model import * 

for i in range(50): 	
	model = GazeboMarkModel(i)
	model.gen_files('./models')
	del model

mref = {}
for i in range(50):
	mref[i] = [0.75*(i%4), 0.5*(i//4)] 

"""
world = ArucoWorld(name="aruco",mrefs=mref)
world.gen_file('./worlds')
"""