import numpy as np
import matplotlib.pyplot as plt
import path_parser
from scipy.spatial import KDTree
import cv2


mname = 'new_indoors'
img = cv2.imread('./maps/'+mname+'/map.png')
(h,w,l) = img.shape
print(h)
del img
map_size_x = w  # cm
map_size_y = h  # cm

def main(map=1, lookahead=0, resolution=10):
	global mname 
	path = './maps/'+mname+'/'
	map_file = path+'new_map_loop'+str(map)+'.txt'
	matrix = np.load(path+'matrix'+str(lookahead)+'cm_lane'+str(map)+'.npy')
	xy = np.array(list(path_parser.read_points(map_file)))
	x, y = xy.T

	tree = KDTree(xy)

	fig = plt.figure(figsize=(12, 10), facecolor='w')

	def show_nearest(target):
	    dist, index = tree.query(target)
	    global lookahead_offset
	    # lookahead_offset = 10#np.int(2 + (8/(8*dist+1)))

	    x1, y1 = target
	    matrix
	    x_index = np.int(round(x1 * 100.0))
	    y_index = np.int(round(y1 * 100.0))
	    x3, y3 = (matrix[x_index,y_index,0],matrix[x_index,y_index,1])

	    plt.scatter(*target, color='r')
	    plt.scatter(*xy[index], color='g')
	    ax = plt.axes()
	    ax.arrow(x1, y1, x3/5, y3/5 , head_width=0.01, head_length=0.01, fc='k', ec='k')
	    #plt.scatter(*lookahead_target, color='m')
	    #plt.show(block=False)
	for i in 	range(0, int(map_size_x / resolution)):
		print(str(i)+'/'+str(int(map_size_x / resolution)-1))
		for j in range(0, int(map_size_y / resolution)):
			show_nearest(((resolution / 100.0) * i, (resolution / 100.0) * j))
	plt.show()

if __name__ == '__main__':
	#main(1,0,25)
	#main(1,25,25)
	#main(2,0,25)
	#main(2,25,25)
	#main(1,0,10)
	main(2,25,16)