#!/usr/bin/env python2
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import path_parser
import cv2


#map_size_x = 600  # cm
#map_size_y = 430  # cm
#map_size_x = 596  # cm
#map_size_y = 596  # cm
mname = 'tmr2024'
img = cv2.imread('./maps/'+mname+'/map.png')
(h,w,l) = img.shape
print(h)
del img
map_size_x = w  # cm
map_size_y = h  # cm
lookahead_offset = 0  # *resolution cm = for example 5 *10= 50 cm //Dynamic: np.int(2 + (8/(8*dist+1)))
resolution = 1
matrix = np.zeros((map_size_x / resolution, map_size_y / resolution, 2), dtype='f')


def main(map_name, lane, la_o):
    global lookahead_offset
    lookahead_offset = la_o
    path = './maps/'+map_name+'/'
    map_file = path+'new_map_loop'+str(lane)+'.txt'
    xy = np.array(list(path_parser.read_points(map_file)))
    x, y = xy.T

    tree = KDTree(xy)

    #fig = plt.figure(figsize=(12, 10), facecolor='w')
    #plt.plot(x, y, ':o', markersize=2)

    #plt.tight_layout()

    def show_nearest(target):
        dist, index = tree.query(target)
        global lookahead_offset
        # lookahead_offset = 10#np.int(2 + (8/(8*dist+1)))
        lookahead_target = xy[(index + lookahead_offset) % len(xy)]

        x1, y1 = target
        x3, y3 = lookahead_target

        #plt.scatter(*target, color='r')
        #plt.scatter(*xy[index], color='g')
        #ax = plt.axes()
        #ax.arrow(x1, y1, (x3-x1)/5, (y3-y1)/5 , head_width=0.01, head_length=0.01, fc='k', ec='k')
        #plt.scatter(*lookahead_target, color='m')
        #plt.show(block=False)
        global matrix
        x_index = np.int(round(x1 * (100.0 / resolution)))
        y_index = np.int(round(y1 * (100.0 / resolution)))

        matrix[x_index, y_index, 0] = x3 - x1
        matrix[x_index, y_index, 1] = y3 - y1

    print('please wait ...')
    for x in range(0, map_size_x / resolution):
        print(str(x)+'/'+str(int(map_size_x / resolution)-1))
        for y in range(0, map_size_y / resolution):
            show_nearest(((resolution / 100.0) * x, (resolution / 100.0) * y))

    np.save(path + 'matrix' + str(lookahead_offset) + 'cm_lane'+str(lane)+'.npy', matrix)
    print('matrix is saved.')
    print(path + 'matrix' + str(lookahead_offset) + 'cm_lane'+str(lane)+'.npy')
    plt.show()


if __name__ == '__main__':
    main(map_name=mname,lane=1, la_o=0)
    main(map_name=mname,lane=1, la_o=25)
    main(map_name=mname,lane=1, la_o=50)
    main(map_name=mname,lane=1, la_o=-25)
    main(map_name=mname,lane=1, la_o=-50)
    main(map_name=mname,lane=2, la_o=0)
    main(map_name=mname,lane=2, la_o=25)
    main(map_name=mname,lane=2, la_o=50)
    main(map_name=mname,lane=2, la_o=-25)
    main(map_name=mname,lane=2, la_o=-50)
    
    #main(map_name=mname,lane=3, la_o=50)
    """
    main(map_name=mname,lane=2, la_o=-25)
    main(map_name=mname,lane=2, la_o=0)
    main(map_name=mname,lane=2, la_o=25)
    """
