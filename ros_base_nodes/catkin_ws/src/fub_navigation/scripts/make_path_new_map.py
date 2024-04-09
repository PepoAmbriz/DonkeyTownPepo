#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import importlib
import maps_info as minfo


fig = plt.figure()

def save(csv_name,lane,map_name):
    npoints = 50000
    mcfg = minfo.pts_ctrl[map_name]
    puntos_ctrl = np.matrix(mcfg[lane])
    print(puntos_ctrl.shape)
    t = np.linspace(0, (puntos_ctrl.shape[0]-1)/3-0.0001, npoints)
    
    x = []
    y = []

    for ti in t: 
        i = int(ti)
        x0 = puntos_ctrl[3*i, 0]
        x1 = puntos_ctrl[3*i+1, 0]
        x2 = puntos_ctrl[3*i+2, 0]
        x3 = puntos_ctrl[3*i+3, 0]
        y0 = puntos_ctrl[3*i, 1]
        y1 = puntos_ctrl[3*i+1, 1]
        y2 = puntos_ctrl[3*i+2, 1]
        y3 = puntos_ctrl[3*i+3, 1]
        t2 = ti-i
        xp = (x0*(1-t2)**3+3*t2*x1*(1-t2)**2+3*t2**2*x2*(1-t2)+x3*t2**3)/100
        yp = (y0*(1-t2)**3+3*t2*y1*(1-t2)**2+3*t2**2*y2*(1-t2)+y3*t2**3)/100
        if(ti==0): 
            x.append(xp)
            y.append(yp)
            print(xp, yp)
        elif(np.abs(xp-x[-1])>=0.01 or np.abs(yp-y[-1])>=0.01):
            x.append(xp)
            y.append(yp)
            print(xp, yp)
    plt.hold(True)
    name=csv_name
    file = open(name,"w")
    for i in range(len(x)):
        if (lane=='lane1'):
            file.write('1.1.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
        if (lane=='lane2'):
            file.write('1.2.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
        if (lane=='lane3'):
            file.write('1.3.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
    
        plt.hold(True)
        plt.plot(x[i],y[i],':o')
    file.close()

def main():
    
    mname = "tmr2024"
    save("./maps/"+mname+"/new_map_loop1.txt",lane='lane1',map_name=mname)
    plt.show()
    save("./maps/"+mname+"/new_map_loop2.txt",lane='lane2',map_name=mname)
    plt.show()
    

    """
    mname = 'inhouse'
    save("./maps/"+mname+"/new_map_loop3.txt",lane='lane3',map_name=mname)
    plt.show()
    """

if __name__ == '__main__':
    main()