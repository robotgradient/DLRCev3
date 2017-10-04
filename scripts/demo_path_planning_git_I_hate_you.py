
from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path
from rick.A_star_planning import *
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import time

rob = [0,0,0]
real_rob_pos = [0,0, pi]
path = []
itera = 0
R = []
R2 = []
plotc = 0
pos1=[70,0]
obj = [120,0]
vel_wheels = np.array([0,0])

P = np.identity(3)

marker_map = np.array([[0,0,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0]])

camino = np.array([np.array(rob[0:2]),np.array(obj)])
print(camino)

#prueba=compute_piecewise_path([0,0],pos1,obj)
obslist=[[50,0],[50,10],[50,-10],[50,20],[50,-20]]
Map=create_map(obslist)



goal=[100,100]


print("origin and objective",rob,obj)

plotc=0
path_plot =compute_A_star_path(rob[0:2],obj,Map)

while 1:


    Ts = 0.1

    #rob,vel_wheels,path = euclidian_path_planning_control(rob,obj, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    #rob,vel_wheels,path = piecewise_path_planning_control(rob,pos1,obj, Ts, path=prueba,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    #KALMAN
    rob,vel_wheels,path = A_star_path_planning_control(rob,obj,Map, Ts,K_y = 1, K_an = 1, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)

    objective=path[-1,:]
    print("las point of the path",objective)
    distance=np.sqrt(np.power(objective[0]-rob[0],2)+np.power(objective[1]-rob[1],2))
    print("Distance to objective", distance)

   #print("odometry: ", vel_wheels[0]*Ts, "  y ", vel_wheels[1]*Ts)
    #print('robot_position: ',rob)
    #print('wheels vel:', vel_wheels)
    #print("Time last: ", itera*Ts)
    #print('path: ', path)
    itera = itera+1


    R.append(rob)
    R2.append(real_rob_pos)
    robot_pos = np.array(R)
    R22 = np.array(R2)

    if plotc%10==1:
        plt.close()
        plt.figure(1)
        plt.plot(robot_pos[:,0],robot_pos[:,1])

        plt.plot(R22[:,0],R22[:,1])

        plt.plot(path_plot[1:-1,0], path_plot[1:-1,1],linestyle='-')
        plt.plot(robot_pos[-1,0],robot_pos[-1,1],'o')
        plt.axis([-int(Map.shape[0]/2), int(Map.shape[0]/2), -int(Map.shape[1]/2), int(Map.shape[1]/2)])
        plt.legend(["estimated position", "real position", "path","current position"])

        plt.show(block=False)
        plt.pause(0.2)
        

    if distance<1:
        time.sleep(1)
        break
    plotc = plotc +1




