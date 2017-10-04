
from rick.mc_please_github_donot_fuck_with_this_ones import A_star_path_planning_control,compute_A_star_path,A_star_kalman
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
#difficult one
obslist=[[20,0],[20,10],[20,-10],[10,15],[10,-15],[0,20],[0,-20],[-10,20],[-10,-20]]
#medium one
obslist=[[20,0],[20,10],[20,-10],[80,0],[80,10],[80,-10]]
Map=create_map(obslist)


firstway=0

print("origin and objective",rob,obj)

plotc=0
path_plot =compute_A_star_path(rob[0:2],obj,Map)

while 1:


    Ts = 0.1

    #rob,vel_wheels,path = euclidian_path_planning_control(rob,obj, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    #rob,vel_wheels,path = piecewise_path_planning_control(rob,pos1,obj, Ts, path=prueba,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    #KALMAN
    #rob,vel_wheels,path = A_star_path_planning_control(rob,obj,Map, Ts,K_y = 1, K_an = 1, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    rob,vel_wheels,path, P, real_rob_pos = A_star_kalman(rob,obj,Map, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts, P=P ,
        marker_map = marker_map, marker_list = [], real_bot= real_rob_pos)
    
    objective=path[0,:]
    print("next_target",objective)
    distance=np.sqrt(np.power(obj[0]-rob[0],2)+np.power(obj[1]-rob[1],2))


   #print("odometry: ", vel_wheels[0]*Ts, "  y ", vel_wheels[1]*Ts)
    print('robot_position:  x ,y and theta',rob[0],rob[1],rob[2])
    print('wheels vel:', vel_wheels)
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
        if firstway==1:
            plt.plot(path_plot2[1:-1,0], path_plot2[1:-1,1],'k')
        else:
            plt.plot(path_plot[1:-1,0], path_plot[1:-1,1],'b')
        plt.plot(robot_pos[-1,0],robot_pos[-1,1],'o')
        r_arrow=[robot_pos[-1,0]+2*np.cos(robot_pos[-1,2]*np.pi/180),robot_pos[-1,1]+2*np.sin(robot_pos[-1,2]*np.pi/180)]
        plt.plot(r_arrow[0],r_arrow[1],'*')
        plt.axis([-int(Map.shape[0]/2), int(Map.shape[0]/2), -int(Map.shape[1]/2), int(Map.shape[1]/2)])
        plt.legend(["estimated position", "real position", "path","current position"])
        #plt.show()
        plt.show(block=False)
        plt.pause(0.2)
        

    if distance<0.5:
        if firstway==1:
            print("Finished")
            break
        print("GOAL REACHED COMING BACK")
        obj=np.array([0,0])
        path_plot2 =compute_A_star_path(rob[0:2],obj,Map)

        plt.figure(2)
        plt.plot(path_plot2[:,0], path_plot2[:,1],'k')
        plt.legend('coming back path')
        plt.show()
        itera=0

        firstway=1


    plotc = plotc +1




