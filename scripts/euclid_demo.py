from brain.motion_control import euclidian_path_planning_control
import numpy as np
import matplotlib.pyplot as plt

rob = [0,0,0]
path = []
itera = 0
R = []
plotc = 0
obj = [100,100]
vel_wheels = np.array([0,0])

camino = np.array([np.array(rob[0:2]),np.array(obj)])
print(camino)

while 1:


    Ts = 0.3

    rob,vel_wheels,path = euclidian_path_planning_control(rob,obj, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    print('robot_position: ',rob)
    print('wheels vel:', vel_wheels)
    print("Time last: ", itera*Ts)
    #print('path: ', path)
    itera = itera+1


    R.append(rob)
    robot_pos = np.array(R)

    if plotc>100:

        plt.figure(1)
        plt.plot(robot_pos[:,0],robot_pos[:,1])
        plt.plot(camino[:,0],camino[:,1])
        plt.axis([0, 150, 0, 150])
        plt.show()
        plotc = 0

    plotc = plotc +1




