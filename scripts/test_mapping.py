from rick.motion_control import euclidian_path_planning_control, euclidian_kalman
from slam import mapping 
import numpy as np
import matplotlib.pyplot as plt
from math import pi


import cv2
import time
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
            
rob = [0,0,0]
real_rob_pos = [0,0, pi]
path = np.ones([5,3])
itera = 0
R = []
R2 = []
plotc = 0
pos1=[70,0]
obj = [100,0]
vel_wheels = np.array([0,0])

P = np.identity(3)

marker_map = np.array([[0,0,0],[50, 0 , 0],[100,0,0],[0,100,0],[100,100,0]])

camino = np.array([np.array(rob[0:2]),np.array(obj)])
print(camino)

n_obs = 50
real_mapa = np.random.randint(200,size=[n_obs,2])
mapa = [];
delete_countdown = 0
robot_trajectory = []

data = np.load('Homography.npz')
H=data["arr_0"]

cap = cv2.VideoCapture(1)
while 1:


    Ts = 0.0001

    #rob,vel_wheels,path = euclidian_path_planning_control(rob,obj, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
    #rob,vel_wheels,path = piecewise_path_planning_control(rob,pos1,obj, Ts, path=prueba,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)


    #KALMAN
   # rob,vel_wheels,path, P, real_rob_pos = euclidian_kalman(rob,obj, Ts, path=path,iteration = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts, P=P ,
   #                                                                                                 marker_map = marker_map, marker_list = [], real_bot= real_rob_pos)



    # FAKE LEGO POSITIONS
    #fake_landmarks = mapping.create_fake_lego_measurements(real_rob_pos, real_mapa)


    #REAL LEGO POSITIONS
    t0 = time.time()
    while time.time()-t0 < 0.05:
    	ret,frame=cap.read()

    ret,frame=cap.read()
    BB_legos=get_lego_boxes(frame)
   

    real_landmarks = mapping.cam2rob(BB_legos,H)
    fake_landmarks = real_landmarks

    #UPDATE MAP

    mapa, delete_countdown,robot_trajectory = mapping.update_mapa(mapa,fake_landmarks,rob,P,delete_countdown, robot_trajectory)
    print("Delete countdown: ", delete_countdown)

    mapa1 = np.array(mapa)


    

    print("odometry: ", vel_wheels[0]*Ts, "  y ", vel_wheels[1]*Ts)
    print('robot_position: ',rob)
    print('wheels vel:', vel_wheels)
    print("Time last: ", itera*Ts)
    #print('path: ', path)
    itera = itera+1


    R.append(rob)
    R2.append(real_rob_pos)
    robot_pos = np.array(R)
    R22 = np.array(R2)

    if plotc>-1:

        plt.figure(1)
        plt.plot(robot_pos[:,0],robot_pos[:,1])

        plt.plot(R22[:,0],R22[:,1])

        plt.plot(camino[:,0],camino[:,1])

        #plt.scatter(real_mapa[:,0],real_mapa[:,1])
        print("mapitaaa: ",mapa1)
        if mapa1.size:
        	plt.scatter(mapa1[:,0],mapa1[:,1])
        plt.axis([-100, 150, -100, 150])
        plt.legend(["estimated position", "real position", "path"])
        plt.show()
        plotc = 0

    plotc = plotc +1

