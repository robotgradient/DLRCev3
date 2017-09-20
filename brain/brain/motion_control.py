import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt


def compute_euclidean_path(pos_rob,pos_obj, points = 5): #pos_rob is a 1x3 matrix with(x,y,teta) &  pos_obj is a 1x2 matrix with(x,y) 

	x = np.linspace(pos_rob[0], pos_obj[0], num=points)
	y = np.linspace(pos_rob[1], pos_obj[1], num=points)

	angle =  math.atan2(pos_obj[1]-pos_rob[1], pos_obj[0]-pos_rob[0]) 
	angle = math.degrees(angle)

	if angle < 0:
		angle = 360-angle

	angle_vec = np.ones(points)
	angle_vec.fill(angle)

	path = np.array([x,y,angle_vec])

	print(path)
	
	return path


def robot_control(pos_rob,target, K_x=1,K_y=1,K_an=1): #pos_rob is a 1x3 matrix with(x,y,teta) &  target is a 1x2 matrix with(x,y) 
	
	# Radius and wheel width in cm
	L = 14.5
	R = 1.7   

	'''error_x = target[0] - pos_rob[0]
	error_y = target[1] - pos_rob[1]

	error_ang = target[2] - pos_rob[2]


	if error_ang > 180:
		error_ang = error_ang - 180

	print("error angle",error_ang)

	vel_x = K_x*error_x
	vel_y = K_y*error_y
	vel_ang = K_an*error_ang




	vel_r= np.sqrt(np.power(vel_x,2)+np.power(vel_y,2))

	vel_teta= vel_ang


	vel_robot= [vel_r , vel_teta]

	M_r2wheels= np.array([[1/R, -L/(2*R) ],[1/R, L/(2*R)]]) # --> (Vr,Vteta) = M * (w_rigth, w_left)

	vel_wheels= np.matmul(M_r2wheels,vel_robot)'''


	# GET wheel velocities through curvature 
	M_r2wheels= np.array([[1/R, -L/(2*R) ],[1/R, L/(2*R)]]) # --> (Vr,Vteta) = M * (w_rigth, w_left)



	vel_wheels = np.ones(2)
	distance_x = (target[0]-pos_rob[0])*np.sin(pos_rob[2]) - (target[1]-pos_rob[1])*np.cos(pos_rob[2])

	l= np.sqrt(np.power(target[0]-pos_rob[0],2)+np.power(target[1]-pos_rob[1],2))
	l = 0.2
	
	C  = -2*distance_x/np.power(l,2)
	w = 1;

	A = (1-(C*L)/2)/(1+(C*L)/2)
	vel_wheels[0] = w*L/(R*(1+A))
	vel_wheels[1] = vel_wheels[0]*A

	vel_robot = np.array([w, w*C])
	vel_wheels =np.matmul(M_r2wheels,vel_robot)

	print(vel_wheels)


	if np.absolute(vel_wheels[0]) >300 : 
		vel_wheels[0] = np.sign(vel_wheels[0])*300
	if np.absolute(vel_wheels[1]) > 300:
		vel_wheels[1] = np.sign(vel_wheels[1])*300


	#print(vel_wheels)

	
	return vel_wheels


def forward_localization(pos_rob, vel_wheels, Ts): # position of the robot (x,y,teta) , vel_wheels 1x2:(vel_right, vel_left) and Ts(sampling time)
	
	L = 14.5
	R = 1.7   

	M_wheels2rob= np.array([[R/2,R/2],[-R/L,R/L]])

	M_rob2w = np.array([[np.cos(pos_rob[2]),0],[np.sin(pos_rob[2]),0],[0,1]])
	#print(M_rob2w)
	vel_robot = np.matmul(M_wheels2rob,vel_wheels)
	#print('vel_robot: ', vel_robot)
	vel_world = np.matmul(M_rob2w,vel_robot) 

	new_pos_rob = np.zeros(3)
	new_pos_rob[0] = pos_rob[0] + Ts*vel_world[0]
	new_pos_rob[1] = pos_rob[1] + Ts*vel_world[1]
	new_pos_rob[2] = pos_rob[2] + Ts*vel_world[2]

	if new_pos_rob[2] >360:
		new_pos_rob[2] = new_pos_rob[2] - 360
	elif new_pos_rob[2] < 0 :
		new_pos_rob[2] = 360 + new_pos_rob[2]

	#print(new_pos_rob)
	return new_pos_rob

def select_target(pos_rob,path,points):

	print(np.size(path))
	shortest_dist  = 100;
	for i in range (0,np.size(path[1,:])): #compute the euclidean distance for all the possible points to go

		#distance = np.sqrt(np.power(path[0,i]-pos_rob[0],2)+np.power(path[1,i]-pos_rob[1],2))
		distance = np.absolute((path[0,i]-pos_rob[0])*np.sin(pos_rob[2]) - (path[1,i]-pos_rob[1])*np.cos(pos_rob[2]))
		print(i," : i : ",distance)
		if distance < shortest_dist :

			shortest_dist = distance
			output = i
			if output == np.size(path[1,:])-1:
				print("hola")
				output = i-1

	new_path = path[:,output:]
	target = path[:,output+1]
	print('target : ',target)
	#print('new path : ',new_path)


	return target , new_path



def main(pos_rob,pos_obj, Ts, points=5,K_x=1,K_y = 1, K_an = 1 , iter = 0, path = []):

	if iter == 0 : 
		path = compute_euclidean_path(pos_rob,pos_obj,points)

	target, new_path = select_target(pos_rob, path,points)

	vel_wheels = robot_control(pos_rob, target, K_x,K_y,K_an)

	estim_rob_pos= forward_localization(pos_rob,vel_wheels,Ts)

	return estim_rob_pos,vel_wheels,new_path









'''rob = [0,0,0]
path = []
itera = 0
R = []
plotc = 0
obj = [10,10]

camino = np.array([np.array(rob[0:2]),np.array(obj)])
print(camino)

while 1:
	
	
	Ts = 0.2

	rob,vel_wheels,path = main(rob,obj, Ts, path=path,iter = itera)
	print('robot_position: ',rob)
	print('wheels vel:', vel_wheels)
	print('path: ', path)
	itera = itera+1


	R.append(rob)
	robot_pos = np.array(R)

	if plotc>100:

		plt.figure(1) 
		plt.plot(robot_pos[:,0],robot_pos[:,1])
		plt.plot(camino[:,0],camino[:,1])
		plt.axis([-10, 30, -10, 10])
		plt.show()
		plotc = 0

	plotc = plotc +1



	

	
	#time.sleep(0.5)
'''