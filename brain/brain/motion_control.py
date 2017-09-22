import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt
from math import pi


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
	distance_x = (target[0]-pos_rob[0])*np.sin(pos_rob[2]*pi/180) - (target[1]-pos_rob[1])*np.cos(pos_rob[2]*pi/180)

	l= np.sqrt(np.power(target[0]-pos_rob[0],2)+np.power(target[1]-pos_rob[1],2))

	
	C  = -2*distance_x/np.power(l,2)
	w = 10*R;

	A = (1-(C*L)/2)/(1+(C*L)/2)
	vel_wheels[0] = w*L/(R*(1+A))
	vel_wheels[1] = vel_wheels[0]*A

	vel_robot = np.array([w, w*C])
	vel_wheels =np.matmul(M_r2wheels,vel_robot)

	vel_wheels[0] = 180/pi * vel_wheels[0]
	vel_wheels[1] = 180/pi * vel_wheels[1]

	print(vel_wheels)


	if np.absolute(vel_wheels[0]) > 500 : 
		vel_wheels[0] = np.sign(vel_wheels[0])*500
	if np.absolute(vel_wheels[1]) > 500:
		vel_wheels[1] = np.sign(vel_wheels[1])*500


	#print(vel_wheels)

	
	return vel_wheels


def forward_localization(pos_rob, vel_wheels, Ts): # position of the robot (x,y,teta) , vel_wheels 1x2:(vel_right, vel_left) and Ts(sampling time)
	
	L = 14.5
	R = 1.7  

	vel_wheels[0] = vel_wheels[0] * pi/180
	vel_wheels[1] = vel_wheels[1] * pi/180

	M_wheels2rob= np.array([[R/2,R/2],[-R/L,R/L]])

	M_rob2w = np.array([[np.cos(pos_rob[2]*pi/180),0],[np.sin(pos_rob[2]*pi/180),0],[0,1]])
	#print(M_rob2w)
	vel_robot = np.matmul(M_wheels2rob,vel_wheels)
	#print('vel_robot: ', vel_robot)
	vel_world = np.matmul(M_rob2w,vel_robot) 

	new_pos_rob = np.zeros(3)
	#new_pos_rob[0] = pos_rob[0] + Ts*vel_world[0]
	#new_pos_rob[1] = pos_rob[1] + Ts*vel_world[1]
	#new_pos_rob[2] = pos_rob[2] + Ts*vel_world[2]

	incr_r = vel_robot[0]*Ts
	incr_teta = vel_robot[1]*Ts * 180/pi

	print(incr_r)
	print(incr_teta)


	print('radial increment:',incr_r,' angular increment: ',incr_teta)

	new_pos_rob[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[2] = pos_rob[2] + incr_teta

	print('new pos: ', new_pos_rob)

	if new_pos_rob[2] >360:
		new_pos_rob[2] = new_pos_rob[2] - 360
	elif new_pos_rob[2] < 0 :
		new_pos_rob[2] = 360 + new_pos_rob[2]

	#print(new_pos_rob)
	return new_pos_rob

def odometry_localization(pos_rob, odom_r, odom_l, Ts): # position of the robot (x,y,teta) , vel_wheels 1x2:(vel_right, vel_left) and Ts(sampling time)
	
	L = 14.5
	R = 1.7  

	M_wheels2rob= np.array([[R/2,R/2],[-R/L,R/L]])

	M_rob2w = np.array([[np.cos(pos_rob[2]*pi/180),0],[np.sin(pos_rob[2]*pi/180),0],[0,1]])
	#print(M_rob2w)

	odom_r = odom_r*pi/180
	odom_l = odom_l*pi/180
	vel_wheels = np.array([odom_r,odom_l])

	vel_robot = np.matmul(M_wheels2rob,vel_wheels)
	#print('vel_robot: ', vel_robot)
	vel_world = np.matmul(M_rob2w,vel_robot) 

	new_pos_rob = np.zeros(3)
	#new_pos_rob[0] = pos_rob[0] + Ts*vel_world[0]
	#new_pos_rob[1] = pos_rob[1] + Ts*vel_world[1]
	#new_pos_rob[2] = pos_rob[2] + Ts*vel_world[2]

	incr_r = vel_robot[0]
	incr_teta = vel_robot[1] * 180/pi
	print(incr_teta)


	print('radial increment:',incr_r,' angular increment: ',incr_teta)

	new_pos_rob[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[2] = pos_rob[2] + incr_teta

	if new_pos_rob[2] >360:
		new_pos_rob[2] = new_pos_rob[2] - 360
	elif new_pos_rob[2] < 0 :
		new_pos_rob[2] = 360 + new_pos_rob[2]

	#print(new_pos_rob)
	return new_pos_rob

def select_target(pos_rob,path):

	print(np.size(path))
	shortest_dist  = 100000000000;
	for i in range (0,np.size(path[1,:])): #compute the euclidean distance for all the possible points to go

		#distance = np.sqrt(np.power(path[0,i]-pos_rob[0],2)+np.power(path[1,i]-pos_rob[1],2))
		distance = np.absolute((path[0,i]-pos_rob[0])*np.sin(pos_rob[2]*pi/180) - (path[1,i]-pos_rob[1])*np.cos(pos_rob[2]*pi/180))
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



def kalman_filter(odom_r,odom_l,pos_rob,marker_list, marker_map,Ts,P):

	L = 14.5
	R = 1.7  

	#From degrees to radians

	odom_l = odom_l*pi/180
	odom_r = odom_r*pi/180

	# get increments

	incr_r = R/2*(odom_r+odom_l)
	incr_teta = R/L*(odom_l-odom_r) * 180/pi

	## A and B matrixes
	increment_R = R/2*(odom_r + odom_l)
	increment_teta = R/L(odom_l-odom_l) * 180/pi # We want the increment in teta in degrees

	A = np.array([[1 , 0, -increment_R*np.sin((pos_rob[2]+increment_teta/2)*pi/180)],[0,1, increment_R*np.cos((pos_rob[2]+increment_teta/2)*pi/180)],[0,0,1]])

	c = np.cos(pos_rob[2]+increment_teta/2); s = np.sin(pos_rob[2]+increment_teta/2)

	B = np.array[[R/2*c+R*increment_R/(2*L)*s, R/2*c-R*increment_R/(2*L)*s],[R/2*s-increment_R/(2*L)*c, R/2*s+increment_R*R/(2*L)*c ],[-R/L,R/L]]


	# H Matrix

	markers = []
	for i in range (0,marker_list[:,1]):

		if marker_list[i,1] < 900:

			markers.append = i

	#The size of the H array is related with the number of markers we see
	H = np.array([len(markers)*3,3])

	R = np.zeros([len(markers),len(markers)])

	for i in range(0,len(markers)):

		distance = np.power(marker_map[markers[i],0]-pos_rob[0],2) + np.power(marker_map[markers[i],1]-pos_rob[1],2)
		H[i*3,0] = (marker_map[markers[i],1]-pos_rob[1])/distance
		H[i*3,1] = -(marker_map[markers[i],0]-pos_rob[0])/distance
		H[i*3,2] = -1

		H[i*3+1,0] = (pos_rob[0]-marker_map[markers[i],0])/np.sqrt(distance)
		H[i*3+1,1]= (pos_rob[1]-marker_map[markers[i],1])/np.sqrt(distance)
		H[i*3+1,2] = 0

		H[i*3+2,0] = 0
		H[i*3+2,1] = 0
		H[i*3+2,2] = -1

		#Noise of the measuremenets

		R[i*3,i*3] = np.power(10,-5)
		R[i*3+1,i*3+1] = np.power(10,-6)
		R[i*3+2,i*3+2] = np.power(10,-6)


	# Process noise

	#noise variance of the encoders
	noise_enc = np.power(10,-7)
	var_noise_enc = np.power(noise_enc/Ts,2)
	#noise variance of the model
	Q = np.zeros([3,3])
	Q[0,0] = np.power(10,-4)
	Q[1,1] = np.power(10,-4)
	Q[2,2] = np.power(7.62,-5)

	# Kalman init

	#Prediction step

	P_pred = np.sum(np.sum(np.multiply(A,np.multiply(P,np.transpose(A))), var_noise_enc*np.multiply(B,np.transpose(B))),Q)

	pos_rob_pred[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[2] = (pos_rob[2] + incr_teta)*pi/180

	if pos_rob_pred[2] > pi:
		pos_rob_pred[2] = pos_rob_pred[2]-(2*pi)
	if pos_rob_pred[2] < -pi:
		pos_rob_pred[2] = pos_rob_pred[2]+(2*pi)	

	#Measurements prediction & measurements


	meas_vec = np.array([len(markers)*3])


	for i in range(0,len(markers)):

		z[i*3] = np.atan2(marker_map[markers[i],1]-pos_rob_pred[1],marker_map[markers[i],0]-pos_rob_pred[0])- pos_rob_pred[2]
		z[i*3+1] = np.sqrt(np.power(marker_map[markers[i],0]-pos_rob_pred[0],2) + np.power(marker_map[markers[i],1]-pos_rob_pred[1],2))
		z[i*3+2] = marker_map[2]- pos_rob_pred[2]

		if z[i*3] > pi:
			z[i*3] = z[i*3]-(2*pi)
		if z[i*3] < -pi:
			z[i*3] = z[i*3]+(2*pi)

		if z[i*3+2] > pi:
			z[i*3+2] = z[i*3+2]-(2*pi)
		if z[i*3+2] < -pi:
			z[i*3+2] = z[i*3+2]+(2*pi)

		meas_vec[i*3] = marker_list[markers[i],0]
		meas_vec[i*3+1] = marker_list[markers[i],1]
		meas_vec[i*3+2] = marker_list[markers[i],2]







	HPHR = np.sum(np.multiply(H,np.multiply(P_pred,np.transpose(H)))+R)

	K = np.multiply(P_pred,np.multiply(np.transpose(H),np.inverse(HPHR)))



	IKH = np.sum(np.identity(3),-np.multiply(K,H))

	P = np.sum(np.multiply(IKH,np.multiply(P_pred,np.transpose(IKH))),np.multiply(K,np.multiply(R,np.transpose(K))))

	#Kalman's state estimation : 

	

	pos_incr = np.multiply(K,np.sum(z,-meas_vec))

	pos_rob = np.sum(pos_rob_pred,pos_incr)


	



	if pos_rob[2] >360:
		pos_rob[2] = new_pos_rob[2] - 360
	elif pos_rob[2] < 0 :
		pos_rob[2] = 360 + new_pos_rob[2]

	#print(new_pos_rob)
	return pos_rob,P


		




def euclidian_path_planning_control(pos_rob,pos_obj, Ts, points=5,K_x=1,K_y = 1, K_an = 1 , iteration = 0, path = [] , odom_r = 0,odom_l= 0, P=np.identity(3), marker_list = [],marker_map=[]):
	
	if iteration == 0 : 

		path = compute_euclidean_path(pos_rob,pos_obj,points)

	target, new_path = select_target(pos_rob, path)

	#Only Odometry
	estim_rob_pos= odometry_localization(pos_rob,odom_r,odom_l,Ts)


	#estim_rob_pos = pos_rob

	vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)

	#estim_rob_pos = forward_localization(pos_rob, vel_wheels, Ts) 

	return estim_rob_pos,vel_wheels,new_path









'''rob = [50,50,180]
path = []
itera = 0
R = []
plotc = 0
obj = [100,100]
vel_wheels = np.array([0,0])

camino = np.array([np.array(rob[0:2]),np.array(obj)])
print(camino)

while 1:
	
	
	Ts = 0.05

	rob,vel_wheels,path = euclidian_path_planning_control(rob,obj, Ts, path=path,iter = itera, odom_r = vel_wheels[0]*Ts , odom_l = vel_wheels[1]*Ts)
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



	

	
	#time.sleep(0.5)'''

