import numpy as np
import math
import time
import random
from math import pi
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

	#print(path)

	return path

## Now this functionc compute a piecewise euclidean path with the intermediate point pos_1
def compute_piecewise_path(pos_rob,pos_1,pos_obj,points=10):
	x1=np.linspace(pos_rob[0],pos_1[0],num=round(points/2))
	y1=np.linspace(pos_rob[1],pos_1[1],num=round(points/2))
	x2=np.linspace(pos_1[0],pos_obj[0],num=round(points/2)+1)
	y2=np.linspace(pos_1[1],pos_obj[1],num=round(points/2)+1)
	x2=x2[1:]
	y2=y2[1:]

	x=np.concatenate((x1,x2))
	y=np.concatenate((y1,y2))

	angle1=math.atan2(pos_1[1]-pos_rob[1],pos_1[0]-pos_rob[0])
	angle2=math.atan2(pos_obj[1]-pos_1[1],pos_obj[0]-pos_1[0])

	angle1=math.degrees(angle1)
	angle2=math.degrees(angle2)

	if angle1<0:
		angle1=360-angle1
	if angle2<0:
		angle2=360-angle2

	angle_vec1 = np.ones(x1.shape)
	angle_vec2=np.ones(x2.shape)

	angle_vec1.fill(angle1)
	angle_vec2.fill(angle2)

	angle_vec=np.concatenate((angle_vec1,angle_vec2))

	path = np.array([x,y,angle_vec])
	plt.plot(path[0,:],path[1,:])
	plt.axis([-100, 300, -100, 300])
	plt.show()

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
	
	#print("L is: ",l)


	C  = -2*distance_x/np.power(l,2)
	w = 3*R;

	#A = (1-(C*L)/2)/(1+(C*L)/2)
	#vel_wheels[0] = w*L/(R*(1+A))
	#vel_wheels[1] = vel_wheels[0]*A

	vel_robot = np.array([w, w*C])
	vel_wheels =np.matmul(M_r2wheels,vel_robot)

	vel_wheels[0] = 180/pi * vel_wheels[0]
	vel_wheels[1] = 180/pi * vel_wheels[1]

	#print(vel_wheels)


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


	#print('radial increment:',incr_r,' angular increment: ',incr_teta)

	new_pos_rob[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[2] = pos_rob[2] + incr_teta

	#print('new pos: ', new_pos_rob)

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
	#print(incr_teta)


	#print('radial increment:',incr_r,' angular increment: ',incr_teta)

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

	#print(np.size(path))
	shortest_dist  = 100000000000;
	shd2 = 100000000000;
	for i in range (path.shape[1]): #compute the euclidean distance for all the possible points to go

		distance2 = np.sqrt(np.power(path[0,i]-pos_rob[0],2)+np.power(path[1,i]-pos_rob[1],2))
		distance = np.absolute((path[0,i]-pos_rob[0])*np.sin(pos_rob[2]*pi/180) - (path[1,i]-pos_rob[1])*np.cos(pos_rob[2]*pi/180))
		#print(i," : i : ",distance)
		if distance <= shortest_dist :

			shortest_dist = distance
			output = i
			if output == np.size(path[1,:])-1:
				#print("hola")
				output = i-1

	new_path = path[:,output:]
	target = path[:,output+1]
	#print('Point to go : ',target)
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
	increment_teta = R/L*(odom_l-odom_r) * 180/pi # We want the increment in teta in degrees

	
	A = np.identity(3)
	A[0,2] = -increment_R*np.sin((pos_rob[2]+increment_teta/2)*pi/180)
	A[1,2] =  increment_R*np.cos((pos_rob[2]+increment_teta/2)*pi/180)

	c = np.cos((pos_rob[2]+increment_teta/2)*pi/180); s = np.sin((pos_rob[2]+increment_teta/2)*pi/180)

	
	B = np.zeros([3,2])

	B[0,0] = R/2*c+R*increment_R*R/(2*L)*s
	B[0,1] = R/2*c-R*increment_R*R/(2*L)*s

	B[1,0] = R/2*s-increment_R*R/(2*L)*c
	B[1,1] = R/2*s+increment_R*R/(2*L)*c

	B[2,0] = -R/L
	B[2,1] = R/L



	# H Matrix
	marker_list=np.array(marker_list)
	markers = []
	for i in range (0,marker_list.shape[0]):
		#print("marker list",marker_list)
		if marker_list[i,0] < 900:
			distance = np.power(marker_map[i,0]-pos_rob[0],2) + np.power(marker_map[i,1]-pos_rob[1],2)
		
			if distance != 0:

				markers.append(i)

	#The size of the H array is related with the number of markers we see

	#H = np.zeros([len(markers)*3,3])
	H = np.zeros([len(markers)*2,3])

	#R = np.zeros([3*len(markers),3*len(markers)])
	R = np.zeros([2*len(markers),2*len(markers)])



	for i in range(0,len(markers)):

		distance = np.power(marker_map[markers[i],0]-pos_rob[0],2) + np.power(marker_map[markers[i],1]-pos_rob[1],2)
		'''
		H[i*3,0] = (marker_map[markers[i],1]-pos_rob[1])/distance
		H[i*3,1] = -(marker_map[markers[i],0]-pos_rob[0])/distance
		H[i*3,2] = -1

		H[i*3+1,0] = (pos_rob[0]-marker_map[markers[i],0])/np.sqrt(distance)
		H[i*3+1,1]= (pos_rob[1]-marker_map[markers[i],1])/np.sqrt(distance)
		H[i*3+1,2] = 0

		H[i*3+2,0] = 0
		H[i*3+2,1] = 0
		H[i*3+2,2] = -1
		'''
		H[i*2,0] = (marker_map[markers[i],1]-pos_rob[1])/distance
		H[i*2,1] = -(marker_map[markers[i],0]-pos_rob[0])/distance
		H[i*2,2] = -1

		H[i*2+1,0] = (pos_rob[0]-marker_map[markers[i],0])/np.sqrt(distance)
		H[i*2+1,1]= (pos_rob[1]-marker_map[markers[i],1])/np.sqrt(distance)
		H[i*2+1,2] = 0



		#Noise of the measuremenets

		#R[i*3,i*3] = 1/np.power(10,5)
		#R[i*3+1,i*3+1] = 1/np.power(10,6)
		#R[i*3+2,i*3+2] = 1/np.power(10,6)
		R[i*2,i*2] = 1/np.power(10,5)
		R[i*2+1,i*2+1] = 1/np.power(10,6)



	# Process noise
	#print(H)

	#noise variance of the encoders
	noise_enc = 1/np.power(10,7)
	var_noise_enc = np.power(noise_enc/Ts,2)
	#noise variance of the model
	Q = np.zeros([3,3])
	Q[0,0] = 1/np.power(10,4)
	Q[1,1] = 1/np.power(10,4)
	Q[2,2] = 1/np.power(7.62,5)

	# Kalman init

	#Prediction step

	P_pred = np.add(np.add(np.multiply(A,np.multiply(P,np.transpose(A))), var_noise_enc*np.dot(B,np.transpose(B))),Q)

	pos_rob_pred = np.ones(3)

	pos_rob_pred[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[2] = (pos_rob[2] + incr_teta)*pi/180

	if pos_rob_pred[2] > pi:
		pos_rob_pred[2] = pos_rob_pred[2]-(2*pi)
	if pos_rob_pred[2] < -pi:
		pos_rob_pred[2] = pos_rob_pred[2]+(2*pi)

	#Measurements prediction & measurements


	#meas_vec = np.zeros(len(markers)*3)
	meas_vec = np.zeros(len(markers)*2)


	#z = np.zeros(3*len(markers))
	z = np.zeros(2*len(markers))

	for i in range(0,len(markers)):

		#z[i*3] = np.arctan2(marker_map[markers[i],1]-pos_rob_pred[1],marker_map[markers[i],0]-pos_rob_pred[0])- pos_rob_pred[2]
		#z[i*3+1] = np.sqrt(np.power(marker_map[markers[i],0]-pos_rob_pred[0],2) + np.power(marker_map[markers[i],1]-pos_rob_pred[1],2))
		#z[i*3+2] = marker_map[markers[i],2]- pos_rob_pred[2]

		z[i*2] = np.arctan2(marker_map[markers[i],1]-pos_rob_pred[1],marker_map[markers[i],0]-pos_rob_pred[0])- pos_rob_pred[2]
		z[i*2+1] = np.sqrt(np.power(marker_map[markers[i],0]-pos_rob_pred[0],2) + np.power(marker_map[markers[i],1]-pos_rob_pred[1],2))
		
		'''
		if z[i*3] > pi:
			z[i*3] = z[i*3]-(2*pi)
		if z[i*3] < -pi:
			z[i*3] = z[i*3]+(2*pi)

		if z[i*3+2] > pi:
			z[i*3+2] = z[i*3+2]-(2*pi)
		if z[i*3+2] < -pi:
			z[i*3+2] = z[i*3+2]+(2*pi)
		'''
		if z[i*2] > pi:
			z[i*2] = z[i*2]-(2*pi)
		if z[i*2] < -pi:
			z[i*2] = z[i*2]+(2*pi)


		'''	
		meas_vec[i*3] = marker_list[markers[i],0]
		meas_vec[i*3+1] = marker_list[markers[i],1]
		meas_vec[i*3+2] = marker_list[markers[i],2]
		'''
		meas_vec[i*2] = marker_list[markers[i],0]
		meas_vec[i*2+1] = marker_list[markers[i],1]






	HPHR = np.add(np.dot(H,np.dot(P_pred,np.transpose(H))),R)

	K = np.dot(P_pred,np.dot(np.transpose(H),np.linalg.inv(HPHR)))



	IKH = np.add(np.identity(3),-np.dot(K,H))

	P = np.add(np.dot(IKH,np.dot(P_pred,np.transpose(IKH))),np.dot(K,np.dot(R,np.transpose(K))))

	#Kalman's state estimation :



	pos_incr = np.dot(K,np.add(z,-meas_vec))

	print('expected: ',z)
	print('real: ', meas_vec)


	print('measurement error : ',pos_incr)

	pos_rob = np.add(pos_rob_pred,-pos_incr)





	pos_rob[2] = pos_rob[2]* 180/pi
	if pos_rob[2] >360:
		pos_rob[2] = pos_rob[2] - 360
	elif pos_rob[2] < 0 :
		pos_rob[2] = 360 + pos_rob[2]

	#print(new_pos_rob)
	return pos_rob,P


def create_fake_measurements(pos_rob, odom_l,odom_r , marker_map, num_mar = 4):

	L = 14.5
	R = 1.7

	# ODOMETRY

	#From degrees to radians

	odom_l = odom_l*pi/180 
	odom_r = odom_r*pi/180 

	# get increments

	incr_r = R/2*(odom_r+odom_l)
	incr_teta = R/L*(odom_l-odom_r) * 180/pi

	# REAL COMPUTATION OF THE STATE : 

	pos_rob_pred = np.zeros(3)
	pos_rob_pred[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	pos_rob_pred[2] = (pos_rob[2] + incr_teta)*pi/180

	# Measurements


	z = np.zeros([num_mar,3])
	for i in range(num_mar):

		z[i,0] = np.arctan2(marker_map[i,1]-pos_rob_pred[1],marker_map[i,0]-pos_rob_pred[0])- pos_rob_pred[2]
		z[i,1] = np.sqrt(np.power(marker_map[i,0]-pos_rob_pred[0],2) + np.power(marker_map[i,1]-pos_rob_pred[1],2))
		z[i,2] = marker_map[i,2]- pos_rob_pred[2]

		if z[i,0] > pi:
			z[i,0] = z[i,0]-(2*pi)
		if z[i,0] < -pi:
			z[i,0] = z[i,0]+(2*pi)

		if z[i,0+2] > pi:
			z[i,0+2] = z[i,0+2]-(2*pi)
		if z[i,0+2] < -pi:
			z[i,0+2] = z[i,0+2]+(2*pi)


	pos_rob = pos_rob_pred

	pos_rob[2] = pos_rob[2]* 180/pi
	if pos_rob[2] >360:
		pos_rob[2] = pos_rob[2] - 360
	elif pos_rob[2] < 0 :
		pos_rob[2] = 360 + pos_rob[2]


	return pos_rob , z





def euclidian_path_planning_control(pos_rob,pos_obj, Ts, points=5,K_x=1,K_y = 1, K_an = 1 , iteration = 0, path = [] , odom_r = 0,odom_l= 0):

	if iteration == 0 :

		path = compute_euclidean_path(pos_rob,pos_obj,points)

	target, new_path = select_target(pos_rob, path)

	#Only Odometry
	estim_rob_pos= odometry_localization(pos_rob,odom_r,odom_l,Ts)
	vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)

	return estim_rob_pos,vel_wheels,new_path

def piecewise_path_planning_control(pos_rob,pos1,pos_obj, Ts, points=5,K_x=1,K_y = 1, K_an = 1 , iteration = 0, path = [] , odom_r = 0,odom_l= 0):
	if iteration == 0 :
		path = compute_piecewise_path(pos_rob,pos1,pos_obj,points)
	target, new_path = select_target(pos_rob, path)
	#Only Odometry
	estim_rob_pos= odometry_localization(pos_rob,odom_r,odom_l,Ts)
	vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)
	return estim_rob_pos,vel_wheels,new_path

	# Only model
	#estim_rob_pos = pos_rob

	#vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)

	#estim_rob_pos = forward_localization(pos_rob, vel_wheels, Ts)

	#return estim_rob_pos,vel_wheels,new_path

	#print('odom_r = 0,odom_l',odom_r,odom_l)
	#print('estim_rob_pos',estim_rob_pos)
	#print('vel_wheels',vel_wheels)
	#print(']]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]')


	#real_robot_pos, marker_list = create_fake_measurements(pos_rob, odom_l,odom_r , marker_map)

	#estim_rob_pos, P  = kalman_filter(odom_r,odom_l,pos_rob,marker_list, marker_map,Ts,P)

	#vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)
	
	#return estim_rob_pos,vel_wheels,new_path , P , real_robot_pos



def euclidian_kalman(pos_rob,pos_obj, Ts, points=5,K_x=1,K_y = 1, K_an = 1 , iteration = 0, path = [] , odom_r = 0,odom_l= 0, P=np.identity(3), marker_list = [],marker_map=[], real_bot=[]):

	if iteration == 0 :

		path = compute_euclidean_path(pos_rob,pos_obj,points)

	target, new_path = select_target(pos_rob, path)

	#Only Odometry
	#estim_rob_pos= odometry_localization(pos_rob,odom_r,odom_l,Ts)

	#vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)

	#return estim_rob_pos,vel_wheels,new_path



	# Only model
	#estim_rob_pos = pos_rob

	#vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)

	#estim_rob_pos = forward_localization(pos_rob, vel_wheels, Ts)

	#return estim_rob_pos,vel_wheels,new_path


	#real_robot_pos, marker_list = create_fake_measurements(real_bot, odom_l,odom_r , marker_map)

	estim_rob_pos, P  = kalman_filter(odom_r,odom_l,pos_rob,marker_list, marker_map,Ts,P)

	vel_wheels = robot_control(estim_rob_pos, target, K_x,K_y,K_an)
	
	return estim_rob_pos,vel_wheels,new_path , P #, real_robot_pos
	









	







