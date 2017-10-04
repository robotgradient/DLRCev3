import numpy as np
import cv2
from math import pi



def points2mapa(landmarks,pos_rob,mapa,P, delete_countdown): #mapa = (x,y, Px,Py, updt)

	new_points2add = []
	update_points = []
	landmarks = np.array(landmarks)
	for i in range(0,landmarks.shape[0]):

			x_mapa = pos_rob[0] + landmarks[i,1]*np.cos((pos_rob[2])*pi/180+landmarks[i,0])

			y_mapa = pos_rob[1] + landmarks[i,1]*np.sin((pos_rob[2])*pi/180+landmarks[i,0])


			new = 1

			mapa_ar = np.array(mapa)

			if delete_countdown ==5 and mapa_ar.size > 0:
				mapa_ar[:,4] = np.zeros([mapa_ar.shape[0]])
				mapa = list(mapa_ar)
				delete_countdown = 0
			else:
				delete_countdown = 0
			sh_dist = 10000;
			p_already = 0
			print('new element', i)
			for j in range(0, mapa_ar.shape[0]):

				distance = np.sqrt(np.power((x_mapa-mapa_ar[j,0]),2) + np.power((y_mapa - mapa_ar[j,1]),2))

				if sh_dist > distance: 
					sh_dist = distance
					p_already = j
			print("shortest distance:", sh_dist)

			if sh_dist < 20 and mapa[p_already][4] != 5:
				print('landmarks', landmarks[i], 'sh distance: ', sh_dist)
				mapa = np.array(mapa)
				mapa[p_already,4] = 1
				mapa = mapa.tolist()
				new = 0

				mapa = relocalize_points(mapa, p_already, x_mapa,y_mapa, i)

			if new ==1:
				print('JODER landmarks', landmarks[i], 'sh distance: ', sh_dist)
				new_points2add.append(i)

	delete_countdown +=1
	return mapa , delete_countdown , new_points2add, update_points


def relocalize_points(mapa, p_already,x_mapa, y_mapa,i):

	# Compute new pos

	mapa_ar = np.array(mapa)
	mapa_ar[p_already, 0] = (mapa_ar[p_already,0] + x_mapa)/2
	mapa_ar[p_already, 1] = (mapa_ar[p_already,1] + y_mapa)/2

	mapa = mapa_ar.tolist()

	return mapa




def cam2rob(BB_legos, H):

	####cam [[[ 270.03048706  448.53890991]]]

	pixel_size =  0.653947100514

	# CENTER OF THE CAMERA
	cam= np.array([242.54,474.87])
	cam2rob_dist = 25

	Lego_list = []
	for box in BB_legos:

		y = box[3]
		x = box[0] + (box[2]-box[0])/2

		input_vector=np.array([[[x,y]]],dtype=np.float32)
		output_vector=cv2.perspectiveTransform(input_vector,np.linalg.inv(H))
		
		distance_x =  (-output_vector[0,0,1]+cam[1])*pixel_size +cam2rob_dist
		distance_x = -0.28*output_vector[0,0,1] +160 
		distance_y = -(output_vector[0,0,0] - cam[0])*pixel_size 
		distance_y =  -(output_vector[0,0,0] - cam[0]) *(0.0063*distance_x)


		#print("data: ", distance_x,distance_y,box[3],box[1])


		angle = np.arctan2(distance_y,distance_x)
		
		distance = np.sqrt(np.power(distance_x,2) + np.power(distance_y,2))
		#print("Distance ", distance, " angle: ", angle)
		if distance < 55:
			Lego_list.append([angle,distance])
			#print("angle" , angle*180/pi)
		else:
			Lego_list.append([0,0])

	Lego_list = np.array(Lego_list)
	return Lego_list

def mapa2grid(mapa):


	obstacle_size = 28

	obstacle_cells = obstacle_size


	# Obstacles position

	obs_pos = mapa
	n_obs = obs_pos.shape[0]

	
	for i in range(0,n_obs):

		

		mapa[obs_pos[i,0]:obs_pos[i,0]+obstacle_cells,obs_pos[i,1]:obs_pos[i,1]+obstacle_cells] = np.ones([obstacle_cells,obstacle_cells])


	target_on = 0
	while(target_on == 1):

		tar_pos = np.random.randint(1000,size=[2])

		if mapa[tar_pos[0],tar_pos[1]] == 0 :

			mapa[tar_pos[0],tar_pos[1]] = 2

			target_on = 1

			

	return mapa





def delete_in_mapa(mapa,robot_trajectory):

	robot_trajectory_ar = np.array(robot_trajectory)

	min_dist = 29
	max_dist = 60
	min_angle = np.arctan2(29,20)
	max_angle = np.arctan2(29,-20)
	mapa_ar = np.array(mapa)

	eliminate_index = []
	for i in range(0, robot_trajectory_ar.shape[0]):

		for j in range(0, mapa_ar.shape[0]):

			x = mapa_ar[j,0] - robot_trajectory_ar[i,0]
			y = mapa_ar[j,1] - robot_trajectory_ar[i,1]

			distance = np.sqrt(np.power(x,2)+np.power(y,2))
			angle = np.arctan2(y,x) - robot_trajectory_ar[i,2]*pi/180
			
			if (distance > min_dist and distance< max_dist and angle > min_angle and angle< max_angle) and  mapa_ar[j,4] == 0 :
				pass
			elif j not in eliminate_index:

				eliminate_index.append(j)

	#print("j: ",eliminate_index)
	eliminate_index = np.array(eliminate_index)
	mapa = np.array(mapa)
	if mapa.size:
		mapa = mapa[eliminate_index,:]
	mapa= mapa.tolist()
	#mapa = mapa[eliminate_index]

	return mapa


def add_points_in_mapa(landmarks,new_points2add,mapa, P ,pos_rob,index):

	landmarks = np.array(landmarks)


	for i in new_points2add:

			x_mapa = pos_rob[0] + landmarks[i,1]*np.cos((pos_rob[2])*pi/180+landmarks[i,0])

			y_mapa = pos_rob[1] + landmarks[i,1]*np.sin((pos_rob[2])*pi/180+landmarks[i,0])

			if landmarks[i,1] != 0:
				mapa.append(np.array([x_mapa,y_mapa,P[0,0],P[1,1],1]))
	

	# Computing target position

	if index !=1000 :
		mapa_ar = np.array(mapa)
		target_pos = 1000
		for j in range(0, mapa_ar.shape[0]):

			if  mapa[j][4] == 5:

				target_pos = j

		x_mapa = pos_rob[0] + landmarks[index,1]*np.cos((pos_rob[2])*pi/180+landmarks[index,0])

		y_mapa = pos_rob[1] + landmarks[index,1]*np.sin((pos_rob[2])*pi/180+landmarks[index,0])

		if target_pos == 1000:
			mapa.append(np.array([x_mapa,y_mapa,P[0,0],P[1,1],5]))
		else:
			mapa[target_pos][0] = x_mapa
			mapa[target_pos][1] = y_mapa
		



	return mapa




def create_fake_lego_measurements(real_rob_pos, mapa):

	min_dist = 29
	max_dist = 60
	min_angle = 0#np.arctan2(29,15)
	max_angle = np.arctan2(29,-15)

	mapa_ar = np.array(mapa)
	fake_landmarks = [];
	for j in range(0, mapa_ar.shape[0]):

			x = mapa_ar[j,0] - real_rob_pos[0]
			y = mapa_ar[j,1] - real_rob_pos[1]

			distance = np.sqrt(np.power(x,2)+np.power(y,2))
			angle = np.arctan2(y,x) - real_rob_pos[2]*pi/180

			if distance > min_dist and distance< max_dist and angle > min_angle and angle< max_angle:
				fake_landmarks.append(np.array([angle,distance]))


	return fake_landmarks






def update_mapa(mapa,landmark_rob,pos_rob,P,delete_countdown, robot_trajectory,index):


	mapa,delete_countdown, new_points2add, update_points = points2mapa(landmark_rob, pos_rob, mapa, P, delete_countdown)

	robot_trajectory.append(pos_rob)

	mapa = add_points_in_mapa(landmark_rob,new_points2add,mapa,P,pos_rob,index)




	if delete_countdown == 5:

		mapa = delete_in_mapa(mapa, robot_trajectory)

		robot_trajectory = [];


	return mapa, delete_countdown,robot_trajectory












	



		