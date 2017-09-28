import numpy as np
import cv2


def points2map(landmarks,pos_rob,map,P, delete_countdown): #MAP = (x,y, Px,Py, updt)

	new_points2add = []
	for i in range(0,len(landmarks[:,1])):

			x_map = pos_rob[0] + landmarks[i,0]*np.cos(pos_rob[2]+landmarks[i,1])

			y_map = pos_rob[1] + landmarks[i,0]*np.sin(pos_rob[2]+landmarks[i,1])


			new = 1

			map_ar = np.array(map)

			if delete_countdown ==5:
				map_ar[:,4] = np.zeros([map_ar.shape(0)])
				delete_countdown = 0

			sh_dist = 100000;
			p_already = 0
			for j in range(0, len(map[:,1])):

				distance = np.power((x_map-map_ar[j,1])/map_ar[j,3],2) + np.power((y_map - map_ar[j,2])/map_ar[j,4],2)

				if sh_dist > distance: 
					sh_dist = distance
					p_already = j
			if sh_dist < 1:
				map[j,4] = 1

			if new ==1:
				new_points2add.append(i)

	delete_countdown +=1
	return map , delete_countdown , new_points2add


def cam2rob(landmarks, H):

	pixel_size =  0.653947100514
	# CENTER OF THE CAMERA
	input_vector_cam_pos=np.array([[[320,480]]],dtype=np.float32)
	
	cam=cv2.perspectiveTransform(input_vector_cam_pos,np.linalg.inv(H))

	landmark_rob = np.array([landmarks.shape(0),2])
	for i in range(0,landmarks.shape(0)):

	input_vector= landmarks[i,:]

	output_vector=cv2.perspectiveTransform(input_vector,np.linalg.inv(H))
	
	# Compute the distance and angle from the center

	landmark_rob[1] = np.sqrt(np.power(output_vector[0,0,0]-cam[0,0,0],2) + np.power(output_vector[0,0,1]-cam[0,0,1],2))*pixel_size
	landmark_rob[0] = np.arctan2(output_vector[0,0,1]-cam[0,0,1], output_vector[0,0,0]-cam[0,0,0])

	# En principio si recibo una lista con (angle, distance) me es suficiente en plan Lego_list = [[angle1,d1],[angle2,d2]]

	return landmark_rob

def map2grid(map):


	obstacle_size = 28

	obstacle_cells = obstacle_size


	# Obstacles position

	obs_pos = map
	n_obs = obs_pos.shape(0)

	
	for i in range(0,n_obs):

		

		map[obs_pos[i,0]:obs_pos[i,0]+obstacle_cells,obs_pos[i,1]:obs_pos[i,1]+obstacle_cells] = np.ones([obstacle_cells,obstacle_cells])


	target_on = 0
	while(target_on == 1):

		tar_pos = np.random.randint(1000,size=[2])

		if map[tar_pos[0],tar_pos[1]] == 0 :

			map[tar_pos[0],tar_pos[1]] = 2

			target_on = 1

			

	return map





def delete_in_map(map,robot_trajectory):

	robot_trajectory_ar = np.array(robot_trajectory)

	min_dist = 29
	max_dist = 60
	min_angle = np.arctan2(29,15)
	max_angle = np.arctan2(29,-15)
	map_ar = np.array(map)

	eliminate_index = []
	for i in range(0, robot_trajectory_ar.shape(0)):

		for j in range(0, map_ar.shape(0)):

			x = map_ar[j,0] - robot_trajectory_ar[i,0]
			y = map_ar[j,1] - robot_trajectory_ar[i,1]

			distance = np.sqrt(np.power(x,2)+np.power(y,2))
			angle = np.arctan2(y,x) - robot_trajectory_ar(i,2)*pi/180

			if distance > min_dist and distance< max_dist and angle > min_angle and angle< max_angle and map_ar[j,4] == 0 and not(j in eliminate_index):
				eliminate_index.append([j])

	del map[eliminate_index]

	return map


def add_points_in_map(landmarks,new_points2add,map, P ):

	for i in range new_points2add:

			x_map = pos_rob[0] + landmarks[i,0]*np.cos(pos_rob[2]+landmarks[i,1])

			y_map = pos_rob[1] + landmarks[i,0]*np.sin(pos_rob[2]+landmarks[i,1])

			map.append(x_map,y_map,P[0,0],P[1,1],1)

	return map










def update_map(map,landmark_rob,pos_rob,P,delete_countdown, robot_trajectory):


	map,delete_countdown, new_points2add = points2map(landmarks_rob, pos_rob, map, P, delete_countdown)

	robot_trajectory.append(pos_rob)

	map = new_points2add(landmarks_rob,new_points2add,map,P)



	if delete_countdown == 5:

		map = delete_in_map(map, robot_trajectory)

		robot_trajectory = [];


	return map, delete_countdown,robot_trajectory












	



		