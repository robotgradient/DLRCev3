import numpy as np


def points2map(landmarks,pos_rob,map,P):

	for i in range(0,len(landmarks[:,1])):

			x_map = pos_rob[0] + landmarks[i,0]*np.cos(pos_rob[2]+landmarks[i,1])

			y_map = pos_rob[1] + landmarks[i,0]*np.sin(pos_rob[2]+landmarks[i,1])


			new = 1

			map_ar = np.array(map)
			for i in range(0, len(map[:,1])):

				distance = np.power((x_map-map_ar[i,1])/map_ar[i,3],2) + np.power((y_map - map_ar[i,2])/map_ar[i,4],2)

				if distance < 1 : 
					new = 0
			if new ==1:
				map.append([x_map,y_map, P[0,0] , P[1,1] ])


			return map


def cam2rob(landmarks, H):

	pixel_size = 20
	# CENTER OF THE CAMERA
	input_vector_cam_pos=np.array([[[320,480]]],dtype=np.float32)
	
	cam=cv2.perspectiveTransform(input_vector_cam_pos,np.linalg.inv(H))

	landmark_rob = np.array([landmarks.shape(0),2])
	for i in range(0,landmarks.shape(0)):

	input_vector= landmarks[i,:]

	output_vector=cv2.perspectiveTransform(input_vector,np.linalg.inv(H))
	
	# Compute the distance and angle from the center

	landmark_rob[0] = np.sqrt(np.power(output_vector[0,0,0]-cam[0,0,0],2) + np.power(output_vector[0,0,1]-cam[0,0,1],2))*pixel_size
	landmark_rob[1] = np.arctan2(output_vector[0,0,1]-cam[0,0,1], output_vector[0,0,0]-cam[0,0,0])

	# En principio si recibo una lista con (angle, distance) me es suficiente en plan Lego_list = [[angle1,d1],[angle2,d2]]

	return landmark_rob






	



		