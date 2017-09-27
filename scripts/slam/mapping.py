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






		