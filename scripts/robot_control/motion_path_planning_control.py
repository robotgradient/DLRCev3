import numpy as np
import math


def compute_euclidean_path(pos_rob,pos_obj, points = 5): #pos_rob is a 1x3 matrix with(x,y,teta) &  pos_obj is a 1x2 matrix with(x,y) 

	x = np.linspace(pos_rob[0], pos_obj[0], num=points)
	y = np.linspace(pos_rob[1], pos_obj[1], num=points)

	angle =  math.atan2(pos_obj[1]-pos_rob[1], pos_obj[0]-pos_rob[0]) 
	angle = math.degrees(angle)

	angle_vec = np.zeros(points)
	angle_vec.fill(angle)

	path = [x,y,angle_vec]

	print(path)
	return path


def robot_control(pos_rob,target, K_x=0,K_y=0,K_an=0): #pos_rob is a 1x3 matrix with(x,y,teta) &  target is a 1x3 matrix with(x,y) 

	error_x = target[0] - pos_rob[0]
	error_y = target[1] - pos_rob[1]
	error_ang = target[2] - pos_rob[2]


	vel_x = 









if __name__ == '__main__':
	rob = [1,1,0]
	obj = [0,0]
	assert compute_euclidean_path(rob,obj) == y