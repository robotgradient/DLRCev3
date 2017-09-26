import numpy as np


def robot_state_update(rob_pos, vel_robot,Ts): # vel_robot = (cm/s ; degrees/s)


	incr_r = vel_robot[0]*Ts
	incr_teta = vel_robot[1]*Ts

	new_pos_rob[0] = pos_rob[0] + incr_r*np.cos((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[1] = pos_rob[1] + incr_r*np.sin((pos_rob[2]+incr_teta/2)*pi/180)
	new_pos_rob[2] = pos_rob[2] + incr_teta

	#print('new pos: ', new_pos_rob)

	if new_pos_rob[2] >360:
		new_pos_rob[2] = new_pos_rob[2] - 360
	elif new_pos_rob[2] < 0 :
		new_pos_rob[2] = 360 + new_pos_rob[2]

	

	return new_pos_rob 


def create_random_map(n_obs = 10): # Each cell is a 1x1 cm in the map

	cell_size = 1

	map = np.zeros([1000,1000])

	obstacle_size = 28

	obstacle_cells = obstacle_size/cell_size


	# Obstacles position

	obs_pos = np.random.randint(size=[2,n_obs],1000-obs_cells*2) 

	w = round(obstacle_cells/2)

	for i in range(0,n_obs):

		map[obs_pos[i,0]+obstacle_cells-w:obs_pos[i,0]+obstacle_cells-w,obs_pos[i,1]+obstacle_cells-w:obs_pos[i,1]+obstacle_cells-w] = np.ones(2*w,2*w)



	while(target_on = 1):

		tar_pos = np.random.randint(size=[2],1000)

		if map[tar_pos[0],tar_pos[1]] == 0 :

			map[tar_pos[0],tar_pos[1]] = 2



			return map














