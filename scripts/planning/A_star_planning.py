import numpy as np
import matplotlib.pyplot as plt

def create_map(obslist):
	#offset ensures probability of negative values in the map

	robot_width=10
	Map=np.ones([200,200])*np.infty
	offsetx=int(round(Map.shape[0]/2))-1
	offsety=int(round(Map.shape[1]/2))-1
	for obs in obslist:
		centerx=int(obs[0])+offsetx
		centery=int(obs[1])+offsety
		Map[centerx,centery]=-100
		robotxant=robot_width
		robotyant=robot_width
		robotxpos=robot_width
		robotypos=robot_width
		if centerx<robot_width:
			robotxant=centerx
		if robot_width>Map.shape[0]-(centerx+1):
			robotxpos=Map.shape[0]-(centerx+1)
		if centery<robot_width:
			robotyant=centery
		if robot_width>Map.shape[1]-(centery+1):
			robotypos=Map.shape[1]-(centery+1)
		Map[centerx-robotxant:centerx+robotxpos+1,centery-robotyant:centery+robotypos+1]=-100
	return Map

def obstacle_set(Map):
	obs_list=[]
	obs=set()
	for i in range(Map.shape[0]):
		for j in range(Map.shape[1]):
			if Map[i,j]==-100:
				obs_list.append([i,j])
				obs.add((i,j))
	return obs,obs_list

def heuristics(point,goal):
	h=abs(point[0]-goal[0])+abs(point[1]-goal[1])
	return h

def list_of_neighbours(point,Map):
	idx=[-1,0,1]
	neighbours=[]
	for i in idx:
		for j in idx:
			if i!=0 or j!=0:
				# For avoiding matrix limits
				if not((point[0]==0 and i==-1) or (point[1]==0 and j==-1) or (point[0]==Map.shape[0]-1 and i==1) or (point[1]==Map.shape[1]-1 and j==1)):
					if Map[point[0]+i,point[1]+j]!=-100:
						neighbours.append([point[0]+i,point[1]+j])
	return neighbours

def get_minimun_cost(openlist,fscore):
	minfscore=1000

	for i in openlist:
		if fscore[i[0],i[1]]<minfscore:
			minfscore=fscore[i[0],i[1]]
			mini=i
	return mini

def reconstruct_path(parent,current,start):
	total_path=[np.array(current)]

	while( not np.array_equal(current,start) ):
		
		current=parent[current[0],current[1]]
		total_path.append(current)
	return total_path


def A_star(start,goal,Map):
	gscore=np.ones(Map.shape)*np.inf
	#parent contain the previous points
	parent=np.ones([Map.shape[0],Map.shape[1],2],dtype=np.int32)
	#nodes to evaluate
	openlist=[start]
	#nodes evaluated
	closelist=[]
	gscore[start[0],start[1]]=0
	fscore=np.ones(Map.shape)*np.inf
	fscore[start[0],start[1]]=heuristics(start, goal)
	while(openlist):
		current=get_minimun_cost(openlist, fscore)
		if np.array_equal(current, goal):
			return reconstruct_path(parent, current, start)
		openlist.remove(current)
		closelist.append(current)

		all_neighbours=list_of_neighbours(current,Map)

		for neighbour in all_neighbours:
			if neighbour in closelist:
				continue
			if neighbour not in openlist:
				openlist.append(neighbour)
			possible_gscore=gscore[current[0],current[1]]+1
			if possible_gscore<gscore[neighbour[0],neighbour[1]]:
				gscore[neighbour[0],neighbour[1]]=possible_gscore
				fscore[neighbour[0],neighbour[1]]=gscore[neighbour[0],neighbour[1]]+heuristics(neighbour, goal)
				parent[neighbour[0],neighbour[1]]=current
	return False

# Main
'''
Map=np.ones([50,50])*np.inf
Map[3:30,10]=-100
Map[5:15,5]=-100
Map[7,10:25]=-100
#obs_set,obs_list=obstacle_set(Map)
obs_set,obs_list=obstacle_set(Map)
obs_array=np.array(obs_list)
#obs_array=np.array(obs_list)
origin=[10,2]
goal=[10,13]

path=A_star(origin,goal,Map)
path_array=np.array(path)
print (path_array)

fig = plt.figure()
plt.plot(origin[0],origin[1],'rx')
plt.plot(path_array[1:-1,0], path_array[1:-1,1],'go')
plt.plot(goal[0],goal[1],'bx')
plt.plot(obs_array[:,0],obs_array[:,1],'k*')

ax = fig.gca()
ax.set_xticks(np.arange(0, 30, 1))
ax.set_yticks(np.arange(0, 30, 1))
plt.grid()
plt.show()
'''
'''

obslist=[[50,50]]

Map=create_map(obslist)
obs_set,obs_list=obstacle_set(Map)
obs_array=np.array(obs_list)


start_robotpos=[0,0]
offsetx=int(round(Map.shape[0]/2))-1
offsety=int(round(Map.shape[1]/2))-1
start_map=[start_robotpos[0]+offsetx,start_robotpos[1]+offsety]

goal=[100,100]
goal_map=[goal[0]+offsetx,goal[1]+offsety]


path=A_star(start_map,goal_map,Map)
path_array=np.array(path)
#print (path_array)

fig = plt.figure()
plt.plot(start_map[0],start_map[1],'rx')
plt.plot(path_array[1:-1,0], path_array[1:-1,1],'go')
plt.plot(goal_map[0],goal_map[1],'bx')
plt.plot(obs_array[:,0],obs_array[:,1],'k*')

#ax = fig.gca()
#ax.set_xticks(np.arange(0, 60, 1))
#ax.set_yticks(np.arange(0, 60, 1))
plt.grid()
plt.show()'''