import numpy as np
import matplotlib.pyplot as plt



def obstacle_check(origin,goal,obs_set):
	x=np.int_(np.linspace(origin[0],goal[0],num=30))
	y=np.int_(np.linspace(origin[1],goal[1],num=30))
	path=np.stack([x,y],axis=1)
	path_set=set(tuple(i) for i in path)

	conflic=path_set.intersection(set(obs_set))
	if len(conflic)==0:
		return True,conflic
	else:
		return False,conflic



def obstacle_set(map):
	obs_list=[]
	obs=set()
	for i in range(map.shape[0]):
		for j in range(map.shape[1]):
			if map[i,j]==1:
				obs_list.append([i,j])
				obs.add((i,j))
	return obs,obs_list

def check(point,Map):
	if Map[point[0],point[1]]==1:
		return False
	else:
		return True

def apply_action(point,action):
	nextpoint=point
	if action==0:
		nextpoint[0]=point[0]+1
	elif action==1:
		nextpoint[0]=point[0]+1
		nextpoint[1]=point[1]+1
	elif action==2:
		nextpoint[1]=point[1]+1
	elif action==3:
		nextpoint[0]=point[0]-1
		nextpoint[1]=point[1]+1
	elif action==4:
		nextpoint[0]=point[0]-1
	elif action==5:
		nextpoint[0]=point[0]-1
		nextpoint[1]=point[1]-1
	elif action==6:
		nextpoint[1]=point[1]-1
	elif action==7:
		nextpoint[0]=point[0]+1
		nextpoint[1]=point[1]-1
	return nextpoint

def next_state(point,goal,Map):
	theta_g=np.arctan2(goal[1]-point[1],goal[0]-point[0])*180/np.pi
	theta_act=np.array([0,45,90,135,180,-135,-90,-45])
	theta_res=abs(theta_act-theta_g)
	order_action=np.argsort(theta_res)
	for i in order_action:
		next_possible=apply_action(point,i)
		if check(next_possible,Map)==True:
			next_point=next_possible
			break
	return next_point

def get_disrete_path(origin,goal,Map):
	path=[]
	nextpoint=next_state(origin,goal,Map)
	path.append(nextpoint)
	while(np.array_equal(nextpoint,goal)==False):
		nowpoint=nextpoint
		nextpoint=next_state(nowpoint,goal,Map)
		#print(nextpoint,type(nextpoint))
		path.append(np.array(nextpoint))
	print("Finish")
	path_final=np.array(path)
	return path_final



#Map=create_random_map(3)
Map=np.zeros([200,200])
Map[10,10]=1
Map[50,30]=1


origin=np.array([10,2])
goal=np.array([10,20])

path=get_disrete_path(origin,goal,Map)
print (type(path),path)

fig = plt.figure()


plt.scatter(path[:,0], path[:,1])
plt.scatter(goal[:,0], goal[:,1],color='red')
plt.scatter(origin[:,0], origin[:,1],color='green')
ax = fig.gca()
ax.set_xticks(np.arange(0, 50, 1))
ax.set_yticks(np.arange(0, 50, 1))
plt.grid()
plt.show()