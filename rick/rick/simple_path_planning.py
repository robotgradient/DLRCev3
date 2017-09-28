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
	return np.array(nextpoint)

'''def next_state(point,goal,Map):
	#for chose the most 
	theta_g=np.arctan2(goal[1]-point[1],goal[0]-point[0])*180/np.pi
	
	theta_act=np.array([0,45,90,135,180,-135,-90,-45])
	theta_res=abs(theta_act-theta_g)
	order_action=np.argsort(theta_res)
	for i in range(order_action.shape[0]):
		next_possible=apply_action(np.array(point),order_action[i])
		if check(next_possible,Map):
			next_point=next_possible
			action=order_action[i]
			print("action accepted in point",order_action[i],point)
			break
		else:
			print("action deleted in point",order_action[i],point)
	print("choosen action at",order_action[i],point)
	return next_point,action'''

def next_state(point,goal,Map,previous_path):
	theta_g=np.arctan2(goal[1]-point[1],goal[0]-point[0])*180/np.pi
	theta_act=np.array([0,45,90,135,180,-135,-90,-45])
	theta_res=abs(theta_act-theta_g)
	#index=np.argmin(theta_res)
	#order_action=np.roll(np.arange(theta_res.shape[0]),theta_res.shape[0]-0)
	order_action=np.argsort(theta_res)

	for i in range(order_action.shape[0]):
		next_possible=apply_action(np.array(point),order_action[i])
		print(next_possible.shape,len(previous_path),type(previous_path[0]))
		if check(next_possible,Map) and (next_possible not in previous_path):
			next_point=next_possible
			action=order_action[i]
			print("action accepted in point",order_action[i],point)
			break
		else:
			print("action deleted in point",order_action[i],point)
	print("choosen action at",order_action[i],point)
	return next_point,action

def get_disrete_path(origin,goal,Map):
	path=[np.array(origin)]
	path2=np.array(path)
	action_list=[]
	nextpoint,i=next_state(origin,goal,Map,path2)
	action_list.append(i)
	path.append(np.array(nextpoint))
	while(np.array_equal(nextpoint,goal)==False):
		nowpoint=np.array(nextpoint)
		path2=np.array(path)

		nextpoint,action=next_state(nowpoint,goal,Map,path)
		#print(nextpoint,type(nextpoint))
		path.append(np.array(nextpoint))
		action_list.append(action)
	print("Finish")
	path_final=np.array(path)
	action_final=np.array(action_list)
	return path_final,action_final



#Map=create_random_map(3)
Map=np.ones([200,200])*np.inf
#Map[3:30,10]=-100
Map[10,10]=-100
obs_set,obs_list=obstacle_set(Map)

obs_array=np.array(obs_list)
first_point=np.array([10,5])
origin=np.array([10,2])
goal=np.array([15,30])

path,action=get_disrete_path(origin,goal,Map)
print(check([25,10],Map),Map[25,10])
print(action)
fig = plt.figure()

print(origin)
plt.plot(origin[0],origin[1],'rx')
plt.plot(path[:-1,0], path[:-1,1],'go')
plt.plot(goal[0],goal[1],'bx')
plt.plot(obs_array[:,0],obs_array[:,1],'k*')

ax = fig.gca()
ax.set_xticks(np.arange(0, 30, 1))
ax.set_yticks(np.arange(0, 30, 1))
plt.grid()
plt.show()