import numpy as np
import matplotlib.pyplot as plt


def list_of_neighbours(point,Map):
	idx=[-1,0,1]
	neighbours=[]
	for i in idx:
		for j in idx:
			if i!=0 or j!=0:
				if Map[point[0]+i,point[1]+j]!=-100:
					neighbours.append([point[0]+i,point[1]+j])
	return neighbours


def nf1(origin,goal,U):
	U[origin[0],origin[1]]=0
	L=[[origin[0],origin[1]]]
	i=0
	while(goal not in L):
		Lnew=[]
		for point in L:
			neighbours=list_of_neighbours(point,Map)
			for vecino in neighbours:
				if U[vecino[0],vecino[1]]==np.inf:
					U[vecino[0],vecino[1]]==U[point[0],point[1]]+1
					Lnew.append([vecino[0],vecino[1]])
		L=Lnew
		i+=1
		print(i)
	return U





Map=np.ones([50,50])*np.inf
Map[3:30,10]=-100
Map[10,10]=-100
#obs_set,obs_list=obstacle_set(Map)
U=Map
#obs_array=np.array(obs_list)
test=list_of_neighbours([4,3],Map)
print(test)
origin=[10,2]
goal=[10,30]
U2=nf1(origin, goal, U)
print(U2)
first_point=np.array([10,5])
origin=np.array([10,2])
goal=np.array([15,30])