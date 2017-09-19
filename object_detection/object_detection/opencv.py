#!/usr/bin/env python3
import cv2
import numpy as np

LowH=0
HighH=186
LowS=80
HighS=255
LowV=100
HighV=255
LowH2=0
HighH2=0
LowS2=0
HighH2=19
LowV2=235
HighV2=255


def imfill(img):
	ret,im_flood = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
	th,inrangeframe = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
	im_flood[0:480,0:1]=np.zeros((480,1))
	h, w = im_flood.shape[:2]
	mask = np.zeros((h+2, w+2), np.uint8)
	#cv2.imshow("0 in corner",im_flood)
	cv2.floodFill(im_flood, mask, (0,0), 255);
	#cv2.imshow("filled",im_flood)
	cv2.bitwise_not(im_flood,im_flood)
	imfilled=cv2.bitwise_or(im_flood,inrangeframe)
	#cv2.imshow("filledOR",inrangeframe)
	return imfilled

def filter_2HSV(img):
	kernel = np.ones((5,5),np.float32)/25
	img = cv2.filter2D(img,-1,kernel)
  # Change colorspace
	hsvframe = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	return hsvframe

def open_and_close(img,size):
	#opening
	morphoimage=cv2.morphologyEx(img,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, size))
	#Closing
	morphoimage=cv2.morphologyEx(morphoimage,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT, size))
	return morphoimage


def get_centers(img,Arearef=130):
	#Apply contours to get the properties of the images
	contourimage, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#matrix to draw the contours
	img2=np.zeros((480,640,3))
	img2 = cv2.drawContours(img2, contours, -1, (0,255,0), 3)
		  # Display the resulting frame
	center_list=[]
	closest_list=[]
	for i in range(len(contours)):
		if  cv2.contourArea(contours[i])>Arearef:
			(x,y),radius = cv2.minEnclosingCircle(contours[i])
			center_list.append([int(x),int(y)])
			contourfigure=contours[i]
			closecontour=np.argmax(contourfigure[:,:,1],axis=0)
			closest_list.append(contourfigure[closecontour,0,:])

	return center_list,closest_list


def get_objective(center_list,closest_list=[]):
	center_array=np.array(center_list)
	center_array[:,0]=abs(center_array[:,0]-320)	
	index=np.argmin(center_array,axis=0)
	objective_center=center_list[index[0]]
	if len(closest_list)>0:
		objective_closest=closest_list[index[0]]
	else:
		objective_closest=[]
	return objective_center,objective_closest

def detection(frame,LowH,HighH,LowS,HighS,LowV,HighV,sizemorph,Arearef=130):


	hsvframe=filter_2HSV(frame)
	lowerboundcolor=np.array([LowH,LowS,LowV])
	upperboundcolor=np.array([HighH,HighS,HighV])
	# Binarization
	inrangeframe=cv2.inRange(hsvframe,lowerboundcolor,upperboundcolor)
	#cv2.imshow("Before morphology",inrangeframe)

	#Morphologic operations
	# Infill
	inrangeframe=imfill(inrangeframe)
	#cv2.imshow("filledOR",inrangeframe)

	#Opening and closing
	morphoimg=open_and_close(inrangeframe,sizemorph)
	sizemorph2=tuple(reversed(sizemorph))
	morphoimg=open_and_close(morphoimg,sizemorph2)
	#Getting the centers
	center_list,closest_list=get_centers(morphoimg,Arearef)
	closest_list=np.array(closest_list)
	#print(closest_list.shape)
	if len(closest_list.shape)>2:
		closest_list=np.squeeze(closest_list,axis=1)
	#print("After squeezing.",closest_list.shape)
	#cv2.imshow("morpho image",morphoimg)
	#plotting
	#print(closest_list[0,0])
	for i in range(len(center_list)):
		cv2.circle(frame,(center_list[i][0],center_list[i][1]),2,(255,255,255),thickness=2)
		cv2.circle(frame,(closest_list[i][0],closest_list[i][1]),2,(0,0,255),thickness=2)
	#print (center_list)

		#Draw the lines that determine the action space
	#cv2.line(frame,(280,0),(260,479),(255,0,0),2)
	#cv2.line(frame,(360,0),(380,479),(255,0,0),2)
	#cv2.line(frame,(220,0),(180,479),(0,0,255),2)
	#cv2.line(frame,(420,0),(460,479),(0,0,255),2)

	if len(center_list)>0:
		#check which center is more in the center
		objective_center,objective_closest=get_objective(center_list,closest_list)
		if len(set(sizemorph))==1:
			cv2.circle(frame,(objective_center[0],objective_center[1]),3,(255,0,0),thickness=2)
			cv2.circle(frame,(objective_closest[0],objective_closest[1]),4,(0,0,0),thickness=2)
		else:
			cv2.circle(frame,(objective_center[0],objective_center[1]),3,(0,0,255),thickness=2)
	else:
		objective_center=[]
		objective_closest=[]

	return objective_center,objective_closest



def nothing(x):
	pass

#For detecting the lego pieces the parameters are
'''LowH=0
HighH=183
LowS=80
HighS=255
LowV=100
HighV=251
morph=(7,7)'''

#For detecting the celo box
'''
LowH2=0
HighH2=123
LowS2=0
HighH2=255
LowV2=147
HighV2=224
morph=11,3
'''

#For detecting the white box
'''
LowH2=0
HighH2=0
LowS2=0
HighH2=19
LowV2=235
HighV2=255
morph=11,3
'''
def detection_lego_outside_white(frame,LowH=0,HighH=183,LowS=59,HighS=255,LowV=0,HighV=236\
	,LowH2=0,HighH2=123,LowS2=0,HighS2=255,LowV2=147\
	,HighV2=224,sizemorph1=(7,7),sizemorph2=(3,11)):
	
	hsvframe=filter_2HSV(frame)
	lowerboundcolor1=np.array([LowH,LowS,LowV])
	upperboundcolor1=np.array([HighH,HighS,HighV])
	lowerboundcolor2=np.array([LowH2,LowS2,LowV2])
	upperboundcolor2=np.array([HighH2,HighS2,HighV2])
	# Binarization
	legobin=cv2.inRange(hsvframe,lowerboundcolor1,upperboundcolor1)
	boxbin=cv2.inRange(hsvframe,lowerboundcolor2,upperboundcolor2)

	legofill=imfill(legobin)		
	boxfill=imfill(boxbin)
	#Opening and closing 
	legomorph=open_and_close(legofill,sizemorph1)
	sizemorpht1=tuple(reversed(sizemorph1))
	legomorph=open_and_close(legomorph,sizemorpht1)

	boxmorph=open_and_close(boxfill,sizemorph2)
	sizemorph2t=tuple(reversed(sizemorph2))
	boxmorph=open_and_close(boxmorph,sizemorph2t)

	boxinv=cv2.bitwise_not(boxmorph)
	lego_no_box=cv2.bitwise_and(boxinv,legomorph)
	cv2.imshow("box invw",boxinv)
	cv2.imshow("lego mopr",legomorph)
	cv2.imshow("lego_no_box",lego_no_box)
	center_list=get_centers(lego_no_box)

	
	#plotting
	for i in range(len(center_list)):
		cv2.circle(frame,(center_list[i][0],center_list[i][1]),2,(255,255,255),thickness=2)
	#print (center_list)
		
		#Draw the lines that determine the action space
	#cv2.line(frame,(280,0),(260,479),(255,0,0),2)
	#cv2.line(frame,(360,0),(380,479),(255,0,0),2)
	#cv2.line(frame,(220,0),(180,479),(0,0,255),2)
	#cv2.line(frame,(420,0),(460,479),(0,0,255),2)
		
	if len(center_list)>0:
		#check which center is more in the center
		objective_center=get_objective(center_list)
		if len(set(sizemorph1))==1:
			cv2.circle(frame,(objective_center[0],objective_center[1]),3,(255,0,0),thickness=2)
		else:
			cv2.circle(frame,(objective_center[0],objective_center[1]),3,(0,0,255),thickness=2)
	else:
		objective_center=[]
	
	return objective_center

def get_lego_piece(frame):
	lego_piece = detection(frame, LowH=0, HighH=186, LowS=59 \
		,HighS=255,LowV=0,HighV=236,sizemorph=(7, 7))
	return lego_piece


def get_white_box(frame):
	white_box = detection(frame, LowH2, HighH2, LowS2, HighV2, LowV2, (3, 11))
	return white_box

def get_brown_box(frame):
	brown_box_center,brown_box_closest = detection(frame, LowH=10, HighH=50, LowS=60, HighS=255, \
	 LowV=90,HighV=255, sizemorph=(3, 11),Arearef=10000)
	return brown_box_center,brown_box_closest

def draw_lines(frame, atol=50):
	cv2.line(frame, (320 - atol, 0), (320 - atol, 479), (255, 0, 0), 2)
	cv2.line(frame, (320 + atol, 0), (320 + atol, 479), (255, 0, 0), 2)
	cv2.line(frame, (220, 0), (180, 479), (0, 0, 255), 2)
	cv2.line(frame, (420, 0), (460, 479), (0, 0, 255), 2)


