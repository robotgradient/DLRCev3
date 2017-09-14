#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep
from time import time


LowH=0
HighH=133
LowS=0
HighS=251
LowV=157
HighV=224


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


def get_centers(img):
	#Apply contours to get the properties of the images
	contourimage, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#matrix to draw the contours
	img2=np.zeros((480,640,3))
	img2 = cv2.drawContours(img2, contours, -1, (0,255,0), 3)
		  # Display the resulting frame
	center_list=[]
	for i in range(len(contours)):
		(x,y),radius = cv2.minEnclosingCircle(contours[i])
		center_list.append([int(x),int(y)])
	return center_list


def get_objective(center_list):
	center_array=np.array(center_list)
	center_array[:,0]=abs(center_array[:,0]-320)	
	index=np.argmin(center_array,axis=0)
	return center_list[index[0]]

def all_operations(frame):
	LowH=cv2.getTrackbarPos("LowH","frame")
	HighH=cv2.getTrackbarPos("HighH","frame")
	LowS=cv2.getTrackbarPos("LowS","frame")
	HighS=cv2.getTrackbarPos("HighS","frame")
	LowV=cv2.getTrackbarPos("LowV","frame")
	HighV=cv2.getTrackbarPos("HighV","frame")
		
	hsvframe=filter_2HSV(frame)
	lowerboundcolor=np.array([LowH,LowS,LowV])
	upperboundcolor=np.array([HighH,HighS,HighV])
	# Binarization
	inrangeframe=cv2.inRange(hsvframe,lowerboundcolor,upperboundcolor)
	cv2.imshow("Before morphology",inrangeframe)

	#Morphologic operations
	# Infill
	inrangeframe=imfill(inrangeframe)
	cv2.imshow("filledOR",inrangeframe)		

	#Opening and closing 
	morphoimg=open_and_close(inrangeframe,(11,3))
	morphoimg=open_and_close(morphoimg,(3,11))
	cv2.imshow("after morphology",morphoimg)	
	#Getting the centers
	center_list=get_centers(morphoimg)

	#plotting
	for i in range(len(center_list)):
		cv2.circle(frame,(center_list[i][0],center_list[i][1]),2,(255,255,255),thickness=2)
	print (center_list)
		
		#Draw the lines that determine the action space
	cv2.line(frame,(280,0),(260,479),(255,0,0),2)
	cv2.line(frame,(360,0),(380,479),(255,0,0),2)
	cv2.line(frame,(220,0),(180,479),(0,0,255),2)
	cv2.line(frame,(420,0),(460,479),(0,0,255),2)
		
	if len(center_list)>0:
		#check which center is more in the center
		objective_center=get_objective(center_list)
		
		cv2.circle(frame,(objective_center[0],objective_center[1]),3,(255,0,0),thickness=2)
	else:
		objective_center=[]
	return objective_center

def init_trackbars():
	
	cv2.namedWindow('frame')
	cv2.createTrackbar("LowH", 'frame',LowH, 255,nothing); 
	cv2.createTrackbar("HighH", 'frame',HighH, 255,nothing);
	cv2.createTrackbar("LowS", 'frame',LowS, 255,nothing); 
	cv2.createTrackbar("HighS", 'frame',HighS, 255,nothing);
	cv2.createTrackbar("LowV", 'frame',LowV, 255,nothing); 
	cv2.createTrackbar("HighV", 'frame',HighV, 255,nothing);
	return 0

def nothing(x):
	pass 

if __name__ == "__main__":
	cap = cv2.VideoCapture(1)
	cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
	init_trackbars()
	sleep(1)
	while(True):
  # Capture frame-by-frame
		t0=time()
		retard=0
		while(retard<0.1):
			ret, frame = cap.read()
			t1=time()
			retard=t1-t0
			
		# Get the trackbar poses
		
		#objective_center=all_operations(frame)
		#print(objective_center)
		#sleep(2)
		cv2.imshow("frame",frame)
		if cv2.waitKey(1) & 0xFF == 27:
			break
		sleep(1)
		
		
		
	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()
