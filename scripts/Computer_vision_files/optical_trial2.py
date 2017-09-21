#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep
from time import time


LowH=0
HighH=50
LowS=59
HighS=255
LowV=0
HighV=236


morph=11,3

'''LowH2=0
HighH2=0
LowS2=0
HighS2=19
LowV2=235
HighV2=255'''
morph=(11,3)


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
			#print (contourfigure.shape)
			closecontour=np.argmax(contourfigure[:,:,1],axis=0)
			#print("close contour:",closecontour,contourfigure[closecontour,0,:])

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

def all_operations(frame):
	LowH2=cv2.getTrackbarPos("LowH","frame")
	HighH2=cv2.getTrackbarPos("HighH","frame")
	LowS2=cv2.getTrackbarPos("LowS","frame")
	HighS2=cv2.getTrackbarPos("HighS","frame")
	LowV2=cv2.getTrackbarPos("LowV","frame")
	HighV2=cv2.getTrackbarPos("HighV","frame")
		
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
	center_list,closest_list=get_centers(morphoimg,1)

	#plotting
	for i in range(len(center_list)):
		#cv2.circle(frame,(center_list[i][0],center_list[i][1]),2,(255,255,255),thickness=2)
		#
		print(len(closest_list))
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

def detection(frame,LowH,HighH,LowS,HighS,LowV,HighV,sizemorph,Arearef=10):


	hsvframe=filter_2HSV(frame)
	lowerboundcolor=np.array([LowH,LowS,LowV])
	upperboundcolor=np.array([HighH,HighS,HighV])
	# Binarization
	inrangeframe=cv2.inRange(hsvframe,lowerboundcolor,upperboundcolor)
	cv2.imshow("Before morphology",inrangeframe)

	#Morphologic operations
	# Infill
	inrangeframe=imfill(inrangeframe)
	#cv2.imshow("filledOR",inrangeframe)

	#Opening and closing
	morphoimg=open_and_close(inrangeframe,sizemorph)
	sizemorph2=tuple(reversed(sizemorph))
	morphoimg=open_and_close(morphoimg,sizemorph2)
	#Getting the centers
	center_list,closest_list=get_centers(morphoimg,1)
	closest_list=np.array(closest_list)
	#print(closest_list.shape)
	if len(closest_list.shape)>2:
		closest_list=np.squeeze(closest_list,axis=1)
	#print("After squeezing.",closest_list.shape)
	cv2.imshow("morpho image",morphoimg)
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

	#return objective_center,objective_closest
	return center_list
	

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
	lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

	cap = cv2.VideoCapture(0)
	#cap.set(cv2.CAP_PROP_BUFFERSIZE,100)
	init_trackbars()
	feature_params = dict( maxCorners = 50,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
	lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


	#sleep(1)
	LowH2=0
	HighH2=50
	LowS2=10
	HighS2=1
	LowV2=0
	HighV2=237
	ret, oldframe = cap.read()	
	
	black_frame_old=np.zeros((480,640,3),dtype=np.uint8)
	old_list= detection(oldframe, LowH2, HighH2, LowS2,HighS2, LowV2, HighV2, (3, 9),1)
	old_list=np.array(old_list)
	for i in range(len(old_list)):
		cv2.rectangle(black_frame_old,(old_list[i,:]),(old_list[i,0]+30,old_list[i,1]+50),color=[0,255,0])
	old_gray = cv2.cvtColor(black_frame_old, cv2.COLOR_BGR2GRAY)
	p0=cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
	
	cv2.imshow("old_gray",old_gray)
	cv2.waitKey()
	while(True):
  # Capture frame-by-frame
		t0=time()
		retard=0
		LowH2=cv2.getTrackbarPos("LowH","frame")
		HighH2=cv2.getTrackbarPos("HighH","frame")
		LowS2=cv2.getTrackbarPos("LowS","frame")
		HighS2=cv2.getTrackbarPos("HighS","frame")
		LowV2=cv2.getTrackbarPos("LowV","frame")
		HighV2=cv2.getTrackbarPos("HighV","frame")
	
		ret, newframe = cap.read()
		black_frame_new=np.zeros((480,640,3),dtype=np.uint8)
		
		new_list= detection(newframe, LowH2, HighH2, LowS2,HighS2, LowV2, HighV2, (3, 9),1)
		new_list=np.array(new_list)
		for i in range(len(new_list)):
			cv2.rectangle(black_frame_new,(new_list[i,0],new_list[i,1]),(new_list[i,0]+30,new_list[i,1]+50),color=(0,255,0))

		new_gray = cv2.cvtColor(black_frame_new, cv2.COLOR_BGR2GRAY)
		cv2.imshow("black",new_gray)
		cv2.waitKey()
		#p0=cv2.goodFeaturesToTrack(new_gray, mask = None, **feature_params)
		p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **lk_params)
	
		good_new = p1[st==1]
		good_old = p0[st==1]
		# Get the trackbar poses
		p1, status, err = cv2.calcOpticalFlowPyrLK(oldframe, newframe, old_list, None, **lk_params)
		

		old_grey_image=new_grey_image
		p0 = good_new.reshape(-1,1,2)
		
		
		if cv2.waitKey(1) & 0xFF == 27:
			break
		#sleep(1)
		
		
		
	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()
