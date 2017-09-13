#!/usr/bin/env python3
import cv2
import numpy as np

LowH=0
HighH=183
LowS=59
HighS=255
LowV=0
HighV=236


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


def nothing(x):
	pass 

if __name__ == "__main__":
	cap = cv2.VideoCapture(1)
	cv2.namedWindow('frame')
	cv2.createTrackbar("LowH", 'frame',LowH, 255,nothing); 
	cv2.createTrackbar("HighH", 'frame',HighH, 255,nothing);
	cv2.createTrackbar("LowS", 'frame',LowS, 255,nothing); 
	cv2.createTrackbar("HighS", 'frame',HighS, 255,nothing);
	cv2.createTrackbar("LowV", 'frame',LowV, 255,nothing); 
	cv2.createTrackbar("HighV", 'frame',HighV, 255,nothing);

	while(True):
  # Capture frame-by-frame
		ret, frame = cap.read()
		# Get the trackbar poses
		LowH=cv2.getTrackbarPos("LowH","frame")
		HighH=cv2.getTrackbarPos("HighH","frame")
		LowS=cv2.getTrackbarPos("LowS","frame")
		HighS=cv2.getTrackbarPos("HighS","frame")
		LowV=cv2.getTrackbarPos("LowV","frame")
		HighV=cv2.getTrackbarPos("HighV","frame")
		
		 
		kernel = np.ones((5,5),np.float32)/25
		frame = cv2.filter2D(frame,-1,kernel)
    # Our operations on the frame come here
		hsvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		print(hsvframe[240,320,:])

		lowerboundcolor=np.array([LowH,LowS,LowV])
		upperboundcolor=np.array([HighH,HighS,HighV])

		inrangeframe=cv2.inRange(hsvframe,lowerboundcolor,upperboundcolor)
		cv2.imshow("Before morphology",inrangeframe)
		#Morphologic operations
		# Infill
		inrangeframe=imfill(inrangeframe)
		cv2.imshow("filledOR",inrangeframe)		

		#Opening and closing 
		morphoimg=open_and_close(inrangeframe,(7,7))

		center_list=get_centers(morphoimg)

		#find contours to get some properties
		for i in range(len(center_list)):
			cv2.circle(frame,(center_list[i][0],center_list[i][1]),2,(255,255,255),thickness=2)
		print (center_list)

		cv2.imshow("frame",frame)
		
		if cv2.waitKey(100) & 0xFF == 27:
			break

	# When everything done, release the capture
	cap.release()
	cv2.destroyAllWindows()
