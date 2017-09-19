import cv2
import numpy as np 
from time import sleep
import sys
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

def nothing(x):
	pass 

cv2.namedWindow('frame')
cv2.createTrackbar("LowH", 'frame',LowH, 255,nothing); 
cv2.createTrackbar("HighH", 'frame',HighH, 255,nothing);
cv2.createTrackbar("LowS", 'frame',LowS, 255,nothing); 
cv2.createTrackbar("HighS", 'frame',HighS, 255,nothing);
cv2.createTrackbar("LowV", 'frame',LowV, 255,nothing); 
cv2.createTrackbar("HighV", 'frame',HighV, 255,nothing);

if len(sys.argv)>=2:
	image_name=sys.argv[1]
else:
	image_name="lego_pieces"
image_complete_name="{}{}".format(image_name,".jpeg")
cap=cv2.VideoCapture(1)
 
while 1:

	ret,frame=cap.read()
	frame_no_boxes=frame.copy()
	LowH=cv2.getTrackbarPos("LowH","frame")
	HighH=cv2.getTrackbarPos("HighH","frame")
	LowS=cv2.getTrackbarPos("LowS","frame")
	HighS=cv2.getTrackbarPos("HighS","frame")
	LowV=cv2.getTrackbarPos("LowV","frame")
	HighV=cv2.getTrackbarPos("HighV","frame")
	lowerbounds=np.array([LowH,LowS,LowV])
	highbounds=np.array([HighH,HighS,HighV])
	#hsvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	hsvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	colours=cv2.inRange(hsvframe,lowerbounds,highbounds)

	#Various inrange with diferent backgrounds and and
	colours=cv2.morphologyEx(colours,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, (7,7)))
	colours=cv2.morphologyEx(colours,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT, (7,7)))
	
	
	#th,colours= cv2.threshold(hsvframe,HighH,255,cv2.THRESH_BINARY)
	colours=imfill(colours)
	contourimage, contours, hierarchy = cv2.findContours(colours,cv2.RETR_TREE,\
		cv2.CHAIN_APPROX_SIMPLE)
	BBcoords=[]
	for i in range(len(contours)):
		if cv2.contourArea(contours[i])>100:
			x,y,w,h = cv2.boundingRect(contours[i])
			cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
			BBcoords.append([x,y,x+w,y+h])

	cv2.imshow("colours",colours)
	cv2.imshow("frame",frame_no_boxes)
	cv2.imshow("frame_with_boxes",frame)
	if cv2.waitKey(1) & 0xFF == 27:
			break
	cv2.imwrite(image_complete_name,frame_no_boxes)
	mask_name="Mask_{}".format(image_complete_name)
	cv2.imwrite(mask_name,colours)

print (BBcoords)
with open("heloloasdfaafsda.txt", 'w') as outf:
	outf.write(repr(BBcoords))
print(colours.shape)
sleep(2)
