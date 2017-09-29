import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2

#robot to camera 29 cm
def detect_purple(frame,BB_list, LowH=113, HighH=142, LowS=72 ,HighS=170,LowV=45,HighV=215,):
	i=0
	hsvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lowerboundcolor=np.array([LowH,LowS,LowV])
	upperboundcolor=np.array([HighH,HighS,HighV])
	inrangeframe=cv2.inRange(hsvframe,lowerboundcolor,upperboundcolor)
	inrangeframe=cv2.morphologyEx(inrangeframe,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT, (7,7)))
	center_list=[]
	purple_box=[]
	for box in BB_list:
		box_image=inrangeframe[box[1]:box[3],box[0]:box[2]]
		#center,closest=get_purple_lego(box_image,Arearef=10)
		#center_list.append(center)
		if box_image.shape[0]>10:
			n_of_purple=np.sum(box_image)/255.
			n_of_pixel=np.size(box_image)
			ratio=n_of_purple/n_of_pixel
			#print(ratio,len(BB_list))
			if ratio>0.2:
				#cv2.imshow("purple_inrange", box_image)
				#cv2.waitKey(100)
				purple_box.append(box)
	return purple_box

def eliminate_grip(frame):
	frame[411:,127:245]=[0,0,0]
	frame[420:,325:426]=[0,0,0]
	return frame

cap = cv2.VideoCapture(1)
while True:
	ret,frame=cap.read()
	frame=eliminate_grip(frame)
	BB_legos=get_lego_boxes(frame,Arearef=50)
	for box in BB_legos:
		cv2.rectangle(frame,(box[0],box[1]),(box[2],box[3]),[0,255,0], thickness=1)
		purple_box=detect_purple(frame, BB_legos)
		for pbox in purple_box:
			cv2.rectangle(frame, (pbox[0],pbox[1]),(pbox[2],pbox[3]), [0,0,255], thickness=3)
	cv2.imshow("frame",frame)
	
	if cv2.waitKey(1) & 0xFF == 27:
			break