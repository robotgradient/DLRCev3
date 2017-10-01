import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
from detection.opencv import detect_purple
from detection.opencv import detect_purple2
import time
import cv2

#robot to camera 29

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
	purple_box=detect_purple2(frame, BB_legos)
	if len(purple_box)>0:
		for pbox in purple_box:
			cv2.rectangle(frame, (pbox[0],pbox[1]),(pbox[2],pbox[3]), [0,0,255], thickness=3)
	cv2.imshow("frame",frame)
	
	if cv2.waitKey(1) & 0xFF == 27:
			break