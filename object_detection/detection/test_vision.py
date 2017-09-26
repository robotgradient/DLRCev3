import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2

cap = cv2.VideoCapture(1)
while True:
	ret,frame=cap.read()
	BB_legos=get_lego_boxes(frame)
	print(BB_legos)
	for box in BB_legos:
		cv2.rectangle(frame,(box[0],box[1]),(box[2],box[3]),(0,255,0))
	cv2.imshow("frame",frame)
	if cv2.waitKey(1) & 0xFF == 27:
			break