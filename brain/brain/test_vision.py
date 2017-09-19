import numpy as np
#from ev3control import Robot
from object_detection.opencv import get_lego_piece
from object_detection.opencv import detection_lego_outside_white
from object_detection.opencv import get_brown_box
from object_detection.opencv import get_purple_lego
import time
import cv2

cap = cv2.VideoCapture(1)
while True:
	ret,frame=cap.read()
	brown_box_center,brown_box_closest=get_purple_lego(frame)
	print (brown_box_closest,brown_box_center)
	cv2.imshow("frame",frame)
	if cv2.waitKey(1) & 0xFF == 27:
			break