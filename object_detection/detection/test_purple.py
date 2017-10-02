#PURE TEST

import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2

from detection.opencv import detect_purple

from math import pi


def cam2rob(BB_legos, H):

	####cam [[[ 270.03048706  448.53890991]]]

	pixel_size =  0.653947100514

	# CENTER OF THE CAMERA
	cam= np.array([258.36773682, 482.33782959])
	cam2rob_dist = 25

	Lego_list = []
	for box in BB_legos:

		y = box[3]
		x = box[0] + (box[2]-box[0])/2


		input_vector=np.array([[[x,y]]],dtype=np.float32)
		output_vector=cv2.perspectiveTransform(input_vector,np.linalg.inv(H))
		print("dentro de cam2rob", output_vector)
		print('lateral:', output_vector[0,0,0] - 258.3677)
		
		distance_x =  (-output_vector[0,0,1]+cam[1])*pixel_size +cam2rob_dist
		distance_x = -0.28*output_vector[0,0,1] +160 
		distance_y = -(output_vector[0,0,0] - cam[0])*pixel_size 
		distance_y =  -(output_vector[0,0,0] - cam[0]) *(0.35-0.00022*output_vector[0,0,0])

		print("data: ", distance_x,distance_y)


		angle = np.arctan2(distance_y,distance_x)
		
		distance = np.sqrt(np.power(distance_x,2) + np.power(distance_y,2))
		
		if distance < 1000:
			Lego_list.append([angle,distance])
			print("angle" , angle*180/pi)

	Lego_list = np.array(Lego_list)
	return Lego_list

data = np.load('Homographygood.npz')
H=data["arr_0"]
print(H)

cap = cv2.VideoCapture(1)
while True:

	ret,frame=cap.read()
	cv2.imshow("frame",frame)
	dst = cv2.warpPerspective(frame,H,(640,480),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)

	BB_legos=get_lego_boxes(frame)


	BB_target = detect_purple(frame,BB_legos)

	BB_target = np.array(BB_target)
	lego_purple = cam2rob(BB_target,H)

	cv2.line(frame, (320,0), (320,480), (0,0,255))
	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == 27:
			break


