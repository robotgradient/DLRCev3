import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2

data = np.load('Homography.npz')
H=data["arr_0"]
print(H)

cap = cv2.VideoCapture(1)
while True:
	ret,frame=cap.read()

	cv2.circle(frame,(200,400),3,(0,255,0),3)
	cv2.imshow("frame",frame)
	dst = cv2.warpPerspective(frame,H,(640,480),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)
	BB_legos=get_lego_boxes(dst)
	#print(BB_legos)
	for box in BB_legos:
		cv2.rectangle(dst,(box[0],box[1]),(box[2],box[3]),(0,255,0))
	#print(dst.shape)
	cv2.rectangle(dst,(0,0),(200,300),(255,0,0))
	input_vector=np.array([[[200,400]]],dtype=np.float32)
	h=H[0:3,0:3]
	#input_vector=np.array([input_vector])
	output_vector=cv2.perspectiveTransform(input_vector,np.linalg.inv(H))
	print("input vector : ",input_vector)
	print("outpt vect: ",output_vector)


	# CENTER OF THE CAMERA
	input_vector_cam_pos=np.array([[[320,480]]],dtype=np.float32)
	cv2.circle(frame,(320,480),3,(0,255,0),3)
	cv2.imshow("frame",frame)

	cam=cv2.perspectiveTransform(input_vector_cam_pos,np.linalg.inv(H))
	cv2.circle(dst,(cam[0,0,0],cam[0,0,1]),3,(255,0,0),3)
	print("cam", cam)
	cv2.imshow("dst",dst)


	# Compute the distance and angle from the center

	distance = np.sqrt(np.power(output_vector[0,0,0]-cam[0,0,0],2) + np.power(output_vector[0,0,1]-cam[0,0,1],2))
	angle = np.arctan2(output_vector[0,0,1]-cam[0,0,1], output_vector[0,0,0]-cam[0,0,0])

	# En principio si recibo una lista con (angle, distance) me es suficiente en plan Lego_list = [[angle1,d1],[angle2,d2]]

	



	cv2.circle(dst,(output_vector[0,0,0],output_vector[0,0,1]),3,(255,0,0),3)
	print(output_vector[0,0,0],output_vector[0,0,1])

	cv2.imshow("dst",dst)


	if cv2.waitKey(1) & 0xFF == 27:
			break