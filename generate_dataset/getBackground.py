import cv2
import numpy as np 
from time import sleep
import sys

if len(sys.argv)>=2:
	image_name=sys.argv[1]
else:
	image_name="Background"
image_complete_name="{}{}".format(image_name,".jpeg")
cap=cv2.VideoCapture(1)

while 1:

	ret,frame=cap.read()
	cv2.imshow("frame_with_boxes",frame)
	if cv2.waitKey(1) & 0xFF == 27:
		break
	cv2.imwrite(image_complete_name,frame)
