#!/usr/bin/env python3
import cv2
import numpy as np
import sys



cap = cv2.VideoCapture(1)
flag=0
if len(sys.argv)>1:
	number_of_photos=int(sys.argv[1])
else:
	number_of_photos=10

image_name="Chessboard"
for i in range(number_of_photos):
	while True:
		ret,frame=cap.read()
		cv2.imshow("camera",frame)
		if cv2.waitKey(10) & 0xFF==32:
			break
		if cv2.waitKey(10) & 0xFF==27:
			flag=1
			break
	if flag==1:
		break
	print("photo taken",i)
	out_name="{}_{}.jpg".format(image_name,i)
	cv2.imwrite(out_name,frame)
