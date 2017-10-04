import numpy as np
#from ev3control import Robot
from detection.opencv import get_sponge_boxes,eliminate_grip
import time
import cv2


def detect_sponges(frame):
	BB_boxes=get_sponge_boxes(frame)
	left_box_th=1000
	right_box_th=0
	left_box=[0,0,0,0]
	right_box=[0,0,0,0]
	if len(BB_boxes)>0:

		for box in BB_boxes:
			if box[1]>300:
				if box[0]<left_box_th:
					left_box_th=box[0]
					left_box=box
				if box[2]>right_box_th:
					right_box=box
					right_box_th=box[2]
		return frame,left_box,right_box
	else:
	 return frame,[],[]

if __name__ == "__main__":
	cap=cv2.VideoCapture(1)

	while True:
		ret,frame=cap.read()
		cv2.imshow("frame", frame)
		frame2,left_box,right_box=detect_sponges(frame)
		if len(left_box)>0 and len(right_box)>0:
			cv2.line(frame2, (left_box[2],0),(left_box[2],480), (0,255,0))
			cv2.line(frame2, (right_box[0],0),(right_box[0],480), (0,0,255))
			print("left",left_box[2],"right",right_box[0])
		cv2.imshow("without sponges", frame2)
		if cv2.waitKey(1) & 0xFF == 27:
			break