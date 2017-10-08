import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2



def load_camera_params():
    data = np.load('camera_parameters.npz')
    mtx=data["cam_matrix"]
    dist=data["dist_coeff"]
    return mtx,dist


data = np.load('Homographygood.npz')
H=data["arr_0"]
print(H)
data2=np.load('newcameramtx.npz')
newcameramtx=data2["arr_0"]
print(newcameramtx)
mtx,dist=load_camera_params()
print("mtx",mtx)
print("dist",dist)
mtx=np.array(mtx)
dist=np.array(dist)

cap = cv2.VideoCapture(1)
while True:
	ret,frame=cap.read()
	cv2.imshow("frame",frame)
	cv2.line(frame, (320,0),(320,480), (0,255,0))
	cv2.line(frame, (0,240),(640,240), (0,255,0))
	dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
	cv2.imshow("distorsion", dst)
	if cv2.waitKey(100) & 0xFF==27:
		break
