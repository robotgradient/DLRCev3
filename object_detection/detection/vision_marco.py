import numpy as np
#from ev3control import Robot
from detection.opencv import get_lego_boxes
from detection.opencv import detection_lego_outside_white
from detection.opencv import get_brown_box
from detection.opencv import get_purple_lego
import time
import cv2

#robot to camera 29 cm


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
	h,  w = frame.shape[:2]
	newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
	distorsion = cv2.undistort(frame, mtx, dist, None, newcameramtx)
	cv2.imshow("distosioned one", distorsion)
	topiew=cv2.warpPerspective(frame,H,(640,480),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)
	topviewdis = cv2.warpPerspective(distorsion,H,(640,480),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)
	
	
	
	

	

	
	cv2.imshow("topview distorsion",topviewdis)
	cv2.imshow("topview normal",topiew)
		#print("y: ", pixel_distance[0], " x : ", pixel_distance[1], "other y: ", pixel_distance[2] , "other x: ", pixel_distance[3])

	if cv2.waitKey(1) & 0xFF == 27:
			break