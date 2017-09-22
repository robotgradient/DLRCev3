import time
import cv2.aruco as aruco
import numpy as np
import cv2
from time import time
from time import sleep
import scipy.io as sio


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    corner=corner[3:5]
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
# SEtting the 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#board = cv2.aruco.CharucoBoard_create(3,4,.025,.0125,auro_dict)
#img = board.draw((200*3,200*3))

#loading camera parameters
data = np.load('camera_parameters.npz')
mtx=data["cam_matrix"]
dist=data["dist_coeff"]

Tc2r=read_Tc2r()
print("transformation matrix",Tc2r)

#Start capturing images for calibration
cap = cv2.VideoCapture(1)
arucoParams = aruco.DetectorParameters_create()
markerLength = 3.5 
#axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

# to detect the markers
while True:
    ret,img=cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # aruco.etectMarkers() requires gray image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams) # Detect aruco
    tinit=time()
    if ids != None: # if aruco marker detected
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist) # For a single marker
        print("rev and tec:",rvec.shape,tvec.shape)
        imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
        
        for i in range(len(ids)):
            imgWithAruco = aruco.drawAxis(imgWithAruco, mtx, dist, rvec[i], tvec[i], 15)  
            
            print("the marker {} has rotation x:{}, y:{},z:{}".format(ids[i],rvec[i,0,0],rvec[i,0,1],rvec[i,0,2]))
            print("the marker {} has coordinates x:{}, y:{},z:{}".format(ids[i],tvec[i,0,0],tvec[i,0,1],tvec[i,0,2]))
            rotmatrix,_=cv2.Rodrigues(rvec[i])
            print("rotation matrix",rotmatrix)
    else:   # if aruco marker is NOT detected
        imgWithAruco = img # assign imRemapped_color to imgWithAruco directly
    #print("retard",time()-tinit)
    sleep(0.05)
    cv2.imshow("aruco", imgWithAruco)   # display
    if cv2.waitKey(50) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
        break