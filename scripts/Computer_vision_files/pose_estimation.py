import numpy as np
import cv2
import glob
import math as m
import time
import timeout_decorator as tm

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img


def silent_timeout(t, *args):
    try:
        return tm.timeout(t)(cv2.findChessboardCorners)(*args)
    except tm.timeout_decorator.TimeoutError:
        print("Timed out")
        return (False, False)

# LOAD THE PARAMETERS
data = np.load('camera_parameters.npz')
mtx=data["cam_matrix"]
dist=data["dist_coeff"]

# Now you have the camera calibration parameters

#FROM HERE IS TO PLOT THE AXIS IN THE CHESSBOARD
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)


## TO DRAW THE AXIS
cap = cv2.VideoCapture(1)
#img = cv2.imread('Chessboard_9.jpg')

while True:
    t0=time.time()
    while (time.time()-t0<0.1):
        ret,img=cap.read()
    tinit=time.time()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = silent_timeout(0.07,gray, (9,6),None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        retval,rvecs2, tvecs2, inliers2= cv2.solvePnPRansac(objp, corners2, mtx, dist)
        tvecs2=2.5*tvecs2
        print("translation x:{},y:{},z:{}".format(tvecs2[0],tvecs2[1],tvecs2[2]))
        #print("rotation x:{},y:{},z:{}".format(rvecs2[0],rvecs2[1],rvecs2[2]))
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs2, tvecs2, mtx, dist)

        img = draw(img,corners2,imgpts)
    print("retard",time.time()-tinit)
    cv2.imshow('img',img)
    if cv2.waitKey(1) & 0xFF==32:
        break



        break

