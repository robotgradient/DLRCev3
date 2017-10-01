import numpy as np
import cv2
import glob
import math as m
#import cv2.aruco as aruco
from time import time
from time import sleep
import cv2.aruco as aruco

def load_camera_params():
    data = np.load('camera_parameters.npz')
    mtx=data["cam_matrix"]
    dist=data["dist_coeff"]
    return mtx,dist

def get_specific_marker_pose(frame,mtx,dist,marker_id,arucoParams=arucoParams,markerLength=4.8):
    T=read_Tc2r()
    rotc2r=T[0:3,0:3]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams) # Detect aruco
    if isinstance(ids, np.ndarray) and (marker_id in ids): # if aruco marker detected
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist) # For a single marker
        position=np.where(ids==marker_id)
        #frame = aruco.drawAxis(frame, mtx, dist, rvec[position], tvec[position], 15)
        p2c=np.concatenate((tvec[position].T,np.array([[1]])),axis=0)
        Tp2c=np.
        p2r=T.dot(p2c)
        rotp2c,jac=cv2.Rodrigues(rvec[position])
        rotp2r=rotc2r.dot(rotp2c)
        #euler angles are x,y,z but the rotation is z,y,x in the moving frames
        eulerrad=rotationMatrixToEulerAngles(rotp2r)
        eulerangles=180*eulerrad/np.pi
        yaw=eulerangles[2]
        x=p2r[0,0]
        y=p2r[1,0]
        
        #print (x,y)
        coords=[x,y,yaw]
    else:
        coords=[]
    return frame,coords

# TO GET CAMERA PARAMETERS

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None, 1 | 4)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
mtx=np.array(mtx)
dist=np.array(dist)
rvecs=np.array(rvecs)
tvecs=np.array(tvecs)
print(mtx,dist)


np.savez("camera_parameters",cam_matrix=mtx,dist_coeff=dist)

#######################################################################################

## GET HOMOGRAPHY MATRIX

#########################################################################################3

cap = cv2.VideoCapture(1)
#img = cv2.imread('Chessboard_9.jpg')
while True:
    ret,img=cap.read()
    cv2.imshow("actual image",img)
    if cv2.waitKey(10) & 0xFF==32:
        break

#img=cv2.imread('Chessboard_10.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (9,6),None, 1 | 4)

#corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams) 



board_w = 10;
board_h = 7;
imgPts = np.zeros([4,2],dtype= "float32")
objPts = np.zeros([4,2],dtype= "float32")

imgPts[0,:] = corners[0,:];
imgPts[1,:] = corners[board_w-2,:];
imgPts[2,:] = corners[(board_h-2)*(board_w-1),:];
imgPts[3,:] = corners[(board_h-2)*(board_w-1) + board_w-2,:];

objPts[0,0] = 0; objPts[0,1] = 0;
objPts[1,0] = board_w -1; objPts[1,1] = 0;
objPts[2,0] = 0; objPts[2,1] = board_h -1;
objPts[3,0] = board_w -1; objPts[3,1] = board_h -1;




img = cv2.drawChessboardCorners(img, (9,6), imgPts,ret)
cv2.imshow('img',img)
cv2.waitKey(20)

H = cv2.getPerspectiveTransform(objPts,imgPts)
_,H2=cv2.findHomography(objPts,imgPts)
H[2,2] = 20
'''H=np.array([[  2.60967808e+01,  -1.09276966e+01,   1.70017319e+02],
 [  2.91876961e+00,   3.07929759e+00 ,  2.71176483e+02],
 [  3.92744157e-03,  -3.78157027e-02,   2.00000000e+01]])'''

print ('Homography matrix:',H)
img = cv2.warpPerspective(frame,H,(480,640),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)


###################################################

#CAMERA TO ROBOT FRAME

############################################





