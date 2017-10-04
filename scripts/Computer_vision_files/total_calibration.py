import numpy as np
import cv2
import glob
import math as m
#import cv2.aruco as aruco
from time import time
from time import sleep
import cv2.aruco as aruco

arucoParams = aruco.DetectorParameters_create()

def load_camera_params():
    data = np.load('camera_parameters.npz')
    mtx=data["cam_matrix"]
    dist=data["dist_coeff"]
    return mtx,dist

def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])
def locate_markers_robot(ids,rvec,tvec,marker_list=[1,2,3,4,5],T=np.ones((4,4))):
    rotc2r=T[0:3,0:3]
    transl=tvec
    located_matrix=0*np.ones((len(marker_list),2))

    if len(transl.shape)==3:
        
        for i,value in enumerate(ids):
            p2c=np.concatenate((tvec[i].T,np.array([[1]])),axis=0)
            roti,jac=cv2.Rodrigues(rvec[i])
            rotp2r=rotc2r.dot(roti)
            p2r=T.dot(p2c)
            x=p2r[0,0]
            y=p2r[1,0]
            d=np.sqrt(np.power(x,2)+np.power(y,2))
            theta=np.arctan2(y,x)
            gamma=rvec[i,0,2]
            index_mat=np.where(value==marker_list)
            print("Rotation vector",rvec[i])
            #print("Rotation Matrix",roti,"until here")
            euler=rotationMatrixToEulerAngles(rotc2r)
            euler2=rotationMatrixToEulerAngles(rotp2r)
            euler2=euler2*180/3.141592
            euler=euler*180/3.141592
            print("Euler angles point with respect cam",euler)
            print("Euler angles point respect robot",euler2)
            print("Coordinates respect robot",x,y,euler2[2])
            located_matrix[index_mat,:]=[d,theta]
    return located_matrix

def get_specific_marker_pose(frame,mtx,dist,marker_id,arucoParams=arucoParams,markerLength=4.8):
    Tp2r=np.ones([4,4])
    rotc2r=T[0:3,0:3]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams) # Detect aruco
    if isinstance(ids, np.ndarray) and (marker_id in ids): # if aruco marker detected
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist) # For a single marker
        position=np.where(ids==marker_id)
        #frame = aruco.drawAxis(frame, mtx, dist, rvec[position], tvec[position], 15)
        tp2c=np.concatenate((tvec[position].T,np.array([[1]])),axis=0)
        rotp2c,jac=cv2.Rodrigues(rvec[position])
        zerosrot=np.zeros([1,3])
        rot4p2c=np.concatenate((rotp2c,zerosrot), axis=0)
        Tp2c=np.concatenate((rot4p2c,tp2c), axis=1)
        Tp2r=np.ones([4,4])
        Tc2r=Tp2r.dot(np.linalg.inv(Tp2c))
    else:
        Tc2r=[]
    return Tc2r

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
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    print("newcameramtx",newcameramtx)
    np.savez("newcameramtx",newcameramtx)
    cv2.imshow("distorsion", dst)
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst2 = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    cv2.imshow("distorsion2",dst2)
    if cv2.waitKey(100) & 0xFF==27:
        break


#img=cv2.imread('Chessboard_10.jpg')
gray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
# Find the chess board corners
ret, corners = cv2.findChessboardCorners(dst, (9,6),None, 1 | 4)

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




dst = cv2.drawChessboardCorners(img, (9,6), imgPts,ret)
cv2.imshow('img',img)
cv2.waitKey(20)

H = cv2.getPerspectiveTransform(objPts,imgPts)
_,H2=cv2.findHomography(objPts,imgPts)
H[2,2] = 20
'''H=np.array([[  2.60967808e+01,  -1.09276966e+01,   1.70017319e+02],
 [  2.91876961e+00,   3.07929759e+00 ,  2.71176483e+02],
 [  3.92744157e-03,  -3.78157027e-02,   2.00000000e+01]])'''

print ('Homography matrix:',H)
img2 = cv2.warpPerspective(img,H,(640,480),flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)
cv2.imshow("waopdst", img2)
cv2.waitKey()
###################################################
print("homography",H)
#CAMERA TO ROBOT FRAME

############################################

np.savez("Homographygood",H)




