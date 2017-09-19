import numpy as np
import cv2
import glob
import math as m

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')
print (len(images))
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None, 1 | 4)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(20)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
mtx=np.array(mtx)
dist=np.array(dist)
rvecs=np.array(rvecs)
tvecs=np.array(tvecs)
print(mtx.shape,dist.shape,rvecs.shape,tvecs.shape)

# Add image to crop
img = cv2.imread('Chessboard_15.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

print(newcameramtx)

## GET BORDERS
cap = cv2.VideoCapture(1)
#img = cv2.imread('Chessboard_9.jpg')
while True:
    ret,img=cap.read()
    cv2.imshow("actual image",img)
    if cv2.waitKey(10) & 0xFF==32:
        break

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (9,6),None, 1 | 4)

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

#img = cv2.drawChessboardCorners(img, (9,6), objPts,ret)
cv2.imshow('img',img)
cv2.waitKey(20)

H = cv2.getPerspectiveTransform(objPts,imgPts)
H[2,2] = 20

print("image size : ", img.shape)
dst = cv2.warpPerspective(img,H,img.shape[0:2],flags= cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)

cv2.imshow('img',img)
cv2.waitKey()


cv2.imshow('distorsion',dst)
cv2.waitKey()
