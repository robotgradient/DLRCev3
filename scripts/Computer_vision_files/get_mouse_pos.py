import cv2
import numpy as np  



class CoordinateStore:
    def __init__(self):
        self.points = []

    def select_point(self,event,x,y,flags,param):
            if event == cv2.EVENT_LBUTTONDBLCLK:
                cv2.circle(img,(x,y),3,(255,0,0),-1)
                self.points.append((x,y))


#instantiate class
coordinateStore1 = CoordinateStore()


# Create a black image, a window and bind the function to window
img = np.zeros((512,512,3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image',coordinateStore1.select_point)

cap = cv2.VideoCapture(1)
ret,img=cap.read()

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()


print ("Selected Coordinates: ")
for i in range(len(coordinateStore1.points)):
    print ("point {} is in {} coordinates".format(i,coordinateStore1.points[i]))
