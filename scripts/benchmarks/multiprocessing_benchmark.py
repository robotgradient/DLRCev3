import cv2
import time
from rick.async import *



bbox = (100, 100, 120, 120)
tracker = cv2.Tracker_create("KCF")
camera = AsyncCamera(0)
frame = camera.frame
tracker.init(camera.frame, bbox)
tracker = AsyncObjectTracker(camera, tracker)

def only_camera():
    cap = cv2.VideoCapture(0)
    t = time.time()
    counter = 0
    while t < 5:
        cv2.imshow("Camera1", cap.read()[1])
        counter+=1
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break

    print("Rate: " , counter    )
def parallel():
    while True:
        frame = camera.frame
        if not frame is None:
            cv2.imshow("Camera1", frame)
            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27: break

parallel()