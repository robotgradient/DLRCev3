import cv2
import sys
from object_detection.opencv import BB_purple
 
if __name__ == '__main__' :
 
    # Set up tracker.
    # Instead of MIL, you can also use
    # BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN
     
    tracker = cv2.Tracker_create("MIL")
 
    # Read video
    video = cv2.VideoCapture(0)
 
    # Exit if video not opened.
    if not video.isOpened():
        print ("Could not open video")
        sys.exit()
 
    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print ('Cannot read video file')
        sys.exit()
     
    # Define an initial bounding box
    while True:
        ret,frame=video.read()
        Box_list=BB_purple(frame)
        print (Box_list)
        for box in Box_list:
            cv2.rectangle(frame,(box[0],box[1]),(box[2],box[3]),(0,255,0))
        cv2.imshow("obtaining bounding box",frame)
        if cv2.waitKey(50) & 0xFF==27:
            break
    bbox=(Box_list[0][0],Box_list[0][1],Box_list[0][2]-Box_list[0][0],Box_list[0][3]-Box_list[0][1])
    print(type(bbox))
    # Uncomment the line below to select a different bounding box
    # bbox = cv2.selectROI(frame, False)
 
    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
 
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        # Update tracker
        ok, bbox = tracker.update(frame)
 
        # Draw bounding box
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0,0,255))
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break