from object_detectors import NNObjectDetector

from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import sys
import tensorflow as tf
sys.path.append("/home/dlrc/projects/tensorflow/models/object_detection")
from utils import label_map_util
import numpy as np

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)


import cv2
import numpy as np

# Actual detection.

cap = cv2.VideoCapture(0)
res = np.asarray([640,480])

try:
    while(True):
        _, img = cap.read()
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(img,-1,kernel)
        #image_np_expanded = np.expand_dims(img, axis=0)
        (boxes, scores, classes, num) = detector.detect(img)
        boxes = boxes.reshape(-1,4)
        scores = scores.reshape(-1)
        for i,box in enumerate(boxes):
            if scores[i] < 0.8:
                break
            print("Box shape: ", box, " Scores shape: ", scores.shape)
            cv2.rectangle(img, (int(box[1]*res[0]), int(box[0]*res[1])), 
                (int(box[3]*res[0]), int(box[2]*res[1])), (0, 255, 0), thickness=3)    
            cv2.putText(img, str(scores[i]) + "%", (int(box[1]*res[0]),
             int(box[0]*res[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

        cv2.imshow("Camera", img)
        if cv2.waitKey(10) & 0xFF==27:
            break

except KeyboardInterrupt as e:
    cap.close()