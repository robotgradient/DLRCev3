from object_detectors import NNObjectDetector

from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import sys
import tensorflow as tf
sys.path.append("/home/dlrc/projects/tensorflow/models/object_detection/utils")
import label_map_util
import numpy as np
from detection.opencv import eliminate_grip

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/faster_rcnn_resnet_lego_v1/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)


import cv2
import numpy as np

# Actual detection.

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
img_res = np.asarray([640,480])

try:
    while(True):
        _, img = cap.read()
        kernel = np.ones((5,5),np.float32)/25
        #img = cv2.filter2D(img,-1,kernel)
        img = eliminate_grip(img)
        #image_np_expanded = np.expand_dims(img, axis=0)
        res = detector.detect_with_threshold(img, threshold=0.9, return_closest=True)[0]
        res = [res]
        if res:
            for lego in res:
                box, score = lego
                print("Main loop: ", box[0], len(box), score)
                # cv2.rectangle(img, (int(box[1]*img_res[0]), int(box[0]*img_res[1])), 
                #     (int(box[3]*img_res[0]), int(box[2]*img_res[1])), (0, 255, 0), thickness=3)
                cv2.rectangle(img, (box[0], box[1]), 
                    (box[2], box[3]), (0, 255, 0), thickness=3)    
                cv2.putText(img, str(score) + "%", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

        cv2.imshow("Camera", img)
        if cv2.waitKey(10) & 0xFF==27:
            break

except KeyboardInterrupt as e:
    cap.close()