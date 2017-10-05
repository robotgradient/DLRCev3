import numpy as np
from nn_object_detection.object_detectors import NNObjectDetector
from rick.utils import plot_bbox
import tensorflow as tf
from scipy import misc

PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/faster_rcnn_resnet_lego_v1/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"



import cv2
cap = cv2.VideoCapture('/home/dlrc/Videos/Webcam/2017-10-04-145623.webm')
detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
counter = 0
bbox_images = []
while(1):
    counter+=1
    ret, frame = cap.read()
    
    if not counter%10 == 0:
        continue
    #fgmask = fgbg.apply(frame)
    
    bboxes = detector.detect_with_threshold(frame)
    for bbox, score in  bboxes:

        snippet = frame[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        if len(snippet) == 0:
            continue
        bbox_images.append(snippet)
        snippet = cv2.resize(snippet, (224,224))
        print(len(snippet))
        #plot_bbox(frame, bbox, score)

        #cv2.imshow('frame',snippet)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
    break

bbox_images = np.asarray(bbox_images)
np.save('bbox_images', bbox_images)
cap.release()
cv2.destroyAllWindows()