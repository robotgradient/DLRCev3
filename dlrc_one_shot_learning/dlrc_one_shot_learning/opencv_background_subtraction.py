import numpy as np
from nn_object_detection.object_detectors import NNObjectDetector
from rick.utils import plot_bbox
import tensorflow as tf


PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/faster_rcnn_resnet_lego_v1/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"



import cv2
cap = cv2.VideoCapture('/home/dlrc/Videos/Webcam/2017-10-04-145623.webm')
detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
while(1):
    tensors = [n.name for n in detector.detection_graph.as_graph_def().node]
    print(tensors)
    break
    ret, frame = cap.read()
    #fgmask = fgbg.apply(frame)
    
    bboxes = detector.detect_with_threshold(frame)
    for bbox, score in  bboxes:
        plot_bbox(frame, bbox, score)

    cv2.imshow('frame',frame)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()