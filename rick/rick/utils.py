import numpy as np
import cv2
from .utils import *

def bbox_center(xmin, ymin, xmax, ymax):
    return np.asarray([(xmin + xmax) / 2., (ymin + ymax) / 2.])


def bbox_bottom_center(xmin, ymin, xmax, ymax):
    return np.asarray([(xmin + xmax) / 2., ymax])

def bboxes_are_overlapping(bbox1, bbox2):
    return not (bbox1[2]<bbox2[0] or bbox2[2]<bbox1[0] or bbox1[3]<bbox2[1] or bbox2[3]<bbox1[1])


def range_overlap(a_min, a_max, b_min, b_max):
    '''Neither range is completely greater than the other
    '''
    overlapping = True
    if (a_min > b_max) or (a_max < b_min):
        overlapping = False
    return overlapping


class DetectionTrackingCoordinator(object):
    def __init__(self, tracker, detector):
        self.tracker = tracker
        self.detector = detector

    @property
    def first_coords(self, frame, img_res):
        tracker_bbox = self.tracker.bbox
        detector_bbox = self.detector.bbox

        img = frame
        kernel = np.ones((5,5),np.float32)/25
        img = cv2.filter2D(img,-1,kernel)
        res = robot.object_detector.detect_with_threshold(img,threshold=0.7, return_closest=True)

        box, score = res[0] 
        img_res = np.asarray(img_res)
        coords = bbox_center(box[1], box[0], box[3], box[2]) * img_res
        coords = coords.astype(np.int16)
        atol = 10 + coords[1]/480 * 50
        print("Coords: ", coords)
        cv2.rectangle(frame, (int(box[1]*img_res[0]), int(box[0]*img_res[1])), 
                (int(box[3]*img_res[0]), int(box[2]*img_res[1])), (0, 255, 0), thickness=3)    
        cv2.putText(frame, str(score) + "%", (int(box[1]*img_res[0]),
         int(box[0]*img_res[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))
        cv2.circle(frame,tuple(coords),2,(255,255,255),thickness=2)

        if not detector_bbox is None and not tracker_bbox is None:

            if bboxes_are_overlapping(detector_bbox, tracker_bbox):
                tracker_bbox.init(frame, detector_bbox)
                return detector_bbox
            else:
                return tracker_bbox

        if detector_bbox is None and not tracker_bbox is None:
            return tracker_bbox
        elif detector_bbox is not None:
            tracker_bbox.init(frame, detector_bbox)
            return detector_bbox
        else:
            return None


        return coords, frame