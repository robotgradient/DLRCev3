import numpy as np
import cv2
from .utils import *

def flip_coords(bbox):
    """
        Flips the bounding boxes
    """
    return np.asarray((bbox[1], bbox[0], bbox[3], bbox[2]))

def bboxf2i(bbox, res):
    """
        Denormalize bbox
    """
    return np.asarray((int(bbox[0]*res[0]), int(bbox[1]*res[1]), int(bbox[2]*res[0]), int(bbox[3]*res[1])))

def bbox2xywh(bbox):
    return np.asarray((bbox[0], bbox[1], bbox[2]-bbox[0], bbox[3] - bbox[1]))

def xywh2bbox(bbox):
    return np.asarray((bbox[0], bbox[1], bbox[2]+bbox[0], bbox[3]+bbox[1]))


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


def apply_filter(img, kernel=np.ones((5,5),np.float32)/25):
    img = cv2.filter2D(img,-1,kernel)
    return img

def plot_bbox(frame,box, score=0, color=(0,255,0)):
    cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), thickness=3)    
    cv2.putText(frame, str(score) + "%", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

class TrackerWrapper(object):


    def __init__(self, tracker_constructor):
        self.tracker_constructor = tracker_constructor
        self.tracker = None
    def init(self,frame, bbox):
        bbox = bbox2xywh(bbox)
        if self.tracker:
            self.tracker.clear()
        self.tracker = self.tracker_constructor()
        return self.tracker.init(frame, tuple(bbox))

    def update(self,frame):
        if self.tracker is None:
            return False, np.asarray([0,0,0,0])
        ok, bbox = self.tracker.update(frame)
        bbox = xywh2bbox(bbox)
        return ok, bbox.astype(np.int32)

    @property
    def isInit(self):
        return not self.tracker is None



class DetectionTrackingCoordinator(object):
    def __init__(self, tracker, detector):
        self.tracker = tracker
        self.detector = detector
        self.tracking = False

    def bbox(self):
        res = self.object_detector.detect_with_threshold(img,threshold=0.9, return_closest=True)
        detected = True if len(res) > 0 else False

        if detected:
            color = (0,0,255)
            box, score = res[0]
            dbox = np.asarray((int(box[1]*img_res[0]), int(box[0]*img_res[1]), int(box[3]*img_res[0]), int(box[2]*img_res[1])))
            if tracking:
                ok, tbox = robot.tracker.update(frame)
                if bboxes_are_overlapping(tbox,dbox):
                    robot.tracker.init(frame, dbox)
                    bbox = dbox
                else:
                    bbox = tbox
            else:
                robot.tracker.init(frame, dbox)
                bbox = dbox
        elif tracking:
            color = (255,0,0)
            ok, bbox = robot.tracker.update(frame)
            if not ok:
                return "SEARCH", frame, {}

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