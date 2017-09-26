import numpy as np


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
    def bbox(self):
        tracker_bbox = self.tracker.bbox
        detector_bbox = self.detector.bbox

        if not detector_bbox is None and not tracker_bbox is None:
            # Check if they are overlapping, if yes return the detector_box and update tracker, if not, return tracker
            pass
        if detector_bbox is None and not tracker_bbox is None:
            pass
        elif detector_bbox is not None:
            pass

