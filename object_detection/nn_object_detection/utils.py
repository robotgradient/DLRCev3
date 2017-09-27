import numpy as np
import cv2


def flip_coords(bbox):
    """
        Flips the bounding boxes
    """
    return np.asarray((bbox[1], bbox[0], bbox[3], bbox[2]))

def sorting_key(element1):
    bbox1=element1[0]
    return bbox1[2]

def transform_coordinates(bbox, img_res):
    bbox = np.asarray((box[1]*img_res[0], box[0]*img_res[1], box[3]*img_res[0], box[2]*img_res[1]))
    bbox = bbox.astype(np.int16)
    return bbox


def filter_detection_results(results, size, sort_closest=True, threshold=0.95):
    
    results = list(zip(results[0], results[1]))
    results = filter(lambda x: x[1] > threshold, results)    

    if not sort_closest:
        return results
    else:
        sorted_boxes = sorted(results, key=sorting_key, reverse=True)
        if len(sorted_boxes) == 0:
            return sorted_boxes
        elif size == 1:
            return sorted_boxes[0]
        elif len(sorted_boxes) > size:
            return sorted_boxes[:size]
        else:
            return sorted_boxes



def render_bbox(frame,bbox, color=(0, 0, 255)):

    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[2]), int(bbox[3]))
    cv2.rectangle(frame, p1, p2, color)