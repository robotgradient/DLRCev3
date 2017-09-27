from rick.utils import *
from rick.async import *
from nn_object_detection.object_detectors import NNObjectDetector
import cv2 


# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

object_detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)


tracker = TrackerWrapper(cv2.TrackerKCF_create)
#tracker.init(frame,bbox)
#ok, bbox = tracker.update(frame)

detection_tracker = DetectionTrackingCoordinator(tracker, object_detector)
camera = AsyncCamera(0)
img_res = (640, 480)

def plot_bbox(frame,bbox, score=0, color=(0,255,0)):
    cv2.rectangle(frame, (box[1], box[0]), (box[3], box[2]), (0, 255, 0), thickness=3)    
    cv2.putText(frame, str(score) + "%", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

while True:
    ok, frame = camera.frame

    # Display result
    img = apply_filter(frame)


    res = object_detector.detect_with_threshold(img, threshold=0.95, return_closest=True)


    ok, tbox = tracker.update(frame)
    print(tbox)

    if len(res) > 0:
        for r in res:

            box, score = r 


            box = np.asarray((int(box[1]*img_res[0]), int(box[0]*img_res[1]), int(box[3]*img_res[0]), int(box[2]*img_res[1])))
            if ok and not bboxes_are_overlapping(box, tbox):
                cv2.rectangle(frame, (tbox[0], tbox[1]), 
                    (tbox[2], tbox[3]), (255, 0, 0), thickness=3)
                break

            img_res = np.asarray(img_res)
            cv2.rectangle(frame, (box[0], box[1]), 
                    (box[2], box[3]), (0, 255, 0), thickness=3)    
            cv2.putText(frame, str(score) + "%", (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))

            tracker.init(frame, box)

            break
    elif ok:
        cv2.rectangle(frame, (tbox[0], tbox[1]), 
                    (tbox[2], tbox[3]), (255, 0, 0), thickness=3)

    cv2.imshow("Tracking", frame)
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

