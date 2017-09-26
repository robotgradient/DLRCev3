from rick.utils import *
from rick.async import *
from nn_object_detection.object_detectors import NNObjectDetector
import cv2 
from rick.utils import bbf, bbdn, bboxes_are_overlapping


# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/models/ssd_mobilenet_v1_lego/train/frozen_inference_graph.pb"
PATH_TO_LABELS = "/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/label_map.pbtxt"
NUM_CLASSES = 2

object_detector = NNObjectDetector(PATH_TO_CKPT, PATH_TO_LABELS)


tracker = cv2.TrackerKCF_create()
#tracker.init(frame,bbox)
#ok, bbox = tracker.update(frame)

detection_tracker = DetectionTrackingCoordinator(tracker, object_detector)
camera = AsyncCamera(0)
img_res = (640, 480)

while True:
    ok, frame = camera.frame

    # Display result
    img = frame
    kernel = np.ones((5,5),np.float32)/25
    img = cv2.filter2D(img,-1,kernel)
    res = object_detector.detect_with_threshold(img,threshold=0.9, return_closest=True)
    print("Res: ", res)
    ok, bbox = tracker.update(frame)
    bbox = (bbox[0], bbox[1], bbox[0] + bbox[2], bbox[1] + bbox[3])
    ok = False
    if len(res)==0 and ok:
        box = bbox
        print("Plotting tracking")
        cv2.rectangle(frame, (int(box[1]*img_res[0]), int(box[0]*img_res[1])), 
            (int(box[3]*img_res[0]), int(box[2]*img_res[1])), (0, 0, 255), thickness=2)
        cv2.imshow("Tracking", frame)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
        continue
    elif len(res)==0:
        continue
        cv2.imshow("Tracking", frame)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
        continue
    elif ok:
        box_net, score = res[0]
        box_net = bbdn(bbf(box_net), img_res)
        if not bboxes_are_overlapping(box, box_net):
            bbox

    box, score = res[0]
    box = bbdn(bbf(box), img_res)
    print("Initializing tracker...") 
    tracker.clear()
    tracker = cv2.TrackerKCF_create()
    kcf_init =  (box[0], box[1], box[2]-box[0], box[3] - box[1])
    print(kcf_init)
    tracker.init(frame, kcf_init)
    print("Shape: ", res[0])
    img_res = np.asarray(img_res)
    coords = bbox_center(box[1], box[0], box[3], box[2]) * img_res

    coords = coords.astype(np.int16)
    atol = 10 + coords[1]/480 * 50
    print("Coords: ", coords)
    cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), thickness=3)    
    cv2.putText(frame, str(score) + "%", (int(box[1]*img_res[0]),
     int(box[0]*img_res[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(255,0,0))
    cv2.circle(frame,tuple(coords),2,(255,255,255),thickness=2)
    cv2.imshow("Tracking", frame)
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27: break

