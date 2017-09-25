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

from utils import label_map_util
NUM_CLASSES = 2

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


def sorting_key(element1):
    bbox1=element1[0]
    return bbox1[2]/2



class NNObjectDetector(object):
  
  def __init__(self, frozen_graph_path, path_to_labels):
    
    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(frozen_graph_path, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
    self.detection_graph = detection_graph
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    self.categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    self.category_index = label_map_util.create_category_index(categories)
    self.sess =  tf.Session(graph=self.detection_graph)
    
  def detect(self,image):
    
    sess = self.sess
    # Definite input and output Tensors for detection_graph
    image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
    detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image, axis=0)
    # Actual detection.
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})

    boxes = boxes.reshape((-1,4))
    scores = scores.reshape(-1)

    return boxes, scores, classes, num



  def detect_with_threshold(self, image, threshold=0.95, return_closest=False):

    results = self.detect(image)
    results = list(zip(results[0], results[1]))
    results = filter(lambda x: x[1] > threshold, results)    

    print(results)
    if not return_closest:
        return results
    else:
        sorted_boxes = sorted(results, key=sorting_key, reverse=True)
        if len(sorted_boxes) == 0:
            return sorted_boxes
        else:
            closest = sorted_boxes[0]
        return sorted_boxes
 



