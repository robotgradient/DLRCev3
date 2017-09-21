import numpy as np
import tensorflow as tf
from object_detection.utils import dataset_util
import os
from tqdm import tqdm
import keras as ke

flags = tf.app.flags
train_output_path = '/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/train.record'
eval_output_path = '/home/dlrc/projects/DLRCev3/object_detection/nn_object_detection/tf_train_dir/data/eval.record'
FLAGS = flags.FLAGS
IMAGES_DIR = '/home/dlrc/datasets/object_detection_dataset_v2/images'
BBOXES_DIR ='/home/dlrc/datasets/object_detection_dataset_v2/bboxes'

def create_block_tf_proto(encoded_block_image_data, filename, image_format, image_dim=(640,480), bboxes=None):
    """Creates a tf.Example proto from sample camera image.

    Args:
    encoded_block_image_data: The jpg encoded data of the block image.

    Returns:
    tf_example: The created tf.Example.
    """
    
    height, width = image_dim

    classes_text = ['lego_block'.encode()]*bboxes.shape[0]
    classes = [1]*bboxes.shape[0]
    
    bboxes = bboxes.T
    xmins, xmaxs, ymins, ymaxs = bboxes[0], bboxes[2], bboxes[1], bboxes[3]
    xmins = xmins / image_dim[0]
    xmaxs = xmaxs / image_dim[0]
    ymins = ymins / image_dim[1]
    ymaxs = ymaxs / image_dim[1]
    
    assert not np.any(xmaxs < xmins)
    assert not np.any(ymaxs < ymins)
    assert not np.any(xmaxs >= 1)
    assert not np.any(ymaxs >= 1)
  
    
    bboxes = np.asarray([xmins, xmaxs, ymins, ymaxs])
    if np.any(bboxes >= 1):
        raise Exception("Not normalized " + str(bboxes))
    
    tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename.encode()),
      'image/source_id': dataset_util.bytes_feature(filename.encode()),
      'image/encoded': dataset_util.bytes_feature(encoded_block_image_data),
      'image/format': dataset_util.bytes_feature(image_format.encode()),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    
    return tf_example


def generate_records(output_path, files):
    

    writer = tf.python_io.TFRecordWriter(output_path)
    print(files.shape)
    for image_path, bbox_path in tqdm(files):
        image_path = os.path.join(IMAGES_DIR, image_path)
        bbox_path = os.path.join(BBOXES_DIR, bbox_path)
        img_file = open(image_path, 'rb')
        img_encoded = img_file.read()
        bboxes = np.load(bbox_path)
        tf_example = create_block_tf_proto(filename=image_path, bboxes=bboxes, encoded_block_image_data=img_encoded,
                                image_format=u'jpeg')
        writer.write(tf_example.SerializeToString())

def generate_train_eval():
    
    image_files = sorted(os.listdir(IMAGES_DIR))
    bbox_files = sorted(os.listdir(BBOXES_DIR))
    
    train_indices = slice(0,int(0.95*len(image_files)))
    eval_indices = slice(int(0.95*len(image_files)), -1)
    zipped_files = np.vstack((image_files, bbox_files)).T
    train_files = zipped_files[train_indices]
    eval_files = zipped_files[eval_indices]
    
    generate_records(train_output_path, train_files)
    generate_records(eval_output_path, eval_files)
    
with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    generate_train_eval()
    sess.close()