import cv2
import numpy as np
import os
import tensorflow as tf
from tensorflow.contrib.tensorboard.plugins import projector


def img_paths(directory):
    return [os.path.join(root, f) for root, _, files in os.walk(directory)
            for f in files if f != "target.png" and f != ".DS_Store"]


def generate_tensorboard_embeddings_from_path(images_path, features_path, logdir_path, clusters_path):
    """
    Function that generates tensorflow data for embeddings visualization
    :param images_path: Path to the images directory, images have to be stored in this directory, not hierarchically
    :param features_path: Path to a .txt numpy file for the features used for clustering
    :param clusters_path: Path to a .txt numpy file for the resulting clusters
    :param logdir_path:  Output directory where tensorboard is going to be run
    :return:
    """
    # %%
    data_path = images_path
    LOG_DIR = logdir_path


    os.makedirs(logdir_path, exist_ok=True)

    img_list = os.listdir(data_path)
    print('Loaded the images of dataset-' + '{}\n'.format(data_path))
    img_data = []
    for img in img_list:
        input_img = cv2.imread(data_path + '/' + img)
        input_img_resize = cv2.resize(input_img, (224, 224))
        img_data.append(input_img_resize)

    img_data = np.array(img_data)

    # %%

    feature_vectors = np.loadtxt(features_path)
    print("feature_vectors_shape:", feature_vectors.shape)
    print("num of images:", feature_vectors.shape[0])
    print("size of individual feature vector:", feature_vectors.shape[1])

    num_of_samples = feature_vectors.shape[0]

    features = tf.Variable(feature_vectors, name='features')

    if clusters_path is None:
        y = np.ones((num_of_samples,), dtype='int64')
    else:
        clusters = np.loadtxt(clusters_path)
        y= clusters.astype(np.int8)


    # with open(metadata, 'w') as metadata_file:
    #    for row in range(210):
    #        c = y[row]
    #        metadata_file.write('{}\n'.format(c))
    metadata_file = open(os.path.join(LOG_DIR, 'metadata_4_classes.tsv'), 'w')
    metadata_file.write('Class\tName\n')
    k = 100  # num of samples in each class
    j = 0
    # for i in range(210):
    #    metadata_file.write('%06d\t%s\n' % (i, names[y[i]]))
    for i in range(num_of_samples):
        c = y[i]
        if i % k == 0:
            j = j + 1
        metadata_file.write('{}\t{}\n'.format(j, c))
        # metadata_file.write('%06d\t%s\n' % (j, c))
    metadata_file.close()


    # Taken from: https://github.com/tensorflow/tensorflow/issues/6322
    def images_to_sprite(data):
        """Creates the sprite image along with any necessary padding

        Args:
          data: NxHxW[x3] tensor containing the images.

        Returns:
          data: Properly shaped HxWx3 image with any necessary padding.
        """
        if len(data.shape) == 3:
            data = np.tile(data[..., np.newaxis], (1, 1, 1, 3))
        data = data.astype(np.float32)
        min = np.min(data.reshape((data.shape[0], -1)), axis=1)
        data = (data.transpose(1, 2, 3, 0) - min).transpose(3, 0, 1, 2)
        max = np.max(data.reshape((data.shape[0], -1)), axis=1)
        data = (data.transpose(1, 2, 3, 0) / max).transpose(3, 0, 1, 2)
        # Inverting the colors seems to look better for MNIST
        # data = 1 - data

        n = int(np.ceil(np.sqrt(data.shape[0])))
        padding = ((0, n ** 2 - data.shape[0]), (0, 0),
                   (0, 0)) + ((0, 0),) * (data.ndim - 3)
        data = np.pad(data, padding, mode='constant',
                      constant_values=0)
        # Tile the individual thumbnails into an image.
        data = data.reshape((n, n) + data.shape[1:]).transpose((0, 2, 1, 3)
                                                               + tuple(range(4, data.ndim + 1)))
        data = data.reshape((n * data.shape[1], n * data.shape[3]) + data.shape[4:])
        data = (data * 255).astype(np.uint8)
        return data


    # %%
    sprite = images_to_sprite(img_data)
    cv2.imwrite(os.path.join(LOG_DIR, 'sprite_4_classes.png'), sprite)
    # scipy.misc.imsave(os.path.join(LOG_DIR, 'sprite.png'), sprite)

    # %%
    with tf.Session() as sess:
        saver = tf.train.Saver([features])

        sess.run(features.initializer)
        saver.save(sess, os.path.join(LOG_DIR, 'images_4_classes.ckpt'))

        config = projector.ProjectorConfig()
        # One can add multiple embeddings.
        embedding = config.embeddings.add()
        embedding.tensor_name = features.name
        # Link this tensor to its metadata file (e.g. labels).
        embedding.metadata_path = os.path.join(LOG_DIR, 'metadata_4_classes.tsv')
        # Comment out if you don't want sprites
        embedding.sprite.image_path = os.path.join(LOG_DIR, 'sprite_4_classes.png')
        embedding.sprite.single_image_dim.extend([img_data.shape[1], img_data.shape[1]])
        # Saves a config file that TensorBoard will read during startup.
        projector.visualize_embeddings(tf.summary.FileWriter(LOG_DIR), config)

def generate_tensorboard_embeddings(images, feature_vectors, clusters, logdir_path):
    """
    Function that generates tensorflow data for embeddings visualization
    :param images_path: Path to the images directory, images have to be stored in this directory, not hierarchically
    :param features_path: Path to a .txt numpy file for the features used for clustering
    :param clusters_path: Path to a .txt numpy file for the resulting clusters
    :param logdir_path:  Output directory where tensorboard is going to be run
    :return:
    """
    # %%
    LOG_DIR = logdir_path

    os.makedirs(logdir_path, exist_ok=True)

    images = np.array(images)

    # %%

    print("feature_vectors_shape:", feature_vectors.shape)
    print("num of images:", feature_vectors.shape[0])
    print("size of individual feature vector:", feature_vectors.shape[1])

    num_of_samples = feature_vectors.shape[0]

    features = tf.Variable(feature_vectors, name='features')

    y = clusters.astype(np.int8)


    # with open(metadata, 'w') as metadata_file:
    #    for row in range(210):
    #        c = y[row]
    #        metadata_file.write('{}\n'.format(c))
    metadata_file = open(os.path.join(LOG_DIR, 'metadata_4_classes.tsv'), 'w')
    metadata_file.write('Class\tName\n')
    k = 100  # num of samples in each class
    j = 0
    # for i in range(210):
    #    metadata_file.write('%06d\t%s\n' % (i, names[y[i]]))
    for i in range(num_of_samples):
        c = y[i]
        if i % k == 0:
            j = j + 1
        metadata_file.write('{}\t{}\n'.format(j, c))
        # metadata_file.write('%06d\t%s\n' % (j, c))
    metadata_file.close()


    # Taken from: https://github.com/tensorflow/tensorflow/issues/6322
    def images_to_sprite(data):
        """Creates the sprite image along with any necessary padding

        Args:
          data: NxHxW[x3] tensor containing the images.

        Returns:
          data: Properly shaped HxWx3 image with any necessary padding.
        """
        if len(data.shape) == 3:
            data = np.tile(data[..., np.newaxis], (1, 1, 1, 3))
        data = data.astype(np.float32)
        min = np.min(data.reshape((data.shape[0], -1)), axis=1)
        data = (data.transpose(1, 2, 3, 0) - min).transpose(3, 0, 1, 2)
        max = np.max(data.reshape((data.shape[0], -1)), axis=1)
        data = (data.transpose(1, 2, 3, 0) / max).transpose(3, 0, 1, 2)
        # Inverting the colors seems to look better for MNIST
        # data = 1 - data

        n = int(np.ceil(np.sqrt(data.shape[0])))
        padding = ((0, n ** 2 - data.shape[0]), (0, 0),
                   (0, 0)) + ((0, 0),) * (data.ndim - 3)
        data = np.pad(data, padding, mode='constant',
                      constant_values=0)
        # Tile the individual thumbnails into an image.
        data = data.reshape((n, n) + data.shape[1:]).transpose((0, 2, 1, 3)
                                                               + tuple(range(4, data.ndim + 1)))
        data = data.reshape((n * data.shape[1], n * data.shape[3]) + data.shape[4:])
        data = (data * 255).astype(np.uint8)
        return data


    # %%
    sprite = images_to_sprite(images)
    cv2.imwrite(os.path.join(LOG_DIR, 'sprite_4_classes.png'), sprite)
    # scipy.misc.imsave(os.path.join(LOG_DIR, 'sprite.png'), sprite)

    # %%
    with tf.Session() as sess:
        saver = tf.train.Saver([features])

        sess.run(features.initializer)
        saver.save(sess, os.path.join(LOG_DIR, 'images_4_classes.ckpt'))

        config = projector.ProjectorConfig()
        # One can add multiple embeddings.
        embedding = config.embeddings.add()
        embedding.tensor_name = features.name
        # Link this tensor to its metadata file (e.g. labels).
        embedding.metadata_path = os.path.join(LOG_DIR, 'metadata_4_classes.tsv')
        # Comment out if you don't want sprites
        embedding.sprite.image_path = os.path.join(LOG_DIR, 'sprite_4_classes.png')
        embedding.sprite.single_image_dim.extend([images.shape[1], images.shape[1]])
        # Saves a config file that TensorBoard will read during startup.
        projector.visualize_embeddings(tf.summary.FileWriter(LOG_DIR), config)
