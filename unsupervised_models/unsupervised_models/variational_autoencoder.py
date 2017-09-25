import tensorflow as tf
from tensorflow.python import debug as tf_debug
class VariationalAutoencoder(object):

    def __init__(self, n_hidden, optimizer = tf.train.AdamOptimizer(), image_size=(32,32), debug=False):
        self.n_input = image_size[0] * image_size[1]
        self.n_hidden = n_hidden

        network_weights = self._initialize_weights()
        self.weights = network_weights

        # model
        self.x = tf.placeholder(tf.float64, shape=[None,self.n_input])

        #self.image_resize = tf.image.resize_bilinear(self.x, size=(32,32), name="image_resize")

        self.z_mean = tf.add(tf.matmul(self.x, self.weights['w1']), self.weights['b1'])
        self.z_log_sigma_sq = tf.add(tf.matmul(self.x, self.weights['log_sigma_w1']), self.weights['log_sigma_b1'])

        # sample from gaussian distribution
        eps = tf.random_normal(tf.stack([tf.shape(self.x)[0], self.n_hidden]), 0, 1, dtype = tf.float64)
        self.z = tf.add(self.z_mean, tf.multiply(tf.sqrt(tf.exp(self.z_log_sigma_sq)), eps))

        self.reconstruction = tf.add(tf.matmul(self.z, self.weights['w2']), self.weights['b2'])

        # cost
        reconstr_loss = 0.5 * tf.reduce_sum(tf.pow(tf.subtract(self.reconstruction, self.x), 2.0))
        latent_loss = -0.5 * tf.reduce_sum(1 + self.z_log_sigma_sq
                                           - tf.square(self.z_mean)
                                           - tf.exp(self.z_log_sigma_sq), 1)
        self.cost = tf.reduce_mean(reconstr_loss + latent_loss)
        self.optimizer = optimizer.minimize(self.cost)

        init = tf.global_variables_initializer()
        self.sess = tf.Session()
        if debug:
            self.sess = tf_debug.LocalCLIDebugWrapperSession(self.sess)
            self.sess.add_tensor_filter("has_inf_or_nan", tf_debug.has_inf_or_nan)
        self.sess.run(init)

    def _initialize_weights(self):
        all_weights = dict()
        all_weights['w1'] = tf.get_variable("w1", shape=[self.n_input, self.n_hidden],
            initializer=tf.contrib.layers.xavier_initializer())
        all_weights['log_sigma_w1'] = tf.get_variable("log_sigma_w1", shape=[self.n_input, self.n_hidden],
            initializer=tf.contrib.layers.xavier_initializer())
        all_weights['b1'] = tf.Variable(tf.zeros([self.n_hidden], dtype=tf.float64))
        all_weights['log_sigma_b1'] = tf.Variable(tf.zeros([self.n_hidden], dtype=tf.float64))
        all_weights['w2'] = tf.Variable(tf.zeros([self.n_hidden, self.n_input], dtype=tf.float64))
        all_weights['b2'] = tf.Variable(tf.zeros([self.n_input], dtype=tf.float64))
        return all_weights

    def partial_fit(self, X):
        cost, opt = self.sess.run((self.cost, self.optimizer), feed_dict={self.x: X})
        return cost

    def calc_total_cost(self, X):
        return self.sess.run(self.cost, feed_dict = {self.x: X})

    def transform(self, X):
        return self.sess.run(self.z_mean, feed_dict={self.x: X})

    def generate(self, hidden = None):
        if hidden is None:
            hidden = self.sess.run(tf.random_normal([1, self.n_hidden]))
        return self.sess.run(self.reconstruction, feed_dict={self.z: hidden})

    def reconstruct(self, X):
        return self.sess.run(self.reconstruction, feed_dict={self.x: X})

    def getWeights(self):
        return self.sess.run(self.weights['w1'])

    def getBiases(self):
        return self.sess.run(self.weights['b1'])


import numpy as np
import sklearn.preprocessing as prep
import os
from scipy import misc
from tqdm import tqdm

modes = [None, 'L', None, 'RGB', 'RGBA']

def read_images(path_to_images, size=None):
    """
    Return numpy array of images from directories
    :param path_to_images:
    :return:
    """
    image_file_paths = map(lambda x: os.path.join(path_to_images, x), os.listdir(path_to_images))
    image_file_paths = filter(lambda x: "jpeg" in x or "jpg" in x or "png" in x, image_file_paths)
    images = []
    for image_path in tqdm(image_file_paths):
        image = misc.imread(image_path, mode='L')
        images.append(image)

    if size:
        for i, img in enumerate(images):
            print(img.shape[-1])
            images[i] = misc.imresize(img, size, mode='L').reshape(size[0], size[1],1)
    return np.asarray(images)



def min_max_scale(X_train, X_test):
    preprocessor = prep.MinMaxScaler().fit(X_train)
    X_train = preprocessor.transform(X_train)
    X_test = preprocessor.transform(X_test)
    return X_train, X_test


def get_batch(data, batch_size, reset=False):
    data_size = len(data)
    for i in np.arange(0, data_size-batch_size, batch_size):
        yield data[i:i+batch_size]

