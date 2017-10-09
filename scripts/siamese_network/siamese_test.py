import os

os.environ["KERAS_BACKEND"] = "tensorflow"

import keras
import numpy as np
from scipy import misc
import cv2
from keras import backend as K


def contrastive_loss(y_true, y_pred):
    '''Contrastive loss from Hadsell-et-al.'06
    http://yann.lecun.com/exdb/publis/pdf/hadsell-chopra-lecun-06.pdf
    '''
    margin = 1
    return K.mean(y_true * K.square(y_pred) +
                  (1 - y_true) * K.square(K.maximum(margin - y_pred, 0)))


keras.losses.contrastive_loss = contrastive_loss

siamese_network = keras.models.load_model("/Users/Jimmy/Desktop/siamese2/checkpoint.19.hdf5")

# Loading the test images
from matplotlib import pyplot as plt
import cv2

path1 = "/Users/Jimmy/Desktop/training_data/legos/lego2x2/green/block-mini12.png"
path2 =  "/Users/Jimmy/Desktop/training_data/legos/lego2x2/dkred/block-mini21.png"

img1 = misc.imread(path1, mode="RGB")
img2 = misc.imread(path2, mode="RGB")

img1 = cv2.resize(img1, (64,64))
img2 = cv2.resize(img2, (64,64))

import time
t1 = time.time()
prediction = siamese_network.predict([, np.expand_dims(img1, axis=0)])
#print(time.time() - t1)
print("Siamese score: ", prediction)


plt.imshow(img1)
plt.imshow(img2)