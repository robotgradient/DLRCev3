import os
import numpy as np
import cv2
from rick.async import AsyncCamera
from multiprocessing import Process, Pipe

color_feature_indices = [25519, 16964, 10071, 32687, 13174, 48585, 17754, 36306, 24108, 33711, 25198, 17468, 31804, 25622, 17870, 4084, 41417, 24133, 18351, 39441, 31663, 19450, 3958, 30780, 24222, 17988, 9562, 18213, 34724, 18426, 9391, 3927, 42441, 38860, 38295, 32828, 31204, 24978, 24224, 19690, 17989, 17436, 16010, 15938, 10068, 45475, 39964, 34376, 29724, 24029, 18030, 15704, 12122, 11562, 44889, 40465, 36030, 34945, 34618, 32759, 32273, 28503, 28069, 24830, 24507, 24072, 23612, 22592, 21252, 18363, 17291, 16780, 12674, 8209, 7623, 4562, 3924, 2121, 1123, 47838, 47570, 46251, 45477, 40514, 40009, 38417, 34630, 33398, 33212, 32723, 32325, 31922, 27180, 26180, 25778, 25157, 25070, 23828, 21883, 20018]

class SimilarityDetector(object):


    def __init__(self):
        self.target = None


    @property
    def has_target(self):
        return not self.target is None

    def set_target(self, target):
        self.target = target

    def similarity(self,x1,x2):
        pass




class EuclidianNNFeaturesBrickFinder(SimilarityDetector):

    def __init__(self):
        super(EuclidianNNFeaturesBrickFinder, self).__init__()
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        os.environ["KERAS_BACKEND"] = "tensorflow"
        import keras
        self.model = keras.applications.mobilenet.MobileNet(input_shape=(224,224,3), alpha=1.0, 
        depth_multiplier=1, dropout=1e-3, include_top=False, weights='imagenet', input_tensor=None, pooling=None)
        self.target_feature_vector = None

    def set_target(self, target):
        self.target = cv2.resize(target, (224,224))
        self.target_feature_vector = self.model.predict(np.expand_dims(self.target, 0)).reshape(-1)[color_feature_indices]

    def similarity(self, x):
        x = cv2.resize(x, (224,224))
        x_features = self.model.predict(np.expand_dims(x, 0)).reshape(-1)[color_feature_indices]
        similarity = -np.sum(np.square(x_features-self.target_feature_vector))
        return similarity


    def extract_features(self, bboxes):
        features = []
        for bbox in bboxes:
            
            #print("############# ", bbox)
            bbox = cv2.resize(bbox, (224,224))
            features.append(self.model.predict(np.expand_dims(bbox, 0)).reshape(-1)[color_feature_indices])
        return features


from concurrent.futures import ProcessPoolExecutor
from time import sleep

def func(brick_finder, cap):
    ret, frame = cap.read()

    if not brick_finder.has_target:
        brick_finder.set_target(frame)
        return None
    else:
        return brick_finder.similarity(frame)
        



class SiameseSimilarityDetector(SimilarityDetector):


    def __init__(self, path_to_ckpt):
        super(SiameseSimilarityDetector, self).__init__()
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"  # see issue #152
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        os.environ["KERAS_BACKEND"] = "tensorflow"
        import keras
        import keras.backend as K

        def contrastive_loss(y_true, y_pred):
            '''Contrastive loss from Hadsell-et-al.'06
            http://yann.lecun.com/exdb/publis/pdf/hadsell-chopra-lecun-06.pdf
            '''
            margin = 1
            return K.mean(y_true * K.square(y_pred) +
                          (1 - y_true) * K.square(K.maximum(margin - y_pred, 0)))

        keras.losses.contrastive_loss = contrastive_loss

        self.siamese_network = keras.models.load_model(path_to_ckpt)
        self.base_network = self.siamese_network.layers[2]

    def set_target(self, target):
        self.target = np.expand_dims(cv2.resize(target, (64, 64)), axis=0)/255


    def similarity(self, x1, x2=None):
        x1 = np.expand_dims(cv2.resize(x1, (64, 64)), axis=0)/255
        if not x2 is None:
            x2 = np.expand_dims(cv2.resize(x2, (64, 64)), axis=0)/255
            return -self.siamese_network.predict([x1,x2])
        else:
            return -self.siamese_network.predict([x1, self.target])

    def extract_features(self, bboxes):
        features = []
        for bbox in bboxes:
            # print("############# ", bbox)
            bbox = np.expand_dims(cv2.resize(bbox, (64, 64)), axis=0)
            features.append(self.base_network.predict([bbox]).reshape(-1))
        return features






if __name__ == "__main__":

    ckpt = "/Users/Jimmy/Desktop/siamese4/checkpoint.59.hdf5"
    print("Executing test...")
    pool = ProcessPoolExecutor(8)
    siamese = SiameseSimilarityDetector(ckpt)
    cap = AsyncCamera(0)
    import time
    futures = []
    while(1):
        ret, frame = cap.read()
        print(siamese.extract_features([frame]))
        print(siamese.similarity(frame, frame))