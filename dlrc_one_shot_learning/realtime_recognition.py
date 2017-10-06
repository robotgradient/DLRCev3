import keras
import cv2
import numpy as np

model = keras.applications.mobilenet.MobileNet(input_shape=(224,224,3), alpha=1.0, 
    depth_multiplier=1, dropout=1e-3, include_top=False, weights='imagenet', input_tensor=None, pooling=None)

from scipy import misc

bbox_images = np.load('bbox_images.npy')
cap = cv2.VideoCapture(0)
input("Show me a brick")
_, brick = cap.read()
brick = cv2.resize(brick, (224,224))
brick = brick.reshape(-1)
data = []
brick_predictions =  model.predict(np.expand_dims(brick, 0)).reshape(-1)


for img in bbox_images:
    img = cv2.resize(img, (224,224))

    predictions = model.predict(np.expand_dims(img, 0))
    data.append(img.reshape(-1))


data = np.asarray(data)

mse = np.square((data-brick_predictions), axis=1)
print(mse)



