from unsupervised_models.convolutional_variational_autoencoder import ConvolutionalVariationalAutoencoder

from functools import partial

import os
from itertools import cycle
from keras.callbacks import ModelCheckpoint, TensorBoard

from scipy import misc
import numpy as np


IM_SIZE = 64


def target_input_from_directory(directory):
    return [os.path.join(root, "target.png") for root, _, files in os.walk(directory)
            for _ in files]


def input_from_directory(directory):
    return [os.path.join(root, f) for root, _, files in os.walk(directory)
            for f in files if f != "target.png"]


def chunk(file_iter, batch_size):
    files = list(file_iter)
    return (files[i:i + batch_size] for i in range(0, len(files), batch_size))


def blender_data_gen(directory, batch_size):
    inputs = chunk(input_from_directory(directory), batch_size)
    for batch in cycle(inputs):
        images = (misc.imread(img, mode="RGB") for img in batch)
        # images = (misc.imresize(img, (IM_SIZE, IM_SIZE)) for img in images)
        arrays = map(np.array, images)
        arrays = map(lambda img: (img-128)/128, arrays)
        #arrays = (arr / 255 for arr in arrays)
        # arrays = (np.expand_dims(arr, axis=2) for arr in arrays)
        yield (np.stack(list(arrays), axis=0),None)

if __name__ == '__main__':
    batch_size = 32
    vae = ConvolutionalVariationalAutoencoder(image_dims=(IM_SIZE, IM_SIZE, 3), batch_size=batch_size)
    # vae.compile(optimizer='rmsprop', loss=vae.output_layers[0].losses[0])
    vae.compile(optimizer='rmsprop', loss=None,
        # metrics=[vae.output_layers[0].losses[0]]
        )
    print(vae.summary())

    checkpoint_callback  = ModelCheckpoint("/home/dlrc/Desktop/vae_models/weights.{epoch:02d}.hdf5",
        monitor="custom_variational_layer_1/Mean_2:0",
        verbose=1,
        save_weights_only=False, 
        mode='auto', 
        period=1)
    tensorboard_callback = TensorBoard(log_dir='/home/dlrc/Desktop/vae_models/logs', 
        histogram_freq=0,
        batch_size=32, 
        write_graph=True, 
        write_grads=False, 
        write_images=True, 
        embeddings_freq=1, 
        embeddings_layer_names=["conv2d_transpose_3"], 
        embeddings_metadata=None)

    vae.fit_generator(blender_data_gen("/home/dlrc/Desktop/Img/train", batch_size), steps_per_epoch=10, epochs=3, 
        callbacks=[checkpoint_callback, 
        tensorboard_callback
        ],
        # validation_data=blender_data_gen("/home/dlrc/Desktop/Img/validate", batch_size),
        # validation_steps=4
        )
