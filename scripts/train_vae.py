from functools import partial

import os
from itertools import cycle
from keras.callbacks import ModelCheckpoint, TensorBoard

from scipy import misc
import numpy as np

from duckdata.blender import blender_data_gen
from unsupervised_models.cvae import ConvolutionalVariationalAutoencoder


IM_SIZE = 64

if __name__ == '__main__':
    batch_size = 32
    vae = ConvolutionalVariationalAutoencoder(
        image_dims=(IM_SIZE, IM_SIZE, 3), batch_size=batch_size)
    # vae.compile(optimizer='rmsprop', loss=vae.output_layers[0].losses[0])
    vae.compile(optimizer='rmsprop', loss=None,
                # metrics=[vae.output_layers[0].losses[0]]
                )
    print(vae.summary())

    checkpoint_callback = ModelCheckpoint("/home/dlrc/Desktop/vae_models/weights.{epoch:02d}.hdf5",
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
