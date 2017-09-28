from functools import partial

import os
import sys

from itertools import cycle
from keras.callbacks import ModelCheckpoint, TensorBoard

from scipy import misc
import numpy as np

from duckdata.blender import blender_data_gen
from unsupervised_models.cvae import ConvolutionalVariationalAutoencoder

IM_SIZE = 64

if __name__ == '__main__':

    latent_dim = int(sys.argv[1])
    batch_size = 16

    epochs = 1000

    vae = ConvolutionalVariationalAutoencoder(
        image_dims=(IM_SIZE, IM_SIZE, 3), batch_size=batch_size, latent_dim=latent_dim, filters=32)

    # vae.compile(optimizer='rmsprop', loss=vae.output_layers[0].losses[0])
    vae.compile(
        optimizer='rmsprop',
        loss=None,
        # metrics=[vae.output_layers[0].losses[0]]
    )
    print(vae.summary())

    os.makedirs('/data/weights/latent_dims_' + str(latent_dim), exist_ok=True)
    checkpoint_callback = ModelCheckpoint(
        "/data/weights/latent_dims_" + str(latent_dim) +
        "/weights.{epoch:02d}.hdf5",
        # monitor="custom_variational_layer_1/Mean_2:0",
        verbose=1,
        save_weights_only=False,
        mode='auto',
        period=10)

    os.makedirs('/data/tensorboard-logdir/latent_dims_' + str(latent_dim), exist_ok=True)
    tensorboard_callback = TensorBoard(
        log_dir='/data/tensorboard-logdir/latent_dims_' + str(latent_dim),
        histogram_freq=0,
        batch_size=batch_size,
        write_graph=True,
        write_grads=False,
        write_images=True,
        embeddings_freq=5,
        embeddings_layer_names=["conv2d_transpose_3"],
        embeddings_metadata=None)

    vae.fit_generator(
        blender_data_gen("/data/Img/train", batch_size),
        steps_per_epoch=3500,
        epochs=epochs,
        callbacks=[checkpoint_callback, tensorboard_callback],
        # validation_data=blender_data_gen("/home/dlrc/Desktop/Img/validate", batch_size),
        # validation_steps=4
    )
