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

latent_dim = int(sys.argv[1])
batch_size = 16

epochs = 1000

import os
from pathlib import Path
PREFIX = Path.home()
PREFIX = Path("/")
# PREFIX = PREFIX / "dlrc"

vae = ConvolutionalVariationalAutoencoder(
    image_dims=(IM_SIZE, IM_SIZE, 3), batch_size=batch_size, latent_dim=latent_dim, filters=32)

vae.compile(
    optimizer='rmsprop',
    loss=None,)
print(vae.summary())

weights_dir = PREFIX / ('weights/latent_dims_' + str(latent_dim))
os.makedirs(weights_dir, exist_ok=True)
checkpoint_callback = ModelCheckpoint(
    str(weights_dir / ("/weights.{epoch:02d}.hdf5")),
    verbose=1,
    save_weights_only=False,
    mode='auto',
    period=10)

tb_dir = PREFIX / ('tensorboard-logdir/latent_dims_' + str(latent_dim))
os.makedirs(tb_dir, exist_ok=True)
tensorboard_callback = TensorBoard(
    log_dir=str(tb_dir),
    histogram_freq=0,
    batch_size=batch_size,
    write_graph=True,
    write_grads=False,
    write_images=True,
    embeddings_freq=5,
    embeddings_layer_names=["conv2d_transpose_3"],
    embeddings_metadata=None)

data_dir = PREFIX / "data"
vae.fit_generator(
    blender_data_gen(data_dir, batch_size),
    steps_per_epoch=3500,
    epochs=epochs,
    callbacks=[checkpoint_callback, tensorboard_callback],)
