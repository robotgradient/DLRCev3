import os
import sys
import os
from pathlib import Path

from keras.callbacks import ModelCheckpoint, TensorBoard
from keras.optimizers import RMSprop
from scipy import misc
import numpy as np

from duckdata import keras_data_gen
from unsupervised_models.cvae import ConvolutionalVariationalAutoencoder

IM_SIZE = 64

LATENT_DIMS = int(sys.argv[1])
RUN_NAME = 'latent_dims_' + str(LATENT_DIMS)
BATCH_SIZE = 32

N_EPOCHS = 200
LEARNING_RATE = 0.01

# PREFIX = Path.home()
# PREFIX = PREFIX / "dlrc"
PREFIX = Path("/")

vae = ConvolutionalVariationalAutoencoder(
    image_dims=(IM_SIZE, IM_SIZE, 3),
    batch_size=BATCH_SIZE,
    latent_dim=LATENT_DIMS,
    filters=32,
    x_std=0.1,
    intermediate_dim=100)

vae.compile(
    optimizer=RMSprop(lr=LEARNING_RATE, clipvalue=0.2),
    loss=None,)
print(vae.summary())

weights_dir = PREFIX / 'weights' / RUN_NAME
os.makedirs(weights_dir, exist_ok=True)
checkpoint_callback = ModelCheckpoint(
    str(weights_dir / "weights.{epoch:02d}.hdf5"),
    verbose=1,
    save_weights_only=False,
    mode='auto',
    period=10)

tb_dir = PREFIX / 'tensorboard-logdir' / RUN_NAME
os.makedirs(tb_dir, exist_ok=True)
tensorboard_callback = TensorBoard(
    log_dir=str(tb_dir),
    histogram_freq=0,
    batch_size=BATCH_SIZE,
    write_graph=True,
    write_grads=False,
    write_images=True,
    embeddings_freq=10,
    embeddings_layer_names=["conv2d_transpose_3"],
    embeddings_metadata=None)

data_dir = PREFIX / "data"
vae.fit_generator(
    keras_data_gen(data_dir, ("legos", "Data", "rendered2", "2x2"), BATCH_SIZE),
    callbacks=[checkpoint_callback, tensorboard_callback],
    # this is approx dataset size / batch_size
    steps_per_epoch=770,
    epochs=N_EPOCHS)
