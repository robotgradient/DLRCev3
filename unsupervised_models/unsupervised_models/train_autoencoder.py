import tensorflow as tf
from unsupervised_models.vae import VariationalAutoencoder, read_images, get_batch, min_max_scale
IMAGE_PATH = "/Users/Jimmy/Pictures"


data = read_images(IMAGE_PATH, size=(32,32))
print("Read data")
n_samples = int(len(data))
training_epochs = 20
batch_size = 1
display_step = 1

autoencoder = VariationalAutoencoder(
    n_hidden=200,
    optimizer=tf.train.AdamOptimizer(learning_rate = 0.001), debug=False)

from tqdm import tqdm
import numpy as np
X_train =  data
X_test = data[0]

X_train, X_test = min_max_scale(X_train.reshape(data.shape[0], -1), X_test.reshape(1,-1))

print(X_train.min())

get_batch_gen = get_batch(data, batch_size)
for epoch in tqdm(range(training_epochs)):
    avg_cost = 0.
    total_batch = int(n_samples / batch_size)
    # Loop over all batches
    for batch_xs in get_batch(data, batch_size):
        # Fit training using batch data
        cost = autoencoder.partial_fit(batch_xs.reshape(batch_size, -1).astype(np.float64))
        # Compute average loss
        avg_cost += cost / n_samples * batch_size

    # Display logs per epoch step
    if epoch % display_step == 0:
        print("Epoch:", '%d,' % (epoch + 1),
              "Cost:", "{:.9f}".format(avg_cost))

print("Total cost: " + str(autoencoder.calc_total_cost(X_test.reshape(batch_size,-1))))
