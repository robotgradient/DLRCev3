'''This script demonstrates how to build a variational autoencoder
with Keras and deconvolution layers.
Reference: "Auto-Encoding Variational Bayes" https://arxiv.org/abs/1312.6114
'''
import numpy as np

from keras.layers import Input, Dense, Lambda, Flatten, Reshape, Layer
from keras.layers import Conv2D, Conv2DTranspose
from keras.layers.normalization import BatchNormalization
from keras.models import Model
from keras import backend as K
from keras import metrics
from keras.datasets import mnist
import tensorflow as tf

# Custom loss layer
class CustomVariationalLayer(Layer):
    def __init__(self, z_log_var, z_mean, img_rows, img_cols, **kwargs):
        self.is_placeholder = True
        self.z_mean = z_mean
        self.z_log_var = z_log_var
        self.img_rows = img_rows
        self.img_cols =img_cols
        super(CustomVariationalLayer, self).__init__(**kwargs)

    def vae_loss(self, x, x_decoded_mean_squash):
        x = K.flatten(x)
        x_decoded_mean_squash = K.flatten(x_decoded_mean_squash)
        xent_loss = self.img_rows * self.img_cols * metrics.binary_crossentropy(x, x_decoded_mean_squash)
        tf.summary.scalar("xent_loss", xent_loss)
        kl_loss = - 0.5 * K.mean(1 + self.z_log_var - K.square(self.z_mean) - K.exp(self.z_log_var), axis=-1)
        tf.summary.scalar("KL_divergence", kl_loss)
        loss =  K.mean(xent_loss + kl_loss)
        tf.summary.scalar("custom_variational_layer", loss)
        return loss

    def call(self, inputs):
        x = inputs[0]
        x_decoded_mean_squash = inputs[1]
        loss = self.vae_loss(x, x_decoded_mean_squash)
        self.add_loss(loss, inputs=inputs)
        # We don't use this output.
        return x



class ConvolutionalVariationalAutoencoder(Model):


    def __init__(self, latent_dim=2, intermediate_dim=120, epsilon_std=1.0,
                image_dims=(28, 28, 1), filters = 64, kernel_size=3, batch_size=1):

        img_rows, img_cols, img_chns = image_dims

        bnorm = BatchNormalization

        if K.image_data_format() == 'channels_first':
            original_img_size = (img_chns, img_rows, img_cols)

        else:
            original_img_size = (img_rows, img_cols, img_chns)

        x = Input(shape=original_img_size)
        conv_1 = Conv2D(img_chns,
                        kernel_size=(2, 2),
                        padding='same', activation='relu')(x)
        bnorm_1 = bnorm()(conv_1)
        conv_2 = Conv2D(filters,
                        kernel_size=(2, 2),
                        padding='same', activation='relu',
                        strides=(2, 2))(bnorm_1)
        bnorm_2 = bnorm()(conv_2)
        conv_3 = Conv2D(filters,
                        kernel_size=kernel_size,
                        padding='same', activation='relu',
                        strides=2)(bnorm_2)
        bnorm_3 = bnorm()(conv_3)
        conv_4 = Conv2D(filters,
                        kernel_size=kernel_size,
                        padding='same', activation='relu',
                        strides=2)(bnorm_3)
        flat = Flatten()(conv_4)
        hidden = Dense(intermediate_dim, activation='relu', kernel_initializer='glorot_uniform')(flat)
        bnorm_hidden = bnorm()(hidden)

        z_mean = Dense(latent_dim, kernel_initializer='glorot_uniform')(bnorm_hidden)
        z_log_var = Dense(latent_dim, kernel_initializer='glorot_uniform')(hidden)

        self.epsilon = K.random_normal(shape=(K.shape(z_mean)[0], latent_dim),
                                  mean=0., stddev=epsilon_std)
        self.sampling = z_mean + K.exp(z_log_var) * self.epsilon

        def sampling(args):
            z_mean, z_log_var = args
            epsilon = K.random_normal(shape=(K.shape(z_mean)[0], latent_dim),
                                      mean=0., stddev=epsilon_std)
            return z_mean + K.exp(z_log_var) * epsilon

        # note that "output_shape" isn't necessary with the TensorFlow backend
        # so you could write `Lambda(sampling)([z_mean, z_log_var])`
        z = Lambda(sampling, output_shape=(latent_dim,))([z_mean, z_log_var])

        upsample = 8
        # we instantiate these layers separately so as to reuse them later

        decoder_hid = Dense(intermediate_dim, activation='relu', kernel_initializer='glorot_uniform')
        decoder_upsample = Dense(filters * upsample * upsample, activation='relu', kernel_initializer='glorot_uniform')

        if K.image_data_format() == 'channels_first':
            output_shape = (batch_size, filters, upsample, upsample)
        else:
            output_shape = (batch_size, upsample, upsample, filters)

        decoder_reshape = Reshape(output_shape[1:])
        decoder_deconv_1 = Conv2DTranspose(filters,
                                           kernel_size=kernel_size,
                                           padding='same',
                                           strides=1,
                                           activation='relu')
        decoder_deconv_2 = Conv2DTranspose(filters,
                                           kernel_size=kernel_size,
                                           padding='same',
                                           strides=1,
                                           activation='relu')
        decoder_deconv_3_upsamp = Conv2DTranspose(filters,
                                                  kernel_size=(3, 3),
                                                  strides=(2, 2),
                                                  padding='valid',
                                                  activation='relu')
        decoder_mean_squash = Conv2D(img_chns,
                                     kernel_size=2,
                                     padding='valid',
                                     activation='sigmoid')

        hid_decoded = bnorm()(decoder_hid(z))
        up_decoded = bnorm()(decoder_upsample(hid_decoded))
        reshape_decoded = bnorm()(decoder_reshape(up_decoded))
        deconv_1_decoded = bnorm()(decoder_deconv_1(reshape_decoded))
        deconv_2_decoded = bnorm()(decoder_deconv_2(deconv_1_decoded))
        x_decoded_relu = bnorm()(decoder_deconv_3_upsamp(deconv_2_decoded))
        x_decoded_mean_squash = decoder_mean_squash(x_decoded_relu)

        y = CustomVariationalLayer(z_log_var=z_log_var,
            z_mean=z_mean,
            img_cols=img_cols,
            img_rows=img_rows)([x, x_decoded_mean_squash],
            # name="custom_variational_layer"
            )

        # build a model to project inputs on the latent space
        self.encoder =  Model(x, z_mean)

        # build a model to project inputs on the latent space


        # build a digit generator that can sample from the learned distribution
        decoder_input = Input(shape=(latent_dim,))
        _hid_decoded = decoder_hid(decoder_input)
        _up_decoded = decoder_upsample(_hid_decoded)
        _reshape_decoded = decoder_reshape(_up_decoded)
        _deconv_1_decoded = decoder_deconv_1(_reshape_decoded)
        _deconv_2_decoded = decoder_deconv_2(_deconv_1_decoded)
        _x_decoded_relu = decoder_deconv_3_upsamp(_deconv_2_decoded)
        _x_decoded_mean_squash = decoder_mean_squash(_x_decoded_relu)
        self.generator = Model(decoder_input, _x_decoded_mean_squash)

        super(ConvolutionalVariationalAutoencoder, self).__init__(x, y)




if __name__ == '__main__':
    from scipy.stats import norm
    import matplotlib.pyplot as plt

    image_dims = (28, 28, 1)

    vae = ConvolutionalVariationalAutoencoder(image_dims=image_dims)
    # train the VAE on MNIST digits
    (x_train, _), (x_test, y_test) = mnist.load_data()
    vae.compile('rmsprop', None)


    x_train = x_train.astype('float32') / 255.
    x_train = x_train.reshape((x_train.shape[0],) + image_dims)
    x_test = x_test.astype('float32') / 255.
    x_test = x_test.reshape((x_test.shape[0],) + image_dims)

    print('x_train.shape:', x_train.shape)

    vae.fit(x_train,
            shuffle=True,
            epochs=5,
            batch_size=10,
            validation_data=(x_test, None))

    # display a 2D manifold of the digits
    n = 15  # figure with 15x15 digits
    digit_size = 28
    figure = np.zeros((digit_size * n, digit_size * n))
    # linearly spaced coordinates on the unit square were transformed through the inverse CDF (ppf) of the Gaussian
    # to produce values of the latent variables z, since the prior of the latent space is Gaussian
    grid_x = norm.ppf(np.linspace(0.05, 0.95, n))
    grid_y = norm.ppf(np.linspace(0.05, 0.95, n))

    batch_size = 16

    for i, yi in enumerate(grid_x):
        for j, xi in enumerate(grid_y):
            z_sample = np.array([[xi, yi]])
            z_sample = np.tile(z_sample, batch_size).reshape(batch_size, 2)
            x_decoded = vae.generator.predict(z_sample, batch_size=batch_size)
            digit = x_decoded[0].reshape(digit_size, digit_size)
            figure[i * digit_size: (i + 1) * digit_size,
                   j * digit_size: (j + 1) * digit_size] = digit

    plt.figure(figsize=(10, 10))
    plt.imshow(figure, cmap='Greys_r')
    plt.show()
