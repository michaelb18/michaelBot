from random import random

from tensorflow.python.keras import Model

from examples.base_keras_model import BaseKerasModel
from framework.input_formatter.base_input_formatter import BaseInputFormatter
import tensorflow as tf

from framework.output_formatter.base_output_formatter import BaseOutputFormatter


class AutoencoderModel(BaseKerasModel):
    input_dim = 0
    compressed_layer = None

    def __init__(self, encoder=True, decoder=True, compressed_dim=10):
        super().__init__()
        self.compressed_dim = compressed_dim
        self.decoder = decoder
        self.encoder = encoder

    def create_input_layer(self, input_placeholder: BaseInputFormatter):
        self.input_dim = input_placeholder.get_input_state_dimension()[-1]
        self.inputs = tf.keras.layers.Input(shape=(self.input_dim, ))
        # super().create_input_layer(input_placeholder)

    def create_hidden_layers(self, input_layer=None):
        if input_layer is None:
            input_layer = self.inputs
        hidden_layer = input_layer

        if self.encoder:
            hidden_layer = self.create_encoder(hidden_layer)

        # adding compressed layer
        hidden_layer = self.compressed_layer = tf.keras.layers.Dense(self.compressed_dim,
                                                                     kernel_regularizer=self.kernel_regularizer,
                                                                     activation=self.activation)(hidden_layer)

        if self.decoder:
            hidden_layer = self.create_decoder(hidden_layer)

        self.hidden_layer = hidden_layer
        return self.hidden_layer

    def create_output_layer(self, output_formatter: BaseOutputFormatter, hidden_layer=None):
        # sigmoid/tanh all you want on self.model
        if hidden_layer is None:
            hidden_layer = self.hidden_layer
        self.outputs = tf.keras.layers.Dense(output_formatter.get_model_output_dimension()[0],
                                             activation='linear')(hidden_layer)
        self.model = Model(inputs=self.inputs, outputs=self.outputs)

    def create_encoder(self, hidden_layer):
        decrease_per_layer = int((self.input_dim - self.compressed_dim) / 3)
        left_in_layer = self.input_dim - decrease_per_layer
        while left_in_layer > self.compressed_dim:
            hidden_layer = tf.keras.layers.Dense(left_in_layer, kernel_regularizer=self.kernel_regularizer,
                                                 activation=self.activation)(hidden_layer)

            hidden_layer = tf.keras.layers.Dropout(0.1)(hidden_layer)
            left_in_layer -= decrease_per_layer
        return hidden_layer

    def create_decoder(self, hidden_layer):
        decrease_per_layer = int((self.input_dim - self.compressed_dim) / 3)
        left_in_layer = self.compressed_dim + decrease_per_layer
        while left_in_layer < self.input_dim:
            hidden_layer = tf.keras.layers.Dense(left_in_layer, kernel_regularizer=self.kernel_regularizer,
                                                 activation=self.activation)(hidden_layer)
            hidden_layer = tf.keras.layers.Dropout(0.1)(hidden_layer)
            left_in_layer += decrease_per_layer
        return hidden_layer

    def finalize_model(self, logname=str(int(random() * 1000))):

        loss, loss_weights = self.create_loss()
        self.model.compile(tf.keras.optimizers.Nadam(lr=0.001), loss=loss, loss_weights=loss_weights,
                           metrics=[tf.keras.metrics.mean_absolute_error, tf.keras.metrics.binary_accuracy])
        log_name = './logs/' + logname
        self.logger.info("log_name: " + log_name)
        self.tensorboard = tf.keras.callbacks.TensorBoard(log_dir=log_name,
                                                          histogram_freq=1,
                                                          write_images=False,
                                                          batch_size=1000,
                                                          )
        self.tensorboard.set_model(self.model)
        self.logger.info("Model has been finalized")

    def create_loss(self):
        return 'mean_absolute_error', None
