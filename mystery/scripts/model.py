import tensorflow as tf
from tensorflow.keras import layers, models

class SimpleRegressionModel:
    def __init__(self, input_shape):
        self.input_shape = input_shape
        self.model = self._build_model()

    def _build_model(self):
        model = models.Sequential([
            layers.Dense(64, activation='relu', input_shape=(self.input_shape,)),
            layers.Dense(32, activation='relu'),
            layers.Dense(1)  # Output layer with 1 neuron for regression
        ])
        model.compile(optimizer='adam', loss='mse', metrics=['mae']) # Using Mean Squared Error loss for regression
        return model

    def train(self, X_train, y_train, epochs=10, batch_size=32, validation_data=None):
        history = self.model.fit(X_train, y_train, epochs=epochs, batch_size=batch_size, validation_data=validation_data, verbose=0)
        return history

    def evaluate(self, X_test, y_test):
        return self.model.evaluate(X_test, y_test)

    def predict(self, X):
        return self.model.predict(X)