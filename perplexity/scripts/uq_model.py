import numpy as np
from keras.models import Model
from keras.layers import Input, Dense
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping
from keras_uncertainty.models import SimpleEnsemble
import time

experiment_specification = {
    "EXPERIMENT_IDENTIFIER": f"Flatfish",
    "BUFFER_SIZE": 200,
    "NUMBER_OF_LAYERS": 2,
    "UNITS_PER_LAYER": 16,
    "LEARNING_RATE": 10e-3,
    "BATCH_SIZE": 4,
    "PATIENCE": 5,
    "MAX_EPOCHS": 10,
    "INPUT_LAYER_SIZE": 3 + 4, # x, y, yaw, 4 thrusters 
    "OUTPUT_LAYER_SIZE": 3, # x, y, yaw accelerations
    "UNCERTAINTY_THRESHOLD": 0.02,
    "NUMBER_OF_ESTIMATORS": 10,
}

class AIOModel():
    def __init__(self, experiment_specification=experiment_specification, p=0.5) -> None:
        self.experiment_specification = experiment_specification
        self.X_train, self.y_train = (
            np.empty((0, 7), float), np.empty((0, 3), float))
        self.weights = np.empty((0, 1), float)
        self.construct_model()

    def construct_model(self):
        def model_fn():
            inp = Input(
                shape=(self.experiment_specification["INPUT_LAYER_SIZE"],))
            x = Dense(
                self.experiment_specification["UNITS_PER_LAYER"], activation="relu")(inp)
            for _ in range(self.experiment_specification["NUMBER_OF_LAYERS"] - 1):
                x = Dense(
                    self.experiment_specification["UNITS_PER_LAYER"], activation="relu")(x)
            mean = Dense(
                self.experiment_specification["OUTPUT_LAYER_SIZE"], activation="linear")(x)
            train_model = Model(inp, mean)
            print(train_model.summary())
            train_model.compile(loss="mse", optimizer=Adam(
                learning_rate=self.experiment_specification["LEARNING_RATE"]))
            return train_model

        self.model = SimpleEnsemble(
            model_fn, num_estimators=self.experiment_specification["NUMBER_OF_ESTIMATORS"])

    def update_own_training_set(self, samples, targets, weights):
        self.X_train = samples
        self.y_train = targets
        self.weights = weights

    def retrain(self, verbose=False):
        """
        Retrain yourself give the own dataset you have.
        """
        early_stop = EarlyStopping(
            monitor='loss', patience=self.experiment_specification["PATIENCE"])
            
        start_time = time.time()
        history = self.model.fit(
            self.X_train,
            self.y_train,
            sample_weight=self.weights,
            verbose=verbose,
            epochs=self.experiment_specification["MAX_EPOCHS"],
            callbacks=[
            early_stop],
            batch_size=self.experiment_specification["BATCH_SIZE"])
        end_time = time.time()
        print(f"Training took {end_time - start_time} seconds.")
        return True

    def predict(self, points):
        """
        Predict on the given set of points, also output uncertainty.
        """
        pred_mean, pred_std = self.model(points)
        return pred_mean, pred_std
