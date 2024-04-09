import numpy as np
from keras.models import Model
from keras.layers import Input, Dense
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping
from keras_uncertainty.models import SimpleEnsemble

experiment_specification = {
    "EXPERIMENT_IDENTIFIER": f"Flatfish",
    "BUFFER_SIZE": 100,
    "MODEL_MODE": "THRESHOLD",
    "NUMBER_OF_LAYERS": 2,
    "UNITS_PER_LAYER": 16,
    "LEARNING_RATE": 0.001,
    "BATCH_SIZE": 2,
    "PATIENCE": 3,
    "MAX_EPOCHS": 100,
    "ACCEPT_PROBABILITY": 0.7,
    "INPUT_LAYER_SIZE": 10,
    "OUTPUT_LAYER_SIZE": 6,
    "UNCERTAINTY_THRESHOLD": 0.02,
    "NUMBER_OF_ESTIMATORS": 10
}

class AIOModel():
    def __init__(self, training_set, experiment_specification=experiment_specification, p=0.5) -> None:
        self.experiment_specification = experiment_specification
        self.X_train, self.y_train = training_set
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

    def update_own_training_set(self, new_point):
        """"
        Update the training set with the new incoming points. Return false, as long as you did not update, then you keep being fed a new item.
        """

        new_X, new_y = new_point

        ################## BASELINES ##################
        if self.experiment_specification["MODEL_MODE"] == "FIFO":
            self.X_train = self.X_train[1:]
            self.y_train = self.y_train[1:]
            self.X_train = np.concatenate(
                [self.X_train, new_X.reshape(-1, 1).T])
            self.y_train = np.concatenate(
                [self.y_train, new_y.reshape(-1, 1).T])
            return True

        # First in, random out
        elif self.experiment_specification["MODEL_MODE"] == "FIRO":
            # Replace random point in the dataset
            random_index = np.random.choice(self.X_train.shape[0])
            self.X_train[random_index] = new_X
            self.y_train[random_index] = new_y
            # Shuffle the rest of the set
            shuffling_indices = np.arange(self.X_train.shape[0])
            np.random.shuffle(shuffling_indices)
            self.X_train = self.X_train[shuffling_indices]
            self.y_train = self.y_train[shuffling_indices]
            return True

        # Random in, random out
        elif self.experiment_specification["MODEL_MODE"] == "RIRO":
            # Only accept with probability 'p'
            a = np.random.rand()
            print(a)
            if a > self.experiment_specification["ACCEPT_PROBABILITY"]:
                return False
            # Replace random point
            random_index = np.random.choice(self.X_train.shape[0])
            self.X_train[random_index] = new_X
            self.y_train[random_index] = new_y
            # Shuffle the rest of the set
            shuffling_indices = np.arange(self.X_train.shape[0])
            np.random.shuffle(shuffling_indices)
            self.X_train = self.X_train[shuffling_indices]
            self.y_train = self.y_train[shuffling_indices]
            return True

        ################## UQ METHODS ##################
        elif self.experiment_specification["MODEL_MODE"] == "GREEDY":
            # obtain uncertainties on the training set and on the new point
            _, train_set_stds = self.predict(self.X_train)
            _, new_point_std = self.predict(new_X.reshape(1, -1))
            # reject if the uncertainty of the incoming point is lower than the minimum uncertainty in your training set
            train_set_stds_means = np.mean(train_set_stds, axis=1)
            if np.min(train_set_stds_means) > np.mean(new_point_std):
                return False
            # otherwise replace the most certain point with the new point
            idx = np.argmin(train_set_stds_means)
            self.X_train[idx] = new_X
            self.y_train[idx] = new_y
            return True

        elif self.experiment_specification["MODEL_MODE"] == "THRESHOLD":
            # obtain uncertainty on the new point
            _, new_point_std = self.predict(new_X.reshape(1, -1))
            # if the uncertainty is too low, just directly reject the point, it is not interesting enough
            if np.mean(new_point_std) < self.experiment_specification["UNCERTAINTY_THRESHOLD"]:
                return False
            # otherwise replace a random old point with it
            random_index = np.random.choice(self.X_train.shape[0])
            self.X_train[random_index] = new_X
            self.y_train[random_index] = new_y
            # Shuffle the rest of the set
            shuffling_indices = np.arange(self.X_train.shape[0])
            np.random.shuffle(shuffling_indices)
            self.X_train = self.X_train[shuffling_indices]
            self.y_train = self.y_train[shuffling_indices]
            return True

        elif self.experiment_specification["MODEL_MODE"] == "THRESHOLD_GREEDY":
            # obtain uncertainty on the new point
            _, new_point_std = self.predict(new_X.reshape(1, -1))
            # if the uncertainty is too low, just directly reject the point
            if np.mean(new_point_std) < self.experiment_specification["UNCERTAINTY_THRESHOLD"]:
                return False
            # obtain uncertainties on the training set
            _, train_set_stds = self.predict(self.X_train)
            # reject if the uncertainty of the incoming point is lower than the minimum uncertainty in your training set
            train_set_stds_means = np.mean(train_set_stds, axis=1)
            if np.min(train_set_stds_means) > np.mean(new_point_std):
                return False
            # otherwise replace the most certain point with the new point
            idx = np.argmin(train_set_stds_means)
            self.X_train[idx] = new_X
            self.y_train[idx] = new_y
            return True

        raise NotImplemented("This method is not implemented.")

    def retrain(self, verbose=False):
        """
        Retrain yourself give the own dataset you have.
        """
        early_stop = EarlyStopping(
            monitor='loss', patience=self.experiment_specification["PATIENCE"])
        history = self.model.fit(self.X_train, self.y_train, verbose=verbose, epochs=self.experiment_specification["MAX_EPOCHS"], callbacks=[
            early_stop], batch_size=self.experiment_specification["BATCH_SIZE"])
        return history

    def predict(self, points):
        """
        Predict on the given set of points, also output uncertainty.
        """
        pred_mean, pred_std = self.model(points)
        return pred_mean, pred_std


class AIOModelTuning(AIOModel):
    def __init__(self, training_set, experiment_specification, model) -> None:
        # assert experiment_specification["MODEL_MODE"] in ["FIFO", "FIRO", "RIRO", "SPACE_HEURISTIC",
        #                                                  "TIME_HEURISTIC", "GREEDY", "THRESHOLD", "THRESHOLD_GREEDY", "OFFLINE"], "Mode does not exist."
        self.experiment_specification = experiment_specification
        self.X_train, self.y_train = training_set
        self.model = model
