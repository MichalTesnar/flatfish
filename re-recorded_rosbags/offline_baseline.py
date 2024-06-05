from uq_model import AIOModel
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import EvaluationMetrics, ModelWeights, KerasReadyTrainingData
from shutil import rmtree
import numpy as np
import pandas as pd
from sklearn.metrics import r2_score
from sklearn.model_selection import train_test_split
import csv

from matplotlib import pyplot as plt

FEATURES = 7
OVERFIT = True


model = AIOModel()

train_data = pd.read_csv('long_mission2_smooth30.csv', nrows=5000)
# data should be already normalized
# normalize onto [0, 1]
# train_data = (train_data - train_data.min())/(train_data.max() - train_data.min())
train_data = (train_data - train_data.mean())/(train_data.std())
# train_data = train_data.rolling(window=15).mean()
# remove nans
train_data = train_data.dropna()

# X = train_data.iloc[:, :FEATURES].values
# y = train_data.iloc[:, FEATURES:].values

y_cols = [3, 4, 5, 6]
y = train_data.iloc[:, y_cols].values

X_cols = [0, 1, 2, 7, 8, 9]
X = train_data.iloc[:, X_cols].values

if OVERFIT:
    X_train = X
    y_train = y
    X_test = X
    y_test = y
else:
    # split for trainin and testing 70/30
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

weights = np.array([1 for _ in range(len(X_train))])
model.update_own_training_set(X_train, y_train, weights)
model.retrain(verbose=True)


# model.model.load_weights('model_overfit_smooth30_switch')
# model.model.save_weights('model_overfit_test_64_units', verbose=False)

# data = pd.read_csv('long_mission_for_test_set_diff_15.csv', skiprows = 2000, nrows=2800)
# data = (data - data.min())/(data.max() - data.min())
# X_test = data.iloc[:, :FEATURES].values
# y_test = data.iloc[:, FEATURES:].values
test_set_size = len(y_test)

pred_mean, _ = model.predict(X_test)

# plot 3 figures in one plot below one another for each of the 3 features
fig, axs = plt.subplots(4, 1, figsize=(10, 10))

for i in range(3):
    axs[i].plot(y_test[:,i], label='True')
    axs[i].plot(pred_mean[:,i], label='Pred')
plt.show()

MSE = np.sum(np.square(y_test - pred_mean))/test_set_size
R2 = r2_score(y_test, pred_mean)

print(MSE, R2)


