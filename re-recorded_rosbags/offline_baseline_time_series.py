from uq_model_time_series import AIOModel
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

OVERFIT = True

model = AIOModel()

train_data = pd.read_csv('long_mission2_smooth30.csv')

# take just first 6 columns
train_data = train_data.iloc[:, :6]
# normalize
train_data = (train_data - train_data.mean())/(train_data.std())
# apply rolling mean to the data
# train_data = train_data.rolling(window=15).mean()
# remove nans
train_data = train_data.dropna()

lenght = train_data.shape[0]

# make the data into time series
# combine 5 consecutive rows into one row

X = []
y = []

for i in range(5, lenght):
    sample1 = train_data.iloc[i-5]
    sample2 = train_data.iloc[i-4]
    sample3 = train_data.iloc[i-3]
    sample4 = train_data.iloc[i-2]
    sample5 = train_data.iloc[i-1]
    target = train_data.iloc[i]

    X.append(np.concatenate((sample1, sample2, sample3, sample4, sample5)))
    y.append(target)

X = np.array(X)
y = np.array(y)


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
# model.retrain(verbose=True)


model.model.load_weights('model_overfit_timeseries')
# model.model.save_weights('model_overfit_timeseries', verbose=False)

# data = pd.read_csv('long_mission_for_test_set_diff_15.csv', skiprows = 2000, nrows=2800)
# data = (data - data.min())/(data.max() - data.min())
# X_test = data.iloc[:, :FEATURES].values
# y_test = data.iloc[:, FEATURES:].values
test_set_size = len(y_test)

pred_mean, _ = model.predict(X_test)

# plot 3 figures in one plot below one another for each of the 3 features
fig, axs = plt.subplots(6, 1, figsize=(10, 10))

for i in range(len(y_test[0])):
    axs[i].plot(y_test[:,i], label='True')
    axs[i].plot(pred_mean[:,i], label='Pred')
plt.show()

MSE = np.sum(np.square(y_test - pred_mean))/test_set_size
R2 = r2_score(y_test, pred_mean)

print(MSE, R2)


