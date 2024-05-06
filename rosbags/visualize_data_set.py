
import pandas as pd
from sklearn.metrics import r2_score
import csv
import numpy as np

from matplotlib import pyplot as plt

FEATURES = 7

import sys

file_name = sys.argv[1]
train_data = pd.read_csv(f'{file_name}.csv')
train_sample = train_data.iloc[:, :FEATURES].values
train_target = train_data.iloc[:, FEATURES:].values

if False:
    model = AIOModel()
    from uq_model import AIOModel
    from tensorflow.keras import models
    weights = np.array([1 for _ in range(len(train_sample))])
    model.update_own_training_set(train_sample, train_target, weights)
    model.retrain(verbose=True)

    model.model.save_weights('model', verbose=False)
    # model.model.load_weights('model')

if False:
    data = pd.read_csv('long_mission2_for_test_set.csv')
    test_sample = data.iloc[:, :FEATURES].values
    test_target = data.iloc[:, FEATURES:].values
    test_set_size = len(test_sample)
    pred_mean, _ = model.predict(test_sample)

# plot 6 figures in one plot below one another for each of the 6 features
fig, axs = plt.subplots(6, 1, figsize=(10, 10))
for i in range(6):
    axs[i].plot(train_sample[:,i], label='True')



for i in range(3):
    plt.figure()
    plt.plot(train_target[:,i], label='True')
    # plt.plot(pred_mean[:,i], label='Pred')
plt.show()

# MSE = np.sum(np.square(test_target - pred_mean))/test_set_size
# R2 = r2_score(test_target, pred_mean)

# print(MSE, R2)

