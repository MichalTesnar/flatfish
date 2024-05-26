
import pandas as pd
from sklearn.metrics import r2_score
import csv
import numpy as np

from matplotlib import pyplot as plt

FEATURES = 7

import sys

file_name = sys.argv[1]
# train_data = pd.read_csv(f'{file_name}.csv', skiprows = 2000, nrows=2800)
# train_data = pd.read_csv(f'{file_name}.csv', skiprows = 1500, nrows=5000)
# train_data = pd.read_csv(f'{file_name}.csv', skiprows = 2000, nrows=1000)
train_data = pd.read_csv(f'{file_name}.csv', nrows=5000)
# train_data = pd.read_csv(f'{file_name}.csv')
# normalize the data between 0 and 1

# apply rolling mean to the data
# train_data = train_data.rolling(window=25).mean()

# normalize the data by subtracting the mean and dividing by the standard deviation
train_data = (train_data - train_data.mean())/(train_data.std())

# rename the columns to 01234567
train_data.columns = [str(i) for i in range(10)]

print(train_data.min())
print(train_data.max())

# train_data = (train_data - train_data.min())/(train_data.max() - train_data.min())


train_sample = train_data.iloc[:, :FEATURES].values
train_target = train_data.iloc[:, FEATURES:].values

# if False:
from uq_model import AIOModel
model = AIOModel()
from tensorflow.keras import models
weights = np.array([1 for _ in range(len(train_sample))])
# model.update_own_training_set(train_sample, train_target, weights)
# model.retrain(verbose=True)

# model.model.save_weights('model_overfit', verbose=False)
model.model.load_weights('model_overfit')

if False:
    data = pd.read_csv('long_mission2_for_test_set.csv')
    test_sample = data.iloc[:, :FEATURES].values
    test_target = data.iloc[:, FEATURES:].values
    test_set_size = len(test_sample)
pred_mean, _ = model.predict(train_sample)

# plot 6 figures in one plot below one another for each of the 6 features
# fig, axs = plt.subplots(6, 1, figsize=(20, 10))
# for i in range(6):
#     axs[i].plot(train_sample[:,i], label='True')
# fig.tight_layout()


targets_fig, targets_axs = plt.subplots(3, 1, figsize=(20, 10)) 
for i in range(3):
    targets_axs[i].plot(train_target[:,i], label='True')
    targets_axs[i].plot(pred_mean[:,i], label='Pred')
targets_fig.tight_layout()

plt.show()

train_set_size = len(train_sample)

MSE = np.sum(np.square(train_target - pred_mean))/train_set_size
R2 = r2_score(train_target, pred_mean)

print(MSE, R2)


