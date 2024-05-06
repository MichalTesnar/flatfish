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
import csv

from matplotlib import pyplot as plt

FEATURES = 7


model = AIOModel()

train_data = pd.read_csv('long_mission_for_test_set.csv')
train_sample = train_data.iloc[:, :FEATURES].values
train_target = train_data.iloc[:, FEATURES:].values

weights = np.array([1 for _ in range(len(train_sample))])
model.update_own_training_set(train_sample, train_target, weights)
model.retrain(verbose=True)

model.model.save_weights('model', verbose=False)
# model.model.load_weights('model')

data = pd.read_csv('long_mission2_for_test_set.csv')
test_sample = data.iloc[:, :FEATURES].values
test_target = data.iloc[:, FEATURES:].values
test_set_size = len(test_sample)



pred_mean, _ = model.predict(test_sample)

for i in range(3):
    plt.figure()
    plt.plot(test_target[:,i], label='True')
    plt.plot(pred_mean[:,i], label='Pred')
plt.show()

MSE = np.sum(np.square(test_target - pred_mean))/test_set_size
R2 = r2_score(test_target, pred_mean)

print(MSE, R2)


