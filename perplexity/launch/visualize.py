import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

paths = []
for file in ["evaluation_metrics-normal.csv", "evaluation_metrics-priority.csv", "evaluation_metrics-uncertainty.csv", "evaluation_metrics.csv"]:
# for file in ["evaluation_metrics-normal.csv"]:
    if os.path.exists(file):
        paths.append(file)


for csv_path in paths:
    data = pd.read_csv(csv_path)
    column_1_name = data.columns[0]
    column_2_name = data.columns[1]
    mse = data[column_1_name]
    r2 = data[column_2_name]

    mse = np.minimum(mse, 0.1)
    r2 = np.maximum(r2, -1)


    # Create the plot
    try:
        plt.plot(r2, label=csv_path.split("-")[1].split(".")[0])
    except:
        plt.plot(r2, label="normal")

# Add legend
plt.legend()
plt.xlabel("Epoch", fontsize=14)
plt.ylabel("R2", fontsize=14)
plt.title("R2 vs Epoch", fontsize=14)

# Display the plot
plt.grid(True)
plt.show()
