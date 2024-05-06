import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

paths = ["evaluation_metrics-normal.csv", "evaluation_metrics-priority.csv", "evaluation_metrics-uncertainty.csv"]
# paths = ["evaluation_metrics-normal.csv", "evaluation_metrics-priority.csv"]
# paths = ["evaluation_metrics-normal.csv", "evaluation_metrics-uncertainty.csv"]


for csv_path in paths:
    data = pd.read_csv(csv_path)
    column_1_name = data.columns[0]
    column_2_name = data.columns[1]
    mse = data[column_1_name]
    r2 = data[column_2_name]

    mse = np.minimum(mse, 0.2)


    # Create the plot
    plt.plot(mse, label=csv_path.split("-")[1].split(".")[0])

# Add legend
plt.legend()

# Display the plot
plt.grid(True)
plt.show()
