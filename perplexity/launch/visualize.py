import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

import sys

paths = []

if len(sys.argv) > 1:
    prefix = sys.argv[1]

for file in [f"{prefix}/evaluation_metrics-normal.csv", f"{prefix}/evaluation_metrics-priority.csv", f"{prefix}/evaluation_metrics-uncertainty.csv", f"{prefix}/evaluation_metrics.csv"]:
# for file in [f"evaluation_metrics-normal.csv", f"evaluation_metrics-priority.csv", f"evaluation_metrics-uncertainty.csv", f"evaluation_metrics.csv"]:
# for file in ["evaluation_metrics-normal.csv"]:
    if os.path.exists(file):
        paths.append(file)

FONT_SIZE = 15

plt.figure(figsize=(10, 5))

for csv_path in paths:
    data = pd.read_csv(csv_path)
    column_1_name = data.columns[0]
    column_2_name = data.columns[1]
    mse = data[column_1_name]
    r2 = data[column_2_name]

    mse = np.minimum(mse, 0.1)
    r2 = np.maximum(r2, -0.5)


    if "normal" in csv_path:
        name = "Uniform"
    elif "priority" in csv_path:
        name = "Priority"
    elif "uncertainty" in csv_path:
        name = "Uncertainty"

    plt.plot(r2, label=name)

# Add legend
plt.legend(fontsize=FONT_SIZE)
plt.xlabel("Samplings of the buffer", fontsize=FONT_SIZE)
plt.ylabel(r"$R^2$", fontsize=FONT_SIZE)
# plt.title("", fontsize=FONT_SIZE)
plt.tight_layout()

# Display the plot
plt.grid(True)
plt.savefig(f"{prefix}/r2.pdf")
plt.show()
