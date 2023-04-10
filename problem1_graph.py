import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("problem1_data.csv", index_col=0, parse_dates=True)
val = 1
fig, data_plot= plt.subplots()
data_plot.boxplot(data[:], labels = data.columns)
for col in data.columns:
    data_col = list(data[col])
    data_plot.scatter(np.random.normal(val, 0.1, size=len(data_col)).tolist(), data_col, label=col, alpha = 0.5)
    val += 1
data_plot.set_ylabel('Fraction of Optimal')
data_plot.set_title('Subproblem 1 Algorithm Comparison')
plt.show()