import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("problem 3 test data.csv", index_col=0, parse_dates=True)
val = 1
fig, data_plot= plt.subplots()
# data_plot.boxplot(data[:], labels = data.columns)
for col in data.columns:
    data_unfiltered = list(data[col])
    data_col = []
    for value in data_unfiltered:
        if not pd.isna(value):
            data_col.append(value)
    data_plot.boxplot(data_col, labels = [col], positions=[val], widths=[0.5])
    data_plot.scatter(np.random.normal(val, 0.1, size=len(data_col)).tolist(), data_col, label=col, alpha = 0.5)
    val += 1
data_plot.set_ylabel('Unused Memory from Run')
data_plot.set_title('Problem 3 Algorithm Comparison')
plt.show()