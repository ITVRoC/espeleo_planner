#! /usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, h

flattest_data = [0.06,0.05,0.06,0.06,0.05,0.05,0.06,0.05,0.06,0.07,0.05,0.06,0.06,0.06,0.06,0.06,0.05,0.06,0.06,0.05,0.06,0.06,0.06,0.05,0.05,0.05,0.06,0.05,0.05,0.05]
op_data = [31.07,34.48,33.50,36.00,31.67,31.81,36.58,32.56,35.98,31.07,35.37,32.67,35.08,36.77,33.89,32.27,31.44,30.44,35.51,31.66,37.02,36.05,35.05,32.07,34.91,35.69,33.18,33.99,32.40,35.66]
py_data = [101.71,103.15,103.80,111.37,104.24,110.52,110.03,110.23,103.43,111.18,103.23,101.21,106.07,109.13,108.47,108.10,110.86,104.76,108.74,112.79,104.36,105.93,108.97,110.62,113.90,107.54,106.86,104.20,113.97,115.73]
op_normal_data = [13.99,13.41,13.77,13.64,13.34,13.55,13.34,13.88,13.70,17.70,15.80,13.68,13.72,13.84,15.77,13.68,13.44,13.53,13.71,17.00,15.92,14.66,14.30,14.30,13.72,14.42,13.96,13.56,14.05,17.65]
py_normal_data = [24.42,25.37,26.33,25.35,27.79,25.80,25.72,24.78,24.68,24.90,25.83,24.49,26.95,24.65,28.78,26.38,24.97,24.50,24.86,24.86,23.92,24.65,26.16,24.80,25.42,24.97,28.18,25.07,25.35,31.03]

flattest_mean, flattest_ci = mean_confidence_interval(flattest_data)
op_mean, op_ci = mean_confidence_interval(op_data)
py_mean, py_ci = mean_confidence_interval(py_data)
op_normal_mean, op_normal_ci = mean_confidence_interval(op_normal_data)
py_normal_mean, py_normal_ci = mean_confidence_interval(py_normal_data)

group_mean = [flattest_mean, op_mean, py_mean, op_normal_mean, py_normal_mean]
group_ci = [flattest_ci, op_ci, py_ci, op_normal_ci, py_normal_ci]

bar_width = 0.40

fig, ax = plt.subplots()

labels_x = ['Normal angle', 'Optimization', 'Pybullet', 'Optimization\n(selective)', 'Pybullet\n(selective)']

ax.grid()
ax.set_axisbelow(True)

rects1 = ax.bar(labels_x, group_mean, width=bar_width, label='Errors', yerr=group_ci, color=['#aaf2a9', (0.011, 0.317, 0.988), (0.937, 0.023, 0.317), (0.011, 0.317, 0.988), (0.937, 0.023, 0.317)])

# patterns = ('+', 'x', '\\', '*', 'o', 'O', '.')
# for bar, pattern in zip(rects1, patterns):
#     bar.set_hatch(pattern)

ax.set_ylabel('Processing time in seconds (0.95 confidence)')
ax.set_title('Time comparison of angle estimation models (22m rugged path)', fontweight='bold')

plt.xlabel('Terrain interaction models', fontweight='bold')

def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for idx in xrange(len(rects)):
    	rect = rects[idx]
        height = rect.get_height()
        ax.annotate('{:.2f} ({:.2f})'.format(group_mean[idx], group_ci[idx]),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 4),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', 
                    va='bottom',
                    fontsize=7)

autolabel(rects1)

colors = ['#aaf2a9', (0.011, 0.317, 0.988), (0.937, 0.023, 0.317)]
labels = ["Normal angle only", "Optimization", "Pybullet"]

f = lambda m,c: plt.plot([],[],marker=m, color=c, ls="none")[0]
handles = [f("s", colors[i]) for i in range(len(colors))]
legend = plt.legend(handles, labels, loc=1, framealpha=1, frameon=True)

plt.tight_layout()
plt.show()
