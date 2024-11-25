#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from scipy.interpolate import interp1d

# Density
x1 = np.array([20, 17, 15.5, 9, 6.5, 5.0, 26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 23.425, 16.925, 13.425, 12.425, 31.325, 28.325, 26.825, 20.325, 16.825, 15.825])
y1 = np.array([1.35222731338367, 2.47646925851325, 3.33526836621588, 5.58810742426104, 5.46064609558763, 5.86311441656544, 0.517028090411402, 1.45443432642842, 2.49033371344119, 5.47483497647197, 5.36484528689311, 5.84636266108953, 0.278399740990755, 0.432399394343583, 1.6453990606665, 4.07780812040671, 4.85390764052234, 5.47782404061971, 0.039771391570108, 0, 0.400232203945905, 3.88902070742492, 4.45637801, 4.74074679968005])
# x1 = np.array([26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 23.425, 16.925, 13.425, 12.425, 31.325, 28.325, 26.825, 20.325, 16.825, 15.825])
# y1 = np.array([0.517028090411402, 1.45443432642842, 2.49033371344119, 5.47483497647197, 5.36484528689311, 5.84636266108953, 0.278399740990755, 0.432399394343583, 1.6453990606665, 4.07780812040671, 4.85390764052234, 5.47782404061971, 0.039771391570108, 0, 0.400232203945905, 3.88902070742492, 4.45637801, 4.74074679968005])
# Spread
x2 = np.array([20, 17, 15.5, 9, 6.5, 5.0, 26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 16.925, 13.425, 12.425, 26.825, 20.325, 16.825, 15.825])
y2 = np.array([0.281283563801308, 0.241350086759652, 0.217878870730697, 0.152245514068486, 0.140857950339467, 0.109420850847834, 0.359859924939888, 0.38667445578246, 0.381027503787732, 0.268466430421299, 0.164235135886524, 0.243940667503406, 0.399578809518845, 0.377679802672095, 0.26702124311395, 0.235519243495821, 0.243174220814384, 0.385417100628619, 0.383827369892889, 0.291688957711852, 0.32621443797731])

min_x1 = np.min(x1)
min_x2 = np.min(x2)
min_x = int(np.min([min_x1, min_x2])) - 2

max_x1 = np.max(x1)
max_x2 = np.max(x2)
max_x = int(np.max([max_x1, max_x2])) + 2

min_y1 = np.min(y1) - abs( (np.max(y1) - np.min(y1)) * 0.1)
min_y2 = np.min(y2) - abs( (np.max(y2) - np.min(y2)) * 0.1)

print(min_y1, np.min(y1), (np.min(y1) * 0.1))

max_y1 = np.max(y1) + abs( (np.max(y1) - np.min(y1)) * 0.1)
max_y2 = np.max(y2) + abs( (np.max(y2) - np.min(y2)) * 0.1)

# Perform linear regression to find the best-fit lines
model1 = LinearRegression()
model1.fit(x1.reshape(-1, 1), y1)
slope1 = model1.coef_[0]
intercept1 = model1.intercept_

model2 = LinearRegression()
model2.fit(x2.reshape(-1, 1), y2)
slope2 = model2.coef_[0]
intercept2 = model2.intercept_

# Interpolate or resample the longer dataset to match the length of the shorter dataset
# f = interp1d(x2, y2)
# x2_resampled = np.linspace(x2.min(), x2.max(), len(x1))
# y2_resampled = f(x2_resampled)
x2_resampled = x2
y2_resampled = y2

# Plotting
# fig, ax1 = plt.subplots(figsize=(8, 6))
fig, ax1 = plt.subplots()

COLOR_1 = 'saddlebrown'
COLOR_2 = 'royalblue'

# Scatter plot for the first set of data
ax1.scatter(x1, y1, color=COLOR_1, label='Density', marker='o')

# Plotting the best-fit line for the first set of data
x1_start = np.min(x1) - 10
x1_end = np.max(x1) + 10
y1_start = slope1*x1_start + intercept1
y1_end = slope1*x1_end + intercept1
ax1.plot([x1_start, x1_end], [y1_start, y1_end], color=COLOR_1, linestyle='-', label='Density fit', lw=0.45)
ax1.plot([x1_start, x1_end], [y1_start, y1_end], color=COLOR_1, linestyle='-', label='Expected density', lw=105, alpha=0.15)

# Setting the labels and colors for the first y-axis
ax1.set_xlabel('Distance to power line [m]')
ax1.set_ylabel('Point density along power line reconstruction [points/m]', color=COLOR_1)
ax1.tick_params(axis='y', labelcolor=COLOR_1)
ax1.set_ylim([min_y1, max_y1])
# ax1.grid(True)

# Creating a secondary y-axis
ax2 = ax1.twinx()

# Scatter plot for the resampled second set of data
ax2.scatter(x2_resampled, y2_resampled, color=COLOR_2, label='Spread', marker='D')

# Plotting the best-fit line for the resampled second set of data
x2_start = np.min(x2) - 10
x2_end = np.max(x2) + 10
y2_start = slope2*x2_start + intercept2
y2_end = slope2*x2_end + intercept2
ax2.plot([x2_start, x2_end], [y2_start, y2_end], color=COLOR_2, linestyle='-', label='Spread fit', lw=0.45)
ax2.plot([x2_start, x2_end], [y2_start, y2_end], color=COLOR_2, linestyle='-', label='Expected spread', lw=105, alpha=0.15)

# Setting the labels and colors for the secondary y-axis
ax2.set_ylabel('Mean point distance to pose estimate [m]', color=COLOR_2)
ax2.tick_params(axis='y', labelcolor=COLOR_2)
ax2.set_ylim([min_y2, max_y2])
# ax2.grid(True)

# Adding legend
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
lines = lines1 + lines2
labels = labels1 + labels2
# ax1.legend(lines, labels, loc='center right')

plt.title('Radar data quality as function of distance to power line \n(4 m/s tracking speed, 25 Hz sensor data rate)')
plt.grid(False)

plt.xlim(min_x, max_x)
plt.show()

# 'Radar data quality as function of distance to power line'
# 'Effect of distance on radar power line measurements'