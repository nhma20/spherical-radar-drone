#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Generate some sample data
np.random.seed(0)
x1 = np.array([20, 17, 15.5, 9, 6.5, 5.0])
y1 = np.array([1.35222731338367, 2.47646925851325, 3.33526836621588, 5.58810742426104, 5.46064609558763, 5.86311441656544])

x2 = np.array([26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 23.425, 16.925, 13.425, 12.425, 31.325, 28.325, 26.825, 20.325, 16.825, 15.825])
y2 = np.array([0.517028090411402, 1.45443432642842, 2.49033371344119, 5.47483497647197, 5.36484528689311, 5.84636266108953, 0.278399740990755, 0.432399394343583, 1.6453990606665, 4.07780812040671, 4.85390764052234, 5.47782404061971, 0.039771391570108, 0, 0.400232203945905, 3.88902070742492, 4.45637801, 4.74074679968005])

# Perform linear regression to find the best-fit lines
model1 = LinearRegression()
model1.fit(x1.reshape(-1, 1), y1)
slope1 = model1.coef_[0]
print("Slope 1: ", slope1)
intercept1 = model1.intercept_
print("Intercept 1: ", intercept1)

model2 = LinearRegression()
model2.fit(x2.reshape(-1, 1), y2)
slope2 = model2.coef_[0]
print("Slope2: ", slope2)
intercept2 = model2.intercept_
print("Intercept 2: ", intercept2)

# Plotting
plt.figure(figsize=(8, 6))

# Scatter plot for the first set of data
plt.scatter(x1, y1, color='blue', label='10 mm Ø cable')

# Plotting the best-fit line for the first set of data
plt.plot(x1, slope1*x1 + intercept1, color='blue', linestyle='-', label='Best Fit')

# Scatter plot for the second set of data
plt.scatter(x2, y2, color='red', label='20 mm Ø cable')

# Plotting the best-fit line for the second set of data
plt.plot(x2, slope2*x2 + intercept2, color='red', linestyle='-', label='Best Fit')

# Adding labels and legend
plt.xlabel('Distance to power line [m]')
plt.ylabel('Point density [points/m]')
plt.title('Point density of reconstructed power lines \n (4 m/s tracking speed, 25 Hz sensor data rate)')
plt.legend()

plt.grid(True)
plt.show()





