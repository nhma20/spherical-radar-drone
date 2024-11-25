#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# #  data
# x1 = np.array([20, 17, 15.5, 9, 6.5, 5.0])
# y1 = np.array([0.281283563801308, 0.241350086759652, 0.217878870730697, 0.152245514068486, 0.140857950339467, 0.109420850847834])

# x2 = np.array([26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 16.925, 13.425, 12.425, 26.825, 20.325, 16.825, 15.825])
# y2 = np.array([0.359859924939888, 0.38667445578246, 0.381027503787732, 0.268466430421299, 0.164235135886524, 0.243940667503406, 0.399578809518845, 0.377679802672095, 0.26702124311395, 0.235519243495821, 0.243174220814384, 0.385417100628619, 0.383827369892889, 0.291688957711852, 0.32621443797731])

# # Perform linear regression to find the best-fit lines
# model1 = LinearRegression()
# model1.fit(x1.reshape(-1, 1), y1)
# slope1 = model1.coef_[0]
# print("Slope 1: ", slope1)
# intercept1 = model1.intercept_
# print("Intercept 1: ", intercept1)

# model2 = LinearRegression()
# model2.fit(x2.reshape(-1, 1), y2)
# slope2 = model2.coef_[0]
# print("Slope2: ", slope2)
# intercept2 = model2.intercept_
# print("Intercept 2: ", intercept2)

# # Plotting
# plt.figure(figsize=(8, 6))

# # Scatter plot for the first set of data
# plt.scatter(x1, y1, color='blue', label='10 mm Ø cable')

# # Plotting the best-fit line for the first set of data
# plt.plot(x1, slope1*x1 + intercept1, color='blue', linestyle='-', label='Best Fit')

# # Scatter plot for the second set of data
# plt.scatter(x2, y2, color='red', label='20 mm Ø cable')

# # Plotting the best-fit line for the second set of data
# plt.plot(x2, slope2*x2 + intercept2, color='red', linestyle='-', label='Best Fit')

# # Adding labels and legend
# plt.xlabel('Distance to power line [m]')
# plt.ylabel('Mean point spread [m]')
# plt.title('Mean point spread of reconstructed power lines \n (4 m/s tracking speed, 25 Hz sensor data rate)')
# plt.legend()

# plt.grid(True)
# plt.show()


#  data
# x1 = np.array([20, 17, 15.5, 9, 6.5, 5.0])
# y1 = np.array([0.281283563801308, 0.241350086759652, 0.217878870730697, 0.152245514068486, 0.140857950339467, 0.109420850847834])

x2 = np.array([20, 17, 15.5, 9, 6.5, 5.0, 26.25, 24.25, 21.75, 15.25, 11.75, 10.75, 27.925, 25.925, 16.925, 13.425, 12.425, 26.825, 20.325, 16.825, 15.825])
y2 = np.array([0.281283563801308, 0.241350086759652, 0.217878870730697, 0.152245514068486, 0.140857950339467, 0.109420850847834, 0.359859924939888, 0.38667445578246, 0.381027503787732, 0.268466430421299, 0.164235135886524, 0.243940667503406, 0.399578809518845, 0.377679802672095, 0.26702124311395, 0.235519243495821, 0.243174220814384, 0.385417100628619, 0.383827369892889, 0.291688957711852, 0.32621443797731])

# Perform linear regression to find the best-fit lines
# model1 = LinearRegression()
# model1.fit(x1.reshape(-1, 1), y1)
# slope1 = model1.coef_[0]
# print("Slope 1: ", slope1)
# intercept1 = model1.intercept_
# print("Intercept 1: ", intercept1)

model2 = LinearRegression()
model2.fit(x2.reshape(-1, 1), y2)
slope2 = model2.coef_[0]
print("Slope2: ", slope2)
intercept2 = model2.intercept_
print("Intercept 2: ", intercept2)

# Plotting
plt.figure(figsize=(8, 6))

# # Scatter plot for the first set of data
# plt.scatter(x1, y1, color='blue', label='10 mm Ø cable')

# # Plotting the best-fit line for the first set of data
# plt.plot(x1, slope1*x1 + intercept1, color='blue', linestyle='-', label='Best Fit')

# Scatter plot for the second set of data
plt.scatter(x2, y2)

# Plotting the best-fit line for the second set of data
plt.plot(x2, slope2*x2 + intercept2, color='black', linestyle='-', label='Best Fit')

# Adding labels and legend
plt.xlabel('Distance to power line [m]')
plt.ylabel('Mean point spread [m]')
plt.title('Mean point spread of reconstructed power lines \n (4 m/s tracking speed, 25 Hz sensor data rate)')
plt.legend()

plt.grid(True)
plt.show()



