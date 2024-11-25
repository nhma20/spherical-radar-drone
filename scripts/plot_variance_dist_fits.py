#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import numpy as np
import pandas as pd 
import pyransac3d as pyrsc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error


# X = np.genfromtxt('straight_power_lines_cloud.csv', delimiter=',')
# X = np.genfromtxt('height1_lines_cloud.csv', delimiter=',')
# X = np.genfromtxt('height2_lines_cloud.csv', delimiter=',')
# X = np.genfromtxt('height3_lines_cloud.csv', delimiter=',')
# X = np.genfromtxt('height4_lines_cloud.csv', delimiter=',')
X = np.genfromtxt('height5_lines_cloud.csv', delimiter=',')

print(X.shape)
print(X[0])

points = X # Load your point cloud as a numpy array (N, 3)


# FIRST LINE
line1 = pyrsc.Line()
slope, intercept, inliers = line1.fit(points, 1.5, 100)

line1_points = points[inliers]
print("Line 1 points: ", line1_points.shape)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(line1_points[0:,0], line1_points[0:,1], line1_points[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()


# POINTS MINUS ONE
points_minus1 = []
inliner_count = 0
for i in range(len(points)):
    if i != inliers[inliner_count]:
        points_minus1.append(points[i])
    else:
        if inliner_count < inliers.shape[0]-1:
            inliner_count = inliner_count + 1

points_minus1 = np.array(points_minus1)
# print(points_minus1.shape)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(points_minus1[0:,0], points_minus1[0:,1], points_minus1[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()


# SECOND LINE
line2 = pyrsc.Line()
slope2, intercept2, inliers2 = line2.fit(points_minus1, 1.5, 100)

line2_points = points_minus1[inliers2]
print("Line 2 points: ", line2_points.shape)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(line2_points[0:,0], line2_points[0:,1], line2_points[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()


# POINTS MINUS TWO
points_minus2 = []
inliner2_count = 0
for i in range(len(points_minus1)):
    if i != inliers2[inliner2_count]:
        points_minus2.append(points_minus1[i])
    else:
        if inliner2_count < inliers2.shape[0]-1:
            inliner2_count = inliner2_count + 1

points_minus2 = np.array(points_minus2)
# print(points_minus2.shape)


# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(points_minus2[0:,0], points_minus2[0:,1], points_minus2[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()



# THIRD LINE
line3 = pyrsc.Line()
slope3, intercept3, inliers3 = line3.fit(points_minus2, 1.5, 100)

line3_points = points_minus2[inliers3]
print("Line 3 points: ", line3_points.shape)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(line3_points[0:,0], line3_points[0:,1], line3_points[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()



# POINTS MINUS THREE == LINE FOUR
points_minus3 = []
inliner3_count = 0
for i in range(len(points_minus2)):
    if i != inliers3[inliner3_count]:
        points_minus3.append(points_minus2[i])
    else:
        if inliner3_count < inliers3.shape[0]-1:
            inliner3_count = inliner3_count + 1

points_minus3 = np.array(points_minus3)
# print(points_minus3.shape)

line4_points = points_minus3
print("Line 4 points: ", line4_points.shape)

# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# ax.scatter(line4_points[0:,0], line4_points[0:,1], line4_points[0:,2])
# # show the plot
# ax.set_proj_type('ortho')  # FOV = 0 deg
# plt.show()



def distance_point_to_line(point, slope, intercept):
    """
    Calculate the distance between a point and a line defined by slope and intercept in 3D space.
    
    Parameters:
        point (numpy array): Coordinates of the point in 3D space, e.g., [x, y, z].
        slope (numpy array): Slope of the line in 3D space, e.g., [a, b, c].
        intercept (numpy array): Intercept of the line in 3D space, e.g., [x0, y0, z0].
        
    Returns:
        float: Distance between the point and the line.
    """
    point = np.array(point)
    slope = np.array(slope)
    intercept = np.array(intercept)
    
    numerator = np.linalg.norm(np.cross(slope, point - intercept))
    denominator = np.linalg.norm(slope)
    
    distance = numerator / denominator
    return distance




# # Print the distances
# print("Distances between each data point and the best-fit line:")
# for i in range(len(line1_points)):
#     dist = distance_point_to_line(line1_points[i], slope, intercept)
#     print(dist)



all_lines = [] #line1_points, line2_points, line3_points, line4_points]

if len(line1_points) != 0:
    all_lines.append(line1_points)

if len(line2_points) != 0:
    all_lines.append(line2_points)

if len(line3_points) != 0:
    all_lines.append(line3_points)

if len(line4_points) != 0:
    all_lines.append(line4_points)

all_lines = np.array(all_lines)

print(all_lines.shape)


import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_proj_type('ortho')  # FOV = 0 deg

names = ["Cable 1", "Cable 2", "Cable 3", "Cable 4"]
colors = ["red", "green", "blue", "black"]

# Generate some data that lies along a line
for i in range(len(all_lines)):
    x = all_lines[i][0:,0] #line1_points[0:,0]
    y = all_lines[i][0:,1] #line1_points[0:,1]
    z = all_lines[i][0:,2] #line1_points[0:,2]

    data = np.concatenate((x[:, np.newaxis], 
                        y[:, np.newaxis], 
                        z[:, np.newaxis]), 
                        axis=1)

    # Calculate the mean of the points, i.e. the 'center' of the cloud
    datamean = data.mean(axis=0)

    # Do an SVD on the mean-centered data.
    uu, dd, vv = np.linalg.svd(data - datamean)

    # Now vv[0] contains the first principal component, i.e. the direction
    # vector of the 'best fit' line in the least squares sense.

    # Now generate some points along this best fit line, for plotting.

    # I use -7, 7 since the spread of the data is roughly 14
    # and we want it to have mean 0 (like the points we did
    # the svd on). Also, it's a straight line, so we only need 2 points.
    linepts = vv[0] * np.mgrid[-35:35:2j][:, np.newaxis]


    # shift by the mean to get the line in the right place
    linepts += datamean


    # print(datamean)
    # print(vv[0])
    # Start by subtracting Ax_support:
    P_centered = all_lines[i] - datamean
    # The points on the line through 0 with direction Ax_direction with the shortest distance in the L2 sense to each element of P_centered is given by
    P_projected = P_centered.dot(np.linalg.pinv(
                vv[0][np.newaxis, :])).dot(vv[0][np.newaxis, :])
    # Thus the formula is
    distances = np.sqrt(((P_centered - P_projected) ** 2).sum(axis=1))

    # print(distances) # probably works

    print("Cable ", i+1, " Mean: ", np.mean(distances))
    print("Cable ", i+1, " Variance: ", np.var(distances))


    # Verify that everything looks right.
    
    ax.scatter(*data.T, color=colors[i], label=names[i])
    ax.plot(*linepts.T, color=colors[i])
 
    # ax.set_aspect('equal', adjustable='box') # equal axes

plt.legend()
plt.show()





