#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import cv2 as cv

import numpy as np
import pandas as pd 

# import time
# import copy
# import sys
# import os
# import math 
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import pyransac3d as pyrsc


X = np.genfromtxt('height5_lines_cloud.csv', delimiter=',')
# X = np.genfromtxt('straight_power_lines_cloud.csv', delimiter=',')



print(X.shape)
print(X[0])

# points = []
# for i in range(X.shape[0]):
#     if X[i][0] > 20:
#         points.append([X[i][0], X[i][1], X[i][2]])

# points = np.array(points)

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


x_min = np.min(line1_points[0:,0])
x_max = np.max(line1_points[0:,0])
print("Line 1 MIN: ", x_min)
print("Line 1 MAX: ", x_max)

linelen = x_max - x_min

print("Line 1 density: ", line1_points.shape[0]/linelen)
print("Line 2 density: ", line2_points.shape[0]/linelen)
print("Line 3 density: ", line3_points.shape[0]/linelen)
print("Line 4 density: ", line4_points.shape[0]/linelen)

# print("Line 2 MIN: ", np.min(line2_points[0:,0]))
# print("Line 2 MAX: ", np.max(line2_points[0:,0]))

# print("Line 3 MIN: ", np.min(line3_points[0:,0]))
# print("Line 3 MAX: ", np.max(line3_points[0:,0]))

# print("Line 4 MIN: ", np.min(line4_points[0:,0]))
# print("Line 4 MAX: ", np.max(line4_points[0:,0]))




#SCATTER PLOT ALL LINE CLUSTERS TOGETHER
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d') 
ax.scatter(line1_points[0:,0], line1_points[0:,1], line1_points[0:,2], color='r')
ax.scatter(line2_points[0:,0], line2_points[0:,1], line2_points[0:,2], color='g')
ax.scatter(line3_points[0:,0], line3_points[0:,1], line3_points[0:,2], color='b')
ax.scatter(line4_points[0:,0], line4_points[0:,1], line4_points[0:,2], color='black')
ax.set_proj_type('ortho')  # FOV = 0 deg
ax.set_aspect('equal', adjustable='box') # equal axes
plt.show()




















