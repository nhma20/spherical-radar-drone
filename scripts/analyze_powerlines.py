#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import struct
from xml.sax.handler import property_xml_string
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 #sudo apt install ros-foxy-sensor-msgs-py

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


###############################################################################
# Class
###############################################################################

class PCLDrawer(Node):

    def __init__(self):

        super().__init__("pcl_drawer")
        self.pl_sub_ = self.create_subscription(
            PointCloud2,
            "/world_pcl",	
            self.on_pcl_msg,
            10
        )

        self.cloud = []


    def on_pcl_msg(self, msg):
        print("got world_pcl message!")
        print(msg.width)

        min_x = 99999
        max_x = 0


        for p in point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            # print (" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            if p[0] < min_x:
                min_x = p[0]

            if p[0] > max_x:
                max_x = p[0]

            self.cloud.append([p[0],p[1],p[2]])

        print("MIN: ", min_x, "MAX: ", max_x)   

        tempcloud = np.array(self.cloud)
        print(tempcloud.shape)

        # convert array into dataframe 
        DF = pd.DataFrame(tempcloud) 
        
        # save the dataframe as a 
        DF.to_csv("testname.csv")

        # gaussian mixture clustering
        from numpy import unique
        from numpy import where
        from sklearn.datasets import make_classification
        from sklearn.mixture import GaussianMixture
        from matplotlib import pyplot
        # define dataset
        X = tempcloud
        print(X.shape)
        # define the model
        model = GaussianMixture(n_components=4)
        # fit the model
        model.fit(X)
        # assign a cluster to each example
        yhat = model.predict(X)
        # retrieve unique clusters
        clusters = unique(yhat)
        # create scatter plot for samples from each cluster
        fig = pyplot.figure(figsize=(12, 12))
        ax = fig.add_subplot(projection='3d') 
        for cluster in clusters:
        # get row indexes for samples with this cluster
            row_ix = where(yhat == cluster)

        #  fig = pyplot.figure(figsize=(12, 12))
        #  ax = fig.add_subplot(projection='3d') 
        #  print(X[0:,0])
        #  ax.scatter(X[0:,0], X[0:,1], X[0:,2])
        #  pyplot.show()

        # create scatter of these samples
        #  pyplot.scatter(X[row_ix, 0], X[row_ix, 1])
        
            ax.scatter(X[row_ix,0], X[row_ix,1], X[row_ix,2])
        # show the plot
        ax.set_proj_type('ortho')  # FOV = 0 deg
        pyplot.show()





###############################################################################
# Main
###############################################################################


if __name__ == "__main__":
    rclpy.init()

    print("Starting PCLDrawer node")

    minimal_publisher = PCLDrawer()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



# # gaussian mixture clustering
# from numpy import unique
# from numpy import where
# from sklearn.datasets import make_classification
# from sklearn.mixture import GaussianMixture
# from matplotlib import pyplot
# # define dataset
# X, _ = make_classification(n_samples=1000, n_features=3, n_informative=2, n_redundant=0, n_clusters_per_class=1, random_state=4)
# print(X.shape)
# # define the model
# model = GaussianMixture(n_components=2)
# # fit the model
# model.fit(X)
# # assign a cluster to each example
# yhat = model.predict(X)
# # retrieve unique clusters
# clusters = unique(yhat)
# # create scatter plot for samples from each cluster
# fig = pyplot.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d') 
# for cluster in clusters:
#  # get row indexes for samples with this cluster
#  row_ix = where(yhat == cluster)

# #  fig = pyplot.figure(figsize=(12, 12))
# #  ax = fig.add_subplot(projection='3d') 
# #  print(X[0:,0])
# #  ax.scatter(X[0:,0], X[0:,1], X[0:,2])
# #  pyplot.show()

#  # create scatter of these samples
# #  pyplot.scatter(X[row_ix, 0], X[row_ix, 1])
 
#  ax.scatter(X[row_ix,0], X[row_ix,1], X[row_ix,2])
# # show the plot
# pyplot.show()