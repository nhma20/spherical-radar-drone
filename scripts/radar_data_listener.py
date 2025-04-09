#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

from time import sleep

from threading import Lock


class RcNodeLauncher(Node):
    def __init__(self):
        super().__init__("radar_listener_node_launcher")

        self.get_logger().info("Initializing radar listener node")


        # ROS2 timer
        # self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.sub = self.create_subscription(
            PointCloud2,
            "/top_pcl",
            self.callback_top,
            10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/bot_pcl",
            self.callback_bot,
            10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/right_pcl",
            self.callback_right,
            10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/left_pcl",
            self.callback_left,
            10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/front_pcl",
            self.callback_front,
            10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/rear_pcl",
            self.callback_rear,
            10
        )


    def callback_top(self, msg: PointCloud2):
        self.get_logger().warning("Received top pcl message")




def main(args=None):
    rclpy.init(args=args)

    publisher = RcNodeLauncher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
