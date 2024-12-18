from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('radar_cable_follower'),
        'config',
        'params.yaml'
    )

    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        #arguments=["0", "0", "0.05", "0", "0", "1.5", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, pitch, roll)
        arguments=["0", "0.0", "-0.08", "1.570796", "0.12", "-1.570796", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)   --  0.1 r good fix
    )

    # tf_drone_to_mmwave = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0.05", "0", "0", "1.5", "drone", "mmwave"] # (x, y, z, yaw, pitch, roll)
    # )

    world_to_drone = Node(
        package="radar_cable_follower",
        executable="drone_frame_broadcaster"
    )

    lidar_to_mmwave = Node(
        package="radar_cable_follower",
        executable="lidar_to_mmwave_node"
    )

    radar_pointcloud_filter = Node(
        package="radar_cable_follower",
        executable="radar_pointcloud_filter",
        parameters=[config]
    )

    offboard_control = Node(
        package="radar_cable_follower",
        executable="offboard_control",
        parameters=[config]
    )


    return LaunchDescription([
        tf_drone_to_iwr,
        world_to_drone,
        #lidar_to_mmwave,
        # tf_drone_to_mmwave,
        radar_pointcloud_filter,
        offboard_control
    ])
