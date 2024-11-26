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
        get_package_share_directory('spherical-radar-drone'),
        'config',
        'params.yaml'
    )

    tf_drone_to_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.20", "0.0", "0.0", "-1.570796", "-1.570796", "0.0", "drone", "front_frame"] #  (x, y, z, yaw, roll, pitch) # arguments=["0", "0.0", "-0.08", "1.570796", "0.0", "-1.570796", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.20", "0.0", "0.0", "1.570796", "-1.57079", "0.0", "drone", "rear_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_top = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.20", "-1.570796", "0.0", "1.570796", "drone", "top_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_bot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0.0", "-0.20", "1.570796", "0.0", "-1.570796", "drone", "bot_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "-0.20", "0.0", "3.141593", "1.570796", "0.0", "drone", "right_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.20", "0.0", "0.0", "1.570796", "0.0", "drone", "left_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    ### FOR TESTING, REMOVE 
    world_to_drone = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "drone"] #  (x, y, z, yaw, roll, pitch)
    )
    # world_to_drone = Node(
    #     package="spherical-radar-drone",
    #     executable="drone_frame_broadcaster"
    # )



    radar_pointcloud_filter = Node(
        package="spherical-radar-drone",
        executable="radar_pointcloud_filter",
        parameters=[config]
    )

    offboard_control = Node(
        package="spherical-radar-drone",
        executable="offboard_control",
        parameters=[config]
    )

    lidar_to_mmwave_front = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_front'},
            {'output_topic': '/front_pcl'},
            {'frame_id': 'front_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    lidar_to_mmwave_rear = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_rear'},
            {'output_topic': '/rear_pcl'},
            {'frame_id': 'rear_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    lidar_to_mmwave_right = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_right'},
            {'output_topic': '/right_pcl'},
            {'frame_id': 'right_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    lidar_to_mmwave_left = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_left'},
            {'output_topic': '/left_pcl'},
            {'frame_id': 'left_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    lidar_to_mmwave_top = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_top'},
            {'output_topic': '/top_pcl'},
            {'frame_id': 'top_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    lidar_to_mmwave_bot = Node(
        package="spherical-radar-drone",
        executable="lidar_to_mmwave_node",
        parameters=[
            {'input_topic': '/dist_sensor/laser_scan_bot'},
            {'output_topic': '/bot_pcl'},
            {'frame_id': 'bot_frame'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )


    return LaunchDescription([
        tf_drone_to_rear,
        tf_drone_to_front,
        tf_drone_to_left,
        tf_drone_to_right,
        tf_drone_to_top,
        tf_drone_to_bot,
        world_to_drone,
        lidar_to_mmwave_front,
        lidar_to_mmwave_rear,
        lidar_to_mmwave_right,
        lidar_to_mmwave_left,
        lidar_to_mmwave_top,
        lidar_to_mmwave_bot
        #radar_pointcloud_filter,
        #offboard_control
    ])