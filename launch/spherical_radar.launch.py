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
        arguments=["0.10", "0.0", "0.0", "-1.570796", "-1.570796", "0.0", "drone", "front_frame"] #  (x, y, z, yaw, roll, pitch) # arguments=["0", "0.0", "-0.08", "1.570796", "0.0", "-1.570796", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_rear = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.10", "0.0", "0.0", "1.570796", "0.0", "0.0", "drone", "rear_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_top = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.10", "0.0", "0.10", "-1.570796", "0.0", "1.570796", "drone", "top_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_bot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0.0", "-0.10", "-1.570796", "0.0", "-1.570796", "drone", "bot_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "-0.10", "-0.04", "0.26179938", "0.0", "3.141593", "drone", "right_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    tf_drone_to_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.10", "-0.04", "-0.26179938", "0.0", "0.0", "drone", "left_frame"] #  (x, y, z, yaw, roll, pitch)
    )

    ### TESTING, REMOVE 
    world_to_drone = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "drone"] #  (x, y, z, yaw, roll, pitch)
    )
    # world_to_drone = Node(
    #     package="spherical-radar-drone",
    #     executable="drone_frame_broadcaster"
    # )

    mmwave_front = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/front_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_front_CLI'},
            {'data_port': '/dev/radar_front_DATA'},
            {'frame_id': 'front_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '30'},
            {'minimum_range', '0.2'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mmwave_rear = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/rear_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_rear_CLI'},
            {'data_port': '/dev/radar_rear_DATA'},
            {'frame_id': 'rear_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '120'},
            {'minimum_range', '0.2'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mmwave_top = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/top_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_top_CLI'},
            {'data_port': '/dev/radar_top_DATA'},
            {'frame_id': 'top_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '120'},
            {'minimum_range', '0.2'}
            
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mmwave_bot = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/bot_pcl')
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_bot_CLI'},
            {'data_port': '/dev/radar_bot_DATA'},
            {'frame_id': 'bot_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '120'},
            {'minimum_range', '0.2'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mmwave_right = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/right_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_right_CLI'},
            {'data_port': '/dev/radar_right_DATA'},
            {'frame_id': 'right_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '120'},
            {'minimum_range', '0.2'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    mmwave_left = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/left_pcl'),
            ],
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/radar_left_CLI'},
            {'data_port': '/dev/radar_left_DATA'},
            {'frame_id': 'left_frame'},
            {'radar_azimuth_fov', '120'},
            {'radar_elevation_fov', '120'},
            {'minimum_range', '0.2'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

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


    return LaunchDescription([
        tf_drone_to_rear,
        tf_drone_to_front,
        tf_drone_to_left,
        tf_drone_to_right,
        tf_drone_to_top,
        tf_drone_to_bot,
        world_to_drone,
        mmwave_bot,
        mmwave_top,
        mmwave_left,
        mmwave_right,
        mmwave_rear,
        mmwave_front,
        #radar_pointcloud_filter,
        #offboard_control
    ])