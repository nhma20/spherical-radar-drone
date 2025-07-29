from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    latency_offset_ns = 10_000_000
    cfg_path_ISK = '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr6843ISK_profile_10Hz_v2.cfg' #'xwr68xx_profile_25Hz_Elev_43m.cfg' #
    cfg_path_AOP = '/home/ubuntu/ros2_ws/src/xwr6843_ros2/cfg_files/xwr6843AOP_profile_10Hz_v2.cfg' #'xwr68xx_profile_25Hz_Elev_43m.cfg' #
    AZIMUTH_AOP = 85
    ELEVATION_AOP = 85
    AZIMUTH_ISK = 140
    ELEVATION_ISK = 60

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
    # world_to_drone = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "drone"] #  (x, y, z, yaw, roll, pitch)
    # )
    # drone_to_drone_yaw_only = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "drone", "drone_yaw_only"] #  (x, y, z, yaw, roll, pitch)
    # )
    world_to_drone = Node(
        package="spherical-radar-drone",
        executable="drone_frame_broadcaster",
        respawn=True
    )

    mmwave_front = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/front_pcl'),
            ],
        parameters=[
            {'cfg_path': cfg_path_ISK},
            {'cli_port': '/dev/radar_front_CLI'},
            {'data_port': '/dev/radar_front_DATA'},
            {'frame_id': 'front_frame'},
            {'radar_azimuth_fov': AZIMUTH_ISK},
            {'radar_elevation_fov': ELEVATION_ISK},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )

    mmwave_rear = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/rear_pcl'),
            ],
        parameters=[
            {'cfg_path': cfg_path_AOP},
            {'cli_port': '/dev/radar_rear_CLI'},
            {'data_port': '/dev/radar_rear_DATA'},
            {'frame_id': 'rear_frame'},
            {'radar_azimuth_fov': AZIMUTH_AOP},
            {'radar_azimuth_fov': ELEVATION_AOP},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )

    mmwave_top = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/top_pcl'),
            ],
        parameters=[
            {'cfg_path': cfg_path_AOP},
            {'cli_port': '/dev/radar_top_CLI'},
            {'data_port': '/dev/radar_top_DATA'},
            {'frame_id': 'top_frame'},
            {'radar_azimuth_fov': AZIMUTH_AOP},
            {'radar_azimuth_fov': ELEVATION_AOP},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
            
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )

    mmwave_bot = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/bot_pcl')
            ],
        parameters=[
            {'cfg_path': cfg_path_AOP},
            {'cli_port': '/dev/radar_bot_CLI'},
            {'data_port': '/dev/radar_bot_DATA'},
            {'frame_id': 'bot_frame'},
            {'radar_azimuth_fov': AZIMUTH_AOP},
            {'radar_azimuth_fov': ELEVATION_AOP},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )

    mmwave_right = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/right_pcl'),
            ],
        parameters=[
            {'cfg_path': cfg_path_AOP},
            {'cli_port': '/dev/radar_right_CLI'},
            {'data_port': '/dev/radar_right_DATA'},
            {'frame_id': 'right_frame'},
            {'radar_azimuth_fov': AZIMUTH_AOP},
            {'radar_azimuth_fov': ELEVATION_AOP},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )

    mmwave_left = Node(
        package="xwr6843_ros2",
        executable="pcl_pub",
        remappings=[
                ('/xwr6843_pcl', '/left_pcl'),
            ],
        parameters=[
            {'cfg_path': cfg_path_AOP}, #cfg_path_AOP
            {'cli_port': '/dev/radar_left_CLI'},
            {'data_port': '/dev/radar_left_DATA'},
            {'frame_id': 'left_frame'},
            {'radar_azimuth_fov': AZIMUTH_AOP},
            {'radar_azimuth_fov': ELEVATION_AOP},
            {'minimum_range': 0.3},
            {'publish_snr': False},
            {'publish_noise': False},
            {'publish_velocity': True},
            {'latency_offset_ns': latency_offset_ns}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        output='screen',
        emulate_tty=True,
        respawn=True
    )


    offboard_control = Node(
        package="spherical-radar-drone",
        executable="offboard_control",
        parameters=[config]
    )

    radar_pointcloud_combiner = Node(
        package="spherical-radar-drone",
        executable="radar_pointcloud_combiner",
        parameters=[
            {'pointcloud_update_rate': 10}, #10
            {'_enable_temporal_filter': False},
            {'concatenate_capacity': 2},
            # {'temporal_filter_horizon': 5},
            # {'temporal_filter_radius': 1.0}
         ],
        arguments=['--ros-args', '--log-level', 'warn'],
        respawn=True,
        emulate_tty=True
    )


    radar_toggler = Node(
        package="spherical-radar-drone",
        executable="radar_toggler",
    )


    micrortps_agent = ExecuteProcess(
        cmd=['micrortps_agent', '-d', '/dev/FTDI_USB_to_Serial'],
        output='screen'
    )


    return LaunchDescription([
        micrortps_agent,
        world_to_drone,
        tf_drone_to_rear,
        tf_drone_to_front,
        tf_drone_to_left,
        tf_drone_to_right,
        tf_drone_to_top,
        tf_drone_to_bot,
        mmwave_bot,
        mmwave_top,
        mmwave_left,
        mmwave_right,
        mmwave_rear,
        mmwave_front,
        radar_pointcloud_combiner,
        # radar_toggler,
        offboard_control
    ])