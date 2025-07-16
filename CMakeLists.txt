cmake_minimum_required(VERSION 3.5)
project(spherical-radar-drone)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(toggle_radar_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(PCL 1.2 REQUIRED COMPONENTS common filters)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(drone_frame_broadcaster
  src/drone_frame_broadcaster_node.cpp
  src/geometry.cpp
)
  ament_target_dependencies(drone_frame_broadcaster
  rclcpp
  Eigen3
  geometry_msgs
  px4_msgs
  tf2
  tf2_ros
)
install(TARGETS drone_frame_broadcaster DESTINATION lib/${PROJECT_NAME})


add_executable(offboard_control
  src/offboard_control.cpp
  src/geometry.cpp
)
  ament_target_dependencies(offboard_control
  rclcpp
  Eigen3
  geometry_msgs
  px4_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_msgs
  visualization_msgs
  PCL
)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})


add_executable(radar_pointcloud_combiner
src/radar_pointcloud_combiner.cpp
src/geometry.cpp
)
target_link_libraries (radar_pointcloud_combiner
${PCL_LIBRARIES}
)
  ament_target_dependencies(radar_pointcloud_combiner
  rclcpp
  sensor_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_msgs
  PCL
)
install(TARGETS radar_pointcloud_combiner DESTINATION lib/${PROJECT_NAME})


add_executable(lidar_to_mmwave_node
src/lidar_to_mmwave.cpp
)
  ament_target_dependencies(lidar_to_mmwave_node
  rclcpp sensor_msgs
  geometry_msgs
  toggle_radar_msgs
)
install(TARGETS lidar_to_mmwave_node DESTINATION lib/${PROJECT_NAME})


add_executable(radar_toggler
src/radar_toggler.cpp
)
  ament_target_dependencies(radar_toggler
  rclcpp sensor_msgs
  geometry_msgs
  toggle_radar_msgs
  tf2
  tf2_ros
  tf2_msgs
  px4_msgs
)
install(TARGETS radar_toggler DESTINATION lib/${PROJECT_NAME})


add_executable(radar_zone_visualizer
src/radar_zone_visualizer.cpp
)
  ament_target_dependencies(radar_zone_visualizer
  rclcpp
  ament_index_cpp
  toggle_radar_msgs
  visualization_msgs
)
install(TARGETS radar_zone_visualizer DESTINATION lib/${PROJECT_NAME})


add_executable(look_ahead_cone_republisher
  src/look_ahead_cone_republisher.cpp
)
  ament_target_dependencies(look_ahead_cone_republisher
  rclcpp
  ament_index_cpp
  visualization_msgs
)
install(TARGETS look_ahead_cone_republisher DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/rc_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# install the mesh directory
install(DIRECTORY
  mesh
  DESTINATION share/${PROJECT_NAME}/
)

# install the urdf directory
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# install the rviz directory
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# # copy look_ahead_cone.stl mesh to home dir
# install(PROGRAMS
#   mesh/look_ahead_cone.stl
#   DESTINATION $ENV{HOME}
# )

ament_package()


