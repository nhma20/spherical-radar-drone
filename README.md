# Spherical Radar Drone ROS2 package
ROS2 package for the spherical radar drone

For publication information, see:


For hardware setup, see: 



```sh
ros2 launch spherical-radar-drone spherical_radar.launch.py
```


In `PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml` include 
```
  - msg:     vehicle_global_position
    send:    true
  - msg:     manual_control_setpoint
    send:    true
```

Then, `/PX4-Autopilot/msg/tools` in run 

```
python3 uorb_to_ros_urtps_topics.py -i urtps_bridge_topics.yaml -o ~/ros2_ws/src/px4_ros_com/templates/urtps_bridge_topics.yaml
```

followed by 
```
./uorb_to_ros_msgs.py ../ ~/uzh_ws/ros2_ws/src/px4_msgs/msg/
```

And build changes by going to `/ros2_ws/src/px4_ros_com/scripts` and run
```
./build_ros2_workspace.bash --verbose
```

## ROS2 Parameters


