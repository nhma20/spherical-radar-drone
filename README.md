# Spherical Radar Drone ROS2 package
ROS2 package for the spherical radar drone

For publication information, see:


## Hardware: 
Hardware needed:
- Holybro QAV250 or similar drone platform
- Pixhawk Mini 4 or similar flight controller
- Radio links, GPS, and battery of choice
- 1x Raspberry Pi 4 (>2GB memory recommended)
- 1x IWR6843ISK evm
- 5x IWR6843AOP evm
- 4-port USB hub modified to receive 5V from DCDC
- 5V 5A DCDC converter
- 6x USB-A to micro-USB cable
- 1x DCDC to USB-C cable


## Pixhawk Mini 4 setup
- Connect via USB to Host PC running QGroundControl
- On host PC:
  - Load Pixhawk with default firmware (1.12.3) through QGroundControl
  - Reset all parameters
  - Build specific version of firmware (on host PC):
    - `git clone -n https://github.com/PX4/PX4-Autopilot.git`
    - `git checkout d7a962b4269d3ca3d2dcae44da7a37177af1d8cd`
    - `git submodule update --init --recursive`
    - Delete the micro_dds directory in the firmware (`PX4-Autopilot/src/modules/microdds_client/`)
    - Go to `PX4-Autopilot/boards/px4/fmu-v5/rtps.px4board` (create if does not exist) and set CONFIG_MODULES_MICRODDS_CLIENT=n
    - `cd ~/PX4-Autopilot/`
    - `make px4_fmu-v5_rtps`
  - Load Pixhawk with newly built firmware (`~/PX4-Autopilot/build/px4_fmu-v5_rtps/px4_fmu-v5_rtps.px4`) through QGroundControl
  - Set parameters (file provided in `/pixhawk_parameters/` directory of https://github.com/nhma20/radar_cable_follower_HW):
    - `RTPS_CONFIG TELEM1`
    - `MAV_0_CONFIG TELEM/SERIAL4`
    - `RTPS_MAV_CONFIG Disabled`
    - `SER_TEL1_BAUD 460800 8N1`
    - `SER_TEL4_BAUD 57600 8N1`
    Optional speed parameters:
    - `MPC_Z_VEL_ALL 3`
    - `MPC_XY_VEL_ALL 5`
- Validate connection between Raspberry Pi and Pixhawk:
  - Connect FTDI-USB-to-Serial between Raspberry Pi USB port and Pixhawk TELEM1 port.
  - Power on Pixhawk (battery or debug micro-USB port).
  - On Raspberry Pi:
    - `source ~/ros2_ws/install/setup.sh`
    - `micrortps_agent -d /dev/ttyUSB0` (find correct port first)
    - `ros2 topic echo /fmu/vehicle_odometry/out` (should print the odometry messages from the Pixhawk)
    - If there is no connection, check that the micrortps_client is running on the Pixhawk with the MAVLink Console:
      - `micrortps_client status`
      - debug from there


## Software setup:

Flash SD card with `Ubuntu 20.04 server` image.

Install ROS2 Foxy:
  - `sudo apt update`
  - `sudo apt upgrade`
  - `sudo apt update && sudo apt install curl gnupg2 lsb-release`
  - `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg`
  - `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
  - `pip3 install pyros-genmsg`
  - `sudo apt-get install build-essential`
  - `sudo apt install ros-foxy-desktop`
  - `sudo apt install python3-colcon-common-extensions`
  - `sudo apt install libpcl-dev`
  - ```sh
    cd ~
    mkdir -p ~/ros2_ws/src
    ```

Install PX4:
  - Foonathan memory:
    - `cd ~`
    - `git clone https://github.com/eProsima/foonathan_memory_vendor.git`
    - `cd foonathan_memory_vendor`
    - `mkdir build && cd build`
    - `cmake ..`
    - `sudo cmake --build . --target install`
   
  - Fast-RTPS-Gen
    - `sudo apt-get install openjdk-8-jdk` or `sudo apt-get install default-jdk`
    - `export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-arm64` or one similar in that directory.
    - `cd ~`
    - `git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
      && cd ~/Fast-RTPS-Gen \
      && ./gradlew assemble \
      && sudo ./gradlew install`

Clone repos into your `../ros2_ws/src/` directory:
- `git clone https://github.com/nhma20/spherical-radar-drone.git`
- `git clone https://github.com/nhma20/toggle_radar_msgs.git`
- `git clone https://github.com/nhma20/px4_msgs.git`
- `git clone https://github.com/nhma20/px4_ros_com.git`
- `git clone -b toggle https://github.com/nhma20/xwr6843_ros2.git`

To build workspace including `micro_rtps` and related message definitions, go to `../ros2_ws/src/px4_ros_com/scripts/` and run

```sh
./build_ros2_workspace.bash --verbose
```

The launch file expects a certain naming convention of the mmWave radar devices, i.e. `/dev/radar_front_CLI` and `/dev/radar_front_DATA`. To set up udev rules:
1. Find attributes to match device against: udevadm info -a /dev/ttyUSB*
2. Create rules in /etc/udev/rules.d/
3. Reload udev rules (also need to replug device): sudo udevadm control --reload

An example rule `11-spherical-radar-drone.rules` could look like:

```
SUBSYSTEMS=="usb", KERNELS=="1-1.1:1.0", DRIVERS=="cp210x", SYMLINK+="radar_rear_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.1:1.1", DRIVERS=="cp210x", SYMLINK+="radar_rear_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.3:1.0", DRIVERS=="cp210x", SYMLINK+="radar_bot_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.3:1.1", DRIVERS=="cp210x", SYMLINK+="radar_bot_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.1:1.0", DRIVERS=="cp210x", SYMLINK+="radar_top_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.1:1.1", DRIVERS=="cp210x", SYMLINK+="radar_top_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.2:1.0", DRIVERS=="cp210x", SYMLINK+="radar_right_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.2:1.1", DRIVERS=="cp210x", SYMLINK+="radar_right_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.3:1.0", DRIVERS=="cp210x", SYMLINK+="radar_left_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.3:1.1", DRIVERS=="cp210x", SYMLINK+="radar_left_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.4:1.0", DRIVERS=="cp210x", SYMLINK+="radar_front_CLI"
SUBSYSTEMS=="usb", KERNELS=="1-1.2.4:1.1", DRIVERS=="cp210x", SYMLINK+="radar_front_DATA"
SUBSYSTEMS=="usb", KERNELS=="1-1.4:1.0", DRIVERS=="ftdi_sio", SYMLINK+="FTDI_USB_to_Serial"
```

To launch full system, run:
 
```sh
ros2 launch spherical-radar-drone spherical_radar.launch.py
```

To visualize on ground control station host PC, consider the `https://github.com/nhma20/spherical-radar-drone-visualizer` package.

For simulation setup, follow `https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo` and instead launch system with:

```sh
ros2 launch spherical-radar-drone spherical_radar_SIM.launch.py
```
