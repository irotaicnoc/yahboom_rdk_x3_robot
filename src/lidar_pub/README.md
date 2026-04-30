# ORADAR ROS Package

The ORADAR ROS package is used to connect the Oradar MS200 LiDAR. This ROS package supports both ROS and ROS2 environments. For ROS, it supports versions such as Indigo, Kinetic, and Melodic. For ROS2, it supports Ubuntu 20.04 ROS2 Foxy and later versions.

## Usage:

1. Install the ROS environment on your system. Refer to the following links for installation instructions:

   ROS installation link: http://wiki.ros.org/kinetic/Installation/Ubuntu

   ROS2 installation link: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

   **Do not install both ROS and ROS2 on the same computer to avoid potential version conflicts and the hassle of manually installing other libraries.**

2. Copy the `oradar_ros` source code to the `src` directory of your ROS workspace and modify the corresponding files:

   ```
   mkdir -p ~/lidar_ros_ws/src
   cp -ar oradar_ros ~/lidar_ros_ws/src/
   ```

   (1) When using ROS, open the CMakeLists.txt file in the root directory of the oradar_ros source code and change the variable COMPILE_METHOD at the top of the file to CATKIN:

   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD CATKIN)
   ```

   Then copy the package_ros1.xml file and rename it to package.xml.

   (2) When using ROS2, open the CMakeLists.txt file in the root directory of the oradar_ros source code and change the variable COMPILE_METHOD at the top of the file to COLCON:
   
   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD COLCON)
   ```

   Then copy the package_ros2.xml file and rename it to package.xml.
   
   
   Compile the project and set environment variables:
   
   For ROS:
   ```
   cd ~/lidar_ros_ws
   catkin_make
   source devel/setup.sh
   ```

   For ROS2:
   ```
   cd ~/lidar_ros_ws
   colcon build
   source install/setup.bash
   ```

   Configure the serial port port_name and baud rate. The default configuration is port_name as /dev/ttyACM0 and baud rate as 230400.

   Configure LiDAR parameters:
   
   Open oradar_ros/launch/ms200_scan.launch for parameter configuration or oradar_ros/launch/ms200_scan.launch.py for parameter configuration.
   
   Parameter descriptions are as follows:

   | Name        | Type   | Description                                                                                        |
   |-------------|--------|----------------------------------------------------------------------------------------------------|
   | frame_id    | string | Name of the LiDAR coordinate frame. Default is laser_frame                                         |
   | scan_topic  | string | LaserScan topic name. Default is scan                                                              |
   | port_name   | string | LiDAR serial port name. Default is /dev/ttyACM0                                                    |
   | baudrate    | int    | LiDAR serial port baud rate. Default is 230400                                                     |
   | angle_min   | double | Minimum angle in degrees, range [0, 360]. Default is 0                                             |
   | angle_max   | double | Maximum angle in degrees, range [0, 360]. Default is 360                                           |
   | range_min   | double | Minimum range in meters. Default is 0.05                                                           |
   | range_max   | double | Maximum range in meters. Default is 20.0                                                           |
   | clockwise   | bool   | Configures point cloud direction. true for clockwise, false for counterclockwise. Default is false |
   | motor_speed | int    | LiDAR rotation speed in Hz, range [5, 15]. Default is 10Hz                                         |


   Start the Oradar ROS node:
   
   For ROS:
   ```
   roslaunch oradar_lidar ms200_scan.launch
   ```
   Or:
   ```
   roslaunch oradar_lidar ms200_scan_view.launch (to display using RViz)
   ```

   For ROS2:
   ```
   ros2 launch oradar_lidar ms200_scan.launch.py
   ```
   Or:
   ```
   ros2 launch oradar_lidar ms200_scan_view.launch.py (to display using RViz2)
   ```