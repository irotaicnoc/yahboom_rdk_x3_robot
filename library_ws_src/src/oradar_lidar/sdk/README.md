
# MS200 SDK Basic Introduction
The MS200 SDK is a software development kit specifically designed for the Oradar MS200 LiDAR product. It provides an
easy-to-use C/C++ style API. With the MS200 SDK, users can quickly connect to the Oradar MS200 LiDAR and receive LiDAR
point cloud data. 

# Requirements
- Linux system：Ubuntu 14.04 LTS, Ubuntu 16.04 LTS, Ubuntu 18.04 LTS
- Windows 7/10
- C++ 11 compiler
- CMake，version 3.5 or higher

# Compilation and Installation Method
First, extract the SDK package. The extracted folder name is sdk.

On Linux, use the following commands:

```
cd sdk
mkdir build
cd build
cmake ..    (you can use cmake -DCMAKE_INSTALL_PREFIX=out .. to specify the installation directory as the current path's out directory)
make
sudo make install
```

On Windows：

(Here, taking Windows 10 system and QT's MinGW compiler as an example, you need to import the compiler installation
path into the system environment variables).
Hold the shift key, right-click, and open PowerShell

```
cd sdk
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=out ..
mingw32-make -j8
mingw32-make install
```

Generate liboradar_sdk.a library file, blocking_test and non-blocking_test executable files

# SDK Main Function API Description
| Function Name        | Function Description                                                                                                                                     |
|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| Connect              | Check and open the LiDAR serial port, create LiDAR serial port read/write thread                                                                         |
| Disconnect           | Close the LiDAR serial port read/write thread, close the serial port                                                                                     |
| GrabOneScan          | Get the latest packet of point cloud data, non-blocking. The point cloud data includes the angle, distance, and intensity information of all points      |
| GrabOneScanBlocking  | Get the latest packet of point cloud data, blocking. The point cloud data includes the angle, distance, and intensity information of all points          |
| GrabFullScan         | Get the latest full circle of point cloud data, non-blocking. The point cloud data includes the angle, distance, and intensity information of all points |
| GrabFullScanBlocking | Get the latest full circle of point cloud data, blocking. The point cloud data includes the angle, distance, and intensity information of all points     |
| GetRotationSpeed     | Get the latest motor speed                                                                                                                               |
| SetRotationSpeed     | Set the motor speed                                                                                                                                      |
| GetTimestamp         | Get the timestamp of the latest packet                                                                                                                   |
| GetFirmwareVersion   | Get the firmware version number of the upper and lower parts                                                                                             |
| GetDeviceSN          | Get the LiDAR device SN number                                                                                                                           |
| Activate             | The LiDAR enters the ranging state from the standby state                                                                                                |
| Deactive             | The LiDAR enters the standby state from the ranging state                                                                                                |


# Example Usage Instructions
On Linux：

Connect the MS200 LiDAR device to the Ubuntu system via a USB to serial cable. Open the terminal in the Ubuntu system
and enter ls /dev/ttyACM* to check if the serial device is connected. If the serial device is detected, use the sudo
chmod 777 /dev/ttyACM* command to grant the highest permissions. Then execute the SDK Sample, enter the following
command:

```
cd sdk/build
./blocking_test                 # Blocking test program to get a full circle of data
```
or
```
./non-blocking_test             # Non-blocking test program to get a full circle of data
```

Note: If the command ls /dev/ttyACM* detects a device, and * is not 0, you need to replace the device name /dev/ttyACM0
in the Sample test code with the corresponding device name (modify the port_name variable in the code).

On Windwos：

Connect the MS200 LiDAR device to the Windows system PC via a USB to serial cable. Check the serial port name through
the device manager, such as com10, and modify the port_name variable in the sample code to com10, then recompile. Then
double-click the blocking_test.exe or non-blocking_test.exe or blocking_c_api_test.exe executable program to run it.