#! /bin/bash

###############################################################################
# ubuntu 20.04 
# add additional startup programs
# main thread
###############################################################################

source /opt/ros/foxy/setup.bash
source /root/marco_ros2_ws/install/local_setup.bash
source /root/software/library_ws/install/local_setup.bash
python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/main_thread.py --gui_mode=False

wait
exit 0