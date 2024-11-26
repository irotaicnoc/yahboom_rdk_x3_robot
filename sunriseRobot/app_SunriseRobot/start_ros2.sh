#! /bin/bash

###############################################################################
# ubuntu 20.04 
# start ros2
# bash /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/start_ros2.sh
# start app program
###############################################################################

# ubuntu 20.04
gnome-terminal -- bash -c "python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/start_ros2.py;exec bash"

wait
exit 0
