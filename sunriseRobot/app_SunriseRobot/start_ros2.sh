#! /bin/bash

###############################################################################
# ubuntu 20.04 
# start ros2
# bash /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/start_ros2.sh
# start app program
###############################################################################

sleep 1

# ubuntu 20.04
gnome-terminal -- bash -c "cd /root/marco_ros2_ws/;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"

wait
exit 0
