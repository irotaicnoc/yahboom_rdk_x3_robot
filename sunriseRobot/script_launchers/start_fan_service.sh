#! /bin/bash

###############################################################################
# ubuntu 20.04
# start fan
# main thread
###############################################################################

gnome-terminal -- bash -c "python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/fan.py;exec bash"

exit 0