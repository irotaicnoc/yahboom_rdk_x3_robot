#! /bin/bash

###############################################################################
# ubuntu 20.04 
# add additional startup programs
# main thread
###############################################################################

gnome-terminal -- bash -c "python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/main_thread.py;exec bash"

wait
exit 0