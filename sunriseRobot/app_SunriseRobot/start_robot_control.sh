#! /bin/bash

###############################################################################
# ubuntu 20.04 
# add Additional startup programs
# main thread
# bash /root/sunriseRobot/app_SunriseRobot/main_thread.sh
# start app program
###############################################################################


sleep 2

# ubuntu 20.04
gnome-terminal -- bash -c "python3 /root/sunriseRobot/app_SunriseRobot/main_thread.py;exec bash"


wait
exit 0
