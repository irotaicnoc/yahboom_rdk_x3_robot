#! /bin/bash

###############################################################################
# ubuntu 20.04 
# add Additional startup programs
# start_app
# bash /root/sunriseRobot/app_SunriseRobot/start_app.sh
# start app program
###############################################################################


sleep 8

# ubuntu 20.04
gnome-terminal -- bash -c "python3 /root/sunriseRobot/app_SunriseRobot/app_SunriseRobot.py;exec bash"


wait
exit 0
