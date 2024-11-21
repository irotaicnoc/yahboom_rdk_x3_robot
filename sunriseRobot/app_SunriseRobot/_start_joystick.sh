#! /bin/bash

###############################################################################
# ubuntu 20.04 
# add Additional startup programs
# start_joystick
# bash /root/sunriseRobot/app_SunriseRobot/start_joystick.sh
# start app program
###############################################################################


sleep 8

# ubuntu 20.04
gnome-terminal -- bash -c "python3 /root/sunriseRobot/app_SunriseRobot/joystick.py;exec bash"


wait
exit 0
