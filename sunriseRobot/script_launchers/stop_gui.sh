# This command stops the graphical user interface and switches to a multi-user target.
systemctl isolate multi-user.target
# This will also kill the main robot control thread, so it needs to be run after the GUI is stopped.
source /opt/ros/foxy/setup.bash
source /root/marco_ros2_ws/install/local_setup.bash
source /root/software/library_ws/install/local_setup.bash
python3 /root/GIT/yahboom_rdk_x3_robot/sunriseRobot/app_SunriseRobot/main_thread.py --gui_mode=False

exit 0