#! /bin/bash

###############################################################################
# ubuntu 20.04 
# start ros2:
# - import ros2
# - go to project workspace
# - import project
# - execute launch file:
#   - start tcp server
#   - start controller subscriber (listener)
#   - start camera publisher (sender)

###############################################################################

gnome-terminal -- bash -c "source /opt/ros/foxy/setup.bash;cd /root/marco_ros2_ws/;source install/local_setup.bash;ros2 launch ros_tcp_endpoint endpoint_launch.py;exec bash"

wait
exit 0