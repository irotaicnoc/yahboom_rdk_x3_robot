import os
import time

from launch_ros.actions import Node
from launch import LaunchDescription


# Read the local IP address
def get_local_ip():
    ip = os.popen(
        "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0: ip.find('\n')]
    # ip = ''
    if ip == '' or len(ip) > 15:
        ip = os.popen(
            "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\n')]
        if ip == '':
            ip = 'x.x.x.x'
    if len(ip) > 15:
        ip = 'x.x.x.x'
    return ip


def generate_launch_description():
    ip_address = get_local_ip()
    print(f'Local IP {ip_address} (server will bind on all interfaces)')
    print('Starting server_node...', end='')
    server_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        # Bind on 0.0.0.0 so the server is reachable on every interface
        # (eth0, wlan0, hotspot, tailscale0, ...). Without this the bind
        # is pinned to the eth0/wlan0 address and remote-network clients
        # (e.g. VR/mobile over Tailscale) cannot connect.
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )
    print('Done')

    print('Starting camera_publisher_node...', end='')
    time.sleep(1)
    camera_publisher_node = Node(
        package='camera_pub',
        executable='camera_publisher_node',
    )
    print('Done.')

    print('Starting arm_camera_relay_node...', end='')
    time.sleep(1)
    # Relays the Jetson-attached arm camera over the wired link and republishes it as 'arm_camera_stream'.
    arm_camera_relay_node = Node(
        package='camera_pub',
        executable='arm_camera_relay_node',
    )
    print('Done.')

    node_list = [
        server_node,
        camera_publisher_node,
        arm_camera_relay_node,
    ]
    return LaunchDescription(node_list)

