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
    print(f'Local IP {ip_address}')
    print('Starting server_node...', end='')
    server_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": ip_address}, {"ROS_TCP_PORT": 10000}],
    )
    print('Done.')

    print('Starting controller_subscriber_node...', end='')
    time.sleep(1)
    controller_subscriber_node = Node(
        package='controller_sub',
        executable='controller_subscriber_node',
    )
    print('Done.')
    print('Starting camera_publisher_node...', end='')
    time.sleep(1)
    camera_publisher_node = Node(
        package='camera_pub',
        executable='camera_publisher_node',
    )
    print('Done.')

    node_list = [
        server_node,
        controller_subscriber_node,
        camera_publisher_node,
    ]
    return LaunchDescription(node_list)

