import time

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    print('Starting server_node...', end='')
    server_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_TCP_PORT": 10000}],
    )
    print('Done')

    print('Starting camera_publisher_node...', end='')
    time.sleep(1)
    camera_publisher_node = Node(
        package='camera_pub',
        executable='camera_publisher_node',
    )
    print('Done.')

    node_list = [
        server_node,
        camera_publisher_node,
    ]
    return LaunchDescription(node_list)
