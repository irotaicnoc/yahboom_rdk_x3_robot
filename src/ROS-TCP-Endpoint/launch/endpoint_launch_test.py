import launch
from launch import LaunchDescription
import launch_ros.events.lifecycle

import os


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
    server_node = launch_ros.actions.LifecycleNode(
        name='server_node',
        namespace='',
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        emulate_tty=True,
        parameters=[{'ROS_IP': ip_address}, {'ROS_TCP_PORT': 10000}],
    )
    print('Done.')

    # When the server_node reaches the 'active' state, log a message and start the
    # controller_subscriber_node and camera_publisher_node.
    register_event_handler_for_server_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=server_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(msg="'server_node' reached the 'active' state. Launching other nodes."),
                launch_ros.actions.LifecycleNode(
                    name='controller_subscriber_node',
                    namespace='',
                    package='controller_sub',
                    executable='controller_subscriber_node',
                    ),
                launch_ros.actions.LifecycleNode(
                    name='camera_publisher_node',
                    namespace='',
                    package='camera_pub',
                    executable='camera_publisher_node',
                    ),
            ],
        )
    )

    node_list = [
        server_node,
        register_event_handler_for_server_reaches_active_state,
        # controller_subscriber_node,
        # camera_publisher_node,
    ]
    return LaunchDescription(node_list)

