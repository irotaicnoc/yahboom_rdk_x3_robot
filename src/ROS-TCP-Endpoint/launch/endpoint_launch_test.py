import launch
from launch import LaunchDescription
import launch_ros.events.lifecycle


def generate_launch_description():
    print('Starting server_node...', end='')
    server_node = launch_ros.actions.LifecycleNode(
        name='server_node',
        namespace='',
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        emulate_tty=True,
        parameters=[{'ROS_TCP_PORT': 10000}],
    )
    print('Done')

    # When the server_node reaches the 'active' state, log a message and start the camera_publisher_node.
    register_event_handler_for_server_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=server_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(msg="'server_node' reached the 'active' state. Launching other nodes"),
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
        # camera_publisher_node,
    ]
    return LaunchDescription(node_list)
