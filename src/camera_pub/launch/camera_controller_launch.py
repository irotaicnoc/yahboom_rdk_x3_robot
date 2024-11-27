from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Camera publisher node
    camera_publisher_node = Node(
        package='camera_pub',
        executable='camera_publisher_node',
    )

    # Controller subscriber node
    controller_subscriber_node = Node(
        package='controller_sub',
        executable='controller_subscriber_node',
    )

    # Create list of nodes to launch
    node_list = [
        camera_publisher_node,
        controller_subscriber_node,
    ]

    return LaunchDescription(node_list)
