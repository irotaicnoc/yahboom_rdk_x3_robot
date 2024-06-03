import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'),
         '/laser_bringup_launch.py'
        ])
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'),
         '/cartographer_launch.py'
        ])
    )

    return LaunchDescription([laser_bringup_launch, cartographer_launch])