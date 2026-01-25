from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('husky_nav2')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        nav2
    ])
