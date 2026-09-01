from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# husky_ai_pkg = get_package_share_directory('husky_ai_inspection')
husky_nav2_pkg = get_package_share_directory('husky_nav2')
husky_gazebo_pkg = get_package_share_directory('husky_gazebo')
nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

def generate_launch_description():

    # --------------------
    # Launch arguments
    # --------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world_path')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_world = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(
            husky_gazebo_pkg,
            'worlds',
            'construction_site_fire_extuinguisher.world'
        )
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            husky_nav2_pkg,
            'maps',
            'small_factory.yaml'
        )
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            husky_nav2_pkg,
            'config',
            'nav2_params.yaml'
        )
    )

    # --------------------
    # Gazebo
    # --------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                husky_gazebo_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world_path': world_path
        }.items()
    )

    # --------------------
    # Nav2 Localization
    # --------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_pkg,
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params
        }.items()
    )

    # --------------------
    # Initial pose node
    # --------------------
    initial_pose_node = Node(
        package='husky_init_pose',
        executable='auto_initial_pose',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------------------
    # Nav2 Navigation
    # --------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_pkg,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params
        }.items()
    )

    # --------------------
    # Waypoint executor
    # --------------------
    waypoint_node = Node(
        package='husky_waypoint_recorder',
        executable='go_to_waypoint',
        output='screen'
    )

    # --------------------
    # Vision / camera service
    # --------------------
    vision_node = Node(
        package='husky_vision_detector',
        executable='image_capture_service',
        output='screen'
    )

    yolo_node = Node(
        package='husky_vision_detector',
        executable='yolo_fire_extinguisher_node',
        output='screen'
    )
    mission_bt_node = Node(
        package='husky_mission_bt',
        executable='mission_bt_node',
        output='screen'
    )
    mission_interface_node = Node(
        package='husky_mission_interface',
        executable='mission_interface_node',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_map,
        declare_params,

        gazebo_launch,
        localization_launch,
        initial_pose_node,
        navigation_launch,
        waypoint_node,
        vision_node,
        yolo_node,
        mission_bt_node,
        mission_interface_node
    ])
