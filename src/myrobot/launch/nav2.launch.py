import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_myrobot = get_package_share_directory('myrobot')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_myrobot, 'maps', 'mart_map.yaml')
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_myrobot, 'config', 'nav2_params.yaml')
    )
    rviz_config = os.path.join(pkg_myrobot, 'rivz', 'mart_nav2.rviz')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'False',
            'autostart': 'true',
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(pkg_myrobot, 'maps', 'mart_map.yaml'),
                              description='Path to map yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_myrobot, 'config', 'nav2_params.yaml'),
                              description='Path to nav2 params'),
        nav2_bringup,
        rviz,
    ])
