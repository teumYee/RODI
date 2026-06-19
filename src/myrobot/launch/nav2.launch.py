import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_myrobot = get_package_share_directory('myrobot')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_enabled = LaunchConfiguration('rviz', default='true')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose', default='true')
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
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    publish_initial_pose = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'sleep 6 && ros2 topic pub --once /initialpose '
            'geometry_msgs/msg/PoseWithCovarianceStamped '
            '"{header: {frame_id: map}, pose: {pose: {position: {x: 8.5, y: 0.0, z: 0.0}, '
            'orientation: {z: 0.7071068, w: 0.7071068}}, '
            'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"'
        ],
        output='screen',
        condition=IfCondition(auto_initial_pose),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true',
                              description='Publish demo initial pose to AMCL'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(pkg_myrobot, 'maps', 'mart_map.yaml'),
                              description='Path to map yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_myrobot, 'config', 'nav2_params.yaml'),
                              description='Path to nav2 params'),
        nav2_bringup,
        publish_initial_pose,
        rviz,
    ])
