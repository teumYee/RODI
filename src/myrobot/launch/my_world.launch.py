import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_with_state_plugin(world_path):
    with open(world_path, 'r') as infp:
        world_xml = infp.read()

    if 'libgazebo_ros_state.so' in world_xml:
        return world_path

    plugin_xml = '''
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>5.0</update_rate>
    </plugin>
'''
    insert_at = world_xml.find('>', world_xml.find('<world'))
    if insert_at == -1:
        return world_path

    patched_world = world_xml[:insert_at + 1] + plugin_xml + world_xml[insert_at + 1:]
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.world',
        prefix='mart_state_',
        delete=False,
    )
    tmp.write(patched_world)
    tmp.close()
    return tmp.name


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_myrobot = get_package_share_directory('myrobot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_path = _world_with_state_plugin(
        os.path.join(pkg_myrobot, 'worlds', 'mart.world')
    )
    urdf_path = os.path.join(pkg_myrobot, 'urdf', 'my_robot.urdf')
    my_model_path = os.path.join(pkg_myrobot, 'models')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + my_model_path
    else:
        gazebo_model_path = my_model_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # 서버만 먼저 실행
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot',
                   '-spawn_service_timeout', '30.0',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
        output='screen',
    )

    # /spawn_entity 서비스가 올라올 때까지 기다린 후 gzclient 실행
    # → 월드가 완전히 로딩된 뒤에야 GUI가 열리므로 "not responding" 안 뜸
    gzclient_after_load = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "[launch] Gazebo 서버 로딩 대기 중..." && '
            'until ros2 service list 2>/dev/null | grep -q "/spawn_entity"; '
            'do sleep 2; done && '
            'echo "[launch] 로딩 완료! gzclient 실행" && '
            'gzclient'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        gzserver,
        robot_state_publisher,
        spawn_robot,
        gzclient_after_load,
    ])
