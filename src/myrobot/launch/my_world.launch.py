import os
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _world_with_state_plugin(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        return world_path

    changed = False

    has_state_plugin = any(
        elem.tag == 'plugin' and elem.attrib.get('filename') == 'libgazebo_ros_state.so'
        for elem in world
    )
    if not has_state_plugin:
        plugin = ET.Element('plugin', {
            'name': 'gazebo_ros_state',
            'filename': 'libgazebo_ros_state.so',
        })
        ros = ET.SubElement(plugin, 'ros')
        namespace = ET.SubElement(ros, 'namespace')
        namespace.text = '/gazebo'
        update_rate = ET.SubElement(plugin, 'update_rate')
        update_rate.text = '5.0'
        world.insert(0, plugin)
        changed = True

    # The saved world contains an older my_robot model without RGB/depth cameras.
    # Remove it from the temporary runtime world so spawn_entity owns the only robot.
    for elem in list(world):
        if elem.tag == 'model' and elem.attrib.get('name') == 'my_robot':
            world.remove(elem)
            changed = True

    for state in world.findall('state'):
        for elem in list(state):
            if elem.tag == 'model' and elem.attrib.get('name') == 'my_robot':
                state.remove(elem)
                changed = True

    if not changed:
        return world_path

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.world',
        prefix='mart_state_',
        delete=False,
    )
    tree.write(tmp, encoding='unicode', xml_declaration=False)
    tmp.close()
    return tmp.name


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_myrobot = get_package_share_directory('myrobot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    spawn_x = LaunchConfiguration('spawn_x', default='8.5')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.01')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='1.5708')

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
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
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
        output='screen',
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        set_gazebo_model_path,
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start Gazebo client GUI'),
        DeclareLaunchArgument('spawn_x', default_value='8.5',
                              description='Gazebo robot spawn X'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='Gazebo robot spawn Y'),
        DeclareLaunchArgument('spawn_z', default_value='0.01',
                              description='Gazebo robot spawn Z'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708',
                              description='Gazebo robot spawn yaw'),
        gzserver,
        robot_state_publisher,
        spawn_robot,
        gzclient_after_load,
    ])
