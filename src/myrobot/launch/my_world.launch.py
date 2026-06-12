import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 설정
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_myrobot = get_package_share_directory('myrobot')
    pkg_turtlebot3_desc = get_package_share_directory('turtlebot3_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. 파일 및 모델 경로 지정
    world_path = os.path.join(pkg_myrobot, 'worlds', 'mart.world')
    urdf_path = os.path.join(pkg_myrobot, 'urdf', 'my_robot.urdf')
    
    # 🚨 [핵심 추가] 내 패키지 안의 models 폴더 경로 확보
    my_model_path = os.path.join(pkg_myrobot, 'models')

    # URDF 파일 내용 읽어오기
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 🚨 [핵심 추가] 가제보에게 내 패키지의 models 폴더를 감시하라고 명령 환경 변수 지정
    # 기존 가제보 모델 경로에 내 경로를 추가(:)하는 방식입니다.
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + my_model_path
    else:
        gazebo_model_path = my_model_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # 2. 가제보 서버 및 클라이언트 실행
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # 3. 로봇 상태 발행자 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # 4. 가제보에 로봇 스폰 노드
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '25.811', '-y', '-11.9989', '-z', '0.007827'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path, # 🚨 환경 변수 설정을 최우선으로 실행!
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity
    ])