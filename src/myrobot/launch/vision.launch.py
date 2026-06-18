from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    alert_hold = LaunchConfiguration('alert_hold_sec', default='5.0')
    wander_prob = LaunchConfiguration('alert_wander_prob', default='0.25')
    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/overlay/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth_camera/depth/image_raw')
    depth_info_topic = LaunchConfiguration(
        'depth_info_topic',
        default='/camera/depth_camera/depth/camera_info'
    )
    classification_mode = LaunchConfiguration('classification_mode', default='depth')

    child_detector = Node(
        package='myrobot_vision',
        executable='child_detector',
        name='child_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'depth_info_topic': depth_info_topic,
            'classification_mode': classification_mode,
        }],
    )

    guardian_logic = Node(
        package='myrobot_vision',
        executable='guardian_logic',
        name='guardian_logic',
        output='screen',
        emulate_tty=True,
        parameters=[{'alert_hold_sec': alert_hold}],
    )

    model_mover = Node(
        package='myrobot_vision',
        executable='model_mover',
        name='model_mover',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'alert_wander_prob': wander_prob,
            'mart_x_min': -3.0,
            'mart_x_max': 15.0,
            'mart_y_min': -3.0,
            'mart_y_max': 6.0,
            'demo_scenario': True,
        }],
    )

    child_response = Node(
        package='myrobot_vision',
        executable='child_response',
        name='child_response',
        output='screen',
        emulate_tty=True,
    )

    image_overlay = Node(
        package='myrobot_vision',
        executable='image_overlay',
        name='image_overlay',
        output='screen',
        emulate_tty=True,
        parameters=[{'overlay_alpha': 0.85, 'max_render_dist': 8.0}],
    )

    patrol = Node(
        package='myrobot_vision',
        executable='patrol',
        name='patrol_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('alert_hold_sec',    default_value='5.0',
                              description='보호자 부재 경보 임계 시간(초). 데모: 5.0, 실사용: 60.0'),
        DeclareLaunchArgument('alert_wander_prob', default_value='0.25',
                              description='성인 이탈 확률 0~1'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/overlay/image_raw',
                              description='child_detector RGB 입력 토픽'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/depth_camera/depth/image_raw',
                              description='child_detector Depth 입력 토픽(32FC1)'),
        DeclareLaunchArgument('depth_info_topic',
                              default_value='/camera/depth_camera/depth/camera_info',
                              description='child_detector Depth CameraInfo 토픽'),
        DeclareLaunchArgument('classification_mode', default_value='depth',
                              description='어린이/성인 분류 방식: bbox 또는 depth'),
        image_overlay,
        child_detector,
        guardian_logic,
        model_mover,
        child_response,
        patrol,
    ])
