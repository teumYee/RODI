"""
test_image_publisher.py

동영상 파일 또는 웹캠 영상을 ROS2 카메라 토픽으로 발행해서
Gazebo 없이도 YOLO → 보호자 로직 → TTS 전체 파이프라인을 테스트.

사용 방법:
  # 웹캠(기본):
  ros2 run myrobot_vision test_publisher

  # 동영상 파일:
  ros2 run myrobot_vision test_publisher --ros-args -p video_path:=/path/to/video.mp4

  # 반복 재생 비활성화:
  ros2 run myrobot_vision test_publisher --ros-args -p video_path:=/path/to/video.mp4 -p loop:=false

발행 토픽:
  /myrobot/camera/image_raw   (sensor_msgs/Image, BGR8)
  /camera/depth/image_raw     (sensor_msgs/Image, 32FC1, 모든 픽셀 2.5m 고정)
  /camera/depth/camera_info   (sensor_msgs/CameraInfo)
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')

        self.declare_parameter('video_path', '')
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('loop', True)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('fake_depth_m', 2.5)

        video_path  = self.get_parameter('video_path').value
        use_webcam  = self.get_parameter('use_webcam').value
        self.loop   = self.get_parameter('loop').value
        fps         = self.get_parameter('fps').value
        self.fake_depth = self.get_parameter('fake_depth_m').value

        self.bridge = CvBridge()

        # 카메라 소스 열기
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            self.get_logger().info(f'동영상 파일 사용: {video_path}')
        elif use_webcam:
            self.cap = cv2.VideoCapture(0)
            self.get_logger().info('웹캠 사용 (인덱스 0)')
        else:
            self.cap = None
            self.get_logger().warn(
                'video_path 또는 use_webcam 파라미터가 없습니다. '
                '테스트용 더미 이미지를 발행합니다.'
            )

        # Publishers
        self.rgb_pub   = self.create_publisher(Image, '/myrobot/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.create_timer(1.0 / fps, self._timer_cb)
        self.get_logger().info(
            f'테스트 이미지 발행 시작 ({fps:.0f} fps)\n'
            '  /myrobot/camera/image_raw\n'
            '  /camera/depth/image_raw\n'
            '  /camera/depth/camera_info'
        )

    def _timer_cb(self):
        frame = self._get_frame()
        if frame is None:
            return

        h, w = frame.shape[:2]
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='depth_camera_link')

        # RGB 이미지 발행
        rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        rgb_msg.header = header
        self.rgb_pub.publish(rgb_msg)

        # Depth 이미지 발행 (모든 픽셀을 fake_depth_m으로 설정)
        depth_arr = np.full((h, w), self.fake_depth, dtype=np.float32)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_arr, encoding='32FC1')
        depth_msg.header = header
        self.depth_pub.publish(depth_msg)

        # CameraInfo 발행
        fx = fy = 554.256  # 640x480, 60도 FOV 기준
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.width  = w
        info_msg.height = h
        info_msg.k = [fx, 0.0, w/2, 0.0, fy, h/2, 0.0, 0.0, 1.0]
        info_msg.distortion_model = 'plumb_bob'
        self.info_pub.publish(info_msg)

    def _get_frame(self):
        if self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().info('동영상 재생 완료')
                    return None
            return frame
        else:
            # 더미 이미지: 밝은 회색 배경에 텍스트
            dummy = np.ones((480, 640, 3), dtype=np.uint8) * 180
            cv2.putText(dummy, 'TEST MODE: no video source',
                        (60, 240), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (50, 50, 200), 2)
            cv2.putText(dummy, 'Use --ros-args -p video_path:=<path>',
                        (30, 280), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (50, 50, 200), 1)
            return dummy

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
