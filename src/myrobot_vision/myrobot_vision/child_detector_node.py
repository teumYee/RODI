"""
child_detector_node.py

RGB + Depth 카메라 동기화 → YOLOv8 사람 감지 → bbox/depth 기반 어린이/성인 분류.

발행 토픽: /detected_persons  (std_msgs/String, JSON)
"""

import json
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO


# bbox 기반 기본 임계값: Gazebo overlay 인물 기준으로 조정 가능
CHILD_HEIGHT_PX      = 260
CHILD_ASPECT_MAX     = 5.0   # person_standing 메쉬는 세로로 길어 aspect 높음
# depth 기반 실제 키(m) 임계값: 1.6m 미만 → 어린이 (0.6 스케일 모델 ~1.08m, 여유 포함)
CHILD_HEIGHT_M       = 1.6
CONFIDENCE_THRESHOLD = 0.50  # 데모 영상에서 중복/노이즈 감지 제거용
CHILD_NOMINAL_HEIGHT_M = 1.10
ADULT_NOMINAL_HEIGHT_M = 1.70


class ChildDetectorNode(Node):
    def __init__(self):
        super().__init__('child_detector')

        self.bridge = CvBridge()

        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value or self._find_model()
        self.get_logger().info(f'YOLOv8 모델 로딩 중: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLOv8 모델 로딩 완료')

        # 카메라 내부 파라미터 (camera_info로 업데이트)
        self.fx = 554.256
        self.fy = 554.256
        self.cx = 320.0
        self.cy = 240.0
        self.has_intrinsics = False

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.declare_parameter('rgb_topic', '/myrobot/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('classification_mode', 'bbox')
        self.declare_parameter('child_height_px', CHILD_HEIGHT_PX)
        self.declare_parameter('child_aspect_max', CHILD_ASPECT_MAX)
        self.declare_parameter('child_height_m', CHILD_HEIGHT_M)
        self.declare_parameter('child_nominal_height_m', CHILD_NOMINAL_HEIGHT_M)
        self.declare_parameter('adult_nominal_height_m', ADULT_NOMINAL_HEIGHT_M)

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        self.classification_mode = self.get_parameter('classification_mode').value
        self.child_height_px = float(self.get_parameter('child_height_px').value)
        self.child_aspect_max = float(self.get_parameter('child_aspect_max').value)
        self.child_height_m = float(self.get_parameter('child_height_m').value)
        self.child_nominal_h = float(
            self.get_parameter('child_nominal_height_m').value
        )
        self.adult_nominal_h = float(
            self.get_parameter('adult_nominal_height_m').value
        )

        if self.classification_mode not in ('bbox', 'depth'):
            self.get_logger().warn(
                f'알 수 없는 classification_mode={self.classification_mode}, bbox 사용'
            )
            self.classification_mode = 'bbox'

        self.get_logger().info(f'RGB 입력 토픽: {rgb_topic}')
        self.get_logger().info(f'Depth 입력 토픽: {depth_topic}')
        self.get_logger().info(f'Depth CameraInfo 토픽: {depth_info_topic}')
        self.get_logger().info(
            f'분류 모드: {self.classification_mode} | '
            f'bbox child_h<{self.child_height_px:.0f}px aspect<{self.child_aspect_max:.2f} | '
            f'depth child_h<{self.child_height_m:.2f}m'
        )

        self.create_subscription(
            CameraInfo, depth_info_topic,
            self._camera_info_cb, 10
        )

        rgb_sub   = message_filters.Subscriber(self, Image, rgb_topic,
                                               qos_profile=best_effort_qos)
        depth_sub = message_filters.Subscriber(self, Image,
                                               depth_topic,
                                               qos_profile=best_effort_qos)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.2
        )
        self.ts.registerCallback(self._detection_cb)

        self.pub = self.create_publisher(String, '/detected_persons', 10)
        self._escorting = False
        self.create_subscription(String, '/escort_status', self._escort_cb, 10)
        self.get_logger().info('Child detector 시작. RGB+Depth 동기화 대기 중...')

    # ── 에스코트 상태 ─────────────────────────────────────────────────────
    def _escort_cb(self, msg: String):
        self._escorting = (msg.data == 'busy')
        self.get_logger().info(
            f'[detector] 에스코트 {"시작 → 감지 중단" if self._escorting else "종료 → 감지 재개"}'
        )

    # ── 카메라 내부 파라미터 ───────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if not self.has_intrinsics:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.has_intrinsics = True
            self.get_logger().info(
                f'카메라 내부 파라미터 수신: fx={self.fx:.1f} fy={self.fy:.1f}'
            )

    # ── RGB + Depth 동기화 콜백 ───────────────────────────────────────────
    def _detection_cb(self, rgb_msg: Image, depth_msg: Image):
        if self._escorting:
            return

        try:
            frame     = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg,
                                                  desired_encoding='passthrough')
            depth = np.array(depth_raw, dtype=np.float32)

            img_h, img_w = frame.shape[:2]

            results = self.model(frame, classes=[0], verbose=False)

            # 임계값 미만 박스도 로그 출력 (디버깅용)
            all_boxes = results[0].boxes
            if all_boxes is not None and len(all_boxes) > 0:
                low_conf = [float(b.conf[0]) for b in all_boxes
                            if float(b.conf[0]) < CONFIDENCE_THRESHOLD]
                if low_conf:
                    self.get_logger().debug(
                        f'임계값 미만 감지 {len(low_conf)}개: '
                        f'conf={[round(c,2) for c in low_conf]}'
                    )

            persons = []
            for box in results[0].boxes:
                conf = float(box.conf[0])
                if conf < CONFIDENCE_THRESHOLD:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                bbox_w = max(1, x2 - x1)
                bbox_h = max(1, y2 - y1)

                cx_px = np.clip((x1 + x2) // 2, 0, img_w - 1)
                cy_px = np.clip((y1 + y2) // 2, 0, img_h - 1)

                # bbox 중심 5×5 패치 중앙값으로 depth 노이즈 제거
                r = 2
                patch = depth[
                    max(0, cy_px - r):min(img_h, cy_px + r + 1),
                    max(0, cx_px - r):min(img_w,  cx_px + r + 1)
                ]
                valid = patch[np.isfinite(patch) & (patch > 0) & (patch < 10.0)]
                z = float(np.median(valid)) if valid.size > 0 else -1.0
                depth_source = 'depth'

                if z <= 0:
                    prelim_label, _ = self._classify_person_bbox(bbox_w, bbox_h)
                    nominal_h = (
                        self.child_nominal_h
                        if prelim_label == 'child'
                        else self.adult_nominal_h
                    )
                    z = float(self.fy * nominal_h / bbox_h)
                    z = float(np.clip(z, 0.3, 10.0))
                    depth_source = 'bbox_estimate'

                # 3D 위치 (카메라 좌표계)
                if z > 0:
                    X = (cx_px - self.cx) * z / self.fx
                    Y = (cy_px - self.cy) * z / self.fy
                else:
                    X, Y = 0.0, 0.0

                label, reason = self._classify_person(bbox_w, bbox_h, z)
                if depth_source != 'depth':
                    reason = f'{reason} {depth_source}'

                persons.append({
                    'label':      label,
                    'confidence': round(conf, 3),
                    'bbox':       [x1, y1, x2, y2],
                    'depth_m':    round(z, 3),
                    'pos3d':      [round(X, 3), round(Y, 3), round(z, 3)],
                    'class_reason': reason,
                })

            out = String()
            out.data = json.dumps({
                'persons':   persons,
                'stamp_sec': rgb_msg.header.stamp.sec,
            })
            self.pub.publish(out)

            if persons:
                children = sum(1 for p in persons if p['label'] == 'child')
                adults   = sum(1 for p in persons if p['label'] == 'adult')
                detail = ' | '.join(
                    f"{p['label']}(conf={p['confidence']} {p['class_reason']})"
                    for p in persons
                )
                self.get_logger().info(
                    f'감지: 어린이 {children}명 / 성인 {adults}명 → {detail}'
                )

        except Exception as e:
            self.get_logger().error(f'감지 오류: {e}')

    def _classify_person(self, bbox_w: int, bbox_h: int, z: float):
        aspect = bbox_h / bbox_w
        if self.classification_mode == 'depth' and z > 0:
            real_height_m = bbox_h * z / self.fy
            label = 'child' if real_height_m < self.child_height_m else 'adult'
            return label, f'depth_height_m={real_height_m:.2f}'

        return self._classify_person_bbox(bbox_w, bbox_h)

    def _classify_person_bbox(self, bbox_w: int, bbox_h: int):
        aspect = bbox_h / bbox_w
        is_child = bbox_h < self.child_height_px and aspect < self.child_aspect_max
        label = 'child' if is_child else 'adult'
        return label, f'bbox_h={bbox_h}px aspect={aspect:.2f}'

    @staticmethod
    def _find_model() -> str:
        candidates = [
            os.path.join(os.path.expanduser('~'), 'RODI', 'yolov8n.pt'),
            os.path.join(os.path.dirname(__file__),
                         '..', '..', '..', '..', 'yolov8n.pt'),
            'yolov8n.pt',
        ]
        for p in candidates:
            if os.path.isfile(p):
                return os.path.abspath(p)
        return 'yolov8n.pt'


def main(args=None):
    rclpy.init(args=args)
    node = ChildDetectorNode()
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
