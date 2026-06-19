"""
image_overlay_node.py  ── "덧입히기" 핵심 노드

동작 원리
─────────────────────────────────────────────────────────────────────
1. Gazebo /model_states → 각 사람 모델의 월드 좌표 획득
2. /gazebo/model_states 중 로봇(my_robot) 의 pose → 카메라 월드 위치·자세 계산
3. 각 사람 모델 위치를 카메라 좌표계로 변환 → 이미지 픽셀 투영
4. 거리(Z)에 맞게 크기 조정한 투명 PNG 를 Gazebo 카메라 영상 위에 합성
5. 합성 결과를 /camera/overlay/image_raw 로 발행
   → child_detector_node 가 이 토픽을 YOLO 입력으로 사용

토픽 흐름
─────────────────────────────────────────────────────────────────────
  /myrobot/camera/image_raw  ──┐
  /gazebo/model_states        ─┤→ [image_overlay_node] → /camera/overlay/image_raw
  /person_model_positions     ─┘                              ↓
                                                     child_detector_node (YOLO)

파라미터
─────────────────────────────────────────────────────────────────────
  camera_offset_x/y/z  : 카메라가 base_footprint 기준 오프셋 (URDF 값)
  overlay_alpha        : 합성 투명도 0~1 (기본 0.85)
  min_render_dist      : 이 거리 이하 모델은 무시 (기본 0.5m)
  max_render_dist      : 이 거리 이상 모델은 무시 (기본 8.0m)
  fx, fy, cx0, cy0     : 카메라 내부 파라미터 (camera_info 로 자동 갱신)
"""

import json
import math
import os

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge


# ── 사람 모델 정보 테이블 ─────────────────────────────────────────────────
# (모델명, 빌보드 높이[m], 오버레이 PNG 경로 키)
PERSON_MODELS = {
    'person_adult_1':  (1.70, 'adult1'),
    'person_adult_2':  (1.65, 'adult2'),
    'person_adult_3':  (1.72, 'adult3'),
    'person_senior_1': (1.60, 'senior1'),
    'person_child_1':  (1.00, 'child1'),
    'person_child_2':  (1.10, 'child2'),
    'person_child_3':  (1.05, 'child3'),
    'adult_guard_1':   (1.70, 'adult1'),
    'adult_guard_2':   (1.65, 'adult2'),
    'adult_guard_3':   (1.72, 'adult3'),
    'adult_guard_4':   (1.70, 'adult1'),
    'adult_solo_1':    (1.65, 'adult2'),
    'adult_solo_2':    (1.72, 'adult3'),
    'child_1':         (1.00, 'child1'),
    'child_2':         (1.10, 'child2'),
    'child_3':         (1.05, 'child3'),
    'child_4':         (1.00, 'child1'),
}

# 카메라 오프셋 (URDF: camera_joint xyz="0.1 0.0 0.2", base_joint z=0.010)
CAM_OFFSET_X = 0.1
CAM_OFFSET_Z = 0.21   # 0.010 + 0.2


class ImageOverlayNode(Node):
    def __init__(self):
        super().__init__('image_overlay')

        self.declare_parameter('overlay_alpha',    0.85)
        self.declare_parameter('min_render_dist',  0.5)
        self.declare_parameter('max_render_dist',  8.0)
        self.declare_parameter('fx',  554.256)
        self.declare_parameter('fy',  554.256)
        self.declare_parameter('cx0', 320.0)
        self.declare_parameter('cy0', 240.0)

        self.alpha    = self.get_parameter('overlay_alpha').value
        self.min_d    = self.get_parameter('min_render_dist').value
        self.max_d    = self.get_parameter('max_render_dist').value
        self.fx       = self.get_parameter('fx').value
        self.fy       = self.get_parameter('fy').value
        self.cx0      = self.get_parameter('cx0').value
        self.cy0      = self.get_parameter('cy0').value

        self.bridge   = CvBridge()

        # 오버레이 PNG 로드 (투명 배경)
        self._overlays = self._load_overlays()

        # 모델 위치 캐시 {모델명: (wx, wy, wz, yaw)}
        self._model_poses: dict[str, tuple] = {}
        self._completed_children: set[str] = set()
        # 로봇 pose
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0

        be_qos = QoSProfile(depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)

        # Subscribers
        self.create_subscription(Image,       '/myrobot/camera/image_raw',
                                 self._img_cb,         be_qos)
        self.create_subscription(CameraInfo,  '/myrobot/camera/camera_info',
                                 self._info_cb,        10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self._model_states_cb, 10)
        self.create_subscription(String, '/completed_child',
                                 self._completed_child_cb, 10)

        # Publisher
        self.pub = self.create_publisher(Image, '/camera/overlay/image_raw', 10)

        self.get_logger().info(
            f'image_overlay 시작 | '
            f'alpha={self.alpha} | '
            f'거리 {self.min_d}~{self.max_d}m | '
            f'모델 {len(self._overlays)}종 PNG 로드됨'
        )

    def _completed_child_cb(self, msg: String):
        name = msg.data.strip()
        if not name:
            return
        self._completed_children.add(name)
        self.get_logger().info(f'완료된 아이 오버레이 제외: {name}')

    # ── PNG 로드 ──────────────────────────────────────────────────────────
    def _load_overlays(self) -> dict[str, np.ndarray]:
        """각 사람 모델의 투명 PNG(BGRA)를 미리 로드."""
        overlays = {}

        # 탐색 우선순위: ament share 설치 경로 → src 소스 경로
        candidate_bases = []
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('myrobot')
            candidate_bases.append(os.path.join(share, 'models'))
        except Exception:
            pass

        script_dir = os.path.dirname(os.path.abspath(__file__))
        src_models  = os.path.normpath(
            os.path.join(script_dir, '..', '..', '..', '..', 'src', 'myrobot', 'models'))
        candidate_bases.append(src_models)

        name_map = {
            'adult1': 'adult1_texture_overlay.png',
            'adult2': 'adult2_texture_overlay.png',
            'adult3': 'adult3_texture_overlay.png',
            'senior1': 'senior1_texture_overlay.png',
            'child1': 'child1_texture_overlay.png',
            'child2': 'child2_texture_overlay.png',
            'child3': 'child3_texture_overlay.png',
        }
        model_key_map = {
            'person_adult_1': 'adult1', 'person_adult_2': 'adult2',
            'person_adult_3': 'adult3', 'person_senior_1': 'senior1',
            'person_child_1': 'child1', 'person_child_2': 'child2',
            'person_child_3': 'child3',
            'adult_guard_1': 'adult1', 'adult_guard_2': 'adult2',
            'adult_guard_3': 'adult3', 'adult_guard_4': 'adult1',
            'adult_solo_1': 'adult2', 'adult_solo_2': 'adult3',
            'child_1': 'child1', 'child_2': 'child2',
            'child_3': 'child3', 'child_4': 'child1',
        }

        for model_name, key in model_key_map.items():
            png_name = name_map[key]
            asset_model_name = self._asset_model_name(model_name)
            found = False
            for base in candidate_bases:
                path = os.path.join(base, asset_model_name,
                                    'materials', 'textures', png_name)
                if os.path.exists(path):
                    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
                    if img is not None:
                        overlays[model_name] = img
                        found = True
                        break
            if not found:
                self.get_logger().warn(
                    f'오버레이 PNG 없음: {model_name} — '
                    '먼저 generate_person_textures.py 를 실행하세요'
                )
        return overlays

    @staticmethod
    def _asset_model_name(model_name: str) -> str:
        asset_map = {
            'adult_guard_1': 'person_adult_1',
            'adult_guard_2': 'person_adult_2',
            'adult_guard_3': 'person_adult_3',
            'adult_guard_4': 'person_adult_1',
            'adult_solo_1': 'person_adult_2',
            'adult_solo_2': 'person_adult_3',
            'child_1': 'person_child_1',
            'child_2': 'person_child_2',
            'child_3': 'person_child_3',
            'child_4': 'person_child_1',
        }
        return asset_map.get(model_name, model_name)

    # ── CameraInfo 수신 ───────────────────────────────────────────────────
    def _info_cb(self, msg: CameraInfo):
        self.fx  = msg.k[0]
        self.fy  = msg.k[4]
        self.cx0 = msg.k[2]
        self.cy0 = msg.k[5]

    # ── model_states 수신 ────────────────────────────────────────────────
    def _model_states_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name == 'my_robot':
                q = pose.orientation
                self._robot_x   = pose.position.x
                self._robot_y   = pose.position.y
                # 쿼터니언 → yaw
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._robot_yaw = math.atan2(siny, cosy)

            if name in PERSON_MODELS:
                q   = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw  = math.atan2(siny, cosy)
                self._model_poses[name] = (
                    pose.position.x, pose.position.y, pose.position.z, yaw
                )

    # ── 카메라 이미지 수신 → 합성 ─────────────────────────────────────────
    def _img_cb(self, msg: Image):
        if not self._model_poses:
            self.pub.publish(msg)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            self.pub.publish(msg)
            return

        frame = self._composite(frame)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

    # ── 합성 메인 ────────────────────────────────────────────────────────
    def _composite(self, frame: np.ndarray) -> np.ndarray:
        img_h, img_w = frame.shape[:2]

        # 카메라 월드 위치
        yaw   = self._robot_yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        cam_wx = self._robot_x + CAM_OFFSET_X * cos_y
        cam_wy = self._robot_y + CAM_OFFSET_X * sin_y
        cam_wz = CAM_OFFSET_Z

        # 카메라 축 (월드 좌표)
        # Z_cam = 로봇 전방
        fwd = np.array([cos_y,  sin_y, 0.0])
        rgt = np.array([sin_y, -cos_y, 0.0])   # 오른쪽
        up  = np.array([0.0,    0.0,   1.0])    # 위 (이미지 y↓ 이므로 부호 반전)

        result = frame.copy()

        # 거리순 정렬 (먼 것부터 그려야 앞 것이 위에 올라옴)
        sorted_models = sorted(
            self._model_poses.items(),
            key=lambda item: -self._depth_to_model(item[1], cam_wx, cam_wy, fwd),
        )

        for model_name, (wx, wy, wz, _) in sorted_models:
            if model_name in self._completed_children:
                continue
            if model_name not in self._overlays:
                continue

            h_world, key = PERSON_MODELS[model_name]

            # 모델 중심 (세계 좌표)
            model_center_z = wz + h_world / 2.0

            # 카메라 좌표계 변환
            dx = wx - cam_wx
            dy = wy - cam_wy
            dz = model_center_z - cam_wz

            Zc = dx * fwd[0] + dy * fwd[1] + dz * fwd[2]   # 깊이
            Xc = dx * rgt[0] + dy * rgt[1]                   # 수평
            Yc = -(dz)                                        # 수직 (이미지 아래↓)

            if Zc < self.min_d or Zc > self.max_d:
                continue

            # 이미지 픽셀 투영
            u = int(self.fx * Xc / Zc + self.cx0)
            v = int(self.fy * Yc / Zc + self.cy0)

            # 빌보드 높이 → 픽셀 높이
            ph = int(self.fy * h_world / Zc)
            if ph < 10:
                continue
            pw = int(ph * self._overlays[model_name].shape[1]
                     / self._overlays[model_name].shape[0])

            # 오버레이 PNG 리사이즈
            png = cv2.resize(self._overlays[model_name], (pw, ph))

            # 붙일 위치 (중앙 하단 기준)
            x0 = u - pw // 2
            y0 = v - ph // 2
            x1 = x0 + pw
            y1 = y0 + ph

            # 클리핑
            sx0 = max(0, -x0);  sx1 = pw - max(0, x1 - img_w)
            sy0 = max(0, -y0);  sy1 = ph - max(0, y1 - img_h)
            dx0 = max(0, x0);   dx1 = min(img_w, x1)
            dy0 = max(0, y0);   dy1 = min(img_h, y1)

            if sx1 <= sx0 or sy1 <= sy0 or dx1 <= dx0 or dy1 <= dy0:
                continue

            roi  = result[dy0:dy1, dx0:dx1]
            tile = png[sy0:sy1, sx0:sx1]

            if tile.shape[2] == 4:
                # BGRA 투명 합성
                a      = tile[:, :, 3:4].astype(np.float32) / 255.0 * self.alpha
                fg     = tile[:, :, :3].astype(np.float32)
                bg     = roi.astype(np.float32)
                result[dy0:dy1, dx0:dx1] = (fg * a + bg * (1 - a)).astype(np.uint8)
            else:
                result[dy0:dy1, dx0:dx1] = cv2.addWeighted(
                    roi, 1 - self.alpha, tile, self.alpha, 0)

        return result

    @staticmethod
    def _depth_to_model(pose, cam_wx, cam_wy, fwd):
        dx = pose[0] - cam_wx
        dy = pose[1] - cam_wy
        return dx * fwd[0] + dy * fwd[1]


def main(args=None):
    rclpy.init(args=args)
    node = ImageOverlayNode()
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
