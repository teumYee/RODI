"""
model_mover_node.py

Gazebo 내 빌보드 모델들을 마트 범위 안에서 자유롭게 이동시키는 노드.
- 성인/어린이 짝(pair)을 정의하고, 성인은 대부분 어린이 근처에 있다가
  일정 확률로 멀어짐 → 보호자 부재 시나리오를 자연스럽게 생성.
- /gazebo/set_entity_state 서비스로 모델 위치를 갱신.

파라미터:
  move_interval_sec  : 이동 주기 (기본 8.0초)
  alert_wander_prob  : 성인이 멀리 이탈할 확률 (기본 0.25 = 25%)
  mart_x_min/max     : 마트 X 범위
  mart_y_min/max     : 마트 Y 범위
"""

import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String


# ── 페어 정의: (어린이 모델명, 담당 성인 모델명) ──────────────────────────
PAIRS = [
    ('child_1', 'adult_guard_1'),
    ('child_2', 'adult_guard_2'),
    ('child_3', 'adult_guard_3'),
    ('child_4', 'adult_guard_4'),
]

# 단독 배회 성인 (어린이 없음)
SOLO_ADULTS = ['adult_solo_1', 'adult_solo_2']

ALL_MODELS = [name for pair in PAIRS for name in pair] + SOLO_ADULTS

PATROL_ROUTE = [
    (2.5, 0.0),
    (4.5, 2.5),
    (8.4, 3.1),
    (12.5, 5.0),
    (14.0, 2.0),
    (13.0, -1.5),
    (9.0, -2.5),
    (5.0, -2.0),
    (2.0, -0.5),
    (6.0, 5.0),
]

PAIR_ANCHORS = {
    'child_1': (1.2,  3.4),
    'child_2': (5.0,  4.7),
    'child_3': (9.6,  3.5),
    'child_4': (12.8, -2.2),
}

SOLO_ANCHORS = {
    'adult_solo_1': (1.0, -2.4),
    'adult_solo_2': (13.6, 4.7),
}

WANDER_ANCHORS = [
    (1.0, -2.4),
    (1.2, 5.1),
    (9.8, -2.4),
    (13.6, 4.7),
]

DEMO_POSITIONS = {
    # 미아 시나리오: child_1 단독, 보호자는 마트 반대편 (~9m)
    'child_1':       (5.0,   2.5),
    'adult_guard_1': (14.0,  2.0),

    # Pair 2 — 마트 좌상단 구역 (child_1과 ~7m)
    'child_2':       (1.5,   4.5),
    'adult_guard_2': (2.3,   4.5),

    # Pair 3 — 마트 우하단 구역 (child_1과 ~6.2m)
    'child_3':       (12.0, -2.0),
    'adult_guard_3': (12.8, -2.0),

    # Pair 4 — 마트 좌하단 구역 (child_1과 ~8.5m)
    'child_4':       (2.0,  -2.5),
    'adult_guard_4': (2.8,  -2.5),

    # 솔로 어른들 — 구석구석 (child_1 3m 초과 거리 유지)
    'adult_solo_1':  (4.5,   4.0),
    'adult_solo_2':  (11.5,  0.0),
}


class ModelMoverNode(Node):
    def __init__(self):
        super().__init__('model_mover')

        self.declare_parameter('move_interval_sec', 18.0)
        self.declare_parameter('alert_wander_prob',  0.25)
        self.declare_parameter('mart_x_min',        -5.0)
        self.declare_parameter('mart_x_max',        15.0)
        self.declare_parameter('mart_y_min',        -6.0)
        self.declare_parameter('mart_y_max',         6.0)
        self.declare_parameter('guardian_radius',    3.0)
        self.declare_parameter('wander_radius',     10.0)
        self.declare_parameter('smooth_move_duration_sec', 14.0)
        self.declare_parameter('smooth_move_steps', 100)
        self.declare_parameter('path_clearance_radius', 1.0)
        self.declare_parameter('person_spacing_radius', 1.25)
        self.declare_parameter('anchor_radius', 0.45)
        self.declare_parameter('child_step_radius', 0.35)
        self.declare_parameter('solo_step_radius', 0.4)
        self.declare_parameter('demo_scenario', True)

        self.interval     = self.get_parameter('move_interval_sec').value
        self.wander_prob  = self.get_parameter('alert_wander_prob').value
        self.x_min        = self.get_parameter('mart_x_min').value
        self.x_max        = self.get_parameter('mart_x_max').value
        self.y_min        = self.get_parameter('mart_y_min').value
        self.y_max        = self.get_parameter('mart_y_max').value
        self.g_radius     = self.get_parameter('guardian_radius').value
        self.w_radius     = self.get_parameter('wander_radius').value
        self.move_duration = float(self.get_parameter('smooth_move_duration_sec').value)
        self.move_steps    = max(1, int(self.get_parameter('smooth_move_steps').value))
        self.path_clearance = float(self.get_parameter('path_clearance_radius').value)
        self.person_spacing = float(self.get_parameter('person_spacing_radius').value)
        self.anchor_radius  = float(self.get_parameter('anchor_radius').value)
        self.child_step     = float(self.get_parameter('child_step_radius').value)
        self.solo_step      = float(self.get_parameter('solo_step_radius').value)
        self.demo_scenario  = bool(self.get_parameter('demo_scenario').value)
        self._demo_applied  = False

        # 현재 위치 저장 딕셔너리 (mart.world 초기 pose 와 일치)
        self._pos = dict(DEMO_POSITIONS) if self.demo_scenario else {
            'child_1':       (1.2,  3.4),
            'adult_guard_1': (2.0,  4.8),
            'child_2':       (5.0,  4.7),
            'adult_guard_2': (6.5,  5.2),
            'child_3':       (9.6,  3.5),
            'adult_guard_3': (11.0, 4.2),
            'child_4':       (12.8, -2.2),
            'adult_guard_4': (14.2, -1.2),
            'adult_solo_1':  (1.0, -2.4),
            'adult_solo_2':  (13.6, 4.7),
        }
        self._wandering = {name: False for name in ALL_MODELS}
        self._escorted  = ''   # 에스코트 중인 child 모델명 (이동 금지)

        self.create_subscription(String, '/escorting_child', self._escort_cb, 10)

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self._service_connected = False

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '/gazebo/set_entity_state 서비스 대기 중 — Gazebo 준비되면 자동 연결됩니다'
            )
        else:
            self._service_connected = True
            self.get_logger().info(
                f'model_mover 시작: {len(PAIRS)}쌍, demo={self.demo_scenario}, '
                f'이동 주기={self.interval}s, 이동시간={self.move_duration}s, '
                f'경로 회피={self.path_clearance}m, 이탈 확률={self.wander_prob*100:.0f}%'
            )

        self.create_timer(1.0 if self.demo_scenario else self.interval, self._move_all)

    # ── /escorting_child 수신 ─────────────────────────────────────────────
    def _escort_cb(self, msg: String):
        self._escorted = msg.data   # 빈 문자열이면 에스코트 없음

    # ── 전체 이동 루틴 ────────────────────────────────────────────────────
    def _move_all(self):
        if not self.cli.service_is_ready():
            return
        if not self._service_connected:
            self._service_connected = True
            self.get_logger().info(
                f'Gazebo 서비스 연결됨 — model_mover 시작: {len(PAIRS)}쌍, '
                f'이동 주기={self.interval}s, 이동시간={self.move_duration}s, '
                f'경로 회피={self.path_clearance}m, 사람 간격={self.person_spacing}m, '
                f'이탈 확률={self.wander_prob*100:.0f}%'
            )

        if self.demo_scenario:
            self._apply_demo_scene()
            return

        escorted = self._escorted
        reserved = []

        for child_name, adult_name in PAIRS:
            cx, cy = self._pos[child_name]
            # 에스코트 중인 child는 model_mover가 건드리지 않음
            if child_name == escorted:
                reserved.append(self._pos[child_name])
                continue

            new_cx, new_cy = self._sample_child_target(child_name, cx, cy, reserved)
            reserved.append((new_cx, new_cy))
            self._move_model(child_name, (cx, cy), (new_cx, new_cy))

            # 성인: wander_prob 확률로 멀리 이탈, 아니면 어린이 근처 유지
            if random.random() < self.wander_prob:
                # 멀리 이탈 (보호자 부재 시나리오)
                self._wandering[adult_name] = True
                ax, ay = self._sample_wander_target(new_cx, new_cy, reserved)
                self.get_logger().info(
                    f'[이탈] {adult_name} → ({ax:.1f}, {ay:.1f}) '
                    f'(어린이 {child_name}으로부터 {self._dist((new_cx, new_cy), (ax, ay)):.1f}m)'
                )
            else:
                # 어린이 근처 유지
                self._wandering[adult_name] = False
                ax, ay = self._sample_guardian_target(new_cx, new_cy, reserved)

            adult_start = self._pos[adult_name]
            self._pos[adult_name] = (ax, ay)
            self._pos[child_name] = (new_cx, new_cy)
            reserved.append((ax, ay))
            self._move_model(adult_name, adult_start, (ax, ay))

        # senior 단독 배회
        for solo in SOLO_ADULTS:
            sx, sy = self._pos[solo]
            nx, ny = self._sample_solo_target(solo, sx, sy, reserved)
            self._pos[solo] = (nx, ny)
            reserved.append((nx, ny))
            self._move_model(solo, (sx, sy), (nx, ny))

    def _apply_demo_scene(self):
        if self._demo_applied:
            return
        for name, (x, y) in DEMO_POSITIONS.items():
            self._set_pose(name, x, y)
        self._pos = dict(DEMO_POSITIONS)
        self._demo_applied = True
        self.get_logger().info(
            '데모 시나리오 배치 완료: child_1 단독 배치, adult_guard_1 원거리 배치, 군중 이동 비활성화'
        )

    # ── Gazebo 서비스 호출 ────────────────────────────────────────────────
    def _move_model(self, model_name: str, start: tuple, target: tuple):
        if self.move_steps <= 1 or self.move_duration <= 0.0:
            self._set_pose(model_name, target[0], target[1])
            return

        threading.Thread(
            target=self._smooth_move,
            args=(model_name, start, target),
            daemon=True,
        ).start()

    def _smooth_move(self, model_name: str, start: tuple, target: tuple):
        sx, sy = start
        tx, ty = target
        delay = self.move_duration / self.move_steps
        for i in range(1, self.move_steps + 1):
            if not rclpy.ok():
                return
            ratio = i / self.move_steps
            x = sx + (tx - sx) * ratio
            y = sy + (ty - sy) * ratio
            self._set_pose(model_name, x, y)
            time.sleep(delay)

    def _set_pose(self, model_name: str, x: float, y: float):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = model_name
        state.reference_frame = 'world'

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        state.pose = pose
        state.twist = Twist()

        req.state = state
        self.cli.call_async(req)

    def _sample_child_target(self, child_name: str, x: float, y: float, reserved: list):
        anchor = PAIR_ANCHORS.get(child_name, (x, y))
        nx, ny = self._sample_valid_near(x, y, self.child_step, reserved)
        if self._dist((nx, ny), anchor) <= self.anchor_radius:
            return nx, ny
        return self._sample_valid_near(anchor[0], anchor[1], self.anchor_radius, reserved)

    def _sample_guardian_target(self, child_x: float, child_y: float, reserved: list):
        return self._sample_valid_offset(child_x, child_y, 1.4, min(2.4, self.g_radius * 0.85), reserved)

    def _sample_wander_target(self, child_x: float, child_y: float, reserved: list):
        anchors = sorted(
            WANDER_ANCHORS,
            key=lambda p: self._dist((child_x, child_y), p),
            reverse=True,
        )
        for anchor in anchors:
            if self._dist((child_x, child_y), anchor) < self.g_radius * 1.6:
                continue
            target = self._sample_valid_near(anchor[0], anchor[1], self.anchor_radius, reserved)
            if self._dist((child_x, child_y), target) >= self.g_radius * 1.5:
                return target
        return self._sample_valid_offset(child_x, child_y, self.g_radius * 1.5, self.w_radius, reserved)

    def _sample_solo_target(self, solo_name: str, x: float, y: float, reserved: list):
        anchor = SOLO_ANCHORS.get(solo_name, (x, y))
        nx, ny = self._sample_valid_near(x, y, self.solo_step, reserved)
        if self._dist((nx, ny), anchor) <= self.anchor_radius:
            return nx, ny
        return self._sample_valid_near(anchor[0], anchor[1], self.anchor_radius, reserved)

    def _sample_valid_near(self, x: float, y: float, radius: float, reserved: list = None):
        for _ in range(40):
            nx = self._clamp(x + random.uniform(-radius, radius), self.x_min, self.x_max)
            ny = self._clamp(y + random.uniform(-radius, radius), self.y_min, self.y_max)
            if self._is_valid_position(nx, ny, reserved):
                return nx, ny
        return self._sample_valid_anywhere(reserved)

    def _sample_valid_offset(self, x: float, y: float, min_dist: float, max_dist: float, reserved: list = None):
        for _ in range(60):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(min_dist, max_dist)
            nx = self._clamp(x + dist * math.cos(angle), self.x_min, self.x_max)
            ny = self._clamp(y + dist * math.sin(angle), self.y_min, self.y_max)
            if self._is_valid_position(nx, ny, reserved):
                return nx, ny
        return self._sample_valid_anywhere(reserved)

    def _sample_valid_anywhere(self, reserved: list = None):
        for _ in range(80):
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            if self._is_valid_position(x, y, reserved):
                return x, y
        return random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max)

    def _is_valid_position(self, x: float, y: float, reserved: list = None):
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return False
        for a, b in zip(PATROL_ROUTE, PATROL_ROUTE[1:]):
            if self._point_segment_dist((x, y), a, b) < self.path_clearance:
                return False
        if reserved:
            for pos in reserved:
                if self._dist((x, y), pos) < self.person_spacing:
                    return False
        return True

    @staticmethod
    def _point_segment_dist(p: tuple, a: tuple, b: tuple):
        px, py = p
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        denom = dx * dx + dy * dy
        if denom == 0.0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / denom
        t = max(0.0, min(1.0, t))
        cx = ax + t * dx
        cy = ay + t * dy
        return math.hypot(px - cx, py - cy)

    @staticmethod
    def _dist(a: tuple, b: tuple):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))


def main(args=None):
    rclpy.init(args=args)
    node = ModelMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
