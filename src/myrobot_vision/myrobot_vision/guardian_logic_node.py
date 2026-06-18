"""
guardian_logic_node.py  (개선판)

/detected_persons (JSON) 수신 → 동적 보호자 짝 배정 → 부재 판단 → 경보 발행.

변경 사항:
  - 미리 짝을 고정하지 않고, 매 프레임 어린이와 가장 가까운 성인을 동적 배정
  - 어린이 위치(3D) 를 /child_alert 에 포함 → child_response_node 가 사용
  - alert_hold_sec 파라미터로 임계값 설정 가능 (기본 60초, 테스트 시 10초)

발행 토픽:
  /child_alert  (std_msgs/String, JSON)
  {
    "alert": true,
    "message": "...",
    "child_count": N,
    "unguarded_children": [
      {"pos3d": [x, y, z], "depth_m": d}, ...
    ]
  }
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GuardianLogicNode(Node):
    def __init__(self):
        super().__init__('guardian_logic')

        self.declare_parameter('alert_hold_sec',   60.0)   # 실 데모: 60초
        self.declare_parameter('guardian_dist_m',   3.0)   # 보호자 인정 거리
        self.declare_parameter('min_depth_m',       0.3)   # 너무 가까운 감지 무시
        self.declare_parameter('max_depth_m',      10.0)   # depth 카메라 최대 범위
        self.declare_parameter('grace_sec',        60.0)   # 감지 끊김 유예 시간 (순찰 루프 한 바퀴 대비)
        self.declare_parameter('guardian_confirm_frames', 10)  # 타이머 리셋에 필요한 연속 보호자 감지 프레임

        self.alert_hold   = self.get_parameter('alert_hold_sec').value
        self.guard_dist   = self.get_parameter('guardian_dist_m').value
        self.min_d        = self.get_parameter('min_depth_m').value
        self.max_d        = self.get_parameter('max_depth_m').value
        self.grace_sec       = self.get_parameter('grace_sec').value
        self.confirm_frames  = int(self.get_parameter('guardian_confirm_frames').value)

        self.create_subscription(String, '/detected_persons', self._cb, 10)
        self.create_subscription(String, '/escort_status', self._escort_cb, 10)
        self.alert_pub = self.create_publisher(String, '/child_alert', 10)

        self._escorting = False  # 에스코트 중엔 감지 중단

        # 어린이별 부재 타이머: key=어린이 인덱스(순서), value=누적 초
        self._absence_timers:   dict[int, float] = {}
        self._alerted_set:      set[int]          = set()
        self._last_time: float | None = None
        self._last_seen:        dict[int, float]  = {}
        self._guardian_frames:  dict[int, int]    = {}  # 연속 보호자 감지 프레임 수

        self.get_logger().info(
            f'Guardian logic 시작 | 부재 임계={self.alert_hold}s | '
            f'보호자 거리={self.guard_dist}m | 유예={self.grace_sec}s | '
            f'보호자 확인={self.confirm_frames}프레임'
        )

    def _escort_cb(self, msg: String):
        self._escorting = (msg.data == 'busy')
        self.get_logger().info(
            f'[guardian] 에스코트 {"시작 → 감지 중단" if self._escorting else "종료 → 감지 재개"}'
        )

    def _cb(self, msg: String):
        if self._escorting:
            return

        data    = json.loads(msg.data)
        persons = data.get('persons', [])

        now = time.monotonic()
        dt = 0.0
        if self._last_time is not None:
            dt = max(0.0, min(now - self._last_time, 2.0))
        self._last_time = now

        # 유효 범위 필터링
        valid = [p for p in persons
                 if p['depth_m'] > self.min_d and p['depth_m'] < self.max_d]

        children = [p for p in valid if p['label'] == 'child']
        adults   = [p for p in valid if p['label'] == 'adult']

        current_keys = set(range(len(children)))

        # 현재 프레임에 보인 어린이의 last_seen 갱신
        for idx in current_keys:
            self._last_seen[idx] = now

        # 이번 프레임에 보호자 없는 어린이 목록
        unguarded_now: list[int] = []
        for idx, child in enumerate(children):
            if self._find_guardian(child, adults) is None:
                unguarded_now.append(idx)

        # 현재 보이는 어린이 타이머 업데이트
        for idx in range(len(children)):
            if idx in unguarded_now:
                self._absence_timers[idx] = self._absence_timers.get(idx, 0.0) + dt
                self._guardian_frames[idx] = 0  # 보호자 없으면 카운터 초기화
            else:
                # 보호자가 confirm_frames 연속으로 보여야만 타이머 리셋
                self._guardian_frames[idx] = self._guardian_frames.get(idx, 0) + 1
                if self._guardian_frames[idx] >= self.confirm_frames:
                    self._absence_timers[idx] = 0.0
                    self._guardian_frames[idx] = 0
                    self._alerted_set.discard(idx)

        # 사라진 어린이: grace_sec 이내면 타이머 유지(동결), 초과하면 삭제
        for k in list(self._absence_timers.keys()):
            if k not in current_keys:
                gone = now - self._last_seen.get(k, now)
                if gone > self.grace_sec:
                    del self._absence_timers[k]
                    self._alerted_set.discard(k)
                    self._last_seen.pop(k, None)
                    self._guardian_frames.pop(k, None)
                # grace period 안: 타이머 값 동결(리셋 안 함)

        # 경보 체크
        alert_children = []
        for idx, child in enumerate(children):
            t = self._absence_timers.get(idx, 0.0)
            if t > 0:
                self.get_logger().info(
                    f'어린이[{idx}] 보호자 없음 {t:.1f}s / {self.alert_hold}s'
                )
            if t >= self.alert_hold and idx not in self._alerted_set:
                alert_children.append(child)
                self._alerted_set.add(idx)
                self._absence_timers[idx] = 0.0

        if alert_children:
            self._fire_alert(alert_children)

    def _find_guardian(self, child, adults):
        """어린이에서 guard_dist 이내에 있는 가장 가까운 성인 반환."""
        best, best_dist = None, float('inf')
        for adult in adults:
            d = self._dist3d(child['pos3d'], adult['pos3d'])
            if d < self.guard_dist and d < best_dist:
                best, best_dist = adult, d
        return best

    @staticmethod
    def _dist3d(a, b):
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    def _fire_alert(self, alert_children: list):
        n = len(alert_children)
        payload = {
            'alert': True,
            'message': f'보호자 없는 어린이 {n}명 감지',
            'child_count': n,
            'unguarded_children': [
                {'pos3d': c['pos3d'], 'depth_m': c['depth_m']}
                for c in alert_children
            ],
        }
        out = String()
        out.data = json.dumps(payload)
        self.alert_pub.publish(out)
        self.get_logger().error(
            f'[경보 발령] 보호자 없는 어린이 {n}명 → child_response_node 트리거'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GuardianLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
