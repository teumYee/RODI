"""
patrol_node.py

/child_alert 없을 때 마트 구역을 자동 순찰.
- auto_waypoints: true  → x/y 범위 + strip_width로 지그재그 루트 자동 생성
- auto_waypoints: false → patrol_waypoints.yaml 의 수동 좌표 사용
- /child_alert 수신 → 현재 Nav2 목표 취소 + 순찰 정지
- /escort_status 'idle' 수신 → 순찰 재개
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('auto_waypoints', True)
        self.declare_parameter('patrol_x_min',  -3.0)
        self.declare_parameter('patrol_x_max',  15.0)
        self.declare_parameter('patrol_y_min',  -3.0)
        self.declare_parameter('patrol_y_max',  43.0)
        self.declare_parameter('strip_width',    6.0)
        self.declare_parameter('waypoints_x', [0.0])
        self.declare_parameter('waypoints_y', [0.0])

        if self.get_parameter('auto_waypoints').value:
            self._waypoints = self._generate_waypoints()
            self.get_logger().info(
                f'[patrol] 자동 생성 웨이포인트 {len(self._waypoints)}개')
        else:
            xs = self.get_parameter('waypoints_x').value
            ys = self.get_parameter('waypoints_y').value
            if len(xs) != len(ys):
                self.get_logger().error('waypoints_x / waypoints_y 길이가 다릅니다!')
            self._waypoints = list(zip(xs, ys))
            self.get_logger().info(
                f'[patrol] 수동 웨이포인트 {len(self._waypoints)}개')
        if not self._waypoints:
            self.get_logger().error('[patrol] 웨이포인트가 없어 기본 좌표 (0, 0)을 사용합니다')
            self._waypoints = [(0.0, 0.0)]

        self._nav  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._busy = False
        self._gh   = None
        self._lock = threading.Lock()
        self._idx  = 0

        self.create_subscription(String, '/child_alert',   self._alert_cb,  10)
        self.create_subscription(String, '/escort_status', self._status_cb, 10)

        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info('patrol_node 시작')

    def _generate_waypoints(self):
        x_min = self.get_parameter('patrol_x_min').value
        x_max = self.get_parameter('patrol_x_max').value
        y_min = self.get_parameter('patrol_y_min').value
        y_max = self.get_parameter('patrol_y_max').value
        strip = self.get_parameter('strip_width').value

        waypoints = []
        y = y_min
        left_to_right = True
        while y <= y_max + 0.01:
            y_clamped = min(y, y_max)
            if left_to_right:
                waypoints.append((x_min, y_clamped))
                waypoints.append((x_max, y_clamped))
            else:
                waypoints.append((x_max, y_clamped))
                waypoints.append((x_min, y_clamped))
            left_to_right = not left_to_right
            y += strip

        for wp in waypoints:
            self.get_logger().info(f'  waypoint: ({wp[0]:.1f}, {wp[1]:.1f})')
        return waypoints

    # ── 경보 수신 ─────────────────────────────────────────────────────────
    def _alert_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get('alert', False):
            with self._lock:
                self._busy = True
                gh = self._gh
            if gh:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                with self._lock:
                    self._gh = None
            self.get_logger().info('[patrol] 에스코트 경보 → 순찰 정지')

    def _status_cb(self, msg: String):
        if msg.data == 'idle':
            with self._lock:
                self._busy = False
            self.get_logger().info('[patrol] 에스코트 완료 → 순찰 재개')

    # ── 순찰 루프 ─────────────────────────────────────────────────────────
    def _loop(self):
        self.get_logger().info('[patrol] Nav2 대기...')
        while rclpy.ok() and not self._nav.wait_for_server(timeout_sec=3.0):
            time.sleep(1.0)
        self.get_logger().info('[patrol] Nav2 연결 → 순찰 시작')

        while rclpy.ok():
            with self._lock:
                busy = self._busy
            if busy:
                time.sleep(1.0)
                continue
            idx = self._idx % len(self._waypoints)
            wp = self._waypoints[idx]
            self.get_logger().info(f'[patrol] → ({wp[0]:.1f}, {wp[1]:.1f})')
            if self._go(*wp):
                self._idx += 1
            else:
                time.sleep(2.0)
            time.sleep(0.5)

    def _go(self, x: float, y: float) -> bool:
        goal = PoseStamped()
        goal.header.frame_id    = 'map'
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position.x    = x
        goal.pose.position.y    = y
        goal.pose.orientation.w = 1.0

        gmsg = NavigateToPose.Goal()
        gmsg.pose = goal

        send_event   = threading.Event()
        result_event = threading.Event()
        handle_holder = [None]

        def _on_goal(fut):
            try:
                handle_holder[0] = fut.result()
            except Exception as e:
                self.get_logger().warn(f'[patrol] goal 응답 오류: {e}')
            send_event.set()

        self._nav.send_goal_async(gmsg).add_done_callback(_on_goal)

        if not send_event.wait(timeout=10.0):
            self.get_logger().warn('[patrol] goal 전송 timeout')
            return False

        handle = handle_holder[0]
        if not handle or not handle.accepted:
            self.get_logger().warn(f'[patrol] goal rejected: ({x:.1f}, {y:.1f})')
            return False

        with self._lock:
            self._gh = handle

        result_holder = [None]

        def _on_result(fut):
            try:
                result_holder[0] = fut.result()
            except Exception as e:
                self.get_logger().warn(f'[patrol] result 응답 오류: {e}')
            result_event.set()

        handle.get_result_async().add_done_callback(_on_result)

        deadline = time.monotonic() + 600.0
        while not result_event.is_set():
            with self._lock:
                if self._busy:
                    if self._gh is handle:
                        self._gh = None
                    return False
            if time.monotonic() > deadline:
                self.get_logger().warn(f'[patrol] goal timeout: ({x:.1f}, {y:.1f})')
                break
            time.sleep(0.5)

        with self._lock:
            if self._gh is handle:
                self._gh = None

        result = result_holder[0]
        if result is None:
            return False
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'[patrol] goal 종료 상태={result.status}, 같은 목적지를 유지합니다'
            )
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
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
