"""
patrol_node.py

/child_alert 없을 때 마트 구역을 자동 순찰.
- /child_alert 수신 → 현재 Nav2 목표 취소 + 순찰 정지
- /escort_status 'idle' 수신 → 순찰 재개
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


PATROL_WAYPOINTS = [
    (2.5, 0.0),
    (4.5, 2.5),
    (7.0, 4.5),
    (12.5, 5.0),
    (14.0, 2.0),
    (13.0, -1.5),
    (9.0, -2.5),
]


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self._nav  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._busy = False
        self._gh   = None   # current goal handle
        self._lock = threading.Lock()
        self._idx  = 0

        self.create_subscription(String, '/child_alert',   self._alert_cb,  10)
        self.create_subscription(String, '/escort_status', self._status_cb, 10)

        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info('patrol_node 시작')

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
            wp = PATROL_WAYPOINTS[self._idx % len(PATROL_WAYPOINTS)]
            self._idx += 1
            self.get_logger().info(f'[patrol] → ({wp[0]:.1f}, {wp[1]:.1f})')
            self._go(*wp)
            time.sleep(0.5)

    def _go(self, x: float, y: float):
        goal = PoseStamped()
        goal.header.frame_id      = 'map'
        goal.header.stamp         = self.get_clock().now().to_msg()
        goal.pose.position.x      = x
        goal.pose.position.y      = y
        goal.pose.orientation.w   = 1.0

        gmsg = NavigateToPose.Goal()
        gmsg.pose = goal

        fut = self._nav.send_goal_async(gmsg)
        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not fut.done():
            self.get_logger().warn('[patrol] goal 전송 timeout')
            time.sleep(2.0)
            return

        handle = fut.result()
        if not handle or not handle.accepted:
            self.get_logger().warn(f'[patrol] goal rejected: ({x:.1f}, {y:.1f})')
            time.sleep(2.0)
            return

        with self._lock:
            self._gh = handle

        rfut = handle.get_result_async()
        end  = time.monotonic() + 120.0
        while not rfut.done() and time.monotonic() < end:
            with self._lock:
                if self._busy:
                    return
            time.sleep(0.3)

        with self._lock:
            self._gh = None


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
