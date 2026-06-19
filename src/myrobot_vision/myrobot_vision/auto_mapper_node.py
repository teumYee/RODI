"""
auto_mapper_node.py

SLAM 중 자동으로 로봇을 몰아서 InformationCounter까지 맵을 그려주는 노드.
/scan 기반 장애물 회피 + 목표 방향 추종.

사용법:
  터미널1: ros2 launch myrobot my_world.launch.py
  터미널2: ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
  터미널3: ros2 run myrobot_vision auto_mapper
  다 됐으면 터미널4: ros2 run nav2_map_server map_saver_cli -f ~/RODI/src/myrobot/maps/mart_map
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# 방문할 Gazebo world 좌표 목록 (현재 맵 영역 + InformationCounter 방향)
WAYPOINTS = [
    (  5.0,  2.5),
    ( 10.0,  3.0),
    ( 14.0,  2.0),
    ( 12.0, -2.0),
    (  5.0, -2.0),
    (  0.0,  0.0),
    ( -5.0,  1.0),
    (-10.0,  1.0),
    (-15.0,  1.0),
    (-20.0,  1.0),
    (-25.0,  1.0),
    (-30.0,  1.0),
    (-33.6,  1.15),  # InformationCounter
    (-30.0,  1.0),
    (-20.0,  1.0),
    (-10.0,  1.0),
    (  0.0,  0.0),
]

LINEAR_SPEED  = 0.22
ANGULAR_SPEED = 0.5
OBSTACLE_DIST = 0.35  # 이 거리 이내면 장애물 회피 (너무 민감하면 증가)
ARRIVAL_DIST  = 1.0   # 웨이포인트 도착 판정 거리


class AutoMapperNode(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        self._pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._scan = None
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_sub = self.create_subscription(
            __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
            '/odom', self._odom_cb, 10)

        self._wp_idx = 0
        self.create_timer(0.1, self._loop)
        self.get_logger().info('자동 맵퍼 시작 — SLAM 실행 중인지 확인하세요!')

    def _scan_cb(self, msg):
        self._scan = msg

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self._x = p.position.x
        self._y = p.position.y
        q = p.orientation
        self._yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def _loop(self):
        if self._wp_idx >= len(WAYPOINTS):
            self._stop()
            self.get_logger().info('모든 웨이포인트 완료! 이제 맵 저장하세요:')
            self.get_logger().info(
                'ros2 run nav2_map_server map_saver_cli '
                '-f ~/RODI/src/myrobot/maps/mart_map'
            )
            return

        wx, wy = WAYPOINTS[self._wp_idx]
        dx = wx - self._x
        dy = wy - self._y
        dist = math.hypot(dx, dy)

        if dist < ARRIVAL_DIST:
            self.get_logger().info(
                f'웨이포인트 [{self._wp_idx+1}/{len(WAYPOINTS)}] '
                f'({wx:.1f},{wy:.1f}) 도착'
            )
            self._wp_idx += 1
            return

        # 장애물 확인 (전방 ±20도만)
        if self._scan:
            ranges = self._scan.ranges
            n = len(ranges)
            step = max(1, n // 18)   # 20도 분량
            front_idx = list(range(0, step)) + list(range(n - step, n))
            left_idx  = list(range(step, step * 4))
            right_idx = list(range(n - step * 4, n - step))

            def valid(idxs):
                return [r for i in idxs
                        if i < n and math.isfinite(r := ranges[i]) and r > 0.05]

            front_vals = valid(front_idx)
            min_front  = min(front_vals) if front_vals else 99.0

            if min_front < OBSTACLE_DIST:
                # 좌우 중 공간 더 넓은 쪽으로 회피
                left_min  = min(valid(left_idx),  default=99.0)
                right_min = min(valid(right_idx), default=99.0)
                turn_dir  = 1.0 if left_min >= right_min else -1.0
                tw = Twist()
                tw.angular.z = ANGULAR_SPEED * turn_dir
                self._pub.publish(tw)
                return

        # 목표 방향 계산
        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self._yaw
        while err >  math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        tw = Twist()
        if abs(err) > 0.3:
            # 방향 맞추기
            tw.angular.z = ANGULAR_SPEED * (1.0 if err > 0 else -1.0)
        else:
            # 직진
            tw.linear.x = LINEAR_SPEED
            tw.angular.z = 0.5 * err

        self._pub.publish(tw)

    def _stop(self):
        self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
