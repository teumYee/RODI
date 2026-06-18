"""
child_response_node.py

시나리오:
1. /child_alert 수신
2. 어린이 위치로 이동
3. 친근한 TTS: 발견 인사 (아이 눈높이)
4. 고객센터로 이동 — 이동 중 child_X 모델이 로봇 뒤를 따라옴
5. 20초마다 팔로우 확인 TTS (model_states 거리 체크 → 멀면 잠깐 대기)
6. 고객센터 도착 TTS + 직원 인계 → 에스코트 즉시 종료
7. patrol_node 재개 신호 발행 → 순찰 재시작
"""

import json
import math
import random
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

import tf2_ros
import tf2_geometry_msgs

from .tts_utils import speak_ko


CUSTOMER_SERVICE_X = 18.0
CUSTOMER_SERVICE_Y = 31.6

# ── 단계별 TTS (아이가 알아듣도록 친근하고 길게) ──────────────────────────
TTS_FOUND = (
    '안녕! 나는 RODI야. 쇼핑몰을 지키는 안전 로봇이에요. '
    '엄마나 아빠가 잠깐 안 보이는 것 같아서 내가 도와주러 왔어! '
    '괜찮아, 걱정하지 마. 내가 고객센터에 있는 선생님한테 데려다 줄게. '
    '선생님이 엄마 아빠를 금방 찾아주실 거야! 자, 나를 따라와!'
)
TTS_WALKING = (
    '잘하고 있어, 조금만 더 걸어가면 돼! '
    '고객센터가 저기 있거든. 나를 놓치지 말고 천천히 따라와!'
)
TTS_CHECK_OK = [
    '잘하고 있어! 이렇게 씩씩하게 따라오다니, 정말 대단한데? 조금만 더 가면 돼!',
    '우와, 발이 엄청 빠른데? 거의 다 왔어! 조금만 더 힘내자!',
    '너 혹시 다리 안 아파? 잘 걷고 있어. 선생님 만나면 엄마 아빠 금방 올 거야!',
    '잘 따라오고 있어, 걱정하지 마! 로봇이랑 같이 가니까 안전해. 조금만 더!',
    '어? 생각보다 엄청 빠른데! 이러다 나보다 먼저 도착하겠는걸? 조금만 더 가자!',
]
TTS_CHECK_WAIT = [
    '잠깐, 내가 너무 빨리 갔나? 미안해! 여기서 기다릴게. 천천히 와도 괜찮아!',
    '어, 어디 있어? 무서워하지 마~ 내가 기다릴게! 천천히 와!',
    '다리가 조금 아프면 잠깐 쉬어도 돼. 내가 여기 있을게, 괜찮아!',
    '나 너무 빨랐지? 미안! 숨 한 번 쉬고 천천히 따라와. 기다릴게!',
]
TTS_ARRIVED = (
    '다 왔어! 여기가 고객센터야. '
    '이 선생님이 엄마 아빠를 찾아주실 거야. '
    '여기서 잠깐만 기다리면 금방 만날 수 있어! '
    '나는 이제 다시 순찰하러 갈게. 걱정하지 마, 금방 만날 수 있을 거야!'
)


class EscortState(str, Enum):
    PATROL = 'patrol'
    DETECTED = 'detected'
    APPROACH = 'approach'
    ESCORT = 'escort'
    WAIT_CHILD = 'wait_child'
    ARRIVED = 'arrived'
    RETURN = 'return'
    IDLE = 'idle'


class ChildResponseNode(Node):
    def __init__(self):
        super().__init__('child_response')

        self.declare_parameter('customer_service_x', CUSTOMER_SERVICE_X)
        self.declare_parameter('customer_service_y', CUSTOMER_SERVICE_Y)
        self.declare_parameter('camera_frame',       'depth_camera_link')
        self.declare_parameter('map_frame',          'map')
        self.declare_parameter('follow_check_sec',    20.0)  # 팔로우 확인 주기
        self.declare_parameter('follow_max_dist',      2.5)  # 이 거리 이상이면 대기

        self.cs_x      = self.get_parameter('customer_service_x').value
        self.cs_y      = self.get_parameter('customer_service_y').value
        self.cam_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.check_sec = self.get_parameter('follow_check_sec').value
        self.max_dist  = self.get_parameter('follow_max_dist').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._set_state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # 로봇 위치 (odom ≈ world)
        self._rx   = 0.0
        self._ry   = 0.0
        self._ryaw = 0.0
        self._rx_lock = threading.Lock()

        # Gazebo 모델 위치
        self._mpos: dict = {}
        self._mpos_lock  = threading.Lock()
        self._mpos_last_update = 0.0

        self._escort_status_pub   = self.create_publisher(String, '/escort_status',   10)
        self._escorting_child_pub = self.create_publisher(String, '/escorting_child', 10)
        self._escort_state_pub    = self.create_publisher(String, '/escort_state',    10)

        self.create_subscription(String,      '/child_alert',         self._alert_cb,   10)
        self.create_subscription(Odometry,    '/odom',                self._odom_cb,    10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._mstates_cb, 10)

        self._navigating    = False
        self._speak_lock    = threading.Lock()
        self._escort_model  = None
        self._escort_paused = False
        self._state = EscortState.PATROL

        self.get_logger().info(
            f'child_response 시작 | 고객센터: ({self.cs_x:.1f}, {self.cs_y:.1f}) | '
            f'팔로우 확인: {self.check_sec}s 주기'
        )

    # ── odom 콜백 ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._rx_lock:
            self._rx   = msg.pose.pose.position.x
            self._ry   = msg.pose.pose.position.y
            self._ryaw = math.atan2(siny, cosy)

    # ── Gazebo 모델 위치 (0.5s 스로틀) ───────────────────────────────────
    def _mstates_cb(self, msg: ModelStates):
        now = time.monotonic()
        if now - self._mpos_last_update < 0.5:
            return
        self._mpos_last_update = now
        pos = {}
        for i, name in enumerate(msg.name):
            p = msg.pose[i].position
            pos[name] = (p.x, p.y)
        with self._mpos_lock:
            self._mpos = pos

    # ── /child_alert ─────────────────────────────────────────────────────
    def _alert_cb(self, msg: String):
        data = json.loads(msg.data)
        if not data.get('alert', False):
            return
        if self._navigating:
            self.get_logger().warn('이미 안내 중 — 새 경보 무시')
            return

        children = data.get('unguarded_children', [])
        if not children:
            return

        target = min(children, key=lambda c: c['depth_m'])
        self._set_escort_state(EscortState.DETECTED)

        threading.Thread(target=self._full_escort, args=(target,), daemon=True).start()

    # ── 전체 에스코트 시나리오 ────────────────────────────────────────────
    def _full_escort(self, target: dict):
        self._navigating = True
        self._pub_escort('busy')
        self.get_logger().info('=== 에스코트 시나리오 시작 ===')
        time.sleep(1.5)   # patrol_node cancel_goal_async 처리 대기

        # STEP 1: 가장 가까운 child_X 모델 선택 + 팔로우 즉시 시작
        self._set_escort_state(EscortState.APPROACH)
        with self._mpos_lock:
            mpos = dict(self._mpos)
        with self._rx_lock:
            rx, ry = self._rx, self._ry

        self._escort_model = self._nearest_child((rx, ry), mpos)

        if self._escort_model:
            self.get_logger().info(f'에스코트 모델: {self._escort_model}')
            pub_msg = String(); pub_msg.data = self._escort_model
            self._escorting_child_pub.publish(pub_msg)
            threading.Thread(target=self._follow_loop,
                             args=(self._escort_model,), daemon=True).start()
        else:
            self.get_logger().warn('child_X 모델을 찾지 못함 — 팔로우 없이 진행')

        # STEP 2: 어린이 발견 TTS
        self.get_logger().info('[1/3] 어린이 발견 인사')
        self._speak(TTS_FOUND)
        time.sleep(1.0)

        # STEP 3: 고객센터로 이동 (팔로우 확인 포함)
        self._set_escort_state(EscortState.ESCORT)
        self.get_logger().info(f'[2/3] 고객센터로 이동 ({self.cs_x:.1f}, {self.cs_y:.1f})')
        self._speak(TTS_WALKING)

        # 팔로우 확인 타이머 시작
        threading.Thread(target=self._follow_check_loop, daemon=True).start()

        ok = self._navigate_to(self.cs_x, self.cs_y, '고객센터', timeout=300.0)

        # STEP 5: 도착 → 센터 직원에게 인계 후 에스코트 종료
        if ok:
            self._set_escort_state(EscortState.ARRIVED)
            self.get_logger().info('[3/3] 고객센터 도착 — 직원 인계 후 종료')
            self._speak(TTS_ARRIVED)
            time.sleep(3.0)   # TTS 끝날 때까지 잠깐 대기
        else:
            self.get_logger().error('고객센터 이동 실패')
            self._speak(
                '어, 길 찾기가 조금 어렵네. 미안해! '
                '여기 선생님한테 도움 요청해 줄게. 잠깐만 기다려!'
            )
            time.sleep(3.0)

        # 정리 → 순찰 재개
        self._set_escort_state(EscortState.RETURN)
        self.get_logger().info('=== 에스코트 종료 — 직원 인계 완료, 순찰 재개 ===')
        self._navigating    = False
        self._escort_model  = None
        self._escort_paused = False

        clear = String(); clear.data = ''
        self._escorting_child_pub.publish(clear)
        self._pub_escort('idle')
        self._set_escort_state(EscortState.PATROL)

    # ── child_X 모델이 로봇 뒤를 따라오도록 ──────────────────────────────
    def _follow_loop(self, model_name: str):
        while self._navigating and rclpy.ok():
            if self._escort_paused:
                time.sleep(0.3)
                continue
            if not self._set_state_cli.service_is_ready():
                time.sleep(0.3)
                continue

            with self._rx_lock:
                rx, ry, ryaw = self._rx, self._ry, self._ryaw
            cx = rx - 0.7 * math.cos(ryaw)
            cy = ry - 0.7 * math.sin(ryaw)

            req   = SetEntityState.Request()
            state = EntityState()
            state.name            = model_name
            state.reference_frame = 'world'
            pose = Pose()
            pose.position.x  = cx
            pose.position.y  = cy
            pose.position.z  = 0.0
            pose.orientation.w = 1.0
            state.pose  = pose
            state.twist = Twist()
            req.state = state
            self._set_state_cli.call_async(req)
            time.sleep(0.15)

    # ── 팔로우 확인 루프 (check_sec 주기) ────────────────────────────────
    def _follow_check_loop(self):
        last_check = time.monotonic()
        while self._navigating and rclpy.ok():
            now = time.monotonic()
            if now - last_check < self.check_sec:
                time.sleep(1.0)
                continue
            last_check = now

            if self._escort_model is None:
                continue

            # model_states에서 child 거리 확인
            with self._mpos_lock:
                child_pos = self._mpos.get(self._escort_model)
            with self._rx_lock:
                rx, ry = self._rx, self._ry

            if child_pos is None:
                continue

            dist = math.hypot(child_pos[0] - rx, child_pos[1] - ry)
            self.get_logger().info(
                f'[팔로우 확인] {self._escort_model}까지 {dist:.1f}m'
            )

            if dist > self.max_dist:
                # 너무 멀면 일시 정지
                self.get_logger().warn(f'어린이가 {dist:.1f}m 뒤처짐 — 대기')
                self._escort_paused = True
                self._set_escort_state(EscortState.WAIT_CHILD)
                self._speak(random.choice(TTS_CHECK_WAIT))
                time.sleep(4.0)   # 잠깐 기다림
                self._escort_paused = False
                self._set_escort_state(EscortState.ESCORT)
            else:
                self._speak(random.choice(TTS_CHECK_OK))

    # ── 가장 가까운 child_X 모델명 반환 ──────────────────────────────────
    def _nearest_child(self, ref_xy, mpos: dict):
        best, best_d = None, float('inf')
        for name, (mx, my) in mpos.items():
            if not name.startswith('child_'):
                continue
            d = math.hypot(mx - ref_xy[0], my - ref_xy[1])
            if d < best_d:
                best_d, best = d, name
        return best if best_d < 15.0 else None

    # ── Nav2 이동 ─────────────────────────────────────────────────────────
    def _navigate_to(self, x, y, label='', timeout=120.0, yaw=0.0) -> bool:
        gp = PoseStamped()
        gp.header.frame_id    = self.map_frame
        gp.header.stamp       = self.get_clock().now().to_msg()
        gp.pose.position.x    = x
        gp.pose.position.y    = y
        gp.pose.orientation.z = math.sin(yaw / 2.0)
        gp.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Nav2 목표: {label} ({x:.2f}, {y:.2f})')
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 서버 없음')
            return False

        gmsg = NavigateToPose.Goal()
        gmsg.pose = gp
        fut = self._nav_client.send_goal_async(gmsg)

        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not fut.done():
            return False

        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Nav2 목표 거부: {label}')
            return False

        rfut = gh.get_result_async()
        end  = time.monotonic() + timeout
        while not rfut.done() and time.monotonic() < end:
            if not self._navigating:
                break
            time.sleep(0.1)

        if not rfut.done():
            self.get_logger().warn(f'Nav2 타임아웃: {label}')
            return False

        self.get_logger().info(f'Nav2 완료: {label}')
        return True

    # ── TF2: 카메라 → map ────────────────────────────────────────────────
    def _cam_to_map(self, pos3d):
        try:
            pt = PointStamped()
            pt.header.frame_id = self.cam_frame
            pt.header.stamp    = self.get_clock().now().to_msg()
            pt.point.x = float(pos3d[0])
            pt.point.y = float(pos3d[1])
            pt.point.z = float(pos3d[2])
            t = self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=1.0))
            return t.point.x, t.point.y
        except Exception as e:
            self.get_logger().warn(f'TF 실패: {e}')
            return None

    def _pub_escort(self, status: str):
        m = String(); m.data = status
        self._escort_status_pub.publish(m)

    def _set_escort_state(self, state: EscortState):
        if self._state == state:
            return
        self._state = state
        msg = String()
        msg.data = state.value
        self._escort_state_pub.publish(msg)
        self.get_logger().info(f'[state] {state.value}')

    # ── TTS ──────────────────────────────────────────────────────────────
    def _speak(self, text: str):
        with self._speak_lock:
            speak_ko(self.get_logger(), text)


def main(args=None):
    rclpy.init(args=args)
    node = ChildResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
