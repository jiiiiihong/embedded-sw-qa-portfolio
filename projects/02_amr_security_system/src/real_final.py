#!/usr/bin/env python3

import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav2_msgs.action import NavigateToPose

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


# ======================
# robot8 기준 설정
# ======================

INITIAL_POSE_POSITION = [-0.1, 0.1]
INITIAL_POSE_DIRECTION = 315

GOAL_POSES = [
    ([2.05, 2.04], 135),
    ([0.65, 3.28], 225),
    ([-0.79, 2.01], 315),
    ([-2.25, 0.84], 225),
    ([-4.37, -1.21], 315),
    ([-3.02, -2.55], 45),
    ([-0.37, 0.4], 315),
]

INITIAL_POSE_TOPIC = '/robot8/initialpose'

MY_START_PATROL_TOPIC = '/robot8/start_patrol_signal'
PEER_START_PATROL_TOPIC = '/robot2/start_patrol_signal'

MY_THIEF_POSITION_TOPIC = '/robot8/thief_position'
PEER_THIEF_POSITION_TOPIC = '/robot2/thief_position'

AMCL_POSE_TOPIC = '/robot8/amcl_pose'
NAV_ACTION_NAME = '/robot8/navigate_to_pose'

ARRIVE_POSITION_TOPIC = '/robot8/arrive_position'
SECURITY_STATUS_TOPIC = '/api/security_status'
SITUATION_END_TOPIC = '/situation_end'

ARRIVE_SIGNAL_SPOTS = {
    (2.05, 2.04): 1,      # pot
    (-2.25, 0.84): 2,    # ball
}

START_IMMEDIATELY = True

CHASE_DISTANCE_M = 0.30
CHASE_START_DELAY_SEC = 2.0
GOAL_UPDATE_PERIOD = 0.5
MIN_GOAL_SHIFT_M = 0.05


class PatrolApp(TurtleBot4Navigator):
    def __init__(self):
        super().__init__()

        self.cb_group = ReentrantCallbackGroup()

        self.is_patrolling = False
        self.chase_mode = False
        self.chase_requested = False
        self.chase_transitioning = False
        self.situation_end = False

        self.latest_my_thief_position = None
        self.latest_peer_thief_position = None
        self.latest_amcl_pose = None

        self.start_patrol_pub = self.create_publisher(
            Bool,
            PEER_START_PATROL_TOPIC,
            10
        )

        self.arrive_position_pub = self.create_publisher(
            Int32,
            ARRIVE_POSITION_TOPIC,
            10
        )

        self.security_status_pub = self.create_publisher(
            Bool,
            SECURITY_STATUS_TOPIC,
            10
        )

        initial_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            INITIAL_POSE_TOPIC,
            initial_pose_qos
        )

        self.create_subscription(
            PointStamped,
            MY_THIEF_POSITION_TOPIC,
            self.my_thief_position_callback,
            10
        )

        self.create_subscription(
            PointStamped,
            PEER_THIEF_POSITION_TOPIC,
            self.peer_thief_position_callback,
            10
        )

        self.create_subscription(
            Bool,
            SITUATION_END_TOPIC,
            self.situation_end_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_POSE_TOPIC,
            self.amcl_pose_callback,
            10
        )

    def my_thief_position_callback(self, msg):
        self.latest_my_thief_position = msg
        self.info('내 thief_position 수신 → 추적 전환 요청')
        self.request_chase_mode()

    def peer_thief_position_callback(self, msg):
        self.latest_peer_thief_position = msg
        self.info('동료 thief_position 수신 → 추적 전환 요청')
        self.request_chase_mode()

    def situation_end_callback(self, msg):
        if msg.data:
            self.info('상황 종료 신호 수신')
            self.situation_end = True
            self.chase_mode = False
            self.chase_requested = False
            self.chase_transitioning = False

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg

    def publish_bool(self, pub, value):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_security_status(self, value):
        msg = Bool()
        msg.data = value
        self.security_status_pub.publish(msg)
        self.info(f'{SECURITY_STATUS_TOPIC} → {value} 발행')

    def request_chase_mode(self):
        if self.chase_mode or self.chase_transitioning:
            return

        self.chase_requested = True

        if self.is_patrolling:
            self.warn('순찰 중 추적 요청 발생 → 순찰 종료 후 추적 전환')
            return

        threading.Thread(
            target=self.enter_chase_mode_after_delay,
            daemon=True
        ).start()

    def enter_chase_mode_after_delay(self):
        if self.chase_mode or self.chase_transitioning:
            return

        self.chase_transitioning = True

        self.info(f'{CHASE_START_DELAY_SEC}초 후 추적 모드 전환')
        time.sleep(CHASE_START_DELAY_SEC)

        if not rclpy.ok():
            return

        self.chase_requested = False
        self.chase_mode = True
        self.situation_end = False

        self.info('침입자 추적 모드 진입')

        try:
            self._waitForNodeToActivate('bt_navigator')

            if self.getDockedStatus():
                self.info('추적 모드: dock 상태이므로 undock')
                self.undock()

            self.info('추적 준비 완료. KeepChaseActionNode가 goal을 갱신합니다.')

        except Exception as e:
            self.error(f'추적 모드 전환 중 오류: {e}')

        finally:
            self.chase_transitioning = False

    def publish_initial_pose_once_reliable(self, wait_timeout=10.0):
        self.info('AMCL 활성화 대기 중...')
        self._waitForNodeToActivate('amcl')

        pose_stamped = self.getPoseStamped(
            INITIAL_POSE_POSITION,
            INITIAL_POSE_DIRECTION
        )

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose = pose_stamped.pose

        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.068

        self.info('AMCL initialpose subscriber 대기 중...')

        start_time = time.time()
        while rclpy.ok() and self.initial_pose_pub.get_subscription_count() == 0:
            if time.time() - start_time > wait_timeout:
                self.warn('initialpose subscriber 대기 timeout. 그래도 1회 전송합니다.')
                break
            time.sleep(0.1)

        self.info('초기 위치 Reliable QoS로 1회 전송')
        self.initial_pose_pub.publish(initial_pose)

        time.sleep(2.0)
        self.info('초기 위치 전송 완료')

    def wait_nav_complete(self):
        while rclpy.ok() and not self.isTaskComplete():
            if self.chase_requested:
                self.warn('추적 요청 감지 → 현재 순찰 goal 취소')
                try:
                    self.cancelTask()
                except Exception:
                    pass
                return None

            time.sleep(0.1)

        if not rclpy.ok():
            return None

        result = self.getResult()
        self.info(f'Navigation result: {result}')
        return result

    def move_and_rotate(self, goal):
        position, direction = goal

        goal_pose = self.getPoseStamped(position, direction)
        self.startToPose(goal_pose)

        result = self.wait_nav_complete()

        if self.chase_requested or result is None:
            return

        pos_key = (round(position[0], 2), round(position[1], 2))

        if pos_key in ARRIVE_SIGNAL_SPOTS:
            msg = Int32()
            msg.data = ARRIVE_SIGNAL_SPOTS[pos_key]

            self.arrive_position_pub.publish(msg)

            self.info(
                f'도착 신호 발행: position={position}, '
                f'topic={ARRIVE_POSITION_TOPIC}, data={msg.data}'
            )

            time.sleep(5.0)

    def run_patrol(self):
        if self.is_patrolling:
            self.warn('이미 순찰 중입니다.')
            return

        if self.chase_mode:
            self.warn('추적 모드 중이므로 순찰 시작 안 함')
            return

        self.is_patrolling = True
        self.chase_requested = False

        try:
            self.info('순찰 시작')
            self.publish_security_status(True)

            self._waitForNodeToActivate('bt_navigator')

            if self.getDockedStatus():
                self.info('Undocking...')
                self.undock()

            for goal in GOAL_POSES:
                if self.chase_requested:
                    self.warn('추적 요청으로 순찰 루프 중단')
                    break

                if self.chase_mode:
                    self.warn('추적 모드 진입으로 순찰 중단')
                    break

                self.move_and_rotate(goal)

            if self.chase_requested:
                self.warn('순찰 종료 후 추적 모드로 전환 예정')
                return

            if not self.chase_mode:
                self.info('순찰 완료')
                self.request_peer_and_dock()

        except Exception as e:
            self.error(f'순찰 중 오류 발생: {e}')

        finally:
            self.is_patrolling = False
            self.info('순찰 상태 해제 완료. 다음 start_patrol_signal 대기 중')

            if self.chase_requested and not self.chase_mode:
                threading.Thread(
                    target=self.enter_chase_mode_after_delay,
                    daemon=True
                ).start()

    def request_peer_and_dock(self):
        self.info('동료 터틀봇 순찰 시작 토픽 반복 발행')

        msg = Bool()
        msg.data = True

        for i in range(20):
            self.start_patrol_pub.publish(msg)
            self.info(f'동료 순찰 시작 신호 발행 {i + 1}/20')
            time.sleep(0.1)

        self.info('이제 도킹 시작')
        self.dock()


class PatrolSignalReceiver(Node):
    def __init__(self, patrol_app):
        super().__init__('patrol_signal_receiver')
        self.patrol_app = patrol_app

        self.create_subscription(
            Bool,
            MY_START_PATROL_TOPIC,
            self.start_callback,
            10
        )

        self.get_logger().info(f'순찰 시작 토픽 수신 노드 시작: {MY_START_PATROL_TOPIC}')

    def start_callback(self, msg):
        self.get_logger().info(
            f'[SIGNAL NODE] start_patrol 수신: {msg.data}, '
            f'is_patrolling={self.patrol_app.is_patrolling}, '
            f'chase_mode={self.patrol_app.chase_mode}'
        )

        if not msg.data:
            return

        if self.patrol_app.is_patrolling:
            self.get_logger().warn('아직 순찰/도킹 중이라 무시')
            return

        if self.patrol_app.chase_mode:
            self.get_logger().warn('추적 모드 중이라 순찰 시작 안 함')
            return

        self.get_logger().info('대기 상태에서 순찰 시작')
        threading.Thread(target=self.patrol_app.run_patrol, daemon=True).start()


class KeepChaseActionNode(Node):
    def __init__(self, patrol_app):
        super().__init__('keep_chase_action_node')

        self.patrol_app = patrol_app
        self.latest_thief_position = None
        self.latest_amcl_pose = None
        self.last_goal_xy = None

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            NAV_ACTION_NAME
        )

        self.create_subscription(
            PointStamped,
            MY_THIEF_POSITION_TOPIC,
            self.my_thief_position_callback,
            10
        )

        self.create_subscription(
            PointStamped,
            PEER_THIEF_POSITION_TOPIC,
            self.peer_thief_position_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_POSE_TOPIC,
            self.amcl_pose_callback,
            10
        )

        self.timer = self.create_timer(
            GOAL_UPDATE_PERIOD,
            self.update_goal_loop
        )

        self.get_logger().info('추적 goal 업데이트 노드 시작')

    def my_thief_position_callback(self, msg):
        self.latest_thief_position = msg

    def peer_thief_position_callback(self, msg):
        self.latest_thief_position = msg

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg

    def get_robot_xy(self):
        if self.latest_amcl_pose is None:
            return None

        p = self.latest_amcl_pose.pose.pose.position
        return p.x, p.y

    def yaw_to_quaternion(self, yaw_rad):
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return 0.0, 0.0, qz, qw

    def make_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw_rad)

        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        return goal

    def update_goal_loop(self):
        if not self.patrol_app.chase_mode:
            return

        if self.latest_thief_position is None:
            self.get_logger().info('추적 모드: 도둑 좌표 대기 중')
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn('AMCL pose 없음')
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose action server 대기 중')
            return

        rx, ry = robot_xy
        tx = self.latest_thief_position.point.x
        ty = self.latest_thief_position.point.y

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        self.get_logger().info(f'도둑까지 거리: {dist:.2f} m')

        if dist <= CHASE_DISTANCE_M:
            self.get_logger().info('도둑 30cm 이내. goal 갱신 중지')
            self.last_goal_xy = None
            return

        if dist < 0.05:
            return

        ux = dx / dist
        uy = dy / dist

        goal_x = tx - ux * CHASE_DISTANCE_M
        goal_y = ty - uy * CHASE_DISTANCE_M
        yaw_deg = math.degrees(math.atan2(dy, dx))

        if self.last_goal_xy is not None:
            last_x, last_y = self.last_goal_xy
            shift = math.hypot(goal_x - last_x, goal_y - last_y)

            if shift < MIN_GOAL_SHIFT_M:
                return

        self.get_logger().info(
            f'Nav2 추적 goal 갱신: goal=({goal_x:.2f}, {goal_y:.2f}), yaw={yaw_deg:.1f}'
        )

        goal_msg = self.make_goal(goal_x, goal_y, yaw_deg)
        self.nav_client.send_goal_async(goal_msg)

        self.last_goal_xy = (goal_x, goal_y)


def main():
    rclpy.init()

    app = PatrolApp()
    signal_receiver = PatrolSignalReceiver(app)
    chase_node = KeepChaseActionNode(app)

    executor = MultiThreadedExecutor()
    executor.add_node(app)
    executor.add_node(signal_receiver)
    executor.add_node(chase_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app.publish_initial_pose_once_reliable()

    try:
        if START_IMMEDIATELY:
            app.info('시작 로봇입니다. 순찰을 시작합니다.')
            threading.Thread(target=app.run_patrol, daemon=True).start()
        else:
            app.info('대기 로봇입니다. start_patrol 토픽 요청을 기다립니다.')

        while rclpy.ok():
            time.sleep(0.5)

    except KeyboardInterrupt:
        app.info('사용자 종료 요청')
        app.situation_end = True
        app.chase_mode = False
        app.chase_requested = False
        app.chase_transitioning = False

    finally:
        executor.shutdown()
        chase_node.destroy_node()
        signal_receiver.destroy_node()
        app.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()