import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
import math

from .turn_logic import TurnLogic


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ParkingDebug(Node):
    def __init__(self):
        super().__init__('parking_debug')

        self.sensor_data = {
            'sign_intersection': 0,
            'odom_yaw': None,
            'lane_cmd': Twist(),
        }

        self.last_sign_time = {
            'sign_intersection': self.get_clock().now(),
        }

        self.sign_score = {
            'sign_intersection': 0.0,
        }

        self.FRESHNESS_TIMEOUT = 0.5
        self.LEFT_SIGN_VALUE = 2
        self.LEFT_SIGN_REQUIRED_DURATION = 2.0

        self.current_state = 'FOLLOW_LANE'
        self.turn_state = TurnLogic(
            self,
            direction='left',
            turn_angle_degrees=90.0,
            straight_seconds=6.0,
            turn_speed=0.3,
            straight_speed=0.03,
        )
        self.turn_started = False

        self.create_subscription(UInt8, '/detect/sign_intersection', self.cb_sign_intersection, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(Twist, '/control/cmd_vel_lane', self.cb_lane, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def cb_sign_intersection(self, msg):
        self.sensor_data['sign_intersection'] = msg.data
        self.last_sign_time['sign_intersection'] = self.get_clock().now()

    def cb_odom(self, msg):
        self.sensor_data['odom_yaw'] = yaw_from_odom(msg)

    def cb_lane(self, msg):
        self.sensor_data['lane_cmd'] = msg

    def _check_sign(self, topic_key, expected_value, required_duration):
        now = self.get_clock().now()
        last_seen = self.last_sign_time[topic_key]
        elapsed_since_last_msg = (now - last_seen).nanoseconds / 1e9
        is_currently_visible = elapsed_since_last_msg < self.FRESHNESS_TIMEOUT
        matches = is_currently_visible and self.sensor_data[topic_key] == expected_value

        self.get_logger().info(
            f'Elapsed since last {topic_key}: {elapsed_since_last_msg:.2f}s, currently visible: {is_currently_visible}, matches: {matches}'
        )
        if matches:
            self.sign_score[topic_key] += 0.05
            self.get_logger().info(
                f'{topic_key} detected, score={self.sign_score[topic_key]:.2f}'
            )
        else:
            self.sign_score[topic_key] = max(0.0, self.sign_score[topic_key] - 0.02)

        if self.sign_score[topic_key] >= required_duration:
            self.sign_score[topic_key] = 0.0
            return True
        return False

    def control_loop(self):
        if self.current_state == 'FOLLOW_LANE':
            if self._check_sign('sign_intersection', self.LEFT_SIGN_VALUE, self.LEFT_SIGN_REQUIRED_DURATION):
                self.get_logger().info('Left sign confirmed. Entering turn logic.')
                self.current_state = 'LEFT_TURN'
                self.turn_started = True
                self.turn_state.on_enter()
                _, cmd_vel = self.turn_state.execute(self.sensor_data)
                self.pub_cmd_vel.publish(cmd_vel)
                return

            self.pub_cmd_vel.publish(self.sensor_data['lane_cmd'])
            return

        if self.current_state == 'LEFT_TURN' and self.turn_started:
            turn_done, cmd_vel = self.turn_state.execute(self.sensor_data)
            if turn_done:
                self.get_logger().info('Left turn finished. Returning to FOLLOW_LANE.')
                self.current_state = 'FOLLOW_LANE_AFTER_TURN'
                self.turn_started = False
            self.pub_cmd_vel.publish(cmd_vel)
            return

        if self.current_state == 'FOLLOW_LANE_AFTER_TURN':
            self.pub_cmd_vel.publish(self.sensor_data['lane_cmd'])
            return

        self.pub_cmd_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()