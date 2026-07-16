#!/usr/bin/env python3
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MissionTunnel(Node):

    def __init__(self):
        super().__init__('mission_tunnel')

        self.declare_parameter('init_pose.position.x', 0.0)
        self.declare_parameter('init_pose.position.y', 0.0)
        self.declare_parameter('init_pose.position.z', 0.0)
        self.declare_parameter('init_pose.orientation.x', 0.0)
        self.declare_parameter('init_pose.orientation.y', 0.0)
        self.declare_parameter('init_pose.orientation.yaw', 0)
        self.declare_parameter('goal_pose.position.x', 0.0)
        self.declare_parameter('goal_pose.position.y', 0.0)
        self.declare_parameter('goal_pose.position.z', 0.0)
        self.declare_parameter('goal_pose.orientation.x', 0.0)
        self.declare_parameter('goal_pose.orientation.y', 0.0)
        self.declare_parameter('goal_pose.orientation.yaw', 0)
        self.declare_parameter('forward_seconds', 10.0)
        self.declare_parameter('forward_speed', 0.08)

        self.init_position_x = self.get_parameter('init_pose.position.x').value
        self.init_position_y = self.get_parameter('init_pose.position.y').value
        self.init_position_z = self.get_parameter('init_pose.position.z').value
        self.init_orientation_x = self.get_parameter('init_pose.orientation.x').value
        self.init_orientation_y = self.get_parameter('init_pose.orientation.y').value
        self.init_orientation_yaw = self.get_parameter('init_pose.orientation.yaw').value
        self.goal_position_x = self.get_parameter('goal_pose.position.x').value
        self.goal_position_y = self.get_parameter('goal_pose.position.y').value
        self.goal_position_z = self.get_parameter('goal_pose.position.z').value
        self.goal_orientation_x = self.get_parameter('goal_pose.orientation.x').value
        self.goal_orientation_y = self.get_parameter('goal_pose.orientation.y').value
        self.goal_orientation_yaw = self.get_parameter('goal_pose.orientation.yaw').value
        self.FORWARD_SECONDS = self.get_parameter('forward_seconds').value
        self.forward_speed = self.get_parameter('forward_speed').value

        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.start_timer = self.create_timer(1.0, self.start_initial_phase)
        self.started = False

        self.init_timer = None
        self.phase_timer = None
        self.forward_timer = None
        self.forward_start_time = None

    def start_initial_phase(self):
        if self.started:
            return
        self.started = True
        self.start_timer.cancel()

        self.init_timer = self.create_timer(0.1, self.publish_initial_pose)
        self.phase_timer = self.create_timer(10.0, self.send_nav_goal)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.init_position_x
        msg.pose.pose.position.y = self.init_position_y
        msg.pose.pose.position.z = self.init_position_z

        yaw = math.radians(self.init_orientation_yaw)
        msg.pose.pose.orientation.x = self.init_orientation_x
        msg.pose.pose.orientation.y = self.init_orientation_y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance = [0.0] * 36
        self.init_pose_pub.publish(msg)

    def send_nav_goal(self):
        if self.init_timer is not None:
            self.init_timer.cancel()
        if self.phase_timer is not None:
            self.phase_timer.cancel()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.goal_position_x
        goal_msg.pose.pose.position.y = self.goal_position_y
        goal_msg.pose.pose.position.z = self.goal_position_z

        yaw = math.radians(self.goal_orientation_yaw)
        goal_msg.pose.pose.orientation.x = self.goal_orientation_x
        goal_msg.pose.pose.orientation.y = self.goal_orientation_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Esperando action server de Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Enviando goal a Nav2...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distancia restante al goal: {dist:.2f} m')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado por Nav2. Cerrando nodo.')
            self.shutdown_node()
            return
        self.get_logger().info('Goal aceptado. Esperando llegada...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        self.get_logger().info('Goal succeeded. Iniciando avance final de %.1fs' % self.FORWARD_SECONDS)
        self.forward_start_time = self.get_clock().now()
        self.forward_timer = self.create_timer(0.05, self._forward_loop)

    def _forward_loop(self):
        elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
        twist = Twist()
        if elapsed < self.FORWARD_SECONDS:
            twist.linear.x = self.forward_speed
            self.cmd_pub.publish(twist)
        else:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Avance final completado. Cerrando nodo.')
            self.shutdown_node()

    def shutdown_node(self):
        if self.forward_timer is not None:
            self.forward_timer.cancel()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MissionTunnel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()