import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.action import RotateAngle


class RotateNode(Node):
    def __init__(self):
        super().__init__('rotate_node')

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            RotateAngle,
            '/rotate',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.current_yaw = 0.0
        self.odom_received = False

        self.get_logger().info('Rotate node iniciado. Aguardando goals em /rotate.')

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        self.odom_received = True

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def execute_callback(self, goal_handle):
        target_degrees = goal_handle.request.degrees
        target_radians = math.radians(abs(target_degrees))

        if target_radians == 0.0:
            self.stop_robot()
            result = RotateAngle.Result()
            result.success = True
            goal_handle.succeed()
            return result

        while not self.odom_received:
            self.get_logger().info('Aguardando primeira leitura de /odom...')
            time.sleep(0.1)

        direction = 1.0 if target_degrees >= 0.0 else -1.0
        angular_speed = 0.4 * direction

        accumulated_rotation = 0.0
        last_yaw = self.current_yaw

        self.get_logger().info(f'Executando rotação de {target_degrees:.2f} graus.')

        while accumulated_rotation < target_radians:
            current_yaw = self.current_yaw
            delta_yaw = self.normalize_angle(current_yaw - last_yaw)
            accumulated_rotation += abs(delta_yaw)
            last_yaw = current_yaw

            remaining_radians = max(0.0, target_radians - accumulated_rotation)

            feedback = RotateAngle.Feedback()
            feedback.degrees_remaining = math.degrees(remaining_radians)
            goal_handle.publish_feedback(feedback)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = angular_speed
            self.cmd_vel_pub.publish(cmd)

            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().info('Rotação concluída com sucesso.')

        result = RotateAngle.Result()
        result.success = True
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
