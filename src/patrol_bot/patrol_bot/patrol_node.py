import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Estado inicial
        self.patrol_active = False

        # Leitura do lidar
        self.front_distance = float('inf')
        self.obstacle_threshold = 0.5
        self.scan_received = False

        # Controle para evitar spam de logs
        self.last_motion_state = 'IDLE'

        # Publisher de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber do lidar
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Service server para iniciar patrulha
        self.start_service = self.create_service(
            SetBool,
            '/start_patrol',
            self.start_patrol_callback
        )

        # Timer principal
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Patrol node iniciado em IDLE. Aguardando /start_patrol.')

    def start_patrol_callback(self, request, response):
        if request.data:
            if not self.patrol_active:
                self.patrol_active = True
                response.success = True
                response.message = 'Patrulha iniciada.'
                self.get_logger().info('Serviço recebido: patrulha iniciada.')
            else:
                response.success = False
                response.message = 'Robô já está em operação.'
                self.get_logger().info('Serviço recebido, mas o robô já está operando.')
        else:
            response.success = False
            response.message = 'Use data=true para iniciar a patrulha.'
            self.get_logger().info('Serviço recebido com data=false.')

        return response

    def scan_callback(self, msg):
        self.scan_received = True

        # Índice central do LaserScan = frente do robô
        center_index = len(msg.ranges) // 2
        distance = msg.ranges[center_index]

        # Trata leituras inválidas
        if math.isinf(distance) or math.isnan(distance):
            self.front_distance = float('inf')
        else:
            self.front_distance = distance

    def timer_callback(self):
        msg = Twist()

        if not self.patrol_active:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publish_and_log(msg, 'IDLE')
            return

        if not self.scan_received:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publish_and_log(msg, 'WAITING_SCAN')
            return

        if self.front_distance < self.obstacle_threshold:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publish_and_log(msg, 'OBSTACLE_STOP')
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.publish_and_log(msg, 'MOVING_FORWARD')

    def publish_and_log(self, msg, state_name):
        self.cmd_vel_pub.publish(msg)

        if state_name != self.last_motion_state:
            if state_name == 'IDLE':
                self.get_logger().info('Estado: IDLE. Robô parado.')
            elif state_name == 'WAITING_SCAN':
                self.get_logger().info('Patrulha ativa, aguardando leituras do /scan.')
            elif state_name == 'OBSTACLE_STOP':
                self.get_logger().info(
                    f'Obstáculo detectado à frente: {self.front_distance:.2f} m. Robô parado.'
                )
            elif state_name == 'MOVING_FORWARD':
                self.get_logger().info(
                    f'Caminho livre ({self.front_distance:.2f} m). Andando para frente.'
                )

            self.last_motion_state = state_name


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
