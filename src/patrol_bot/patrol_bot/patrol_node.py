import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

from custom_interfaces.action import RotateAngle


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Estado geral
        self.patrol_active = False
        self.rotation_in_progress = False

        # Controle da missão:
        # 0 -> ainda não bateu no obstáculo do lado oposto
        # 1 -> já bateu no lado oposto, girou e está voltando
        self.obstacle_count = 0

        # Lidar
        self.front_distance = float('inf')
        self.scan_received = False
        self.obstacle_threshold = 0.5

        # Ângulo da rotação quando encontra obstáculo
        self.rotation_angle_degrees = 180.0

        # Controle de logs
        self.last_motion_state = ''

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Service server
        self.start_service = self.create_service(
            SetBool,
            '/start_patrol',
            self.start_patrol_callback
        )

        # Action client
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate')

        # Timer principal
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Patrol node iniciado em IDLE. Aguardando /start_patrol.')

    def start_patrol_callback(self, request, response):
        if request.data:
            if not self.patrol_active:
                self.patrol_active = True
                self.rotation_in_progress = False
                self.obstacle_count = 0
                self.last_motion_state = ''
                response.success = True
                response.message = 'Patrulha iniciada.'
                self.get_logger().info('Serviço recebido: patrulha iniciada.')
            else:
                response.success = False
                response.message = 'Robô já está em operação.'
                self.get_logger().info('Serviço recebido, mas o robô já está operando.')
        else:
            if self.patrol_active:
                self.patrol_active = False
                self.rotation_in_progress = False
                self.obstacle_count = 0
                self.stop_robot()
                self.last_motion_state = ''
                response.success = True
                response.message = 'Patrulha parada.'
                self.get_logger().info('Serviço recebido: patrulha parada.')
            else:
                response.success = False
                response.message = 'O robô já está parado.'
                self.get_logger().info('Serviço recebido com data=false, mas o robô já estava parado.')

        return response

    def scan_callback(self, msg):
        self.scan_received = True

        center_index = len(msg.ranges) // 2
        distance = msg.ranges[center_index]

        if math.isinf(distance) or math.isnan(distance):
            self.front_distance = float('inf')
        else:
            self.front_distance = distance

    def timer_callback(self):
        if not self.patrol_active:
            self.stop_robot()
            self.log_state('IDLE')
            return

        if not self.scan_received:
            self.stop_robot()
            self.log_state('WAITING_SCAN')
            return

        if self.rotation_in_progress:
            self.log_state('ROTATING')
            return

        if self.front_distance < self.obstacle_threshold:
            self.stop_robot()

            # Primeiro obstáculo: chegou ao lado oposto, então gira e volta
            if self.obstacle_count == 0:
                self.obstacle_count = 1
                self.log_state('OBSTACLE_DETECTED')
                self.send_rotate_goal(self.rotation_angle_degrees)

            # Segundo obstáculo: retornou ao lado inicial, então encerra
            else:
                self.patrol_active = False
                self.rotation_in_progress = False
                self.obstacle_count = 0
                self.stop_robot()
                self.get_logger().info(
                    'Obstáculo detectado novamente. Robô retornou ao ponto inicial. Patrulha concluída.'
                )
                self.log_state('MISSION_COMPLETE')
            return

        self.move_forward()
        self.log_state('MOVING_FORWARD')

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def send_rotate_goal(self, degrees):
        if self.rotation_in_progress:
            return

        if not self.rotate_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Action /rotate não está disponível ainda.')
            return

        self.rotation_in_progress = True
        self.stop_robot()

        goal_msg = RotateAngle.Goal()
        goal_msg.degrees = float(degrees)

        self.get_logger().info(f'Enviando goal de rotação: {degrees:.2f} graus.')

        self.send_goal_future = self.rotate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.rotate_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.rotate_goal_response_callback)

    def rotate_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal de rotação rejeitado.')
            self.rotation_in_progress = False
            return

        self.get_logger().info('Goal de rotação aceito.')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.rotate_result_callback)

    def rotate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback da rotação: faltam {feedback.degrees_remaining:.2f} graus.'
        )

    def rotate_result_callback(self, future):
        result_response = future.result()
        result = result_response.result
        status = result_response.status

        if result.success or status == 4:
            self.get_logger().info('Rotação concluída. Retomando patrulha.')
        else:
            self.get_logger().warn('Rotação terminou sem sucesso.')

        self.rotation_in_progress = False
        self.last_motion_state = ''

    def log_state(self, state_name):
        if state_name == self.last_motion_state:
            return

        if state_name == 'IDLE':
            self.get_logger().info('Estado: IDLE. Robô parado.')
        elif state_name == 'WAITING_SCAN':
            self.get_logger().info('Patrulha ativa, aguardando leituras do /scan.')
        elif state_name == 'ROTATING':
            self.get_logger().info('Robô está rotacionando.')
        elif state_name == 'OBSTACLE_DETECTED':
            self.get_logger().info(
                f'Obstáculo detectado à frente: {self.front_distance:.2f} m. Chamando rotação.'
            )
        elif state_name == 'MOVING_FORWARD':
            self.get_logger().info(
                f'Caminho livre ({self.front_distance:.2f} m). Andando para frente.'
            )
        elif state_name == 'MISSION_COMPLETE':
            self.get_logger().info('Estado: MISSION_COMPLETE. Patrulha encerrada com sucesso.')

        self.last_motion_state = state_name


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()