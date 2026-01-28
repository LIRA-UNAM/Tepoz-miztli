import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

import numpy as np

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge_node')

        # Configuración QoS para DDS (CRÍTICO: PX4 usa Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Suscripción a la odometría de VINS o RealSense (ajusta el tópico según tu config)
        # Ejemplo: '/camera/pose/sample' para T265 o '/vins_estimator/odometry'
        self.subscription = self.create_subscription(
            Odometry,
            '/camera/pose/sample',  # <--- CAMBIA ESTO POR TU TÓPICO DE ODOMETRÍA
            self.listener_callback,
            10)

        # Publicador hacia PX4
        self.publisher_ = self.create_publisher(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            qos_profile)

    def listener_callback(self, msg):
        vehicle_odom_msg = VehicleOdometry()

        # 1. Sincronización de tiempo (Microsegundos)
        vehicle_odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle_odom_msg.timestamp_sample = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)

        # 2. Definición de Frames (Sistemas de referencia)
        # PX4 v1.16 prefiere recibir datos referenciados localmente (FRD)
        vehicle_odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        vehicle_odom_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD

        # 3. Conversión de Coordenadas (ENU a NED/FRD)
        # ROS X (Frente) -> PX4 X (Norte/Frente)
        # ROS Y (Izquierda) -> PX4 -Y (Este/Derecha)  <-- OJO: Signo negativo
        # ROS Z (Arriba) -> PX4 -Z (Abajo)            <-- OJO: Signo negativo
        
        # Posición
        vehicle_odom_msg.position = [
            msg.pose.pose.position.x,
            -msg.pose.pose.position.y,
            -msg.pose.pose.position.z
        ]

        # Orientación (Cuaterniones)
        # Necesitamos rotar el cuaternión de ENU a NED.
        # Una forma simple si la cámara apunta al frente es la transformación estándar:
        # q_ned = [q_enu_w, q_enu_x, -q_enu_y, -q_enu_z] (aproximación básica para frame alignment)
        # Para mayor precisión matemática se debe aplicar una matriz de rotación, 
        # pero para empezar prueba con mapeo directo ajustado:
        
        vehicle_odom_msg.q = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            -msg.pose.pose.orientation.y,
            -msg.pose.pose.orientation.z
        ]

        # Velocidad (Lineal y Angular)
        vehicle_odom_msg.velocity = [
            msg.twist.twist.linear.x,
            -msg.twist.twist.linear.y,
            -msg.twist.twist.linear.z
        ]
        
        vehicle_odom_msg.angular_velocity = [
            msg.twist.twist.angular.x,
            -msg.twist.twist.angular.y,
            -msg.twist.twist.angular.z
        ]

        # Enviar
        self.publisher_.publish(vehicle_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    vio_bridge = VIOBridge()
    rclpy.spin(vio_bridge)
    vio_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()