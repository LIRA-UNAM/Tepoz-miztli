#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_node')

        # --- Configuración de QoS (Calidad de Servicio) ---
        # PX4 usa "Best Effort" para la mayoría de los tópicos. 
        # Si no configuramos esto así, ROS 2 no escuchará nada.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publicadores ---
        # 1. Modo de control (¿Qué queremos controlar? Posición, Velocidad, etc.)
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        # 2. Setpoints de Trayectoria (¿A dónde queremos ir?)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # 3. Comandos al Vehículo (Armar, Desarmar, Cambiar Modo)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # --- Suscriptores ---
        # Escuchamos el estado para saber si el drone ya se armó
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Variables internas
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter_ = 0

        # Timer: PX4 necesita recibir comandos a > 2Hz. Usaremos 10Hz (0.1s)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        print("Nodo de Control Offboard Inicializado...")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def timer_callback(self):
        # Primero publicamos el modo Offboard y el Setpoint
        # IMPORTANTE: PX4 requiere recibir setpoints ANTES de permitir el cambio a modo Offboard
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Esperamos 10 ciclos (1 segundo) enviando datos antes de intentar armar
        if self.offboard_setpoint_counter_ == 10:
            self.engage_offboard_mode()
            self.arm()

        # Incrementamos el contador
        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True  # Queremos controlar posición
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        # --- COORDENADAS NED (North, East, Down) ---
        # X = Norte (positivo), Sur (negativo)
        # Y = Este (positivo), Oeste (negativo)
        # Z = Abajo (positivo), Arriba (negativo)
        
        msg.position = [0.0, 0.0, -2.0] # Ir a 2 metros de ALTURA (Negativo es arriba)
        msg.yaw = 0.0 # Mirar al Norte (0 grados)
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def engage_offboard_mode(self):
        # Comando para cambiar a modo Offboard
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.param1 = 1.0 # custom_mode
        msg.param2 = 6.0 # OFFBOARD mode
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        print("Intentando cambiar a modo Offboard...")

    def arm(self):
        # Comando para Armar motores
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 # 1 = ARM, 0 = DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        print("Enviando comando de ARMADO...")

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()