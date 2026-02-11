#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus
)

class PX4FlowTakeoff(Node):

    def __init__(self):
        super().__init__('px4_flow_takeoff')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.timer = self.create_timer(0.1, self.timer_cb) # 10 Hz
        self.counter = 0
        self.takeoff_height = -2.0  # PX4 usa coordenadas NED (Z negativo es hacia arriba)
        
    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # 1. Modo Offboard: Habilitamos POSICIÓN
        # Al activar .position = True, PX4 usa el EKF2 (y tu Optical Flow) para no moverse
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        # 2. Lógica de Trayectoria (Máquina de estados simple)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.yaw = 0.0 # Mantener el frente hacia el norte

        if self.counter < 150: # Primeros 15 segundos (aprox)
            # Mantenerse en X=0, Y=0 y subir a 2 metros
            setpoint.position = [0.0, 0.0, self.takeoff_height]
        
        elif self.counter < 190: # Siguientes 4 segundos (150 a 190 ciclos)
            # Mantener la posición de 2 metros (Estabilizado con Flow)
            setpoint.position = [0.0, 0.0, self.takeoff_height]
            if self.counter == 151:
                self.get_logger().info('>>> Manteniendo altura 4 segundos...')

        else: # Fase de descenso
            # Volver a 0.0 (suelo)
            setpoint.position = [0.0, 0.0, 0.0]
            if self.counter == 191:
                self.get_logger().info('>>> Descendiendo...')

        self.trajectory_pub.publish(setpoint)

        # 3. Comandos de Armado y Offboard
        if self.counter == 10:
            self.send_cmd(176, param1=1.0, param2=6.0) # OFFBOARD
        
        if self.counter == 20:
            self.send_cmd(400, param1=1.0) # ARM
            self.get_logger().info('>>> Despegando con Optical Flow...')

        self.counter += 1

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = PX4FlowTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()