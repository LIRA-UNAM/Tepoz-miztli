#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleStatus
)

class PX4PreciseHover(Node):
    def __init__(self):
        super().__init__('px4_precise_hover')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Suscriptores
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_cb) 
        self.counter = 0

        # --- VARIABLES ---
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.locked_yaw = None 
        
        self.target_height = -2.0  # 2 metros
        self.hold_time_seconds = 5.0 # <--- AHORA SON 5 SEGUNDOS
        
        # Variable para guardar la "Hora de llegada"
        self.hover_start_time = None 
        
        self.state = "INIT"

    def local_pos_cb(self, msg):
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        # Obtenemos la hora actual en nanosegundos
        now_ns = self.get_clock().now().nanoseconds 
        now_ms = now_ns // 1000

        # 1. MODO OFFBOARD
        offboard = OffboardControlMode()
        offboard.timestamp = now_ms
        offboard.position = True
        offboard.velocity = True 
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # 2. SETPOINT
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now_ms

        if self.locked_yaw is None:
            self.locked_yaw = self.current_yaw
        setpoint.yaw = self.locked_yaw

        # 3. MÁQUINA DE ESTADOS

        if self.state == "INIT":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]

            if self.counter > 20: 
                self.send_cmd(176, param1=1.0, param2=6.0) 
                self.state = "ARMING"

        elif self.state == "ARMING":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]
            
            if self.counter > 30: 
                self.send_cmd(400, param1=1.0) 
                self.get_logger().info(">>> DESPEGANDO...")
                self.state = "CLIMBING"

        elif self.state == "CLIMBING":
            setpoint.position = [0.0, 0.0, self.target_height]
            setpoint.velocity = [0.0, 0.0, -0.8] 

            # Verificamos si llegamos
            error_distancia = abs(self.current_z - self.target_height)

            if error_distancia < 0.20: # Margen de 20cm
                self.state = "HOVERING"
                # AQUI ESTA EL TRUCO: Guardamos la hora exacta en que llegamos
                self.hover_start_time = now_ns 
                self.get_logger().info(f">>> LLEGAMOS A 2M. INICIANDO CRONÓMETRO DE {self.hold_time_seconds}s")

        elif self.state == "HOVERING":
            # Acción: Mantenerse quieto en -2.0 metros
            setpoint.position = [0.0, 0.0, self.target_height]
            
            # TRUCO 2: Ponemos Z en NaN para que el PID de posición trabaje mejor y no rebote
            setpoint.velocity = [0.0, 0.0, float('nan')] 

            # Calculamos cuánto tiempo ha pasado desde que llegamos
            time_passed_ns = now_ns - self.hover_start_time
            time_passed_seconds = time_passed_ns / 1e9 # Convertir nano a segundos

            # Imprimimos en pantalla para que veas el conteo (Debug)
            if self.counter % 10 == 0: # Imprimir cada segundo aprox
                self.get_logger().info(f"Hover: {time_passed_seconds:.1f} / {self.hold_time_seconds} s")

            if time_passed_seconds >= self.hold_time_seconds:
                self.state = "DESCENDING"
                self.get_logger().info(">>> TIEMPO CUMPLIDO. BAJANDO...")

        elif self.state == "DESCENDING":
            setpoint.position = [0.0, 0.0, 0.0] 
            setpoint.velocity = [0.0, 0.0, 0.3] 

            if self.current_z > -0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0) 
                self.get_logger().info(">>> TIERRA.")

        elif self.state == "LANDED":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = PX4PreciseHover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()