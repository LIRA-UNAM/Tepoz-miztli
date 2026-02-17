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

class PX4Mission2Meters(Node):
    def __init__(self):
        super().__init__('px4_mission_2m')
        
        # QoS Profile
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

        # --- VARIABLES DE ESTADO Y POSICIÓN ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.current_yaw = 0.0
        self.locked_yaw = None 
        
        # --- CONFIGURACIÓN DE LA MISIÓN ---
        self.target_height = -2.0  # 2m Arriba
        self.target_dist = 2.0     # 2m Adelante
        self.hover_time = 4.0      # Tiempo de espera
        
        # Contadores
        self.state = "INIT"
        self.timer_ticks = 0       # Contador genérico para esperas

    def local_pos_cb(self, msg):
        # ¡IMPORTANTE! Ahora leemos X y Y también
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # 1. Heartbeat Offboard
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True 
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # 2. Setpoint Base
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        if self.locked_yaw is None:
            self.locked_yaw = self.current_yaw
        setpoint.yaw = self.locked_yaw

        # 3. MÁQUINA DE ESTADOS
        
        if self.state == "INIT":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) # Set Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]
            if self.counter > 30:
                self.send_cmd(400, param1=1.0) # Arm
                self.get_logger().info(">>> ARMADO. Subiendo...")
                self.state = "CLIMBING"

        elif self.state == "CLIMBING":
            # Meta: Ir a [0, 0, -2]
            setpoint.position = [0.0, 0.0, self.target_height]
            # Vel: X/Y quietos, Z subiendo
            setpoint.velocity = [0.0, 0.0, -0.8] 

            # ¿Llegamos a la altura?
            if abs(self.current_z - self.target_height) < 0.15:
                self.state = "STABILIZE"
                self.timer_ticks = 0 # Reiniciar contador
                self.get_logger().info(">>> ALTURA OK. Estabilizando antes de avanzar...")

        elif self.state == "STABILIZE":
            # Breve pausa (2s) antes de avanzar para asegurar el Optical Flow
            setpoint.position = [0.0, 0.0, self.target_height]
            setpoint.velocity = [0.0, 0.0, 0.0] # Freno total

            self.timer_ticks += 1
            if self.timer_ticks >= 20: # 20 ciclos = 2 segundos
                self.state = "MOVING_FORWARD"
                self.get_logger().info(">>> AVANZANDO 2 METROS...")

        elif self.state == "MOVING_FORWARD":
            # LA CLAVE DEL MOVIMIENTO RECTO:
            # Posición Target: X=2.0, Y=0.0, Z=-2.0
            setpoint.position = [self.target_dist, 0.0, self.target_height]
            
            # Velocidad:
            # X = NaN (Deja que el controlador decida la velocidad para llegar a 2m)
            # Y = 0.0 (¡OBLIGATORIO! Riel virtual para que no driftee a los lados)
            # Z = 0.0 (Mantener altura)
            setpoint.velocity = [float('nan'), 0.0, 0.0]

            # ¿Llegamos a los 2 metros en X?
            dist_error = abs(self.current_x - self.target_dist)
            
            if dist_error < 0.20: # Margen de 20cm
                self.state = "HOVER_FINAL"
                self.timer_ticks = 0 # Reiniciamos contador para los 4s exactos
                self.get_logger().info(">>> LLEGAMOS. Iniciando conteo de 4s...")

        elif self.state == "HOVER_FINAL":
            # Quedarse quieto en [2.0, 0.0, -2.0]
            setpoint.position = [self.target_dist, 0.0, self.target_height]
            setpoint.velocity = [0.0, 0.0, 0.0] # Ancla total

            # Aquí contamos los 4 segundos
            self.timer_ticks += 1
            if (self.timer_ticks * 0.1) >= self.hover_time:
                self.state = "DESCENDING"
                self.get_logger().info(">>> TIEMPO CUMPLIDO. Aterrizando...")

        elif self.state == "DESCENDING":
            # Bajar en [2.0, 0.0]
            setpoint.position = [self.target_dist, 0.0, 0.0] # Ir al suelo
            setpoint.velocity = [0.0, 0.0, 0.3] # Bajar lento

            if self.current_z > -0.15: # Casi en el suelo
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0) # Desarmar
                self.get_logger().info(">>> ATERRIZAJE COMPLETADO.")

        elif self.state == "LANDED":
            setpoint.position = [self.target_dist, 0.0, 0.0]
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
    node = PX4Mission2Meters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()