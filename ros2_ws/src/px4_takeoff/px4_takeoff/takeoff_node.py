#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand
)

class PX4AttitudeTakeoff(Node):

    def __init__(self):
        super().__init__('px4_attitude_takeoff')

        # -------------------------------
        # QoS Profile (CRÍTICO PARA UXRCE-DDS)
        # -------------------------------
        # PX4 usa "Best Effort" para sensores/estado. Si usas "Reliable" (default ROS2),
        # a veces no se comunican bien. Esto asegura la compatibilidad.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -------------------------------
        # Publishers
        # -------------------------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            qos_profile
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # Timer a 10 Hz (PX4 necesita > 2Hz para no salir de Offboard)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.counter = 0

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # ------------------------------------------------
        # 1. PUBLICAR HEARTBEAT DE OFFBOARD (Siempre primero)
        # ------------------------------------------------
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = True   # Solo controlamos actitud
        offboard.body_rate = False

        self.offboard_pub.publish(offboard)

        # ------------------------------------------------
        # 2. CALCULAR ATTITUDE SETPOINT
        # ------------------------------------------------
        att = VehicleAttitudeSetpoint()
        att.timestamp = now
        # Cuaternión identidad [w, x, y, z] -> [1, 0, 0, 0] = Plano
        att.q_d = [1.0, 0.0, 0.0, 0.0]

        # THRUST (Potencia): ¡CUIDADO!
        # -0.6 es muy fuerte para interiores si el dron es ligero.
        # Empieza suave. El rango es [-1.0 (max) a 0.0 (min)].
        if self.counter < 100: 
             # Motores girando despacio (Idle) o subiendo muy lento
            att.thrust_body = [0.0, 0.0, -0.2] 
        elif self.counter < 200:
            # Despegue
            att.thrust_body = [0.0, 0.0, -0.55] 
        else:
            # Mantener (ajusta este valor según el peso de tu dron)
            att.thrust_body = [0.0, 0.0, -0.45] 

        self.attitude_pub.publish(att)

        # ------------------------------------------------
        # 3. MÁQUINA DE ESTADOS (CAMBIO DE MODO Y ARMADO)
        # ------------------------------------------------
        # Esperamos unos ciclos para que el flujo de datos sea estable
        if self.counter == 20:
            # Comando 176: DO_SET_MODE
            # Param 1 = 1 (Custom Mode)
            # Param 2 = 6 (OFFBOARD Mode) <-- FALTABA ESTO EN TU CÓDIGO
            self.send_cmd(176, param1=1.0, param2=6.0)
            self.get_logger().info('>>> Intentando cambiar a OFFBOARD...')

        if self.counter == 40:
            # Comando 400: ARM_DISARM
            # Param 1 = 1.0 (Arm)
            self.send_cmd(400, param1=1.0)
            self.get_logger().info('>>> Intentando ARMAR...')

        self.counter += 1

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2) # Param2 es vital para el cambio de modo
        
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = PX4AttitudeTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()