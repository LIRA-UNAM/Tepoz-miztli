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
        
        # QoS Profile (Fiabilidad para comandos)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores y Suscriptores
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile
        )
        
        # Para bloquear el giro (Yaw) y que no de vueltas
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            qos_profile
        )

        # Timer principal a 10 Hz (0.1 segundos por ciclo)
        self.timer = self.create_timer(0.1, self.timer_cb) 
        self.counter = 0

        # --- VARIABLES DE CONTROL ---
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.locked_yaw = None 
        
        self.target_height = -2.0  # 2 metros (Negativo en NED)
        self.hold_time_seconds = 10.0 # Tiempo que quieres esperar ARRIBA
        
        # El contador para la espera (Se inicia en 0)
        self.hover_timer_ticks = 0 
        
        self.state = "INIT"

    def local_pos_cb(self, msg):
        self.current_z = msg.z

    def attitude_cb(self, msg):
        # Convertir cuaternión a Yaw
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # --- 1. MODO OFFBOARD (Habilitamos Posición y Velocidad) ---
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True 
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # --- 2. PREPARAR SETPOINT ---
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        # Bloqueo de Yaw (Para que no gire como loco)
        if self.locked_yaw is None:
            self.locked_yaw = self.current_yaw
        setpoint.yaw = self.locked_yaw

        # --- 3. MÁQUINA DE ESTADOS ---

        if self.state == "INIT":
            # Paso 0: Esperar a que la conexión sea estable
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]

            if self.counter > 20: # 2 segundos de buffer
                self.send_cmd(176, param1=1.0, param2=6.0) # Modo Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]
            
            if self.counter > 30: # 1 segundo después
                self.send_cmd(400, param1=1.0) # Armar motores
                self.get_logger().info(">>> ARMADO. Iniciando ascenso...")
                self.state = "CLIMBING"

        elif self.state == "CLIMBING":
            # Acción: Subir a 2 metros
            setpoint.position = [0.0, 0.0, self.target_height]
            
            # Truco anti-drift: Velocidad X/Y en 0, Z controlada por posición (-0.8 ayuda a subir decidido)
            setpoint.velocity = [0.0, 0.0, -0.8] 

            # Lógica de transición: ¿Ya llegamos?
            # Calculamos el error: Diferencia entre donde estoy y donde quiero ir
            error_distancia = abs(self.current_z - self.target_height)

            # Si el error es menor a 15cm (0.15), consideramos que LLEGAMOS
            if error_distancia < 0.15:
                self.state = "HOVERING"
                self.hover_timer_ticks = 0 # <--- AQUÍ REINICIAMOS EL CONTADOR
                self.get_logger().info(">>> ALTURA ALCANZADA. Iniciando conteo de 4 segundos...")

        elif self.state == "HOVERING":
            # Acción: Mantenerse quieto
            setpoint.position = [0.0, 0.0, self.target_height]
            setpoint.velocity = [0.0, 0.0, 0.0] # Freno total

            # Lógica del Tiempo: Solo contamos aquí
            self.hover_timer_ticks += 1
            
            # Convertimos ticks a segundos (cada tick es 0.1s)
            tiempo_transcurrido = self.hover_timer_ticks * 0.1

            if tiempo_transcurrido >= self.hold_time_seconds:
                self.state = "DESCENDING"
                self.get_logger().info(">>> 4 SEGUNDOS COMPLETADOS. Bajando...")

        elif self.state == "DESCENDING":
            # Acción: Bajar suave
            setpoint.position = [0.0, 0.0, 0.0] # Ir al suelo
            setpoint.velocity = [0.0, 0.0, 0.3] # Bajar lento (30 cm/s)

            # Lógica de Aterrizaje: ¿Estamos cerca del suelo?
            # En NED, el suelo es 0.0. Si z > -0.15 estamos a 15cm del piso.
            if self.current_z > -0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0) # Desarmar (Matar motores)
                self.get_logger().info(">>> ATERRIZAJE TERMINADO. Motores apagados.")

        elif self.state == "LANDED":
            # Seguridad: Seguir enviando comando de apagado y posición cero
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]

        # Publicar el comando calculado
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
