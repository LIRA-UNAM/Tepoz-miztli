#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
import numpy as np
import enum

class FlightState(enum.Enum):
    """
    Definición de los nodos topológicos para la Máquina de Estados Finitos.
    """
    INIT = 0
    TAKEOFF = 1
    HOLD = 2
    LAND = 3
    DONE = 4

class OffboardControlNode(Node):
    """
    Controlador central Offboard operando bajo el framework ROS 2 Humble/Iron.
    Resuelve crónicamente las colisiones de latencia de heartbeats y mitiga la
    deriva estocástica mediante la asignación matemática estricta de variables NaN
    para aislar lazos de control cinemático.
    """
    def __init__(self):
        super().__init__('offboard_control_node')

        # ---------------------------------------------------------------------
        # PERFIL DE CALIDAD DE SERVICIO (QoS)
        # Adaptación microscópica indispensable para que Micro XRCE-DDS
        # puenteé los paquetes con el RTOS interno del Pixhawk 6x.
        # ---------------------------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------------------------------------------------------------
        # PUBLICADORES Y SUSCRIPTORES
        # ---------------------------------------------------------------------
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.local_position_callback, qos_profile)

        # ---------------------------------------------------------------------
        # CONTEXTO DE VARIABLE LOCAL Y PARÁMETROS GEOMÉTRICOS
        # ---------------------------------------------------------------------
        self.state = FlightState.INIT
        self.local_pos = np.array([0.0, 0.0, 0.0])
        # Altura en marco Local NED (North, East, Down). Down negativo implica Ascenso.
        self.target_altitude = -2.5  
        self.hold_time_start = 0.0
        self.hold_duration = 10.0    # Umbral paramétrico del estacionamiento temporal

        # ---------------------------------------------------------------------
        # CRONÓMETRO LATIDO (HEARTBEAT TIMER)
        # 20 Hz = Período de 0.05s. Satisface el requisito paramétrico de >2 Hz 
        # impuesto por el módulo commander de PX4 para preservar la viabilidad.
        # ---------------------------------------------------------------------
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Nodo de control arquitectónico Offboard inicializado.")

    def local_position_callback(self, msg):
        """
        Extracción asíncrona del estimado cartesiano consolidado por el EKF2.
        """
        self.local_pos = msg.x
        self.local_pos = msg.y
        self.local_pos = msg.z

    def timer_callback(self):
        """
        Orquestador cíclico determinista. Es el núcleo que evita la paralización
        del Event Loop, suplantando cualquier instrucción perjudicial tipo 'sleep()'.
        """
        if self.state == FlightState.DONE:
            return

        # 1. EMISIÓN INCONDICIONAL DE LA MÁSCARA BOOLEANA (HEARTBEAT VITAL)
        self.publish_offboard_control_mode()

        # 2. TRANSICIÓN A TRAVÉS DEL LÁTICE DE LA MÁQUINA DE ESTADOS
        if self.state == FlightState.INIT:
            self.arm_vehicle()
            self.set_offboard_mode()
            self.state = FlightState.TAKEOFF
            self.get_logger().info("Inicialización validada. Desencadenando fase TAKEOFF (2.5m).")

        elif self.state == FlightState.TAKEOFF:
            # Consigna Rígida Posicional. Velocidades delegadas al PID (NaN).
            self.publish_position_setpoint(0.0, 0.0, self.target_altitude)
            
            # Condición de superación de umbral geométrico con hiséresis de tolerancia (0.2m)
            # Como Z desciende negativamente durante el ascenso, evaluamos por <=
            if self.local_pos <= (self.target_altitude + 0.2):
                self.state = FlightState.HOLD
                # Retención escalar temporal proveniente del núcleo basal ROS, NO bloqueante.
                self.hold_time_start = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info("Altitud cinemática lograda. Transitando a ciclo estacionario HOLD (10s).")

        elif self.state == FlightState.HOLD:
            # El mantenimiento (Hover) en PX4 exige publicar IDÉNTICA coordenada de 
            # posición que la fijada previamente. No alterar a ceros ni inyectar control veloz.
            self.publish_position_setpoint(0.0, 0.0, self.target_altitude)

            # Escrutinio aritmético del transcurso temporal contra reloj de referencia
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.hold_time_start
            
            if elapsed >= self.hold_duration:
                self.state = FlightState.LAND
                self.get_logger().info("Estabilidad escalar expirada. Desencadenando maniobra LAND.")

        elif self.state == FlightState.LAND:
            # Emisión imperativa de la orden absoluta de Aterrizaje al gestor uORB.
            self.land_vehicle()
            self.state = FlightState.DONE

    def publish_offboard_control_mode(self):
        """
        Arbitraje del enrutador de mensajes. Ordena al sistema ignorar bucles cinemáticos
        y abocarse exclusivamente al error posicional. 
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        """
        Construcción meticulosa del vector de control.
        La saturación rigurosa de velocidades y tirones con NaN (Not a Number)
        fuerza estructuralmente al Firmware PX4 a computarlos endógenamente.
        Si se omitieran o se igualaran a ceros absolutos, chocaría la cascada PID.
        """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # Cumplimiento del estándar IEEE 754 ineludible en arquitecturas complejas PX4.
        msg.velocity = [np.nan, np.nan, np.nan]
        msg.acceleration = [np.nan, np.nan, np.nan]
        msg.jerk = [np.nan, np.nan, np.nan]
        
        # Orientación inercial sostenida
        msg.yaw = 0.0
        msg.yawspeed = np.nan
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """
        Abstracción sistémica para transmisión de códigos MAV_CMD encapsulados.
        """
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm_vehicle(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land_vehicle(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = OffboardControlNode()
    
    try:
        # El comando 'spin' monopoliza asíncronamente el hilo principal del procesador.
        # Gestiona las llamadas retrospectivas (callbacks) de los timers sin paralizar el entorno.
        rclpy.spin(offboard_control_node)
    except KeyboardInterrupt:
        offboard_control_node.get_logger().info("Aborto dictaminado por terminal del usuario.")
    finally:
        offboard_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()