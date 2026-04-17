"""
Mision 3 lazo abierto: Circuito Gate Blue - Aterrizaje

Despegue, Cruce de Gate Blue, Esquivar columnas, Identiica zona de aterizaje, Landing.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor
)

# ==========================================
# PARÁMETROS DEL CIRCUITO m
# ==========================================
TARGET_ALTITUDE = 1.35
TARGET_Z = -1.35       # NED: Negativo es hacia arriba
LANDING_Z = 0.2
HOLD_DURATION = 3.0   # Segundos de espera al despegar/aterrizar

# COORDENADAS ABSOLUTAS DESDE EL PUNTO DE DESPEGUE (0,0)
# X = Metros hacia el Frente | Y = Metros hacia la Derecha
DIST_1_ALINEACION_Y      = 1.5   # Cuánto se mueve a la derecha para centrar
DIST_2_FONDO_X           = 3.2   # Cuánto avanza al frente pasando las gates
DIST_3_CARRIL_REGRESO_Y  = 4.3   # Cuánto se mueve a la derecha para evitar las columnas de regreso
DIST_4_REGRESO_X         = 0.0   # A qué distancia frontal regresa (0.0 = línea de salida)
DIST_5_META_Y            = 0.0   # A qué distancia lateral regresa (0.0 = punto de salida)

# Tolerancias
POS_TOLERANCE = 0.35  # Metros para confirmar llegada a un punto
YAW_TOLERANCE = 0.1   # Radianes (~5.7°) para confirmar que ya rotó

class Mision3LazoAbierto(Node):
    def __init__(self):
        super().__init__('mision3_lazo_abierto')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.flow_cb, sub_qos)

        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        self.hold_x = 0.0
        self.hold_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        self.state = "INIT"
        self.counter = 0
        self.stable_ticks = 0
        self.hold_start_time = None

        self.timer = self.create_timer(0.05, self.timer_cb) 
        self.get_logger().info("Nodo mision3_lazo_abierto Iniciado")

    # ===================== CALLBACKS =====================

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw          = float('nan')
        setpoint.yawspeed     = float('nan') 

        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        yaw_err = self._get_yaw_error(self.target_yaw, self.current_yaw)

        # ===================== MÁQUINA DE ESTADOS =====================

        if self.state == 'INIT':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) 
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f"ARMED | Despegando a {TARGET_ALTITUDE:.1f} m")
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            err_alt = abs(self.current_distance - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0
            
            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info("Estabilizado sobre el inicio.")
                self.state = 'HOLD_TAKEOFF'

        elif self.state == 'HOLD_TAKEOFF':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            
            if elapsed >= HOLD_DURATION:
                self.get_logger().info('Avanza a la derecha...')
                self.set_target(0.0, DIST_1_ALINEACION_Y)
                self.target_yaw = self.locked_yaw 
                self.state = 'MOVE_PASO_1'

        elif self.state == 'MOVE_PASO_1':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if self.has_reached_target():
                self.get_logger().info('Alineado cruzando gates...')
                self.set_target(DIST_2_FONDO_X, DIST_1_ALINEACION_Y)
                self.state = 'MOVE_PASO_2'

        elif self.state == 'MOVE_PASO_2':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if self.has_reached_target():
                self.get_logger().info('Girando 90° derecha...')
                self.set_turn(math.pi / 2)
                self.state = 'TURN_PASO_3'

        elif self.state == 'TURN_PASO_3':
            setpoint.position = [self.hold_x, self.hold_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if abs(yaw_err) < YAW_TOLERANCE:
                self.get_logger().info('Avanzando...')
                self.set_target(DIST_2_FONDO_X, DIST_3_CARRIL_REGRESO_Y)
                self.state = 'MOVE_PASO_3'

        elif self.state == 'MOVE_PASO_3':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if self.has_reached_target():
                self.get_logger().info('Girando 90° derecha...')
                self.set_turn(math.pi)
                self.state = 'TURN_PASO_4'

        elif self.state == 'TURN_PASO_4':
            setpoint.position = [self.hold_x, self.hold_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if abs(yaw_err) < YAW_TOLERANCE:
                self.get_logger().info('Avanzando de regreso...')
                self.set_target(DIST_4_REGRESO_X, DIST_3_CARRIL_REGRESO_Y)
                self.state = 'MOVE_PASO_4'

        elif self.state == 'MOVE_PASO_4':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if self.has_reached_target():
                self.get_logger().info('Girando 90° derecha hacia la meta...')
                self.set_turn(-math.pi / 2)
                self.state = 'TURN_PASO_5'

        elif self.state == 'TURN_PASO_5':
            setpoint.position = [self.hold_x, self.hold_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if abs(yaw_err) < YAW_TOLERANCE:
                self.get_logger().info('Avanzando a la meta...')
                self.set_target(DIST_4_REGRESO_X, DIST_5_META_Y)
                self.state = 'MOVE_PASO_5'

        elif self.state == 'MOVE_PASO_5':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if self.has_reached_target():
                self.get_logger().info('Girando 90° derecha ...')
                self.set_turn(0.0)
                self.state = 'TURN_PASO_6'

        elif self.state == 'TURN_PASO_6':
            setpoint.position = [self.hold_x, self.hold_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            if abs(yaw_err) < YAW_TOLERANCE:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info('Posicionado.en landing gate...')
                self.state = 'HOLD_BEFORE_LAND'

        elif self.state == 'HOLD_BEFORE_LAND':
            setpoint.position = [self.target_x, self.target_y, TARGET_Z]
            setpoint.yaw = self.target_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            
            if elapsed >= HOLD_DURATION:
                self.get_logger().info('Iniciando descenso...')
                self.send_cmd(21) # VEHICLE_CMD_NAV_LAND
                self.state = 'LANDING'

        elif self.state == 'LANDING':
            setpoint.position = [self.target_x, self.target_y, LANDING_Z]
            setpoint.yaw = self.target_yaw
            
            if self.current_distance < 0.15 and self.counter % 20 == 0:
                self.get_logger().info('¡Misión Completada con éxito!')

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    # ===================== FUNCIONES DE NAVEGACIÓN =====================

    def set_target(self, dx: float, dy: float):
        self.target_x = self.locked_x + dx * math.cos(self.locked_yaw) - dy * math.sin(self.locked_yaw)
        self.target_y = self.locked_y + dx * math.sin(self.locked_yaw) + dy * math.cos(self.locked_yaw)

    def set_turn(self, offset_rad: float):
        self.hold_x = self.current_x
        self.hold_y = self.current_y
        self.target_yaw = self._normalize_yaw(self.locked_yaw + offset_rad)

    def has_reached_target(self) -> bool:
        err_x = self.current_x - self.target_x
        err_y = self.current_y - self.target_y
        return math.sqrt(err_x**2 + err_y**2) < POS_TOLERANCE

    # ===================== COMANDOS Y HELPERS =====================

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)
    
    @staticmethod
    def _elapsed_s(start_time, clock) -> float:
        if start_time is None:
            return 0.0
        return (clock.now() - start_time).nanoseconds / 1e9

    @staticmethod
    def _get_yaw_error(target, current) -> float:
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def _normalize_yaw(yaw: float) -> float:
        return math.atan2(math.sin(yaw), math.cos(yaw))
    
def main():
    rclpy.init()
    node = Mision3LazoAbierto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción detectada. Cerrando nodo")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()