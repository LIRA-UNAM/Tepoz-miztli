"""
Mision 1: Circuito completo Blue

Desetpointegue, Cruce de Gates Blue, Esquivar columnas, Encontrar Aruco
Dibujar en el pizarrón, Identiica zona de aterizaje, Landing.

El detector de la columna funciona de la sigueinte manera:

En caso de detectar una columna, el dobyFrame empieza a avanzar de manera suave a la derecha (izquierda)
cuando la columna desaparece empieza a contar los ticks (DOGDE_CLEAR_TICKS)
pasamos al estado dodge_next_state.

Si la columna nunca aparece en DODGE_TIMEOUT salta directamente a dodge_next_state.
"""

import math
import time
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

from geometry_msgs.msg import PoseArray, Point

#Parametros editables para la misión
TARGET_ALTITUDE = 1.0 # metros de altura'(en modelo normal)
TARGET_Z = -1.0 #(Modelo NED)
HOLD_DURATION = 3.0

#Busqueda de Gates azules
GATE_SEARCH_VY  = 0.10          # desetpointlazamiento lateral buscando gate
CROSS_GATE_VX   = 0.60          # velocidad de cruce
CROSS_GATE_TIME = 4.0           # tiempo cruzando la gate
VEL_REDUC_STEPS = [(2.0, 0.4), (4.0, 0.2), (6.0, 0.1)]  # (t_acum_s, vel_m/s)

#Esquivar columna
DODGE_VX          = 0.20        # avance suave durante la esquiva
DODGE_VY_RIGHT    = 0.25        # velocidad lateral DERECHA (Se puede cambiar a la izquierda)
DODGE_CLEAR_TICKS = 15          # ticks sin columna
DODGE_TIMEOUT     = 6.0         # si la columna nunca aparece -> saltar (Segundos)
COLUMN_TIMEOUT    = 0.4         # No hay columna

#Aruco
APPROACH_DIST = 0.80        #distancia entre el aruco y el drone.
APPROACH_KP_FWD = 0.2       # ganancia de profundidad
APPROACH_KP_LAT  = 0.5      # ganancia lateral
APPROACH_KP_VERT = 0.3      # ganancia vertical
APPROACH_MAX_V   = 0.3      # velocidad máx [m/s]
APPROACH_TOL_XY  = 0.10     # tolerancia lateral
APPROACH_TOL_Z   = 0.15     # tolerancia de profundidad
APPROACH_STABLE  = 20       # ticks para confirmar la llegada al aruco

#Velocidad de busqueda girando y deslizado de busqueda
SEARCH_YAWsetpointEED = 0.25 #rad/s
SLIDE_setpointEED = 0.3 # m/s
SLIDE_DURATION = 4.0 # segundos 

#rotación para finalización de tarea
ROTATE_DIR = -1 #-1 = 90 CW
ROTATE_TOL = 0.04 #en radianas
ROTATE_STABLE = 40 #confirmación de rotación con ticks

ARUCO_TIMEOUT = 0.8

#Modo Landing
LAND_SEARCH_VX = 0.10
LAND_DESCENT_VZ = 0.20
LAND_ALT_THRESH = 0.15

class Mision1_Full(Node):
    def __init__(self):
        super().__init__('Paquete1_FullMision')

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

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            pub_qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            pub_qos)
        
        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            sub_qos)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            sub_qos)

        self.flow_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self.flow_cb,
            sub_qos)
        
        self.gates_blue = self.create_subscription(
            Point,
            'm1/blue/coordinates',
            self.gate_cb, 1
        )
        
        self.column_sub = self.create_subscription(
            Point,
            'm_column/coordinates',
            self.column_cb, 1)

        self.aruco_sub = self.create_subscription(
            PoseArray, 
            'aruco/poses',
            self.aruco_cb, 10)
        
        self.landing_sub = self.create_subscription(
            Point, 
            'm4/landing/coordinates', self.landing_cb, sub_qos)
        
        # VARIABLES DEL ENTORNO

        # Posición actual
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        #Vision
        self.gate             = None
        self.column           = None
        self.last_column_time = 0.0
        self.aruco_tvec       = None
        self.last_aruco_time  = 0.0
        self.landing_target   = None

        # Posición bloqueada al armar (origen)
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        self.approach_locked_yaw = None
        self.slide_locked_yaw    = None
        self.end_slide_x         = None
        self.end_slide_y         = None
        self.target_yaw_final    = None

        #State Machine
        self.state = "INIT"
        self.counter = 0
        self.stable_ticks = 0
        self.approach_stable = 0
        self.rotate_stable_cnt = 0
        self.hold_start_time = None
        self.slide_start_time = None

        self.dodge_next_state = None
        self.dodge_clear_cnt = 0

        self.timer = self.create_timer(0.05, self.timer_cb) # 20Hz
        self.get_logger().info(
            f"Paquete de Misiones circuito completo de Gate Blue Iniciado"
        )

    # ===================== CALLBACKS =====================

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosetpoint = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosetpoint = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosetpoint, cosy_cosetpoint)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance
        if self.counter % 20 == 0:
            self.get_logger().debug(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )

    def gate_cb(self, msg: Point):
        self.gate = msg
 
    def column_cb(self, msg: Point):
        """Recibe detección de columna — sólo se llama cuando la columna es visible."""
        self.column = msg
        self.last_column_time = time.time()
 
    def aruco_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.aruco_tvec = [p.x, p.y, p.z]
        self.last_aruco_time = time.time()
        if self.counter % 10 == 0:
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            self.get_logger().info(
                f'ArUco | dist={dist:.3f} m  tx={p.x:.3f} ty={p.y:.3f} tz={p.z:.3f}'
            )
 
    def landing_cb(self, msg: Point):
        self.landing_target = msg


    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        if time.time() - self.last_aruco_time > ARUCO_TIMEOUT:
            self.aruco_tvec = None

        # Publicar OffboardControlMode constantemente
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True   # Controlamos por POSICIÓN
        offboard.velocity     = True   # En este caso lo que hacemos es un control hidrido, se activan los dos
        self.offboard_pub.publish(offboard)

        # Preparar TrajectorySetpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw          = float('nan')
        setpoint.yawsetpointeed     = float('nan')

        # Bloquear origen mientras está en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ===================== Maquina de Estados =====================

        if self.state == 'INIT':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) #Modo Offboard
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f"ARMED | Ascendiendo a {TARGET_ALTITUDE:.1f} m")
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            err_alt = abs(self.current_distance - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"err={err_alt:.2f} m stable={self.stable_ticks}/10"        
                )
            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info(
                    f"Altura estable en {self.current_distance:.2f} m "
                )
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.current_distance:.2f} m'
                )
            if elapsed >= HOLD_DURATION:
                self.get_logger().info('SEARCH_GATE — buscando gate azul')
                self.state = 'SEARCH_GATE'

        elif self.state == 'SEARCH_GATE':
            setpoint.position = [float('nan'), float('nan'), TARGET_Z]
            setpoint.velocity = [0.0, GATE_SEARCH_VY, 0.0]
            setpoint.yaw = safe_yaw
            if self.gate is not None:
                self.phase_start_time = time.time()
                self.get_logger().info('Gate detectada — CROSS_GATE')
                self.state = 'CROSS_GATE'
 
        elif self.state == 'CROSS_GATE':
            setpoint.velocity = [CROSS_GATE_VX, 0.0, 0.0]
            setpoint.yaw = safe_yaw
            elapsed = time.time() - self.phase_start_time
            if self.counter % 10 == 0:
                self.get_logger().info(f'CROSS_GATE | t={elapsed:.1f}/{CROSS_GATE_TIME:.0f}s')
            if elapsed >= CROSS_GATE_TIME:
                self.phase_start_time = time.time()
                self.get_logger().info('VEL_REDUC')
                self.state = 'VEL_REDUC'
 
        elif self.state == 'VEL_REDUC':
            elapsed = time.time() - self.phase_start_time
            vx = 0.0
            done = True
            for t_limit, vel in VEL_REDUC_STEPS:
                if elapsed < t_limit:
                    vx = vel
                    done = False
                    break
            setpoint.velocity = [vx, 0.0, 0.0]
            setpoint.yaw = safe_yaw
            if done:
                self._enter_dodge('SEARCH_ARUCO')       # ← 1ª esquiva
 
        elif self.state == 'DODGE_COLUMN':
            elapsed = time.time() - self.phase_start_time
            yaw = self.current_yaw
 
            # Timeout: columna nunca apareció → saltar
            if elapsed >= DODGE_TIMEOUT and self.dodge_clear_cnt == 0:
                self.get_logger().warn(
                    f'DODGE_COLUMN timeout — columna no detectada, '
                    f'saltando a {self.dodge_next_state}'
                )
                self.state = self.dodge_next_state
            else:
                if self.column is not None:
                    # Columna visible → esquivar hacia la DERECHA en body-frame
                    self.dodge_clear_cnt = 0
                    b_vx = DODGE_VX          # leve avance
                    b_vy = DODGE_VY_RIGHT    # lateral derecha (body +y)
                    ned_vx = b_vx * math.cos(yaw) - b_vy * math.sin(yaw)
                    ned_vy = b_vx * math.sin(yaw) + b_vy * math.cos(yaw)
                    setpoint.velocity = [ned_vx, ned_vy, 0.0]
                    setpoint.yaw = yaw
                    if self.counter % 10 == 0:
                        self.get_logger().info(
                            f'DODGE_COLUMN | VISIBLE  '
                            f'vel_NED=[{ned_vx:.2f}, {ned_vy:.2f}]'
                        )
                else:
                    # Columna no visible → avanzar mientras se confirma que quedó atrás
                    self.dodge_clear_cnt += 1
                    ned_vx = DODGE_VX * math.cos(yaw)
                    ned_vy = DODGE_VX * math.sin(yaw)
                    setpoint.velocity = [ned_vx, ned_vy, 0.0]
                    setpoint.yaw = yaw
                    if self.counter % 10 == 0:
                        self.get_logger().info(
                            f'DODGE_COLUMN | CLARA  '
                            f'clear={self.dodge_clear_cnt}/{DODGE_CLEAR_TICKS}'
                        )
                    if self.dodge_clear_cnt >= DODGE_CLEAR_TICKS:
                        self.get_logger().info(
                            f'Columna esquivada — pasando a {self.dodge_next_state}'
                        )
                        self.state = self.dodge_next_state
 
        elif self.state == 'SEARCH_ARUCO':
            setpoint.position = [self.current_x, self.current_y, TARGET_Z]
            setpoint.yawsetpointeed = SEARCH_YAWsetpointEED
            if self.aruco_tvec is not None:
                self.approach_locked_yaw = self.current_yaw
                self.approach_stable     = 0
                self.get_logger().info(
                    f'ArUco encontrado a {self.aruco_tvec[2]:.2f} m — APPROACH'
                )
                self.state = 'APPROACH'
 
        elif self.state == 'APPROACH':
            setpoint.yaw         = self.approach_locked_yaw
            setpoint.position[2] = TARGET_Z
 
            if self.aruco_tvec is None:
                setpoint.velocity[0] = 0.0
                setpoint.velocity[1] = 0.0
                self.approach_stable = 0
                if self.counter % 10 == 0:
                    self.get_logger().warn('ArUco perdido — esetpointerando')
            else:
                tx    = float(self.aruco_tvec[0])
                ty    = float(self.aruco_tvec[1])
                tz    = float(self.aruco_tvec[2])
                err_z = tz - APPROACH_DIST
 
                bvx = 0.0 if abs(err_z) < APPROACH_TOL_Z else self._clamp(
                    APPROACH_KP_FWD * err_z, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvy = self._clamp(APPROACH_KP_LAT  * tx, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvz = self._clamp(APPROACH_KP_VERT * ty, -0.15, 0.15)
 
                yaw = self.approach_locked_yaw
                setpoint.velocity[0] = bvx * math.cos(yaw) - bvy * math.sin(yaw)
                setpoint.velocity[1] = bvx * math.sin(yaw) + bvy * math.cos(yaw)
                setpoint.velocity[2] = bvz
 
                reached = abs(err_z) < APPROACH_TOL_Z and abs(tx) < APPROACH_TOL_XY
                self.approach_stable = self.approach_stable + 1 if reached else 0
 
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'APPROACH | tz={tz:.3f} (meta {APPROACH_DIST})  '
                        f'tx={tx:.3f}  stable={self.approach_stable}/{APPROACH_STABLE}'
                    )
                if self.approach_stable >= APPROACH_STABLE:
                    self.slide_locked_yaw = self.current_yaw
                    self.phase_start_time = time.time()
                    self.get_logger().info(
                        f'Frente al ArUco — SLIDE_RIGHT ({SLIDE_DURATION:.0f}s a {SLIDE_setpointEED:.2f} m/s)'
                    )
                    self.state = 'SLIDE_RIGHT'
 
        elif self.state == 'SLIDE_RIGHT':
            yaw = self.slide_locked_yaw
            ned_vx = -math.sin(yaw) * SLIDE_setpointEED
            ned_vy =  math.cos(yaw) * SLIDE_setpointEED
            setpoint.velocity = [ned_vx, ned_vy, 0.0]
            setpoint.yaw = yaw
            elapsed = time.time() - self.phase_start_time
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'SLIDE_RIGHT | t={elapsed:.1f}/{SLIDE_DURATION:.0f}s  '
                    f'vel_NED=[{ned_vx:.3f}, {ned_vy:.3f}]'
                )
            if elapsed >= SLIDE_DURATION:
                self.end_slide_x = self.current_x
                self.end_slide_y = self.current_y
                raw = self.current_yaw + ROTATE_DIR * (math.pi / 2.0)
                self.target_yaw_final = math.atan2(math.sin(raw), math.cos(raw))
                self.rotate_stable = 0
                self.get_logger().info(
                    f'Slide completado — ROTATE_90 '
                    f'({math.degrees(self.current_yaw):.1f}° → {math.degrees(self.target_yaw_final):.1f}°)'
                )
                self.state = 'ROTATE_90'
 
        elif self.state == 'ROTATE_90':
            setpoint.position = [self.end_slide_x, self.end_slide_y, TARGET_Z]
            setpoint.yaw = self.target_yaw_final
            yaw_err = math.atan2(
                math.sin(self.target_yaw_final - self.current_yaw),
                math.cos(self.target_yaw_final - self.current_yaw),
            )
            self.rotate_stable = self.rotate_stable + 1 if abs(yaw_err) < ROTATE_TOL else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'ROTATE_90 | actual={math.degrees(self.current_yaw):.1f}°  '
                    f'meta={math.degrees(self.target_yaw_final):.1f}°  '
                    f'err={math.degrees(yaw_err):.1f}°  stable={self.rotate_stable}/{ROTATE_STABLE}'
                )
            if self.rotate_stable >= ROTATE_STABLE:
                self.get_logger().info('Rotación 90° completada')
                self._enter_dodge('SEARCH_LANDING')     # ← 2ª esquiva
 
        elif self.state == 'SEARCH_LANDING':
            setpoint.velocity = [LAND_SEARCH_VX, 0.0, 0.0]
            setpoint.yaw = self.target_yaw_final
            if self.landing_target is not None:
                self.get_logger().info('Landing H detectada — LAND')
                self.state = 'LAND'
 
        elif self.state == 'LAND':
            vy = 0.0
            if self.landing_target is not None:
                vy = self._clamp(-0.002 * self.landing_target.x, -0.3, 0.3)
            setpoint.velocity = [0.0, vy, LAND_DESCENT_VZ]
            setpoint.yaw = self.target_yaw_final
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'LAND | dist={self.current_distance:.2f} m  vy={vy:.3f}'
                )
            if self.current_distance < LAND_ALT_THRESH:
                self.send_cmd(400, param1=0.0)
                self.get_logger().info('=== ATERRIZAJE COMPLETADO — MISIÓN FINALIZADA ===')
                self.state = 'LANDED'
 
        elif self.state == 'LANDED':
            if self.counter % 40 == 0:
                self.get_logger().info('MISIÓN FINALIZADA')
 
        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    def _enter_dodge(self, next_state: str):
        """Configura y activa el estado DODGE_COLUMN."""
        self.dodge_next_state = next_state
        self.dodge_clear_cnt  = 0
        self.phase_start_time = time.time()
        self.get_logger().info(
            f'DODGE_COLUMN — esperando/esquivando columna  |  siguiente: {next_state}'
        )
        self.state = 'DODGE_COLUMN'

    # ===================== COMANDOS =====================

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
    
    # ===================== HELPERS =====================

    @staticmethod
    def _elapsed_s(start_time, clock) -> float:
        if start_time is None:
            return 0.0
        return (clock.now() - start_time).nanoseconds / 1e9
    
    @staticmethod
    def _clamp(value: float, lo:float, hi:float) -> float:
        return max(lo, min(hi, value))
    
def main():
        rclpy.init()
        node = Mision1_Full()
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