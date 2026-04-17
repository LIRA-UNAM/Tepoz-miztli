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
    DistanceSensor,
)
from geometry_msgs.msg import PoseArray

# ── Parámetros editables ──────────────────────────────────────────────────────

TARGET_ALTITUDE = 1.2        # altura objetivo [m] (sensor de distancia)
TARGET_Z        = -1.2       # en NED (negativo = arriba)
HOLD_DURATION   = 3.0        # duración del HOLD inicial tras despegue [s]

# Movimientos de lazo abierto (pos. setpoint + tolerancia de llegada)
MOVE_RIGHT_DIST   = 1.2      # desplazamiento lateral a la derecha [m]
MOVE_FWD_DIST     = 4.4      # desplazamiento al frente [m]
MOVE_TOL_XY       = 0.20     # tolerancia XY para confirmar waypoint [m]
MOVE_STABLE_TICKS = 20       # ticks consecutivos dentro de tolerancia

# Rotación pre-búsqueda de Aruco (90° CW antes de buscar el marcador)
PRE_ROT_DIR    = -1          # -1 = CW, +1 = CCW
PRE_ROT_TOL    = 0.04        # tolerancia angular [rad]
PRE_ROT_STABLE = 40          # ticks para confirmar rotación

# Aruco / pizarrón 
APPROACH_DIST    = 0.80
APPROACH_KP_FWD  = 0.2
APPROACH_KP_LAT  = 0.5
APPROACH_KP_VERT = 0.3
APPROACH_MAX_V   = 0.3
APPROACH_TOL_XY  = 0.10
APPROACH_TOL_Z   = 0.15
APPROACH_STABLE  = 20
ARUCO_TIMEOUT    = 0.8

# Búsqueda girando / slide
SEARCH_YAWSPEED = 0.25       # rad/s
SLIDE_SPEED     = 0.3        # m/s
SLIDE_DURATION  = 4.0        # s

# Rotación post-pizarrón (90° CW)
ROTATE_DIR    = -1
ROTATE_TOL    = 0.04
ROTATE_STABLE = 40

# Movimiento final hacia plataforma de aterrizaje
MOVE_LAND_DIST     = 4.5     # m al frente tras la 2ª rotación
LAND_SEARCH_SPD    = 0.25    # rad/s buscando la plataforma
LAND_APPROACH_KP   = 0.4     # ganancia lateral del centrado
LAND_APPROACH_MAXV = 0.2     # velocidad máx. centrado [m/s]
LAND_TOL_XY        = 0.12    # tolerancia lateral para LAND [m]
LAND_STABLE        = 30      # ticks para confirmar centrado
LANDING_TIMEOUT    = 0.8     # tiempo sin detección para invalidar [s]


class WhiteBoardMission(Node):
    def __init__(self):
        super().__init__('whiteboard_mission')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Publishers
        self.offboard_pub   = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   pub_qos)
        self.cmd_pub        = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',        pub_qos)

        # ── Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub  = self.create_subscription(VehicleAttitude,      '/fmu/out/vehicle_attitude',       self.attitude_cb,  sub_qos)
        self.flow_sub      = self.create_subscription(DistanceSensor,       '/fmu/out/distance_sensor',        self.flow_cb,      sub_qos)
        self.aruco_sub     = self.create_subscription(PoseArray, 'aruco/poses',   self.aruco_poses_cb,   10)
        self.landing_sub   = self.create_subscription(PoseArray, 'landing/poses', self.landing_poses_cb, 10)

        # ── Estado del vehículo
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        # ── Detecciones con timeout
        self.aruco_tvec        = None
        self.last_aruco_time   = 0.0
        self.landing_tvec      = None
        self.last_landing_time = 0.0

        # ── Posiciones bloqueadas
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        self.move_right_target_x = None
        self.move_right_target_y = None
        self.move_fwd_target_x   = None
        self.move_fwd_target_y   = None
        self.move_land_target_x  = None
        self.move_land_target_y  = None

        self.pre_rot_target_yaw = None
        self.pre_rot_locked_pos = None

        self.front_locked_x       = None
        self.front_locked_y       = None
        self.front_locked_z       = None
        self.front_locked_yaw     = None
        
        self.end_slide_locked_x   = None
        self.end_slide_locked_y   = None
        self.end_slide_locked_yaw = None
        
        self.target_yaw_final     = None
        self.slide_locked_yaw     = None
        self.slide_start_time     = None

        self.land_locked_yaw = None
        self.land_stable_cnt = 0

        # ── Contadores
        self.state               = 'INIT'
        self.counter             = 0
        self.stable_ticks        = 0
        self.move_stable_cnt     = 0
        self.approach_stable_cnt = 0
        self.rotate_stable_cnt   = 0
        self.hold_start_time     = None
        self.hold_front_start    = None

        self.timer = self.create_timer(0.05, self.timer_cb)   # 20 Hz
        self.get_logger().info('Misión Pizarrón + Aterrizaje iniciada (Corregida)')

    # ══════════════════════════════ CALLBACKS ════════════════════════════════

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance

    def aruco_poses_cb(self, msg: PoseArray):
        if not msg.poses: return
        p = msg.poses[0].position
        self.aruco_tvec      = [p.x, p.y, p.z]
        self.last_aruco_time = time.time()

    def landing_poses_cb(self, msg: PoseArray):
        if not msg.poses: return
        p = msg.poses[0].position
        self.landing_tvec      = [p.x, p.y, p.z]
        self.last_landing_time = time.time()

    # ══════════════════════════════ LOOP PRINCIPAL ════════════════════════════

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Expirar detecciones
        if time.time() - self.last_aruco_time   > ARUCO_TIMEOUT:   self.aruco_tvec   = None
        if time.time() - self.last_landing_time > LANDING_TIMEOUT: self.landing_tvec = None

        # ── OffboardControlMode
        offboard              = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = True
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        # ── Setpoint base inicializado en NaN por seguridad
        sp              = TrajectorySetpoint()
        sp.timestamp    = now
        sp.position     = [float('nan')] * 3
        sp.velocity     = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        sp.yaw          = float('nan')
        sp.yawspeed     = float('nan')

        if self.state in ('INIT', 'ARMING'):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x   = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y   = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ══════════════════════════════════════════════════════════════════════
        #                        MÁQUINA DE ESTADOS
        # ══════════════════════════════════════════════════════════════════════

        if self.state == 'INIT':
            sp.position = [safe_x, safe_y, TARGET_Z]
            sp.yaw = safe_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)   # Modo Offboard
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            sp.position = [safe_x, safe_y, TARGET_Z]
            sp.yaw = safe_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)               # ARM
                self.get_logger().info(f'ARMED | Ascendiendo a {TARGET_ALTITUDE:.1f} m')
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            sp.position = [safe_x, safe_y, TARGET_Z]
            sp.yaw = safe_yaw
            err_alt = abs(self.current_distance - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0
            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info(f'Altura estable en {self.current_distance:.2f} m')
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            sp.position = [safe_x, safe_y, TARGET_Z]
            sp.yaw = safe_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if elapsed >= HOLD_DURATION:
                yaw = self.locked_yaw
                self.move_right_target_x = safe_x + (-math.sin(yaw)) * MOVE_RIGHT_DIST
                self.move_right_target_y = safe_y + ( math.cos(yaw)) * MOVE_RIGHT_DIST
                self.move_stable_cnt = 0
                self.state = 'MOVE_RIGHT'

        elif self.state == 'MOVE_RIGHT':
            sp.position = [self.move_right_target_x, self.move_right_target_y, TARGET_Z]
            sp.yaw = self.locked_yaw
            err = math.hypot(self.current_x - self.move_right_target_x,
                             self.current_y - self.move_right_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                yaw = self.locked_yaw
                self.move_fwd_target_x = self.current_x + math.cos(yaw) * MOVE_FWD_DIST
                self.move_fwd_target_y = self.current_y + math.sin(yaw) * MOVE_FWD_DIST
                self.move_stable_cnt = 0
                self.state = 'MOVE_FORWARD'

        elif self.state == 'MOVE_FORWARD':
            sp.position = [self.move_fwd_target_x, self.move_fwd_target_y, TARGET_Z]
            sp.yaw = self.locked_yaw
            err = math.hypot(self.current_x - self.move_fwd_target_x,
                             self.current_y - self.move_fwd_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                raw = self.locked_yaw + PRE_ROT_DIR * (math.pi / 2.0)
                self.pre_rot_target_yaw = math.atan2(math.sin(raw), math.cos(raw))
                self.pre_rot_locked_pos = (self.current_x, self.current_y)
                self.pre_rot_stable_cnt = 0
                self.state = 'ROTATE_PRE'

        elif self.state == 'ROTATE_PRE':
            # Verificación de seguridad por si el estado salta muy rápido
            px = self.pre_rot_locked_pos[0] if self.pre_rot_locked_pos else self.current_x
            py = self.pre_rot_locked_pos[1] if self.pre_rot_locked_pos else self.current_y
            
            sp.position = [px, py, TARGET_Z]
            sp.yaw = self.pre_rot_target_yaw
            yaw_err = math.atan2(
                math.sin(self.pre_rot_target_yaw - self.current_yaw),
                math.cos(self.pre_rot_target_yaw - self.current_yaw))
            self.pre_rot_stable_cnt = self.pre_rot_stable_cnt + 1 if abs(yaw_err) < PRE_ROT_TOL else 0
            
            if self.pre_rot_stable_cnt >= PRE_ROT_STABLE:
                self.locked_yaw = self.current_yaw
                self.state = 'SEARCH_ARUCO'

        elif self.state == 'SEARCH_ARUCO':
            px = self.pre_rot_locked_pos[0] if self.pre_rot_locked_pos else self.current_x
            py = self.pre_rot_locked_pos[1] if self.pre_rot_locked_pos else self.current_y
            sp.position = [px, py, TARGET_Z]
            sp.yawspeed = SEARCH_YAWSPEED
            
            if self.aruco_tvec is not None:
                self.locked_yaw = self.current_yaw
                self.approach_stable_cnt = 0
                self.state = 'APPROACH'

        elif self.state == 'APPROACH':
            sp.yaw = self.locked_yaw
            # CORRECCIÓN: Dejamos sp.position en NaN para que use control puro de velocidad.
            if self.aruco_tvec is None:
                sp.velocity[0] = 0.0
                sp.velocity[1] = 0.0
                sp.velocity[2] = 0.0 
                self.approach_stable_cnt = 0
            else:
                tx = float(self.aruco_tvec[0])
                ty = float(self.aruco_tvec[1])
                tz = float(self.aruco_tvec[2])
                dist_err = tz - APPROACH_DIST
                
                bvx = 0.0 if abs(dist_err) < APPROACH_TOL_Z else self._clamp(APPROACH_KP_FWD * dist_err, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvy = self._clamp(APPROACH_KP_LAT  * tx, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvz = self._clamp(APPROACH_KP_VERT * ty, -0.15, 0.15)
                
                yaw = self.locked_yaw
                sp.velocity[0] = bvx * math.cos(yaw) - bvy * math.sin(yaw)
                sp.velocity[1] = bvx * math.sin(yaw) + bvy * math.cos(yaw)
                sp.velocity[2] = bvz
                
                reached = abs(dist_err) < APPROACH_TOL_Z and abs(tx) < APPROACH_TOL_XY
                self.approach_stable_cnt = self.approach_stable_cnt + 1 if reached else 0
                
                if self.approach_stable_cnt >= APPROACH_STABLE:
                    self.front_locked_x   = self.current_x
                    self.front_locked_y   = self.current_y
                    self.front_locked_z   = self.current_z
                    self.front_locked_yaw = self.current_yaw
                    self.hold_front_start = self.get_clock().now()
                    self.state = 'HOLD_FRONT'

        elif self.state == 'HOLD_FRONT':
            sp.position = [self.front_locked_x, self.front_locked_y, self.front_locked_z]
            sp.yaw = self.front_locked_yaw
            elapsed = self._elapsed_s(self.hold_front_start, self.get_clock())
            if elapsed >= HOLD_DURATION:
                self.slide_locked_yaw = self.front_locked_yaw
                self.slide_start_time = self.get_clock().now()
                self.state = 'SLIDE_RIGHT'

        elif self.state == 'SLIDE_RIGHT':
            yaw    = self.slide_locked_yaw
            sp.velocity = [-math.sin(yaw) * SLIDE_SPEED, math.cos(yaw) * SLIDE_SPEED, 0.0]
            sp.yaw = yaw
            elapsed = self._elapsed_s(self.slide_start_time, self.get_clock())
            
            if elapsed >= SLIDE_DURATION:
                self.end_slide_locked_x   = self.current_x
                self.end_slide_locked_y   = self.current_y
                self.end_slide_locked_yaw = self.current_yaw
                self.hold_start_time      = self.get_clock().now()
                self.state = 'HOLD_SLIDE'

        elif self.state == 'HOLD_SLIDE':
            sp.position = [self.end_slide_locked_x, self.end_slide_locked_y, TARGET_Z]
            sp.yaw = self.end_slide_locked_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if elapsed >= HOLD_DURATION:
                raw = self.end_slide_locked_yaw + ROTATE_DIR * (math.pi / 2.0)
                self.target_yaw_final  = math.atan2(math.sin(raw), math.cos(raw))
                self.rotate_stable_cnt = 0
                self.state = 'ROTATE_90'

        elif self.state == 'ROTATE_90':
            sp.position = [self.end_slide_locked_x, self.end_slide_locked_y, TARGET_Z]
            sp.yaw = self.target_yaw_final
            yaw_err = math.atan2(
                math.sin(self.target_yaw_final - self.current_yaw),
                math.cos(self.target_yaw_final - self.current_yaw))
            self.rotate_stable_cnt = self.rotate_stable_cnt + 1 if abs(yaw_err) < ROTATE_TOL else 0
            
            if self.rotate_stable_cnt >= ROTATE_STABLE:
                yaw = self.target_yaw_final
                self.move_land_target_x = self.current_x + math.cos(yaw) * MOVE_LAND_DIST
                self.move_land_target_y = self.current_y + math.sin(yaw) * MOVE_LAND_DIST
                self.move_stable_cnt    = 0
                self.state = 'MOVE_TO_LAND'

        elif self.state == 'MOVE_TO_LAND':
            sp.position = [self.move_land_target_x, self.move_land_target_y, TARGET_Z]
            sp.yaw = self.target_yaw_final
            err = math.hypot(self.current_x - self.move_land_target_x,
                             self.current_y - self.move_land_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                self.land_locked_yaw = self.current_yaw
                self.land_stable_cnt = 0
                self.state = 'SEARCH_LANDING'

        elif self.state == 'SEARCH_LANDING':
            sp.position = [self.move_land_target_x, self.move_land_target_y, TARGET_Z]
            sp.yawspeed = LAND_SEARCH_SPD
            if self.landing_tvec is not None:
                self.land_locked_yaw = self.current_yaw
                self.land_stable_cnt = 0
                self.state = 'LAND_APPROACH'

        elif self.state == 'LAND_APPROACH':
            sp.yaw = self.land_locked_yaw
            # CORRECCIÓN: Fijo la altura en Z, y permito velocidades XY para centrarse
            sp.position = [float('nan'), float('nan'), TARGET_Z] 
            
            if self.landing_tvec is None:
                sp.velocity[0] = 0.0
                sp.velocity[1] = 0.0
                self.land_stable_cnt = 0
            else:
                offset_x = float(self.landing_tvec[0])
                offset_y = float(self.landing_tvec[1])
                sp.velocity[0] = self._clamp(LAND_APPROACH_KP * offset_x, -LAND_APPROACH_MAXV, LAND_APPROACH_MAXV)
                sp.velocity[1] = self._clamp(LAND_APPROACH_KP * offset_y, -LAND_APPROACH_MAXV, LAND_APPROACH_MAXV)
                
                centrado = abs(offset_x) < LAND_TOL_XY and abs(offset_y) < LAND_TOL_XY
                self.land_stable_cnt = self.land_stable_cnt + 1 if centrado else 0
                
                if self.land_stable_cnt >= LAND_STABLE:
                    self.get_logger().info('Centrado sobre plataforma → Aterrizando (MAV_CMD_NAV_LAND)')
                    self.send_cmd(21)   # MAV_CMD_NAV_LAND
                    self.state = 'LANDING'

        elif self.state == 'LANDING':
            # CORRECCIÓN VITAL: Mantener un setpoint válido para que Offboard no se queje de NaNs
            # mientras el dron transiciona internamente al modo Land.
            sp.position = [self.move_land_target_x, self.move_land_target_y, TARGET_Z]
            sp.yaw = self.land_locked_yaw
            
            if self.counter % 40 == 0:
                self.get_logger().info('LANDING — descenso en progreso')

        # ─────────────────────────────────────────────────────────────────────
        self.trajectory_pub.publish(sp)
        self.counter += 1

    # ══════════════════════════════ COMANDOS ═════════════════════════════════

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg                  = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)

    # ══════════════════════════════ HELPERS ══════════════════════════════════

    @staticmethod
    def _elapsed_s(start_time, clock) -> float:
        if start_time is None: return 0.0
        return (clock.now() - start_time).nanoseconds / 1e9

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main():
    rclpy.init()
    node = WhiteBoardMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción detectada. Cerrando nodo')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()