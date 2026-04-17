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

# Aruco / pizarrón (sin cambios respecto al código original)
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

# Rotación post-pizarrón (90° CW, idéntica a la pre-búsqueda)
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
        # NOTA: 'landing/poses' — publica PoseArray con la misma convención que
        # aruco/poses.  Para cámara cenital: position.x = offset NED-X,
        # position.y = offset NED-Y.  Ajustar en landing_poses_cb si difiere.

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

        # ── Posición/yaw bloqueados al armar
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # ── Objetivos de lazo abierto (se calculan al entrar al estado)
        self.move_right_target_x = None
        self.move_right_target_y = None
        self.move_fwd_target_x   = None
        self.move_fwd_target_y   = None
        self.move_land_target_x  = None
        self.move_land_target_y  = None

        # ── Rotación pre-búsqueda
        self.pre_rot_target_yaw = None
        self.pre_rot_locked_pos = None   # (x, y) congelados durante la rotación
        self.pre_rot_stable_cnt = 0

        # ── Pizarrón (variables originales)
        self.front_locked_x       = None
        self.front_locked_y       = None
        self.front_locked_z       = None
        self.front_locked_yaw     = None
        self.end_slide_locked_x   = None
        self.end_slide_locked_y   = None
        self.end_slide_locked_yaw = None
        self.target_yaw_final     = None   # yaw tras ROTATE_90
        self.slide_locked_yaw     = None
        self.slide_start_time     = None

        # ── Aterrizaje
        self.land_locked_yaw = None
        self.land_stable_cnt = 0

        # ── Contadores y tiempos generales
        self.state               = 'INIT'
        self.counter             = 0
        self.stable_ticks        = 0
        self.move_stable_cnt     = 0
        self.approach_stable_cnt = 0
        self.rotate_stable_cnt   = 0
        self.hold_start_time     = None
        self.hold_front_start    = None

        self.timer = self.create_timer(0.05, self.timer_cb)   # 20 Hz
        self.get_logger().info('Misión Pizarrón + Aterrizaje iniciada')

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
        if self.counter % 20 == 0:
            self.get_logger().debug(
                f'Calidad: {msg.signal_quality} | Distancia: {self.current_distance:.4f} m')

    def aruco_poses_cb(self, msg: PoseArray):
        """
        Convención RealSense (cámara frontal):
          position.x = tx → lateral   (+ = derecha)
          position.y = ty → vertical  (+ = abajo)
          position.z = tz → profundidad
        """
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.aruco_tvec      = [p.x, p.y, p.z]
        self.last_aruco_time = time.time()
        if self.counter % 10 == 0:
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            self.get_logger().info(
                f'Aruco | dist={dist:.3f}m  tx={p.x:.3f} ty={p.y:.3f} tz={p.z:.3f}')

    def landing_poses_cb(self, msg: PoseArray):
        """
        Landing pad detector.
        Para cámara cenital se asume:
          position.x = offset lateral  (frame cámara → convertir a NED según montaje)
          position.y = offset longitudinal
        Ajustar según el detector específico.
        """
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.landing_tvec      = [p.x, p.y, p.z]
        self.last_landing_time = time.time()
        if self.counter % 10 == 0:
            self.get_logger().info(
                f'LandingPad | tx={p.x:.3f} ty={p.y:.3f} tz={p.z:.3f}')

    # ══════════════════════════════ LOOP PRINCIPAL ════════════════════════════

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Expirar detecciones por timeout
        if time.time() - self.last_aruco_time   > ARUCO_TIMEOUT:   self.aruco_tvec   = None
        if time.time() - self.last_landing_time > LANDING_TIMEOUT: self.landing_tvec = None

        # ── OffboardControlMode (siempre activo)
        offboard              = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = True
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        # ── Setpoint base (todo NaN)
        sp              = TrajectorySetpoint()
        sp.timestamp    = now
        sp.position     = [float('nan')] * 3
        sp.velocity     = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        sp.yaw          = float('nan')
        sp.yawspeed     = float('nan')

        # Congelar origen mientras está en tierra
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

        # ── TIERRA / DESPEGUE ─────────────────────────────────────────────────

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
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'TAKEOFF | dist={self.current_distance:.2f}m '
                    f'err={err_alt:.2f}m stable={self.stable_ticks}/10')
            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info(f'Altura estable en {self.current_distance:.2f} m')
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            sp.position = [safe_x, safe_y, TARGET_Z]
            sp.yaw = safe_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.current_distance:.2f}m')
            if elapsed >= HOLD_DURATION:
                yaw = self.locked_yaw
                # Derecha en NED: (-sin(yaw), cos(yaw))
                self.move_right_target_x = safe_x + (-math.sin(yaw)) * MOVE_RIGHT_DIST
                self.move_right_target_y = safe_y + ( math.cos(yaw)) * MOVE_RIGHT_DIST
                self.move_stable_cnt = 0
                self.get_logger().info(
                    f'MOVE_RIGHT — {MOVE_RIGHT_DIST}m a la derecha '
                    f'→ NED=({self.move_right_target_x:.2f}, {self.move_right_target_y:.2f})')
                self.state = 'MOVE_RIGHT'

        # ── MOVIMIENTOS DE LAZO ABIERTO ───────────────────────────────────────

        elif self.state == 'MOVE_RIGHT':
            sp.position = [self.move_right_target_x, self.move_right_target_y, TARGET_Z]
            sp.yaw = self.locked_yaw
            err = math.hypot(self.current_x - self.move_right_target_x,
                             self.current_y - self.move_right_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'MOVE_RIGHT | err={err:.3f}m stable={self.move_stable_cnt}/{MOVE_STABLE_TICKS}')
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                yaw = self.locked_yaw
                # Frente en NED: (cos(yaw), sin(yaw))
                self.move_fwd_target_x = self.current_x + math.cos(yaw) * MOVE_FWD_DIST
                self.move_fwd_target_y = self.current_y + math.sin(yaw) * MOVE_FWD_DIST
                self.move_stable_cnt = 0
                self.get_logger().info(
                    f'MOVE_FORWARD — {MOVE_FWD_DIST}m al frente '
                    f'→ NED=({self.move_fwd_target_x:.2f}, {self.move_fwd_target_y:.2f})')
                self.state = 'MOVE_FORWARD'

        elif self.state == 'MOVE_FORWARD':
            sp.position = [self.move_fwd_target_x, self.move_fwd_target_y, TARGET_Z]
            sp.yaw = self.locked_yaw
            err = math.hypot(self.current_x - self.move_fwd_target_x,
                             self.current_y - self.move_fwd_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'MOVE_FORWARD | err={err:.3f}m stable={self.move_stable_cnt}/{MOVE_STABLE_TICKS}')
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                raw = self.locked_yaw + PRE_ROT_DIR * (math.pi / 2.0)
                self.pre_rot_target_yaw = math.atan2(math.sin(raw), math.cos(raw))
                self.pre_rot_locked_pos = (self.current_x, self.current_y)
                self.pre_rot_stable_cnt = 0
                self.move_stable_cnt    = 0
                self.get_logger().info(
                    f'ROTATE_PRE — girando 90° {"CW" if PRE_ROT_DIR < 0 else "CCW"} '
                    f'hasta {math.degrees(self.pre_rot_target_yaw):.1f}°')
                self.state = 'ROTATE_PRE'

        elif self.state == 'ROTATE_PRE':
            px, py = self.pre_rot_locked_pos
            sp.position = [px, py, TARGET_Z]
            sp.yaw = self.pre_rot_target_yaw
            yaw_err = math.atan2(
                math.sin(self.pre_rot_target_yaw - self.current_yaw),
                math.cos(self.pre_rot_target_yaw - self.current_yaw))
            self.pre_rot_stable_cnt = self.pre_rot_stable_cnt + 1 if abs(yaw_err) < PRE_ROT_TOL else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'ROTATE_PRE | actual={math.degrees(self.current_yaw):.1f}° '
                    f'target={math.degrees(self.pre_rot_target_yaw):.1f}° '
                    f'err={math.degrees(yaw_err):.1f}° stable={self.pre_rot_stable_cnt}/{PRE_ROT_STABLE}')
            if self.pre_rot_stable_cnt >= PRE_ROT_STABLE:
                self.locked_yaw = self.current_yaw   # actualizar yaw de referencia
                self.approach_stable_cnt = 0
                self.get_logger().info('SEARCH_ARUCO — girando en busca del marcador')
                self.state = 'SEARCH_ARUCO'

        # ── PIZARRÓN (lógica original sin modificaciones) ─────────────────────

        elif self.state == 'SEARCH_ARUCO':
            px, py = self.pre_rot_locked_pos
            sp.position = [px, py, TARGET_Z]
            sp.yawspeed = SEARCH_YAWSPEED
            if self.aruco_tvec is not None:
                self.locked_yaw = self.current_yaw
                self.approach_stable_cnt = 0
                self.get_logger().info(
                    f'Aruco detectado a {self.aruco_tvec[2]:.3f}m → APPROACH')
                self.state = 'APPROACH'

        elif self.state == 'APPROACH':
            sp.yaw = self.locked_yaw
            sp.position[2] = TARGET_Z
            if self.aruco_tvec is None:
                sp.velocity[0] = 0.0
                sp.velocity[1] = 0.0
                self.approach_stable_cnt = 0
                if self.counter % 10 == 0:
                    self.get_logger().warn('Aruco perdido durante APPROACH — esperando')
            else:
                tx = float(self.aruco_tvec[0])
                ty = float(self.aruco_tvec[1])
                tz = float(self.aruco_tvec[2])
                dist_err = tz - APPROACH_DIST
                bvx = (0.0 if abs(dist_err) < APPROACH_TOL_Z
                       else self._clamp(APPROACH_KP_FWD * dist_err, -APPROACH_MAX_V, APPROACH_MAX_V))
                bvy = self._clamp(APPROACH_KP_LAT  * tx, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvz = self._clamp(APPROACH_KP_VERT * ty, -0.15, 0.15)
                yaw = self.locked_yaw
                sp.velocity[0] = bvx * math.cos(yaw) - bvy * math.sin(yaw)
                sp.velocity[1] = bvx * math.sin(yaw) + bvy * math.cos(yaw)
                sp.velocity[2] = bvz
                reached = abs(dist_err) < APPROACH_TOL_Z and abs(tx) < APPROACH_TOL_XY
                self.approach_stable_cnt = self.approach_stable_cnt + 1 if reached else 0
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'APPROACH | tz={tz:.3f}m (quiero {APPROACH_DIST}m) '
                        f'tx={tx:.3f} ty={ty:.3f} | stable={self.approach_stable_cnt}/{APPROACH_STABLE}')
                if self.approach_stable_cnt >= APPROACH_STABLE:
                    self.front_locked_x   = self.current_x
                    self.front_locked_y   = self.current_y
                    self.front_locked_z   = self.current_z
                    self.front_locked_yaw = self.current_yaw
                    self.hold_front_start = self.get_clock().now()
                    self.get_logger().info(
                        f'POSICIÓN FRENTE AL ARUCO | tz={tz:.3f}m → HOLD_FRONT')
                    self.state = 'HOLD_FRONT'

        elif self.state == 'HOLD_FRONT':
            sp.position = [self.front_locked_x, self.front_locked_y, self.front_locked_z]
            sp.yaw = self.front_locked_yaw
            elapsed = self._elapsed_s(self.hold_front_start, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD_FRONT {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.current_distance:.2f}m')
            if elapsed >= HOLD_DURATION:
                self.slide_locked_yaw = self.front_locked_yaw
                self.slide_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'SLIDE_RIGHT — {SLIDE_DURATION:.0f}s a {SLIDE_SPEED:.2f}m/s '
                    f'(yaw={math.degrees(self.slide_locked_yaw):.1f}°)')
                self.state = 'SLIDE_RIGHT'

        elif self.state == 'SLIDE_RIGHT':
            yaw    = self.slide_locked_yaw
            ned_vx = -math.sin(yaw) * SLIDE_SPEED
            ned_vy =  math.cos(yaw) * SLIDE_SPEED
            sp.velocity = [ned_vx, ned_vy, 0.0]
            sp.yaw = yaw
            elapsed = self._elapsed_s(self.slide_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'SLIDE_RIGHT | t={elapsed:.1f}/{SLIDE_DURATION:.0f}s | '
                    f'vel_NED=[{ned_vx:.3f}, {ned_vy:.3f}, 0.000]')
            if elapsed >= SLIDE_DURATION:
                self.end_slide_locked_x   = self.current_x
                self.end_slide_locked_y   = self.current_y
                self.end_slide_locked_yaw = self.current_yaw
                self.hold_start_time      = self.get_clock().now()
                self.get_logger().info(
                    f'Slide completado | x={self.end_slide_locked_x:.3f} y={self.end_slide_locked_y:.3f}')
                self.state = 'HOLD_SLIDE'

        elif self.state == 'HOLD_SLIDE':
            sp.position = [self.end_slide_locked_x, self.end_slide_locked_y, TARGET_Z]
            sp.yaw = self.end_slide_locked_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(f'HOLD_SLIDE {elapsed:.1f}/{HOLD_DURATION:.0f}s')
            if elapsed >= HOLD_DURATION:
                raw = self.end_slide_locked_yaw + ROTATE_DIR * (math.pi / 2.0)
                self.target_yaw_final  = math.atan2(math.sin(raw), math.cos(raw))
                self.rotate_stable_cnt = 0
                self.get_logger().info(
                    f'ROTATE_90 — de {math.degrees(self.end_slide_locked_yaw):.1f}° '
                    f'a {math.degrees(self.target_yaw_final):.1f}° '
                    f'({"CW" if ROTATE_DIR < 0 else "CCW"})')
                self.state = 'ROTATE_90'

        elif self.state == 'ROTATE_90':
            sp.position = [self.end_slide_locked_x, self.end_slide_locked_y, TARGET_Z]
            sp.yaw = self.target_yaw_final
            yaw_err = math.atan2(
                math.sin(self.target_yaw_final - self.current_yaw),
                math.cos(self.target_yaw_final - self.current_yaw))
            self.rotate_stable_cnt = self.rotate_stable_cnt + 1 if abs(yaw_err) < ROTATE_TOL else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'ROTATE_90 | actual={math.degrees(self.current_yaw):.1f}° '
                    f'target={math.degrees(self.target_yaw_final):.1f}° '
                    f'err={math.degrees(yaw_err):.1f}° stable={self.rotate_stable_cnt}/{ROTATE_STABLE}')
            if self.rotate_stable_cnt >= ROTATE_STABLE:
                # Calcular waypoint final hacia la plataforma
                yaw = self.target_yaw_final
                self.move_land_target_x = self.current_x + math.cos(yaw) * MOVE_LAND_DIST
                self.move_land_target_y = self.current_y + math.sin(yaw) * MOVE_LAND_DIST
                self.move_stable_cnt    = 0
                self.get_logger().info(
                    f'Rotación completada → MOVE_TO_LAND — {MOVE_LAND_DIST}m al frente '
                    f'NED=({self.move_land_target_x:.2f}, {self.move_land_target_y:.2f})')
                self.state = 'MOVE_TO_LAND'

        # ── ATERRIZAJE ────────────────────────────────────────────────────────

        elif self.state == 'MOVE_TO_LAND':
            sp.position = [self.move_land_target_x, self.move_land_target_y, TARGET_Z]
            sp.yaw = self.target_yaw_final
            err = math.hypot(self.current_x - self.move_land_target_x,
                             self.current_y - self.move_land_target_y)
            self.move_stable_cnt = self.move_stable_cnt + 1 if err < MOVE_TOL_XY else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'MOVE_TO_LAND | err={err:.3f}m stable={self.move_stable_cnt}/{MOVE_STABLE_TICKS}')
            if self.move_stable_cnt >= MOVE_STABLE_TICKS:
                self.land_locked_yaw = self.current_yaw
                self.land_stable_cnt = 0
                self.move_stable_cnt = 0
                self.get_logger().info('SEARCH_LANDING — buscando plataforma de aterrizaje')
                self.state = 'SEARCH_LANDING'

        elif self.state == 'SEARCH_LANDING':
            # Mantener posición girando lentamente hasta ver la plataforma
            sp.position = [self.move_land_target_x, self.move_land_target_y, TARGET_Z]
            sp.yawspeed = LAND_SEARCH_SPD
            if self.landing_tvec is not None:
                self.land_locked_yaw = self.current_yaw
                self.land_stable_cnt = 0
                self.get_logger().info(
                    f'Plataforma detectada | tx={self.landing_tvec[0]:.3f} '
                    f'ty={self.landing_tvec[1]:.3f} → LAND_APPROACH')
                self.state = 'LAND_APPROACH'

        elif self.state == 'LAND_APPROACH':
            """
            Centra el dron sobre la plataforma usando la detección.
            Se asume cámara cenital: landing_tvec[0] y [1] son offsets
            directos en NED (x = norte/sur, y = este/oeste).
            Ajustar signos si la cámara tiene otra orientación.
            """
            sp.yaw = self.land_locked_yaw
            sp.position[2] = TARGET_Z
            if self.landing_tvec is None:
                sp.velocity[0] = 0.0
                sp.velocity[1] = 0.0
                self.land_stable_cnt = 0
                if self.counter % 10 == 0:
                    self.get_logger().warn('Landing pad perdido — esperando')
            else:
                offset_x = float(self.landing_tvec[0])
                offset_y = float(self.landing_tvec[1])
                sp.velocity[0] = self._clamp(LAND_APPROACH_KP * offset_x, -LAND_APPROACH_MAXV, LAND_APPROACH_MAXV)
                sp.velocity[1] = self._clamp(LAND_APPROACH_KP * offset_y, -LAND_APPROACH_MAXV, LAND_APPROACH_MAXV)
                sp.velocity[2] = 0.0
                centrado = abs(offset_x) < LAND_TOL_XY and abs(offset_y) < LAND_TOL_XY
                self.land_stable_cnt = self.land_stable_cnt + 1 if centrado else 0
                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'LAND_APPROACH | ox={offset_x:.3f} oy={offset_y:.3f} '
                        f'stable={self.land_stable_cnt}/{LAND_STABLE}')
                if self.land_stable_cnt >= LAND_STABLE:
                    self.get_logger().info('Centrado sobre plataforma → LANDING')
                    self.send_cmd(21)   # MAV_CMD_NAV_LAND
                    self.state = 'LANDING'

        elif self.state == 'LANDING':
            # PX4 gestiona el descenso; sólo logueamos
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
        if start_time is None:
            return 0.0
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