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

from geometry_msgs.msg import PoseArray

#Parametros editables para la misión
TARGET_ALTITUDE = 1.0
TARGET_Z = -1.0
HOLD_DURATION = 3.0

#Aruco
APPROACH_DIST = 0.80        #distancia entre el aruco y el drone.
APPROACH_KP_FWD = 0.4       # ganancia de profundidad
APPROACH_KP_LAT  = 0.5      # ganancia lateral
APPROACH_KP_VERT = 0.3      # ganancia vertical
APPROACH_MAX_V   = 0.3      # velocidad máx [m/s]
APPROACH_TOL_XY  = 0.05     # tolerancia lateral
APPROACH_TOL_Z   = 0.04     # tolerancia de profundidad
APPROACH_STABLE  = 15       # ticks para confirmar la llegada al aruco

#Velocidad de busqueda girando y deslizado de busqueda
SEARCH_YAWSPEED = 0.25 #rad/s
SLIDE_SPEED = 0.3 # m/s
SLIDE_DURATION = 4.0 # segundos 

#rotación para finalización de tarea
ROTATE_DIR = -1 #-1 = 90 CW
ROTATE_TOL = 0.04 #en radianas
ROTATE_STABLE = 40 #confirmación de rotación con ticks

ARUCO_TIMEOUT = 0.8

class WhiteBoardMission(Node):
    def __init__(self):
        super().__init__('whiteboard_mission')

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
        
        self.aruco_sub = self.create_subscription(
            PoseArray, 
            'aruco/poses',
            self.aruco_poses_cb, 10)
        
        self.aruco_tvec = None
        self.last_aruco_time = 0.0

        # Posición actual
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        # Posición bloqueada al armar (origen)
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        #State Machine
        self.state = "INIT"
        self.counter = 0

        self.stable_ticks = 0
        self.approach_stable_cnt = 0
        self.rotate_stable_cnt = 0
        self.hold_start_time = None
        self.hold_front_start = None
        self.slide_start_time = None
        self.slide_locked_yaw = None
        self.slide_yaw_final = None

        self.timer = self.create_timer(0.05, self.timer_cb) # 20Hz
        self.get_logger().info(
            f"Mision Pizarrón iniciada"
        )

    
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
        if self.counter % 20 == 0:
            self.get_logger().debug(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )

    def aruco_poses_cb(self, msg:PoseArray):
        """
        Convención RealSense (cámara apuntando al frente):
        position.x = tx  → lateral  (+ = derecha)
        position.y = ty  → vertical (+ = abajo)
        position.z = tz  → profundidad (distancia frontal)
        """
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.aruco_tvec = [p.x, p.y, p.z]
        self.last_aruco_time = time.time()

        if self.counter % 10 == 0:
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            self.get_logger().info(
                f'Aruco detectado | dist={dist:.3f} m '
                f'tx={p.x:.3f} ty={p.y:.3f} tz={p.z:.3f}'
            )

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        if time.time() - self.last_aruco_time > ARUCO_TIMEOUT:
            self.aruco_tvec = None

        # Publicar OffboardControlMode constantemente
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True   # Controlamos por POSICIÓN
        offboard.velocity     = True   # En este caso lo que hacemos es un control hidrido, se activan los dos
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        # Preparar TrajectorySetpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw          = float('nan')
        setpoint.yawspeed     = float('nan')

        # Bloquear origen mientras está en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        setpoint_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

    # ===================== Maquina de Estados =====================

        if self.state == 'INIT':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) #Modo Offboard
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f"ARMED | Ascendiendo a {TARGET_ALTITUDE:.1f} m")
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
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
            setpoint.yaw = setpoint_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.current_distance:.2f} m'
                )
            if elapsed >= HOLD_DURATION:
                self.get_logger().info('SEARCH_ARUCO — girando en busca del marcador')
                self.state = 'SEARCH_ARUCO'

        elif self.state == 'SEARCH_ARUCO':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yawspeed = SEARCH_YAWSPEED
            if self.aruco_tvec is not None:
                self.locked_yaw = self.current_yaw
                self.approach_stable_cnt = 0
                self.get_logger().info(
                    f'Aruco detectado a {self.aruco_tvec[2]:.3f}m'
                )
                self.state = 'APPROACH'
        
        elif self.state == 'APPROACH':
            setpoint.yaw = self.locked_yaw
            if self.aruco_tvec is None:
                setpoint.position = [self.current_x, self.current_y, TARGET_Z]
                self.approach_stable_cnt = 0
                if self.counter % 10 == 0:
                    self.get_logger().warn('Aruco perdido durante APPROACH - esperando')
            else:
                tx = float(self.aruco_tvec[0])
                ty = float(self.aruco_tvec[1])
                tz = float(self.aruco_tvec[2])

                dist_err = tz - APPROACH_DIST

                bvx = self._clamp(APPROACH_KP_FWD  * dist_err, -APPROACH_MAX_V, APPROACH_MAX_V)
                bvy = self._clamp(APPROACH_KP_LAT  * tx,       -APPROACH_MAX_V, APPROACH_MAX_V)
                bvz = self._clamp(APPROACH_KP_VERT * ty,       -0.2,            0.2)

                yaw = self.locked_yaw
                setpoint.velocity = [
                    bvx*math.cos(yaw) - bvy*math.sin(yaw),
                    bvx*math.sin(yaw) + bvy*math.cos(yaw),
                    bvz,
                ]

                reached = (
                    abs(dist_err) < APPROACH_TOL_Z and
                    abs(tx) < APPROACH_TOL_XY and
                    abs(ty) < APPROACH_TOL_XY
                )
                self.approach_stable_cnt = self.approach_stable_cnt + 1 if reached else 0

                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'APPROACH | tz={tz:.3f} m (quiero {APPROACH_DIST} m) '
                        f'tx={tx:.3f} ty={ty:.3f} | '
                        f'stable={self.approach_stable_cnt}/{APPROACH_STABLE}'
                    )
                if self.approach_stable_cnt >= APPROACH_STABLE:
                    self.locked_x = self.current_x
                    self.locked_y = self.current_y
                    self.hold_front_start = self.get_clock().now()
                    self.get_logger().info(
                        f'POSICIÓN FRENTE AL ARUCO — tz={tz:.3f} m | '
                        f'x={self.locked_x:.3f} y={self.locked_y:.3f} → HOLD_FRONT'
                    )
                    self.state = 'HOLD_FRONT'
        
        elif self.state == 'HOLD_FRONT':
            setpoint.position = [self.locked_x, self.locked_y, TARGET_Z]
            setpoint.yaw = self.locked_yaw
            elapsed = self._elapsed_s(self.hold_front_start, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD_FRONT {elapsed:.1f}/{HOLD_DURATION:.0f}s |'
                    f'dist={self.current_distance:.2f}m'
                )
            if elapsed >= HOLD_DURATION:
                self.slide_locked_yaw = self.current_yaw
                self.slide_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'SLIDE_RIGHT — {SLIDE_DURATION:.0f}s a {SLIDE_SPEED:.2f} m/s '
                    f'(yaw={math.degrees(self.slide_locked_yaw):.1f}°)'
                )
                self.state = 'SLIDE_RIGHT'

        elif self.state == 'SLIDE_RIGHT':
            yaw = self.slide_locked_yaw
            ned_vx = -math.sin(yaw) * SLIDE_SPEED
            ned_vy = math.cos(yaw) * SLIDE_SPEED
            setpoint.velocity = [ned_vx, ned_vy, 0.0]
            setpoint.yaw = yaw

            elapsed = self._elapsed_s(self.slide_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'SLIDE_RIGHT | t={elapsed:.1f}/{SLIDE_DURATION:.0f}s | '
                    f'vel_NED=[{ned_vx:.3f}, {ned_vy:.3f}, 0.000]'
                )

            if elapsed >= SLIDE_DURATION:
                self.locked_x = self.current_x
                self.locked_y = self.current_y
                self.locked_yaw = self.current_yaw
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'Slide completado | x={self.locked_x:.3f} y={self.locked_y:.3f}'
                )
                self.state = 'HOLD_SLIDE'

        elif self.state == 'HOLD_SLIDE':
            setpoint.position = [self.locked_x, self.locked_y, TARGET_Z]
            setpoint.yaw = self.locked_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(f'HOLD_SLIDE {elapsed:.1f}/{HOLD_DURATION:.0f}s')
            if elapsed >= HOLD_DURATION:
                raw = self.locked_yaw + ROTATE_DIR * (math.pi / 2.0)
                self.target_yaw_final = math.atan2(math.sin(raw), math.cos(raw))
                self.rotate_stable_cnt = 0
                self.get_logger().info(
                    f'ROTATE_90 — de {math.degrees(self.locked_yaw):.1f}° '
                    f'a {math.degrees(self.target_yaw_final):.1f}° '
                    f'({"CW" if ROTATE_DIR < 0 else "CCW"})'
                )
                self.state = 'ROTATE_90'

        elif self.state == 'ROTATE_90':
            setpoint.position = [self.locked_x, self.locked_y, TARGET_Z]
            setpoint.yaw = self.target_yaw_final
            yaw_err = math.atan2(
                math.sin(self.target_yaw_final - self.current_yaw),
                math.cos(self.target_yaw_final - self.current_yaw),
            )
            self.rotate_stable_cnt = self.rotate_stable_cnt + 1 if abs(yaw_err) < ROTATE_TOL else 0

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'ROTATE_90 | actual={math.degrees(self.current_yaw):.1f}° '
                    f'target={math.degrees(self.target_yaw_final):.1f}° '
                    f'err={math.degrees(yaw_err):.1f}° '
                    f'stable={self.rotate_stable_cnt}/{ROTATE_STABLE}'
                )
            if self.rotate_stable_cnt >= ROTATE_STABLE:
                self.get_logger().info(
                    f'Rotación 90° COMPLETADA')
                self.state = 'HOLD_FINAL'

        elif self.state == 'HOLD_FINAL':
            setpoint.position = [self.locked_x, self.locked_y, TARGET_Z]
            setpoint.yaw = self.target_yaw_final
            if self.counter % 40 == 0:
                self.get_logger().info('MISION FINALIZADA')
        
        self.trajectory_pub.publish(setpoint)
        self.counter += 1

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
        node = WhiteBoardMission()
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