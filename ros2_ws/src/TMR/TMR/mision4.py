"""
Mision 4: Gates Azules

Despegue, búsqueda de gate azul, alineación, cruce y landing.
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

from geometry_msgs.msg import Point

TARGET_ALTITUDE = 1.0
TARGET_Z        = -1.0
HOLD_DURATION   = 3.0

SEARCH_YAWSPEED = 0.25   # rad/s

IMG_W = 640
IMG_H = 480
IMG_CX = IMG_W / 2   # 320 px   centro horizontal
IMG_CY = IMG_H / 2   # 240 px  centro vertical

ALIGN_KP_LAT  = 0.003   # ganancia lateral  (error px → m/s)
ALIGN_KP_VERT = 0.003   # ganancia vertical (error px → m/s)
ALIGN_MAX_V   = 0.25    # velocidad máxima de corrección [m/s]
ALIGN_TOL_LAT  = 30     # tolerancia lateral  [px]
ALIGN_TOL_VERT = 30     # tolerancia vertical [px]
ALIGN_STABLE  = 20      # ticks estables para confirmar alineación

CROSS_SPEED    = 0.4    # m/s hacia adelante
CROSS_DURATION = 3.0    # segundos cruzando

GATE_TIMEOUT = 0.8      # segundos sin ver la gate → perdida


class Mission4(Node):
    def __init__(self):
        super().__init__('mission4')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # BUG CORREGIDO: era "reliabilty"
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Publicadores ──────────────────────────────────────────────────────
        self.offboard_pub   = self.create_publisher(OffboardControlMode,  '/fmu/in/offboard_control_mode', pub_qos)  # BUG: era "oofboard_pub"
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',  pub_qos)
        self.cmd_pub        = self.create_publisher(VehicleCommand,        '/fmu/in/vehicle_command',      pub_qos)  # BUG: era "/fmu/ib/..."

        # ── Subscriptores ─────────────────────────────────────────────────────
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)  # BUG: era local_pos_pub
        self.attitude_sub  = self.create_subscription(VehicleAttitude,      '/fmu/out/vehicle_attitude',       self.attitude_cb,  sub_qos)
        self.flow_sub      = self.create_subscription(DistanceSensor,       '/fmu/out/distance_sensor',        self.flow_cb,      sub_qos)
        self.gate_sub      = self.create_subscription(Point, '/m1/blue/coordinates', self.gate_cb, 1)

        # ── Estado de la gate ─────────────────────────────────────────────────
        self.gate_center    = None   # [cx_px, cy_px] o None si no se ve
        self.last_gate_time = 0.0

        # ── Posición actual ───────────────────────────────────────────────────
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        # ── Posiciones bloqueadas ─────────────────────────────────────────────
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        self.cross_locked_yaw = None

        # ── Máquina de estados ────────────────────────────────────────────────
        self.state = "INIT"
        self.counter = 0

        self.stable_ticks      = 0
        self.align_stable_cnt  = 0
        self.hold_start_time   = None
        self.cross_start_time  = None

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.get_logger().info("Misión 4 — Gates Azules iniciada")

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
                f"Distancia: {self.current_distance:.4f} m"
            )

    def gate_cb(self, msg: Point):
        """
        Recibe el centro de la bounding box de la gate azul en píxeles.
        msg.x → coordenada horizontal (0 = izquierda, 640 = derecha)
        msg.y → coordenada vertical   (0 = arriba,    480 = abajo)
        msg.z → no usado (siempre 0)

        Solo se guarda si el centro está dentro de la imagen
        (filtra detecciones parciales en los bordes).
        """
        cx = msg.x
        cy = msg.y

        # Filtro de detección parcial: descarta si el centro
        # está demasiado cerca del borde (margen de 10% de la imagen)
        margin_x = IMG_W * 0.10   # 64 px
        margin_y = IMG_H * 0.10   # 48 px

        if not (margin_x < cx < IMG_W - margin_x and
                margin_y < cy < IMG_H - margin_y):
            return   # centro en el borde → probablemente detección parcial

        self.gate_center    = [cx, cy]
        self.last_gate_time = time.time()

        if self.counter % 10 == 0:
            err_x = cx - IMG_CX
            err_y = cy - IMG_CY
            self.get_logger().info(
                f'Gate detectada | cx={cx:.0f} cy={cy:.0f} px | '
                f'err_x={err_x:+.0f} err_y={err_y:+.0f} px'
            )

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Invalidar gate si lleva más de GATE_TIMEOUT sin verse
        if time.time() - self.last_gate_time > GATE_TIMEOUT:
            self.gate_center = None

        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = True
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

        safe_x       = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y       = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw     = self.locked_yaw if self.locked_yaw is not None else 0.0

        if self.state == "INIT":
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = 'ARMING'

        elif self.state == "ARMING":
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f"ARMED | Ascendiendo a {TARGET_ALTITUDE:.1f} m")
                self.state = 'TAKEOFF'

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            err_alt = abs(self.current_distance - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0  # BUG: era "stavle_ticks" y "current_disntance"
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"err={err_alt:.2f} m stable={self.stable_ticks}/10"
                )
            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()  # bug: era "hold_stat_time"
                self.get_logger().info(f"Altura estable en {self.current_distance:.2f} m")
                self.state = 'HOLD'

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'HOLD {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.current_distance:.2f} m'
                )
            if elapsed >= HOLD_DURATION:
                self.get_logger().info('SEARCH_GATE — girando en busca de la gate azul')
                self.state = 'SEARCH_GATE'

        elif self.state == "SEARCH_GATE":
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yawspeed = SEARCH_YAWSPEED
            if self.gate_center is not None:
                self.locked_yaw = self.current_yaw
                self.align_stable_cnt = 0
                self.get_logger().info(
                    f'Gate encontrada en ({self.gate_center[0]:.0f}, {self.gate_center[1]:.0f}) px → ALIGN'
                )
                self.state = 'ALIGN'

        elif self.state == "ALIGN":
            """
            Centra el drone sobre la gate en píxeles antes de cruzar.
            Error lateral  → velocidad en Y del cuerpo (izq/der)
            Error vertical → velocidad en Z NED (arriba/abajo)
            """
            setpoint.yaw = self.locked_yaw
            setpoint.position[2] = TARGET_Z

            if self.gate_center is None:
                # Gate perdida: frenar y esperar
                setpoint.velocity[0] = 0.0
                setpoint.velocity[1] = 0.0
                self.align_stable_cnt = 0
                if self.counter % 10 == 0:
                    self.get_logger().warn('Gate perdida durante ALIGN — esperando')
            else:
                cx, cy = self.gate_center

                # Error en píxeles respecto al centro de imagen
                err_lat  = cx - IMG_CX   # + = gate a la derecha
                err_vert = cy - IMG_CY   # + = gate abajo

                # Velocidades en frame del cuerpo
                bvy = self._clamp( ALIGN_KP_LAT  * err_lat,  -ALIGN_MAX_V, ALIGN_MAX_V)
                bvz = self._clamp( ALIGN_KP_VERT * err_vert, -ALIGN_MAX_V, ALIGN_MAX_V)

                # Rotar lateral de cuerpo → NED
                yaw = self.locked_yaw
                setpoint.velocity[0] = -bvy * math.sin(yaw)
                setpoint.velocity[1] =  bvy * math.cos(yaw)
                setpoint.velocity[2] =  bvz   # NED: + = bajar

                # ¿Alineado?
                aligned = (abs(err_lat) < ALIGN_TOL_LAT and
                           abs(err_vert) < ALIGN_TOL_VERT)
                self.align_stable_cnt = self.align_stable_cnt + 1 if aligned else 0

                if self.counter % 10 == 0:
                    self.get_logger().info(
                        f'ALIGN | err_lat={err_lat:+.0f} err_vert={err_vert:+.0f} px | '
                        f'stable={self.align_stable_cnt}/{ALIGN_STABLE}'
                    )

                if self.align_stable_cnt >= ALIGN_STABLE:
                    self.cross_locked_yaw = self.locked_yaw
                    self.cross_start_time = self.get_clock().now()
                    self.get_logger().info(
                        f'¡ALINEADO! → CROSS | yaw={math.degrees(self.cross_locked_yaw):.1f}°'
                    )
                    self.state = 'CROSS'

        elif self.state == "CROSS":
            """
            Avanza recto a través de la gate durante CROSS_DURATION segundos.
            """
            yaw = self.cross_locked_yaw
            vx_ned =  math.cos(yaw) * CROSS_SPEED
            vy_ned =  math.sin(yaw) * CROSS_SPEED

            setpoint.velocity[0] = vx_ned
            setpoint.velocity[1] = vy_ned
            setpoint.velocity[2] = 0.0
            setpoint.yaw = yaw

            elapsed = self._elapsed_s(self.cross_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'CROSS | t={elapsed:.1f}/{CROSS_DURATION:.0f}s'
                )
            if elapsed >= CROSS_DURATION:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info('Gate cruzada → HOLD_FINAL')
                #self.state = 'HOLD_FINAL'
                self.state = 'LAND'

        # elif self.state == "HOLD_FINAL":
        #     setpoint.position = [self.current_x, self.current_y, TARGET_Z]
        #     setpoint.yaw = self.cross_locked_yaw
        #     if self.counter % 40 == 0:
        #         self.get_logger().info('MISIÓN FINALIZADA — en posición')
        elif self.state == "LAND":
            setpoint.position = [float('nan'), float('nan'), 0.0]
            setpoint.velocity =[0.0, 0.0, 0.2]
            
            if self.current_distance < 0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDING COMPLETED")

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
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main():
    rclpy.init()
    node = Mission4()
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